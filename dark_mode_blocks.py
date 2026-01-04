import cv2
import numpy as np
import urllib.request
import threading
import serial
import time

url = 'http://192.168.4.1/' 
SERIAL_PORT = '/dev/ttyUSB1' 
BAUD_RATE = 115200

MIN_BLOCK_AREA = 1000 
COOLDOWN_TIME = 1.0 

lower_dark = np.array([0, 0, 0])      
upper_dark = np.array([180, 255, 30]) 

class ESP32Camera:
    def __init__(self, url):
        self.url = url
        self.current_frame = None
        self.running = False
        self.lock = threading.Lock()
        try:
            self.stream = urllib.request.urlopen(self.url, timeout=2)
            self.running = True
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()
            print("Camera Connected: STRICT DARK MODE + ROI CROP")
        except Exception as e:
            print(f"Connection Failed: {e}")

    def update(self):
        bytes_data = b''
        while self.running:
            try:
                chunk = self.stream.read(10240) 
                bytes_data += chunk
                last_end = bytes_data.rfind(b'\xff\xd9')
                if last_end != -1:
                    start = bytes_data.rfind(b'\xff\xd8', 0, last_end)
                    if start != -1:
                        jpg = bytes_data[start:last_end+2]
                        bytes_data = bytes_data[last_end+2:]
                        if len(bytes_data) > 10000: bytes_data = b'' 
                        if len(jpg) > 0:
                            try:
                                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if img is not None:
                                    with self.lock:
                                        self.current_frame = img
                            except: pass
                if len(bytes_data) > 20000: bytes_data = b''
            except Exception as e:
                try:
                    self.stream = urllib.request.urlopen(self.url, timeout=2)
                    bytes_data = b''
                except: time.sleep(0.01)

    def get_frame(self):
        with self.lock: return self.current_frame

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 Solenoid on {SERIAL_PORT}")
    time.sleep(2) 
except Exception as e:
    print(f"Serial Error: {e}")

dirty_counts = [0, 0, 0, 0] 
lane_last_sent = [0, 0, 0, 0] 

cam = ESP32Camera(url)
time.sleep(1)

print("System Running... ROI MASK ENABLED (Ignoring Walls)")

print("System Running... Custom Lane Widths")

while True:
    frame = cam.get_frame()

    if frame is not None:
        height, width, _ = frame.shape
        
       
        line1_x = int(width * 0.30) 
        
        line2_x = int(width * 0.50) 
        
        line3_x = int(width * 0.70) 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_dark, upper_dark)
        
        mask[:, 0:40] = 0           
        mask[:, width-40:width] = 0 

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > MIN_BLOCK_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + w // 2
                
                lane_index = -1
                if center_x < line1_x:
                    lane_index = 0 # Lane 1
                elif center_x < line2_x:
                    lane_index = 1 # Lane 2 
                elif center_x < line3_x:
                    lane_index = 2 # Lane 3
                else:
                    lane_index = 3 # Lane 4

                if lane_index != -1:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    actual_lane_number = lane_index + 1

                    if (time.time() - lane_last_sent[lane_index] > COOLDOWN_TIME):
                        dirty_counts[lane_index] += 1
                        lane_last_sent[lane_index] = time.time()
                        print(f"!!! DARK BLOCK: Lane {actual_lane_number} !!!")
                        if ser and ser.is_open:
                            msg = f"{actual_lane_number}\n"
                            ser.write(msg.encode('utf-8'))

        cv2.line(frame, (line1_x, 0), (line1_x, height), (0, 255, 255), 2)
        cv2.line(frame, (line2_x, 0), (line2_x, height), (0, 255, 255), 2)
        cv2.line(frame, (line3_x, 0), (line3_x, height), (0, 255, 255), 2)
        
        cv2.line(frame, (40, 0), (40, height), (0, 0, 255), 2)
        cv2.line(frame, (width-40, 0), (width-40, height), (0, 0, 255), 2)

        cv2.putText(frame, f"Stats: {dirty_counts}", (10, height - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('DEBUG: Mask View', mask) 
        cv2.imshow('Main Camera', frame)

    if cv2.waitKey(1) == ord('q'):
        cam.running = False
        break

if ser: ser.close()
cv2.destroyAllWindows()
if ser: ser.close()
cv2.destroyAllWindows()