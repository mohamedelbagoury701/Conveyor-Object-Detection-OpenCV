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

CROP_MARGIN = 40 

lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])




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
            print("Connected: Red Mode + ROI Active Area")
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
                        
                        if len(bytes_data) > 10000: 
                            bytes_data = b'' 

                        if len(jpg) > 0:
                            try:
                                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if img is not None:
                                    with self.lock:
                                        self.current_frame = img
                            except:
                                pass
                if len(bytes_data) > 20000:
                    bytes_data = b''
            except Exception as e:
                try:
                    self.stream = urllib.request.urlopen(self.url, timeout=2)
                    bytes_data = b''
                except:
                    time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            return self.current_frame


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

print(f"System Running... Logic: Red Detection + Cropping {CROP_MARGIN}px")

while True:
    frame = cam.get_frame()

    if frame is not None:
        height, width, _ = frame.shape
        
        
        active_start_x = CROP_MARGIN
        active_end_x = width - CROP_MARGIN
        active_width = active_end_x - active_start_x
        
        lane_width = active_width / 4.0
        
        line1_x = int(active_start_x + lane_width * 1)
        line2_x = int(active_start_x + lane_width * 2)
        line3_x = int(active_start_x + lane_width * 3)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        
        mask[:, 0:CROP_MARGIN] = 0           
        mask[:, width-CROP_MARGIN:width] = 0 
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > MIN_BLOCK_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + w // 2
                center_y = y + h // 2

                lane_index = -1
                
                if center_x < active_start_x or center_x > active_end_x:
                     lane_index = -1
                else:
                    lane_index = int((center_x - active_start_x) // lane_width)

                if 0 <= lane_index <= 3:
                    
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                    
                    actual_lane_number = lane_index + 1

                    if (time.time() - lane_last_sent[lane_index] > COOLDOWN_TIME):
                        dirty_counts[lane_index] += 1
                        lane_last_sent[lane_index] = time.time()
                        
                        print(f"!!! RED ACTION: Lane {actual_lane_number} (Center at {center_x}) !!!")
                        print(f"Stats: {dirty_counts}")

                        if ser and ser.is_open:
                            msg = f"{actual_lane_number}\n"
                            ser.write(msg.encode('utf-8'))

        cv2.line(frame, (line1_x, 0), (line1_x, height), (0, 255, 255), 2)
        cv2.line(frame, (line2_x, 0), (line2_x, height), (0, 255, 255), 2)
        cv2.line(frame, (line3_x, 0), (line3_x, height), (0, 255, 255), 2)
        
        cv2.line(frame, (active_start_x, 0), (active_start_x, height), (0, 0, 255), 2)
        cv2.line(frame, (active_end_x, 0), (active_end_x, height), (0, 0, 255), 2)

        cv2.putText(frame, f"Stats: {dirty_counts}", (10, height - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Red Sorter + ROI', frame)

    if cv2.waitKey(1) == ord('q'):
        cam.running = False
        break

if ser: ser.close()
cv2.destroyAllWindows()