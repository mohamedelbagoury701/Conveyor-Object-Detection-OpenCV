# 🏭 Smart Industrial Sorting System using Computer Vision & IoT

An ultra-low latency, real-time sorting system designed for high-speed conveyor belts. This project implements a **Distributed Control Architecture** using two ESP32 nodes and a Python-based computer vision server to classify and sort objects based on color and brightness with **Zero Latency** actuation.

![Project Demo](Images/screenshot_red.png)
*(Demo of the computer vision logic detecting objects in real-time)*

## 🚀 Key Features

* **Zero Latency Streaming:** Developed a custom "Buffer Killer" algorithm in Python to parse MJPEG streams manually, eliminating the standard 2-second lag found in `urllib` and ensuring real-time synchronization.
* **Dual Detection Modes:**
    * **Red Mode:** Detects colored objects using HSV masking logic.
    * **Dark Mode:** Detects black/grey objects on dark belts using adaptive `Value` channel thresholding (< 30).
* **Smart ROI & Active Area:** Automatically crops conveyor walls (40px margins) and dynamically calculates 4 virtual lanes to prevent false positives from shadows.
* **Distributed Architecture:** Decouples vision (ESP32-CAM) from actuation (Standard ESP32) to maximize processing efficiency.
* **Queue-Based Actuation:** Implements a non-blocking `ArduinoQueue` system with microsecond precision (`micros()`) to handle multiple objects simultaneously across different lanes.

---

## 🛠️ System Architecture

The system is built on a **Distributed Node Architecture**:

### 1. Vision Node (ESP32-CAM)
* **Role:** Wireless Video Streamer.
* **Configuration:** Acts as a standalone **WiFi Access Point** (`Fast-Cam-Project`) to isolate traffic from local networks.
* **Performance:** Streams QVGA video with an XCLK of 20MHz for high framerate.

### 2. Processing Unit (Python Server)
* **Role:** The "Brain". Handles Image Processing & Decision Making.
* **Logic:**
    * Connects to the ESP32-CAM AP.
    * Processes frames using **OpenCV** (Morphological Ops, Contours, Centroids).
    * Determines the object's lane (1-4) based on its X-coordinate centroid.
    * Sends actuation commands via **UART Serial** (115200 baud).

### 3. Actuator Node (ESP32 Dev Module)
* **Role:** Mechanical Control.
* **Logic:**
    * Controls the Conveyor Belt DC Motor via **L298N Driver** (PWM Speed Control).
    * Controls 4 Sorting Solenoids (Simulated by Lamps/LEDs).
    * Uses a **Time-Window** logic: When a command is received, it's pushed to a queue with a timestamp. The system calculates the exact `micros()` needed for the object to reach the gate before triggering the actuator.

---

## 📦 Hardware Specifications

* **Microcontrollers:** 1x ESP32-CAM (AI-Thinker), 1x ESP32 WROOM.
* **Motor Driver:** L298N H-Bridge.
* **Power:** 12V Lead-Acid Battery for portability.
* **Conveyor Belt:** Custom-designed using **Medical Compression Bandage** material for optimal friction and cost-efficiency.
* **Actuators:** 4x Push Solenoids / Indicators.
* **Chassis:** Custom Wooden Frame.

---

## 💻 Software Logic & Algorithms

### The "Buffer Killer" (Python)
Standard HTTP streams accumulate delay over time due to buffering. We implemented a custom parser that reads large chunks (`10240 bytes`) and searches for the **last** JPEG end-byte (`\xff\xd9`) using `rfind()`. This forces the buffer to flush old frames, ensuring the processed image is always the most recent one available.

### Center-Point Lane Logic
Instead of simple image slicing, the system calculates the **Centroid** of the detected contour.
```python
center_x = x + w // 2
lane_index = int((center_x - active_start_x) // lane_width)

## 🔧 Installation & Usage

### 1. Hardware Setup
* Connect the Laptop to the ESP32-CAM WiFi: SSID `Fast-Cam-Project`.
* Connect the Actuator ESP32 to the Laptop via USB.

### 2. Flash Firmware
* Upload `cam/cam.ino` to the ESP32-CAM.
* Upload `serialesp/serialesp.ino` to the Actuator ESP32 Node.

### 3. Install Python Dependencies
```bash
pip install opencv-python numpy pyserial

### 4. Run the System
* **For Red Objects:**
    ```bash
    python Python_Server/red_blocks.py
    ```
    *(See)*

* **For Dark/Black Objects:**
    ```bash
    python Python_Server/dark_mode_blocks.py
    ```
    *(See)*

---

## 👥 Authors
* **[Mohamed Elbagoury]** 
