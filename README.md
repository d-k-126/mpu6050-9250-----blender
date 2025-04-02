# Research and Development of a Wearable Device for Gait Simulation and Analysis

## Introduction
This project focuses on developing a wearable system for gait simulation and analysis. The system utilizes ESP32 microcontrollers and multiple MPU6050 sensors to capture motion data, process it, and transmit it to Blender for real-time visualization. The goal is to aid in medical diagnostics, health monitoring, and motion research.

![ảnh(3)](https://github.com/user-attachments/assets/a7d3d236-69ef-412a-8b90-cd1d1636b262)


## Device Requirements
- **ESP32**: Minimum 1 unit
- **MPU6050**: Minimum 2 units

## Features
- Wireless data transmission via Wi-Fi
- Real-time motion tracking and analysis
- Noise filtering and signal processing
- 3D motion visualization in Blender

## Installation
1. **ESP32 Setup:**
   - Install ESP-IDF or Arduino IDE
   - Flash the firmware for sensor data acquisition
2. **Blender Setup:**
   - Install Python dependencies
   - Configure Blender for real-time data reception
3. **Network Configuration:**
   - Set up a local Wi-Fi network for ESP32 communication
   
## Usage
1. Power on the ESP32 and ensure MPU6050 sensors are correctly connected.
2. Start the data transmission service.
3. Open Blender and run the Python script to visualize motion in real-time.

## Troubleshooting
- **Lagging Data:** Optimize Wi-Fi communication and data filtering.
- **No Data Received:** Check serial output from ESP32 and ensure correct network settings.
- **Incorrect Motion Output:** Calibrate the MPU6050 sensors and verify data processing algorithms.

## Future Development
- Integration of more advanced IMUs for better accuracy.
- Optimization of data transmission for reduced latency.
- Expansion of the system for full-body motion capture.

## License
This project is open-source and available under the MIT License.

