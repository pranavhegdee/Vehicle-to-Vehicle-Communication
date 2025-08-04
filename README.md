# Vehicle-to-Vehicle-Communication
V2V communication using STM32F407VG where vehicles share useful information like the location, distance between them, acceleration and the speed. It uses ultrasonic sensor, imu sensor, gps module and ir sensor along with esp32 for wifi communication.<br />

## 📌 Features<br />
Distance Detection:<br />
-Uses an Ultrasonic Sensor to measure the distance from the vehicle ahead.<br />
-Alerts the driver if the distance falls below 30 cm.<br />
<br />
Vehicle Location Tracking:<br />
-GPS Module provides real-time coordinates of the vehicle.<br />
-Speed & Acceleration Measurement:<br />

-IR Sensor detects wheel rotations to calculate vehicle speed.<br />

-IMU Sensor (Accelerometer + Gyroscope) provides acceleration and orientation data.<br />

-V2V Data Communication:<br />
-Sensor data is collected by STM32 and sent to ESP32 via UART.<br />
-ESP32 (Transmitter) sends data over Wi-Fi to another ESP32 (Receiver).<br />

-Driver Alerts:<br />
LED indicators for negative acceleration (deceleration) and dangerously close distance.<br />
<br />
## ⚙️ Working Principle
Data Collection (STM32F407VG):<br />
-Ultrasonic → Distance<br />
-GPS → Latitude & Longitude<br />
-IMU → Acceleration & Orientation<br />
-IR → Speed of the vehicle<br />
<br />
Data Transmission (UART → ESP32):<br />
-STM32 sends the collected data to the ESP32 Transmitter via UART.<br />
V2V Communication (ESP32 Wi-Fi):<br />
-ESP32 (Tx) sends the data wirelessly to ESP32 (Rx).<br />
-The receiving ESP32 makes the data available for visualization or alerts.<br />
<br />
Driver Alerts (STM32 LED):<br />
-LED ON if Distance < 30 cm.<br />
-LED ON if Acceleration < 0 (decelerating).<br />
<br />
<br />
<img width="735" height="468" alt="Image" src="https://github.com/user-attachments/assets/b4f957ae-cc85-42fa-a3b1-607c280899fd" />
