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
