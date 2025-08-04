# Vehicle-to-Vehicle-Communication
V2V communication using STM32F407VG where vehicles share useful information like the location, distance between them, acceleration and the speed. It uses ultrasonic sensor, imu sensor, gps module and ir sensor along with esp32 for wifi communication.
The ultrasonic sensor gives the distance between the vehicle in the front and the drivers' vehicle, gps module gives the exact coordinates of the vehicle, IMU sensor gives the acceleration and oerientation, ir sensor is used to calculate speed by marking on the wheel.
These data is collected by the STM32 and then sent to ESP32 by UART. The transmitter ESp32 sends the data to Reciever ESP32 using Wifi,
Using the data, visual indicators are given,
If the acceleration is negative, led is turned on and driver is alerted.
If distance is less than 30cm, led is turned on.
