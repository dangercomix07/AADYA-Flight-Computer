# AADYA-Flight-Computer

> Flight Software for AADYA series of rockets


AADYA Flight Computer (AFC) V1.0 will be the flight computer of AADYA II
It has basic functionalities like data logging, power management and parachute deployment
Has MPU6050 6DOF IMU and BMP280 Pressure and Temperature Sensor and logs data to a microSD card

AFC 2.0 will will also include control algorithms and will fly in AADYA III.
The Active Fin Stablised Model Rocket, our most ambitious project till date.

There are 2 versions of AFC
1) Micropython based for Raspberry Pi Pico
2) C++ for ESP32 using Arduino framework (Platform IO Project)

Currend Status: 
Sensor interfacing and data collection has been tested
Data logging doesnt work due to mistake in SPI pin assignment on the PCB (Will be updated soon)
Kalman Filter implementation for Roll and Pitch angle estimation from IMU (Euler angle based)
