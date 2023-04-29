import board
import busio
import adafruit_mpu6050
import adafruit_bmp280
import time
import math
import digitalio
import storage
import sdcardio


led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

try:
    i2c0 = busio.I2C(board.GP19, board.GP18)
    mpu = adafruit_mpu6050.MPU6050(i2c0)
except:
    print("MPU not working")
    pass
try:
    i2c1 = busio.I2C(board.GP21, board.GP20)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c1)

    bmp280.sea_level_pressure = 1013.25
    #bmp280._iir_filter = 4
    # Set the sensor to high resolution mode
    bmp280.mode = adafruit_bmp280.MODE_NORMAL
    bmp280.standby_period = adafruit_bmp280.STANDBY_TC_500
    bmp280.iir_filter = adafruit_bmp280.IIR_FILTER_X16
    bmp280.overscan_pressure = adafruit_bmp280.OVERSCAN_X16
    bmp280.overscan_temperature = adafruit_bmp280.OVERSCAN_X2
    #bmp280._overscan_pressure = 16

except:
    print("BMP not working")
    pass
# Initialize Kalman filter variables
Q_angle = 0.01
Q_gyro = 0.0003
R_angle = 0.01
x_angle = 0.0
P_00 = 0.0
P_01 = 0.0
P_10 = 0.0
P_11 = 0.0
K_0 = 0.0
K_1 = 0.0

dt =0.001
prev_altitude = bmp280.altitude

while True:
    # Read accelerometer and gyroscope data
    accel_data = mpu.acceleration
    gyro_data = mpu.gyro

    # Filter accelerometer data
    dt = time.monotonic() - dt
    x_angle += gyro_data[0] * dt
    x_angle += (accel_data[0] - x_angle) * Q_angle
    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt
    P_01 +=  - dt * P_11
    P_10 +=  - dt * P_11
    P_11 +=  + Q_gyro * dt

    # Filter gyroscope data
    y_angle = 0.0
    S = P_00 + R_angle
    K_0 = P_00 / S
    K_1 = P_10 / S
    y_angle += K_0 * (accel_data[1] - x_angle)
    x_angle += K_1 * (accel_data[1] - x_angle)
    P_00 -= K_0 * P_00
    P_01 -= K_0 * P_01
    P_10 -= K_1 * P_00
    P_11 -= K_1 * P_01

    # Calculate angles from filtered data
    x_angle = math.atan2(accel_data[1], accel_data[2])
    y_angle = math.atan2(-accel_data[0], math.sqrt(accel_data[1]**2 + accel_data[2]**2))
    z_angle = math.atan2(gyro_data[1], gyro_data[2])
    
    #Read BMP280 data
    temperature = bmp280.temperature
    pressure = bmp280.pressure
    altitude = bmp280.altitude

    #print(math.degrees(x_angle))
    #print(math.degrees(y_angle))
    #print(math.degrees(z_angle))
    #print(pressure)
    #print(temperature)
    #print(altitude)
    dh = (altitude - prev_altitude)/dt
    
    prev_altitude = altitude
    time.sleep(0.001)


