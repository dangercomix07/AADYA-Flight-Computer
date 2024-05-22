"""
 Kalman Filter Testing code
 Author: Ameya Marakarkandy
 Last Update: 22-05-2025

 Comments
 For testing Kalman Filter library
 Implemented and tested Kalman Filter for roll and pitch angle
"""

from machine import Pin,I2C
import mpu6050_lib, time
import math
import KalmanFilter

INT_LED = Pin("LED",Pin.OUT) # Internal LED
STATE2 = Pin(17,Pin.OUT,Pin.PULL_DOWN)

SENSOR_SW = Pin(15,Pin.OUT, Pin.PULL_DOWN) # Sensor switch
# Powering the sensors
SENSOR_SW.on()
time.sleep(2)
print("Sensors powered")

try:
    i2cline = I2C(id = 0, scl=Pin(21), sda = Pin(20),freq = 400000, timeout = 1000)
    time.sleep(0.1)
    print("I2C line established")
except:
    print("I2C Failed")
    
try:
    mpu = mpu6050_lib.accel(i2cline)
    #mpu.calibrate(10,gs_g)
    time.sleep_ms(50)
    print("MPU Initialise")
except:
    print("MPU_INIT FAILED")
    
g = 9.81
phi= 0
theta = 0
psi = 0

# KALMAN FILTER
dt = 0.05

# INITIALISATION STEP
theta_0 = 0
gyro_bias_0 = 0

P00_0 = 0
P01_0 = 0
P10_0 = 0
P11_0 = 0

Qtheta = 0.001
Qgyrobias = 0.003 
R = 0.03 # Measurement variance (accelerometer)

kf_pitch = KalmanFilter.KF(Qtheta,Qgyrobias,R)
kf_roll = KalmanFilter.KF(Qtheta,Qgyrobias,R)

INT_LED.on()

while True:
    mpu.get_values()
    ax = mpu.a[1]
    ay = mpu.a[0]
    az = -mpu.a[2]
    p = mpu.g[1]
    q = mpu.g[0]
    r = -mpu.g[2]
    
    # Attitude measurements from accelerometer
    theta_acc = math.asin(ax/g)
    phi_acc = math.asin(-ay/(g*math.cos(theta)))
    # Gyro measurements
    theta_gyro = kf_pitch.angle_prev + dt*q # Pure numerical integration
    phi_gyro = kf_roll.angle_prev + dt*p
    
    theta_est = kf_pitch.KalmanFilter(dt,q,theta_acc)
    phi_est = kf_roll.KalmanFilter(dt,p,phi_acc)
    
    #print("theta_kalman:",math.degrees(theta_est),"theta_acc:",math.degrees(theta_acc),"theta_gyro:",math.degrees(theta_gyro))
    #print("phi_kalman:",math.degrees(phi_est),"theta_acc:",math.degrees(phi_acc),"theta_gyro:",math.degrees(phi_gyro))
    print("roll:",math.degrees(phi_est),"pitch:",math.degrees(theta_est))
    
    STATE2.toggle()
    time.sleep(0.05) #20Hz sampling
    # As computation increases lower sampling rate provides better results
    
    
