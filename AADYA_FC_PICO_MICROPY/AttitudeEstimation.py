# Attitude Estimation
# Author: Ameya Marakarkandy
# Last Update: 22-05-2025

# Comments
# Implemented complementary filter

from machine import Pin,I2C
import mpu6050_lib, time
import math

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

global phi,theta,psi
global g

g = 9.81
phi= 0
theta = 0
psi = 0

while True:
    INT_LED.on()
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
    
    # Kinematic equations
    phidot = q*math.sin(phi)/math.cos(theta) + r*math.cos(phi)/math.cos(theta)
    thetadot = q*math.cos(phi) -r*math.sin(phi)
    psidot = p + q*math.sin(phi)*math.tan(theta) + r*math.cos(phi)*math.tan(theta)
    
    # Attitude measureents from gyroscope (Integration)
    phi_gyro = phi + phidot
    theta_gyro = theta + thetadot
    psi_gyro = psi + psidot
    
    # Implementation of complementary filterfor pitch angle (static alpha)
    alpha = 0.9 #Higher confidence on accelerometer readings
    theta = alpha*theta_acc +(1-alpha)*theta_gyro
    phi = alpha*phi_acc + (1-alpha)*phi_gyro
    
    # Comparing the angles estimated
    print("theta:",math.degrees(theta),"theta_acc:",math.degrees(theta_acc),"theta_gyro:",math.degrees(theta_gyro))
    #print("roll:",math.degrees(phi),"roll_acc:",math.degrees(phi_acc),"roll_gyro:",math.degrees(phi_gyro))

    STATE2.toggle()
    time.sleep(0.05) #20Hz sampling
    # As computation increases lower sampling rate provides better results
    
    