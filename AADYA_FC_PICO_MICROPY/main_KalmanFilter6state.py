from machine import Pin,I2C, UART, SPI, ADC, PWM
from math import trunc, log, asin, cos, degrees
import mpu6050_lib, time, uos, SDcard_lib
from bmp280_lib import *
import matrixOps as mops

# GS CALIBRATION PARAMETERS

gs_pr = 100900 # Ground Station pressure (Pa)
gs_alt = 75 # Ground Station altitude (m)
gs_temp = 300 # Ground Station temperature (K)
gs_g = 9.81 # Ground Station Gravity (m/s^2)


# SYSTEM STATUS
# [0]Boot [1]Calibrate [2]Ascent [3]Descent [4]Touch Down
flight_state = 0

mpu_status = False
bmp_status = False
sdc_status = False
deploy_status = False

last_time = 0 # time.time_ns() when last run of loop start
pr_altimeter = [0.0,0.0,0.0,0.0,0.0,0.0] # Vector to store last 6 altitudes, 0th element is latest
pr_fall_v = [0.0,0.0,0.0,0.0,0.0,0] # Vector to store last 5 fall velocities as deduced from pressure, 0th element is latest, last element is time.time_ns of last calculation7
# SYSTEM STATUS ENDS

# PIN CONFIGURATION
INT_LED = Pin("LED",Pin.OUT) # Internal LED
pico_temp = ADC(4) # Internal Temperature sensor

STATE1 = Pin(16,Pin.OUT,Pin.PULL_DOWN)
STATE2 = Pin(17,Pin.OUT,Pin.PULL_DOWN)
STATE3 = Pin(18,Pin.OUT,Pin.PULL_DOWN)
STATE4 = Pin(19,Pin.OUT,Pin.PULL_DOWN)

SENSOR_SW = Pin(15,Pin.OUT, Pin.PULL_DOWN) # Sensor switch
DEPLOY_SW = Pin(14,Pin.OUT, Pin.PULL_DOWN) # Deploy switch

#parachute_servo = PWM(27); parachute_servo.freq(50); parachute_servo.duty_u16(1200)
try:
    cs = Pin(9, Pin.OUT, value = 1)
    spi = SPI(0,
              baudrate=1000000,
              polarity=0,
              phase=0,
              bits=8,
              firstbit=SPI.MSB,
              sck=Pin(6),
              mosi=Pin(7),
              miso=Pin(8))
    # Some issue with MISO pin definintion
    # Have used a mix of SPI0 and SPI1 (Have to change)
    
    time.sleep(0.1)
except:
    print("PIN_CONFIGURATION: SPI Failed")
try:
    i2cline = I2C(id = 0, scl=Pin(21), sda = Pin(20),freq = 400000, timeout = 1000)
    time.sleep(0.1)
except:
    print("PIN_CONFIGURATION: I2C Failed")
# PIN CONFIGURATION ENDS

# POWERING ON
def power_sensors(gamma):
    gamma = bool(gamma)
    global mpu_status, bmp_status, sdc_status
    if (not gamma):
        try:
            uos.umount("/sd")
        except:
            None
    SENSOR_SW.value(gamma)
    
    mpu_status = gamma
    bmp_status = gamma
    sdc_status = gamma

    time.sleep_ms(10)
    
def power_deploy(gamma):
    global deploy_status
    DEPLOY_SW.value(gamma)
    deploy_status = gamma
    #time.sleep_ms(10)

def toggle_state_led(state):
    if state == 1:
        STATE1.toggle()
    elif state == 2:
          STATE2.toggle()
    elif state == 3:
          STATE3.toggle()
    elif state == 4:
          STATE4.toggle()
# POWERING ON ENDS
    
# SENSOR INITIALISATION
def init_sensors():
    global bmp, bmp_status, mpu, mpu_status, sd, vfs, sdc_status
    try:
        bmp = BMP280(i2cline)
        bmp.use_case(BMP280_CASE_INDOOR)
        for i in range(0,20): # Filling altimeter readings at the start to avoid the wrong altitude average in the start
            read_bmp()
    except:
        bmp_status = False
        print('SENSOR_INIT: bmp_offline')
    
    try:
        mpu = mpu6050_lib.accel(i2cline)
        #mpu.calibrate(10,gs_g)
        time.sleep_ms(50)
    except:
        mpu_status = False
        print('SENSOR_INIT: mpu_offline')
    
    try:
        uos.stat("/sd")
        sdc_online = True
    except:
        try:
            sd = SDcard_lib.SDCard(spi, cs)
            vfs = uos.VfsFat(sd)
            uos.mount(vfs, "/sd")
        except:
            sdc_online = False
            print('SENSOR_INIT: sdc_offline')
# SENSOR INITIALISATION ENDS

# SENSOR DATA UPDATE
def mpu_update():
    global mpu_status, mpu
    if mpu_status:
        try:
            mpu.get_values()
        except:
            mpu_status = False
            print('ERROR:mpu_update')
            
def read_bmp():
    global bmp_status, bmp, pr_fall_v, pr_altimeter, gs_alt, gs_temp, gs_pr, gs_g
    if bmp_status:
        try:
            pr = bmp.pressure
            pr_altimeter[1:6] = pr_altimeter[0:5]
            pr_fall_v[1:5] = pr_fall_v[0:4]
            
            # Latest altitude computation
            pr_altimeter[0] = gs_alt-8.314*gs_temp/gs_g/0.0289 * log(pr/gs_pr)
            pr_fall_v[0] = (pr_altimeter[0]-pr_altimeter[1])*1e9/(time.time_ns()-pr_fall_v[5])
            pr_fall_v[5] = time.time_ns()
            return pr #pascal
        except:
            bmp_status = False
            print('ERROR:read_bmp')
            return 0

def read_temperature():   # Allows redundency for temperature
    global mpu_status
    if mpu_status:
        try: 
            mpu.get_values()
            return trunc(mpu.temp*10)/10.0
        except:
            mpu_online = False
            print('ERROR:read_temperature_mpu_offline')
            sdc_write('mpu_offline', "/sd/ver_log.csv","a")
            return 0
    else:
        return trunc((27 - (pico_temp.read_u16()*3.3 / (65535) - 0.706)/0.001721)*10)/10.0
# SENSOR DATA UPDATE ENDS

# SD CARD FUNCTIONS
def sdc_write(text, file_path, mode):
    global sdc_status, mpu
    try:
        with open(file_path, mode) as file:
            file.write(text)
    except:
        sdc_status = False
        print('ERROR:sdc_write: sdc_offline')

def sdc_read(file_path):
    global sdc_status, mpu
    try:
        with open(file_path, "r") as file:
            return file.read()
    except:
        sdc_status = False
        print('ERROR:sdc_read: sdc_offline')
        return
# SD CARD FUNCTIONS ENDS

#
global xprev,Pprev,A,Q,R

dt = 0.05
g = 9.8
Qcov = 0.01
Rgyro = 0.01
Racc = 0.01

A = [[1,0,0,dt,0,0],
     [0,1,0,0,dt,0],
     [0,0,1,0,0,dt],
     [0,0,0,1,0,0],
     [0,0,0,0,1,0],
     [0,0,0,0,0,0]]

Q = [[Qcov,0,0,0,0,0],
     [0,Qcov,0,0,0,0],
     [0,0,Qcov,0,0,0],
     [0,0,0,Qcov,0,0],
     [0,0,0,0,Qcov,0],
     [0,0,0,0,0,Qcov]]

H = [[0,0,0,1,0,0],
     [0,0,0,0,1,0],
     [0,0,0,0,0,1],
     [0,2*g,0,0,0,0],
     [-2*g,0,0,0,0,0],
     [0,0,0,0,0,0]]

R = [[Rgyro,0,0,0,0,0],
     [0,Rgyro,0,0,0,0],
     [0,0,Rgyro,0,0,0],
     [0,0,0,Racc,0,0],
     [0,0,0,0,Racc,0],
     [0,0,0,0,0,Racc]]

# INITIALISATION STEP
x0 = [[0],
      [0],
      [0],
      [0],
      [0],
      [0]]
P0 = [[1,0,0,0,0,0],
      [0,1,0,0,0,0],
      [0,0,1,0,0,0],
      [0,0,0,1,0,0],
      [0,0,0,0,1,0],
      [0,0,0,0,0,1]]


xprev = x0
Pprev = P0

# ADCS FUNCTIONS
class ADCS:
    def __init__(self,mpu):
        self.mpu = mpu
        self.xest = 0
        self.Pest = 0
        self.xprev = x0
        self.Pprev = P0
        
    def StateEstimation(self):
        #try:
       # PREDICTION STEP
       x = mops.matrix_multiply(A,self.xprev)
       #print(x)
       P = mops.add_matrices(mops.matrix_multiply(A,mops.matrix_multiply(self.Pprev,mops.matrix_transpose(A))),Q)
       # ESTIMATION STEP
       K = mops.matrix_multiply(P,mops.matrix_multiply(mops.matrix_transpose(H),mops.matrix_inverse(mops.add_matrices(mops.matrix_multiply(H,mops.matrix_multiply(self.Pprev,mops.matrix_transpose(H))),R))))
       # Axis Remapping
       z = [[mpu.g[1]],
            [mpu.g[0]],
            [-mpu.g[2]],
            [mpu.a[1]],
            [mpu.a[0]],
            [-mpu.a[2]]]
       self.xest = mops.add_matrices(x,mops.matrix_multiply(K,mops.add_matrices(z,mops.scalar_multiply(-1,mops.matrix_multiply(H,x)))))
       self.Pest = mops.matrix_multiply(mops.add_matrices(mops.eye(6),mops.scalar_multiply(-1,mops.matrix_multiply(K,H))),P)
       self.xprev = self.xest
       self.Pprev = self.Pest
       return self.xest
        #except:
        #print("ERROR: AttitudeEstimation")
# ADCS FUNCTIONS ENDS


# MAIN CODE

power_sensors(1)
time.sleep(3)
try:
    init_sensors()
    print("Sensors initialised")
    adcs = ADCS(mpu)
    print("ADCS Initialised")
except:
    print("Initialisation Failed")

init_alt = (sum(pr_altimeter)/6)
print("Initial Altitude",init_alt)
start_time = time.time()

while True:
    INT_LED.on()
    mpu_update(); pr = read_bmp(); t = read_temperature()
    #print('ax:',mpu.a[0],'ay:',mpu.a[1],'az:',mpu.a[2])
    #print("pr:",pr)
    #print("Altitude:",sum(pr_altimeter)/6)
    #print(t)
    #print(mpu.a[2])
    
    adcs.StateEstimation()
    print(degrees(adcs.xest[1][0]))
    #print("roll:",degrees(adcs.phi_est),"pitch:",degrees(adcs.theta_est))
    
    state = 2
    toggle_state_led(state)
    time.sleep(0.05) #20Hz sampling
