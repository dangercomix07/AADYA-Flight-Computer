"""
Kalman Filter Library
Author: Ameya Marakarkandy
Last Update: 22-05-2025

Description:
2 state Kalman filter implementation
State(1) = angle
State(2) = gyro measurement bias
Output: Estimated angle


How to use
1) Initialise first
    kf = KalmanFilter.KF(Qangle,Qgyrobias,R)
2) can use parameters defined in class in main code
    For eg. kf.angle_prev
3) For calling functions:
    angle_est = kf.KalmanFilter(dt,gyro_rate,angle_acc)
"""

class KF():
    def __init__(self,Qangle,Qgyrobias,R):
        # State
        self.angle_prev = 0
        self.gyro_bias_prev = 0
        # Error covariance matrix
        self.P00_prev = 0
        self.P01_prev = 0
        self.P10_prev = 0
        self.P11_prev = 0

        self.Qangle = Qangle		# Accelerometer variance
        self.Qgyrobias = Qgyrobias 	# Bias in gyro measurement
        self.R = R 					# variance of measurement
        
    def KalmanFilter(self,dt,
                    gyro_rate,angle_acc): # Measurements
        
        # PREDICTION STEP
        angle = self.angle_prev + dt*(gyro_rate - self.gyro_bias_prev)
        gyro_bias = self.gyro_bias_prev
        
        P00 = self.P00_prev - self.P01_prev*dt -dt*(self.P10_prev - self.P11_prev*dt) + self.Qangle*dt
        P01 = self.P01_prev - self.P11_prev*dt
        P10 = self.P10_prev - self.P11_prev*dt
        P11 = self.P11_prev + self.Qgyrobias*dt
        
        y = angle_acc - self.angle_prev # Innovation
        S = P00 + self.R # Innovation covariance
        
        K0 = P00/S
        K1 = P10/S
        
        angle_est = angle + K0*y
        gyro_bias_est = gyro_bias + K1*y
        
        P00_est = (1-K0)*P00
        P01_est = (1-K0)*P01
        P10_est = -K1*P00 + P10
        P11_est = -K1*P01 + P11
         
        self.angle_prev = angle_est
        self.gyro_bias_prev = gyro_bias_est
        self.P00_prev = P00_est
        self.P01_prev = P01_est
        self.P10_prev = P10_est
        self.P11_prev = P11_est
        
        return angle_est
                    
                    
