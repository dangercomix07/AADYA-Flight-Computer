"""
Extended Kalman Filter
Author: Ameya Marakarkandy
Last updated: 05/06/2024

#STEPS
Initialisation
Prediction
Measurement
Estimation and update
"""
import math
import matrixOps

class EKF():
    def __init__(self,Qangle,Qgyrobias,R,T):
        # State
        self.q_prev = [0.0,0.0,0.0,0.0]
        
        self.q0_prev = self.q_prev[0]
        self.q1_prev = self.q_prev[1]
        self.q2_prev = self.q_prev[2]
        self.q3_prev = self.q_prev[3]
        
        self.gyro_bias_prev = 0.0
        self.acc_bias_prev = 0.0
        
        # Error covariance matrix
        self.P = [
            [0.0,0.0,0.0],
            [0.0,0.0,0.0],
            [0.0,0.0,0.0]
        ]

        self.T = 0 # Sample time
        
        self.Omega = [[0.0,0.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0]]
        
        
    def predict(self):
        q_pred = self.q_prev + matrixOps.scalar_multiply(self.T,self.A)
        
        
           