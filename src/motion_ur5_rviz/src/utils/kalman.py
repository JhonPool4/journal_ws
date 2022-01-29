import numpy as np
import math
from copy import copy
import os
from numpy.linalg import inv
from numpy import matmul as mx

class MultipleKalmanDerivator:
    def __init__(self, deltaT, ndof = 6):
        self.dq = np.zeros(ndof)
        self.ddq = np.zeros(ndof)
        self.dddq = np.zeros(ndof)
        
        self.deltaT = deltaT
        self.ndof = ndof

        self.derivators = []
        for _ in range(ndof):
            self.derivators.append(KalmanDerivator(self.deltaT))

    def initialize(self, dq, ddq, dddq):
        for i in range(self.ndof):
            self.derivators[i].initialize(dq[i], ddq[i], dddq[i])
        

    def update(self, dqr, ddqr):
        dq = []
        ddq = []
        dddq = []

        for i in range(self.ndof):
            dx, ddx, dddx = self.derivators[i].kalman_filter(dqr[i], ddqr[i])#)
            dq.append(dx)
            ddq.append(ddx)
            dddq.append(dddx)

        self.dq = np.array(dq)
        self.ddq = np.array(ddq)
        self.dddq = np.array(dddq)

        return self.dq, self.ddq, self.dddq

class KalmanDerivator:
    def __init__(self, deltaT):
        self.deltaT = deltaT
        
        self.x_k_k = np.zeros((3,1))
        self.x_k1_k = np.zeros((3,1))
        
        self.P_k_k =  np.eye(3)
        self.P_k1_k = np.eye(3)
        
        self.K = np.zeros((3,2)) 
        
        self.Q = 1*np.array([[deltaT**5/30, deltaT**4/24, deltaT**3/6],
                            [deltaT**4/24, deltaT**3/3, deltaT**2/2],
                            [deltaT**3/6, deltaT**2/2, deltaT]]) #

        
        self.R = 0.0001*np.eye(2)
        
        self.I = np.eye(3)

    def initialize(self, vel, acc, jerk):
        self.x_k_k[0][0], self.x_k_k[1][0], self.x_k_k[2][0] = vel, acc, jerk
        
    def kalman_filter(self, vel, acc):
        F = np.array([[1., self.deltaT, self.deltaT**2/2],[0., 1., self.deltaT],[0.,0.,1.]])
        H = np.array([[1., 0., 0.],[0., 1., 0.]])

        z = np.array([[vel],[acc]])

        #Prediction
        self.x_k1_k = mx(F,self.x_k_k)
        self.P_k1_k = mx(F, mx(self.P_k_k, np.transpose(F))) +  self.Q

        #Update
        self.x_k_k = self.x_k1_k + mx(self.K, (z - mx(H, self.x_k1_k)))
        self.P_k_k = mx(mx((self.I - mx(self.K, H)), self.P_k1_k), np.transpose(self.I - mx(self.K, H))) + mx(self.K, mx(self.R, np.transpose(self.K))) 
        #self.P_k_k = mx(self.I - mx(self.K,H), self.P_k1_k)

        self.K = mx(mx(self.P_k1_k, np.transpose(H)), inv(mx(H, mx(self.P_k1_k, np.transpose(H))) + self.R))   
        
        return self.x_k_k[0][0], self.x_k_k[1][0], self.x_k_k[2][0]