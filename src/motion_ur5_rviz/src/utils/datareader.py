import numpy as np
import pandas as pd
import math
from copy import copy
import os

from labpythonlib.lab_functions import *
from labpythonlib.lab_markers import *	


class DataReader:
    def __init__(self, path, dt = 0.01):
        self.datapath = path
        self.xs = np.array([])
        self.dxs = np.array([])
        self.ddxs = np.array([])
        self.dddxs = np.array([])
        self.dt = dt
        self.max_count = 0
        self.df = None
        self.i = 0

    def read_dataset(self, right_arm = False):
        self.df = pd.read_csv(self.datapath, delimiter = r"\s+", header = None)
        df = self.df
        pose = []
        vel = []
        nrows = df.shape[0]
        self.max_count = nrows- 2


        if right_arm == False:
            # Master
            ix, iy, iz  = 1, 2, 3
            iRs, iRe = 4, 12
            idx, idy, idz = 13, 14, 15
            iwx, iwy, iwz = 16, 17, 18

            # Slave
            # ix, iy, iz  = 39,40,41
            # iRs, iRe = 42,50
            # idx, idy, idz = 51,52,53
            # iwx, iwy, iwz = 54,55,56
        else: 
            # Master
            ix, iy, iz  = 20, 21, 22
            iRs, iRe = 23, 31
            idx, idy, idz = 32,33,34
            iwx, iwy, iwz = 35,36,37
        
            # Slave
            # ix, iy, iz  = 58, 59, 60
            # iRs, iRe = 61, 69
            # idx, idy, idz = 70,71,72
            # iwx, iwy, iwz = 73,74, 75

        for i in np.arange(nrows):
            x = df.iloc[i, ix - 1] #- 0.6#- 0.4#+ (right_arm)*(-0.1) + (1 - right_arm)*(0.1) #+0.0#
            y = df.iloc[i, iy - 1] + 0.5 #+ (right_arm)*(-0.45) + (1 - right_arm)*(0.45) #+0.2#
            z = df.iloc[i, iz - 1] - 0.2 #+ 0.8 #+0.5#

            R = df.iloc[i, iRs-1:iRe].to_numpy().reshape(3,3)
            rpy = rot2rpy(R)

            roll = rpy[0]
            pitch = rpy[1]
            yaw = rpy[2]

            dx = df.iloc[i, idx - 1]
            dy = df.iloc[i, idy - 1]
            dz = df.iloc[i, idz - 1]

            wx = df.iloc[i, iwx - 1]
            wy = df.iloc[i, iwy - 1]
            wz = df.iloc[i, iwz - 1]

            pose.append( [x, y, z, roll, pitch, yaw] )
            vel.append( [dx, dy, dz, wx, wy, wz])

        self.xs = np.array(pose)
        self.dxs = np.array(vel)

    def calculate(self):
        self.ddxs = np.diff(self.dxs, axis = 0) / self.dt
        self.dddxs = np.diff(self.ddxs, axis = 0) / self.dt

        self.xs = self.xs[:-4,:]
        self.dxs = self.dxs[:-4,:]
        self.ddxs = self.ddxs[:-3,:]
        self.dddxs = self.dddxs[:-2,:]

    def dataset_trajectory_generator(self):
        x = self.xs[self.i,:]
        dx = self.dxs[self.i,:]
        ddx = self.ddxs[self.i,:]
        dddx = self.dddxs[self.i,:]

        self.i += 1
        return x, dx, ddx, dddx

    def check(self):
        if self.i >= self.max_count-4:
            return True 
        return False


    def reset(self):
        self.i = 0

        
