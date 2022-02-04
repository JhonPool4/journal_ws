import numpy as np
import pandas as pd
import math
from copy import copy
import os

from labpythonlib.lab_functions import *
from labpythonlib.lab_markers import *	


class DataReaderJIGSAW:
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
            ix, iy, iz, iRs, iRe, idx, idy, idz, iwx, iwy, iwz = 0, 1, 2, 3, 12, 13, 14, 15, 16, 17, 18 
        else: 
            ix, iy, iz, iRs, iRe, idx, idy, idz, iwx, iwy, iwz = 38,39,40,41,49, 50,51,52, 53,54, 55
        
        for i in np.arange(nrows):
            x = df.iloc[i, ix] + 0.3
            y = df.iloc[i, iy] + 0.2
            z = df.iloc[i, iz]

            R = df.iloc[i, iRs:iRe].to_numpy().reshape(3,3)
            rpy = rot2rpy(R)

            roll = rpy[0]
            pitch = rpy[1]
            yaw = rpy[2]

            dx = df.iloc[i, idx]
            dy = df.iloc[i, idy]
            dz = df.iloc[i, idz]

            wx = df.iloc[i, iwx]
            wy = df.iloc[i, iwy]
            wz = df.iloc[i, iwz]

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

        
