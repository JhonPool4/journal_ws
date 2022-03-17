#!/usr/bin/env python3

# ============================
# 			Libraries
# ============================ 
import rospy 							# ROS with Python
import os 								# Manipulate path names
import pandas as pd 					# Save data
from labpythonlib.lab_functions import *
from labpythonlib.lab_markers import *	
from utils.datareader import *

import numpy as np
from copy import copy
from numpy import transpose as tr



# =============
#   Save data
# =============
pwd = os.path.dirname(os.path.realpath(__file__))

origin_assets = '../assets'
folder_assets = os.path.join(pwd, origin_assets)

column_tmp = [
    't',
    'x_des', 'y_des', 'z_des', 'roll_des', 'pitch_des', 'yaw_des',
    'dx_des', 'dy_des', 'dz_des', 'droll_des', 'dpitch_des', 'dyaw_des',
    'x_ref', 'y_ref', 'z_ref', 'roll_ref', 'pitch_ref', 'yaw_ref',
    'dx_ref', 'dy_ref', 'dz_ref', 'droll_ref', 'dpitch_ref', 'dyaw_ref',    
    ]
#    'ddx_des', 'ddy_des', 'ddz_des', 'ddroll_des', 'ddpitch_des', 'ddyaw_des',

df_tmp = pd.DataFrame(columns=column_tmp,dtype=object)
path_tmp = os.path.join(pwd, 'tmp1.csv')
df_tmp.to_csv(path_tmp, index=False)

# Loop rate (in Hz)
freq = 30.0
dt = 1/freq                    # 33  [ms]

# ==============
#   Trajectory  
# ==============
trajectory_type = 'dataset_3'

if  trajectory_type.split('_')[0] == 'dataset' and int(trajectory_type.split('_')[1]) in range(1,5):
    datasets = [ 
    'Needle_Passing_E004.txt', #s
    'Knot_Tying_E003.txt', #s
    'Knot_Tying_D001.txt', #s
    'Knot_Tying_D003.txt'] #s

    arms = [
            "l",#s
            "r",#s
            "l",#s
            "r",#s
            ]
            
    dataset_number = int(trajectory_type.split('_')[1]) - 1

    fn = datasets[dataset_number]  
    is_right_arm = True if arms[dataset_number] == 'r' else False    
    dataset_folder = "dataset" 

    dataset_path = os.path.join(folder_assets, dataset_folder, fn)

    dr = DataReader(dataset_path)
    dr.read_dataset(is_right_arm)
    dr.calculate()
    dataset_used = True

elif trajectory_type == 'circular':
    dataset_used = False
else:
    raise ValueError('Trajectory type {} is not valid. Choose only "circular" or "dataset_x [x: 1,2,3,4]".'.format(trajectory_type))


# ================================
# 		Dynamic simulation
# ================================
t = 0.0 				# @param current simulation time
sim_time = 30
kalman_created = False

# reference
p_ref = np.zeros(3)
dp_ref = np.zeros(3)
rpy_ref = np.zeros(3)
drpy_ref = np.zeros(3)

while True:
    x_des, dx_des, _, _ = dr.dataset_trajectory_generator()
    
    rpy_des = x_des[3:6]

    if not kalman_created:
        kalman_created=True
        # create kalman derivators
        kf_pos = MultipleKalmanDerivator(dt, x_des[0:3], dx_des[0:3], np.zeros(3), n_obs=2, sigmaR=1e-3,sigmaQ=1)
        kf_ori = MultipleKalmanDerivator(dt, x_des[3:6], dx_des[3:6], np.zeros(3), n_obs=2, sigmaR=1e-3,sigmaQ=1)


    # smothing
    p_ref, dp_ref, ddp_ref  = kf_pos.update(x_des[0:3], dx_des[0:3])
    rpy_ref, drpy_ref, ddrpy_ref = kf_ori.update(x_des[3:6], dx_des[3:6])


    row_tmp = tl(np.array([t]))+tl(x_des)+tl(dx_des)+tl(p_ref)+tl(rpy_ref)+tl(dp_ref)+tl(drpy_ref)
    row_tmp = tl(np.expand_dims(np.array(row_tmp), axis = 1))
    df_row_tmp = pd.DataFrame.from_dict(dict(zip(column_tmp, row_tmp)))
    df_tmp.append(df_row_tmp, sort = False).to_csv(path_tmp, index=False, mode = 'a', header=False)

    # Update
    t = t + dt


    if t>=(sim_time): # 60 sec
        print("Reached maximum of steps")
        break

print('out because ctrl + c. Data saved.')            

