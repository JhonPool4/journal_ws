# =================================================
#	Course  :   legged robots
# 	Alumno  :   jhon charaja
# 	Info	:	useful functions for laboratories
# =================================================

# ======================
#   required libraries
# ======================
import os
import numpy as np
import pinocchio as pin
from copy import copy

# =============
#   functions
# =============
def sinusoidal_reference_generator(q0, a, f, t_change, t):
    """
    @info: generates a sine signal.

    @inputs: 
    ------
        - q0: initial joint/cartesian position
        - a: amplitude
        - f: frecuency [hz]
        - t_change: change from sinusoidal to constant reference [sec]
        - t: simulation time [sec]
    @outputs:
    -------
        - q, dq, ddq: joint/carteisan position, velocity and acceleration
    """
    w = 2*np.pi*f               # [rad/s]
    if t<=t_change:
        q = q0 + a*np.sin(w*t)      # [rad]
        dq = a*w*np.cos(w*t)        # [rad/s]
        ddq = -a*w*w*np.sin(w*t)    # [rad/s^2]
    else:
        q = q0 + a*np.sin(w*t_change)   # [rad]
        dq = 0                          # [rad/s]
        ddq = 0                         # [rad/s^2]
    return q, dq, ddq

def step_reference_generator(q0, a, t_step, t):
    """
    @info: generate a constant reference.

    @inputs:
    ------
        - q0: initial joint/cartesian position
        - a: constant reference
        - t_step: start step [sec]
        - t: simulation time [sec]
    @outputs:
    -------
        - q, dq, ddq: joint/carteisan position, velocity and acceleration
    """
    if t>=t_step:
        q = q0 + a  # [rad]
        dq = 0      # [rad/s]
        ddq = 0     # [rad/s^2]
    else:
        q = copy(q0)    # [rad]
        dq = 0          # [rad/s]
        ddq = 0         # [rad/s^2]            
    return q, dq, ddq


def tl(array):
    """
    @info: add element to list
    """
    return array.tolist()    

def rot2axisangle(R):
    """
    @info: computes axis/angle values from rotation matrix

    @inputs:
    --------
        - R: rotation matrix
    @outputs:
    --------
        - angle: angle of rotation
        - axis: axis of rotation
    """
    R32 = R[2,1]
    R23 = R[1,2]
    R13 = R[0,2]
    R31 = R[2,0]
    R21 = R[1,0]
    R12 = R[0,1]
    tr  = np.diag(R).sum()
    # angle
    angle = np.arctan2(0.5*np.sqrt( np.power(R21-R12,2)+np.power(R31-R13,2)+np.power(R32-R23,2)), 0.5*(tr-1))
    # axis
    if angle!=0:
        rx = (R32-R23)/(2*np.sin(angle))
        ry = (R13-R31)/(2*np.sin(angle))
        rz = (R21-R12)/(2*np.sin(angle))
        axis = np.array([rx, ry, rz]) 
    else:
        axis = np.zeros(3)
    return angle, axis

def rot2quat(R):
    """
    @info: computes quaternion from rotation matrix
    
    @input:
    ------
        - R: Rotation matrix
    @output:
    -------
        - Q: Quaternion [w, ex, ey, ez]
    """
    dEpsilon = 1e-6
    Q = np.zeros(4)
    
    Q[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        Q[1] = 0.0
    else:
        Q[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        Q[2] = 0.0
    else:
        Q[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        Q[3] = 0.0
    else:
        Q[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return Q      

def quatError(Qdes, Qmed):
    """
    @info: computes quaterion error (Q_e = Q_d . Q_m*).

    @inputs:
    ------
        - Qdes: desired quaternion
        - Q : measured quaternion

    @output:
    -------
        - Qe : quaternion error    
    """

    we = Qdes[0]*Qmed[0] + np.dot(Qdes[1:4].T,Qmed[1:4]) - 1
    e  = -Qdes[0]*Qmed[1:4] + Qmed[0]*Qdes[1:4] - np.cross(Qdes[1:4], Qmed[1:4])
    Qe = np.array([ we, e[0], e[1], e[2] ])

    return Qe               

def rpy2rot(rpy):
    """
    @info: computes rotation matrix from roll, pitch, yaw (ZYX euler angles) representation
    
    @inputs:
    -------
        - rpy[0]: rotation in z-axis (roll)
        - rpy[1]: rotation in y-axis (pitch)
        - rpy[2]: rotation in x-axis (yaw)
    @outputs:
    --------
        - R: rotation matrix        
    """
    Rz = np.array([[ np.cos(rpy[0])  ,  -np.sin(rpy[0]) ,      0],
                [    np.sin(rpy[0])  ,   np.cos(rpy[0]) ,      0],
                [           0      ,        0       ,      1]])

    Ry = np.array([[np.cos(rpy[1])   ,   0   ,   np.sin(rpy[1])],
                [      0            ,   1   ,           0],
                [  -np.sin(rpy[1])   ,   0   ,   np.cos(rpy[1])]])

    Rx =  np.array([ [   1   ,    0           ,        0], 
                        [0   ,    np.cos(rpy[2]) ,  -np.sin(rpy[2])],
                        [0   ,    np.sin(rpy[2]) ,   np.cos(rpy[2])]])

    R =  np.dot(np.dot(Rz, Ry), Rx)
    return R

def rot2rpy(R):
    """
    @info: computes roll, pitch, yaw (ZYX euler angles) from rotation matrix
    
    @inputs:
    -------
        - R: rotation matrix        
    @outputs:
    --------
        - rpy[0]: rotation in z-axis (roll)
        - rpy[1]: rotation in y-axis (pitch)
        - rpy[2]: rotation in x-axis (yaw)
    """
    R32 = R[2,1]
    R31 = R[2,0]
    R33 = R[2,2]
    R21 = R[1,0]
    R11 = R[0,0]
    rpy = np.zeros(3)    
    rpy[1] = np.arctan2(-R31, np.sqrt(R32*R32 + R33*R33))
    rpy[0] = np.arctan2(R21/np.cos(rpy[1]), R11/np.cos(rpy[1]))
    rpy[2] = np.arctan2(R32/np.cos(rpy[1]), R33/np.cos(rpy[1]))

    return rpy

def rot2rpy_unwrapping(R, rpy_old):
    """
    @info: computes roll, pitch, yaw (ZYX euler angles) from rotation matrix
    
    @inputs:
    -------
        - R: rotation matrix        
    @outputs:
    --------
        - rpy[0]: rotation in z-axis (roll)
        - rpy[1]: rotation in y-axis (pitch)
        - rpy[2]: rotation in x-axis (yaw)
    """
    R32 = R[2,1]
    R31 = R[2,0]
    R33 = R[2,2]
    R21 = R[1,0]
    R11 = R[0,0]
    rpy = np.zeros(3)    
    rpy[1] = np.arctan2(-R31, np.sqrt(R32*R32 + R33*R33))
    rpy[0] = np.arctan2(R21/np.cos(rpy[1]), R11/np.cos(rpy[1]))
    rpy[2] = np.arctan2(R32/np.cos(rpy[1]), R33/np.cos(rpy[1]))

    for i in range(3):
        if(rpy[i]<(rpy_old[i]-np.pi)):
            rpy[i] +=2*np.pi
        elif(rpy[i]>(rpy_old[i]+np.pi)):
            rpy[i] -=2*np.pi 
    return rpy 


def angular_velocity_rpy(rpy, drpy):
    """
    @info: compute angular velocity (w) from euler angles (roll, pitch and yaw) and its derivaties
    @inputs:
    -------
        - rpy[0]: rotation in z-axis (roll)
        - rpy[1]: rotation in y-axis (pitch)
        - rpy[2]: rotation in x-axis (yaw)
        - drpy[0]: rotation ratio in z-axis
        - drpy[1]: rotation ratio in y-axis
        - drpy[2]: rotation ratio in x-axis
    @outputs:
    --------
        - w: angular velocity
    """        
    E0 = np.array(  [[0, -np.sin(rpy[0]), np.cos(rpy[0])*np.cos(rpy[1])], \
                    [0,   np.cos(rpy[0]), np.sin(rpy[0])*np.cos(rpy[1])], \
                    [1,         0,          -np.sin(rpy[1])       ]])
    
    w = np.dot(E0, drpy)
    return w

def angular_acceleration_rpy(rpy, drpy, ddrpy):
    """
    @info: compute angular velocity (w) from euler angles (roll, pitch and yaw) and its derivaties
    @inputs:
    -------
        - rpy[0]: rotation in z-axis (roll)
        - rpy[1]: rotation in y-axis (pitch)
        - rpy[2]: rotation in x-axis (yaw)
        - drpy[0]: rotation speed in z-axis
        - drpy[1]: rotation speed in y-axis
        - drpy[2]: rotation speed in z-axis
        - ddrpy[0]: rotation acceleration in z-axis
        - ddrpy[1]: rotation acceleration in y-axis
        - ddrpy[2]: rotation acceleration in x-axis        
    @outputs:
    --------
        - dw: angular acceleration
    """        
    E0 = np.array(  [[0, -np.sin(rpy[0]), np.cos(rpy[0])*np.cos(rpy[1])], \
                    [0,   np.cos(rpy[0]), np.sin(rpy[0])*np.cos(rpy[1])], \
                    [1,         0,          -np.sin(rpy[1])       ]])
    
    E1 = np.array( [[0, -np.cos(rpy[0])*drpy[0], -np.sin(rpy[0])*drpy[0]*np.cos(rpy[1])-np.cos(rpy[0])*np.sin(rpy[1])*drpy[1]], \
                    [0, -np.sin(rpy[0])*drpy[0],  np.cos(rpy[0])*drpy[0]*np.cos(rpy[1])-np.sin(rpy[0])*np.sin(rpy[1])*drpy[1]], \
                    [0,         0,               -np.cos(rpy[1])*drpy[1]   ]])
    dw = np.dot(E1, drpy) + np.dot(E0, ddrpy)
    return dw

def damped_pinv(M, lambda_=0.0000001):
    """
    @info: computes damped pseudo-inverse

    @inputs:
    ------
        - M: matrix
        - lambda_ : damping term (optional)
    @outputs:
    -------
        - M_damped_inv: damped psedu-inverse of M            
    """
    ntask = M.shape[0]
    M_damped_inv =  np.dot(M.T, np.linalg.inv(np.dot(M, M.T) + lambda_*np.eye(ntask)))
    return M_damped_inv

class Robot(object):
    """
    @info: Class to load the .urdf of a robot. For thism Pinocchio library is used

    @methods:
        - foward_kinematics(q0)
        - jacobian(q0)
        - jacobian_time_derivative(q0, dq0)
        - jacobian_damped_pinv(J, lambda)
        - twist(q0, dq0)
        - send_control_command(u)
        - inverse_kinematics_position(x_des, q0)
    """    
    def __init__(self, q0, dq0, dt, urdf_path):
        # robot object
        self.robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_path)
        # degrees of freedom
        self.ndof = self.robot.model.nq
        # joint configuration: position, velocity and acceleration
        self.q = copy(q0)                
        self.dq = copy(dq0)               
        self.ddq = np.zeros(self.ndof)
        # inertia matrix
        self.M = np.zeros([self.ndof, self.ndof])
        # nonlinear effects vector
        self.b = np.zeros(self.ndof)
        # gravivty effects vector
        self.g = np.zeros(self.ndof)
        # vector of zeros
        self.z = np.zeros(self.ndof)
        # sampling time
        self.dt = copy(dt)     
        # frame id: end-effector
        self.frame_ee = self.robot.model.getFrameId('ee_link') 
        # end-effector: position, velocity and acceleration
        self.p = np.zeros(3)
        self.dp = np.zeros(3)
        self.ddp = np.zeros(3)
        # end-effector: orientation
        self.R = np.zeros([3,3])
        # end-effector: angular velocity and acceleration
        self.w = np.zeros(3)
        self.dw = np.zeros(3)
        # initial configuration: position (p) and orientation (R)
        self.p, self.R = self.forward_kinematics(self.q)
        # initial configuration: linear (dp) and angular (w) velocity
        self.dp, self.w = self.twist(self.q, self.dq)
        # initial configuration: linear (ddp) and angular (dw) acceleration
        self.ddp, self.dw = self.dtwist(self.q, self.dq, self.ddq)
        # initial configuration: dynamic model
        self.M = pin.crba(self.robot.model, self.robot.data, self.q)
        self.b = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, self.z)
        self.g = pin.rnea(self.robot.model, self.robot.data, self.q, self.z, self.z)        
  
    def forward_kinematics(self, q0):
        """
        @info: computes the position (xyz) and rotation (R) of the end-effector.

        @inputs:
        -----
            - q0: joint configuration (rad)
        
        @outputs:
        -------
            - p: position of the end-effector (m).
            - R: rotation matrix of the end-effector (rad).
        """      
        # commpute forward kinematics
        pin.forwardKinematics(self.robot.model, self.robot.data, q0) 
        # get position and orientation       
        p = pin.updateFramePlacement(self.robot.model, self.robot.data, self.frame_ee).translation
        R = pin.updateFramePlacement(self.robot.model, self.robot.data, self.frame_ee).rotation
        return p, R

    def jacobian(self, q0):
        """
        @info: computes jacobian matrix of the end-effector.

        @inputs:
        ------
            - q0: joint configuration (rad)
        @outputs:
        -------
            - J: jacobian matrix            
        """
        # compute jacobian matrix (end-effector frame)
        pin.computeJointJacobians(self.robot.model, self.robot.data, q0)
        J = pin.getFrameJacobian(self.robot.model, self.robot.data, self.frame_ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J
    
    def jacobian_time_derivative(self, q0, dq0):
        """
        @info: computes time derivative of jacobian matrix of the end-effector.

        @inputs:
        ------
            - q0: joint position/configuration (rad)
            - dq0: joint velocity (rad/s)
        @outputs:
        -------
            - dJ: time derivative of jacobian matrix            
        """        
        # compute time-derivative of jacobian matrix (end-effector frame)
        pin.computeJointJacobiansTimeVariation(self.robot.model, self.robot.data, q0, dq0)
        dJ = pin.getFrameJacobianTimeVariation(self.robot.model, self.robot.data, self.frame_ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return dJ

    def jacobian_damped_pinv(self, J, lambda_=0.0000001):
        """
        @info: computes inverse jabobian using damped pseudo-inverse method

        @inputs:
        ------
            - J: position jacobian [3 x ndof]
            - lambda_ : damping term (optional)
        @outputs:
        -------
            - J_damped_inv: inverse of jacobian matrix            
        """
        ntask = J.shape[0] # position (3) + orientation (3 or 4)
        J_damped_inv =  np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + lambda_*np.eye(ntask)))
        return J_damped_inv
    
    def twist(self, q0, dq0):
        """
        @info: computes linear and angular velocity of robot end-effector
        @inputs:
        -------
            - q0: joint configuration/position (rad)
            - dq0: joint velocity (rad/s)
        @outputs:
        --------
            - v: linear velocity (m/s)
            - w: angular velocity (rad/s)             
        """
        J = self.jacobian(q0)
        v = J[0:3,0:6].dot(dq0)
        w = J[3:6,0:6].dot(dq0)
        return v, w
    
    def dtwist(self, q0, dq0, ddq0):
        """
        @info: computes linear and angular acceleration of robot end-effector
        @inputs:
        -------
            - q0: joint configuration/position (rad)
            - dq0: joint velocity (rad/s)
            - ddq0: joint acceleration (rad/s^2)
        @outputs:
        --------
            - a: linear acceleration (m/s^2)
            - dw: angular acceleration (rad/s^2)             
        """      
        J = self.jacobian(q0)
        dJ = self.jacobian_time_derivative(q0, dq0)
        a = dJ[0:3,0:6].dot(dq0) + J[0:3,0:6].dot(ddq0)
        dw = dJ[3:6,0:6].dot(dq0) + J[3:6,0:6].dot(ddq0)
        return a, dw

    def send_control_command(self, u):
        """
        @info: uses the control signal (u) to compute forward dynamics (ddq). 
              Then update joint configuration (q) and end-effector pose (p, R)
        """
        tau = np.squeeze(np.asarray(u))
        # compute dynamics model
        self.M = pin.crba(self.robot.model, self.robot.data, self.q)
        self.b = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, self.z)
        self.g = pin.rnea(self.robot.model, self.robot.data, self.q, self.z, self.z)
        # forward dynamics
        self.ddq = np.linalg.inv(self.M).dot(tau-self.b)
        # update joint position/configuration
        self.dq = self.dq + self.dt*self.ddq
        self.q = self.q + self.dt*self.dq + 0.5*self.dt*self.dt*self.ddq
        # update end-effector: linear and angular position, velocity and acceleration
        self.p, self.R = self.forward_kinematics(self.q)
        self.dp, self.w = self.twist(self.q, self.dq)
        self.ddp, self.dw = self.dtwist(self.q, self.dq, self.ddq)
                
    def inverse_kinematics_position(self, x_des, q0):
        """
        @info: computes inverse kinematics with the method of damped pseudo-inverse.

        @inputs:
        -------
            - xdes  :   desired position vector
            - q0    :   initial joint configuration (it's very important)
        @outputs:
        --------        
            - q_best  : joint position
        """         
        best_norm_e     = 1e-6 
        max_iter        = 10
        delta           = 1
        lambda_         = 0.0000001
        q               = copy(q0)

        for i in range(max_iter):
            p, _ = self.forward_kinematics(q) # current position
            e   = x_des - p      # position error
            J   = self.jacobian(q)[0:3, 0:self.ndof] # position jacobian [3x6]
            J_damped_inv =  self.jacobian_damped_pinv(J, lambda_) # inverse jacobian [6x3]
            dq  = np.dot(J_damped_inv, e)
            q   = q + delta*dq
                       
            # evaluate convergence criterion
            if (np.linalg.norm(e)<best_norm_e):
                best_norm_e = np.linalg.norm(e)
                q_best = copy(q) 
        return q_best 

    def inverse_kinematics_pose(self, x_des, q0):
        """
        @info: computes inverse kinematics with the method of damped pseudo-inverse.

        @inputs:
        -------
            - xdes  :   desired pose vector
            - q0    :   initial joint configuration (it's very important)
        @outputs:
        --------        
            - q_best  : joint position
        """         
        best_norm_e     = 1e-6 
        max_iter        = 10
        delta           = 1
        lambda_         = 0.0000001
        q               = copy(q0)

        for i in range(max_iter):
            p, R = self.forward_kinematics(q) # current position
            # error: position (xyz)
            e_p = x_des[0:3] - p                  
            # error: orientation axis/angle
            e_o = quatError(x_des[3:7], rot2quat(R))    # orientation error
            # error: position and orientation
            e = np.concatenate((e_p,e_o), axis=0) # [6x1] 
            # jacobian
            J   = self.jacobian(q)
            # jacobian: pseudo-inverse
            J_damped_inv =  damped_pinv(J, lambda_) # inverse jacobian [6x3]
            dq  = np.dot(J_damped_inv, e)
            q   = q + delta*dq
                       
            # evaluate convergence criterion
            if (np.linalg.norm(e)<best_norm_e):
                best_norm_e = np.linalg.norm(e)
                q_best = copy(q) 
        return q_best 

    def read_joint_position_velocity_acceleration(self):
        return self.q, self.dq, self.ddq

    def read_cartesian_position_velocity_acceleration(self):
        return self.p, self.dp, self.ddp

    def read_ee_position(self):
        return self.p

    def read_ee_orientation(self):
        return self.R

    def read_ee_angular_velocity(self):        
        return self.w

    def read_ee_linear_velocity(self):
        return self.v        

    def get_M(self):
        return self.M

    def get_b(self):
        return self.b
    
    def get_g(self):
        return self.g
    # deprecated
    def get_ee_orientation(self):
        return self.R
    # deprecated        
    def get_ee_position(self):
        return self.p















# currenty not neccesary
def dh(d, theta, a, alpha):
    """
    Info: Computes homogeneous transformation matrix for Denavit-Hartenverg parameters of UR5 robot.

    Inputs:
    ------
        - theta: [rad]
        - alpha: [rad]
        - d: [m]
        - a: [m]
    Output:
    ------
        - T: homogeneous transformation matrix
    """
    T = np.array(
        [[np.cos(theta),    -np.cos(alpha)*np.sin(theta),   +np.sin(alpha)*np.sin(theta),   a*np.cos(theta)],
         [np.sin(theta),    +np.cos(alpha)*np.cos(theta),   -np.sin(alpha)*np.cos(theta),   a*np.sin(theta)],
         [      0      ,            +np.sin(alpha)      ,           +np.cos(alpha)      ,           d      ],
         [      0      ,                    0           ,                   0           ,           1      ]])

    return T

def fkine_ur5(q):
    """
    Info: Computes forward kinematics of UR5 robot. With respect to base frame

    Inputs:
    -----
        - q: joint configuration [6x1 rad]
    Outputs:
    -------
        - T: homogenoeus transformation matrix that relates end-effector with base.
    """
    #           d               th              a               alpha
    T01 = dh(0.08916,          +q[0],            0.0,            np.pi/2)
    T12 = dh(    0.0,          +q[1],         -0.425,                0.0)
    T23 = dh(    0.0,          +q[2],         -0.392,                0.0)
    T34 = dh(0.10915,          +q[3],            0.0,            np.pi/2)    
    T45 = dh(0.09465,     np.pi+q[4],            0.0,            np.pi/2)
    T56 = dh( 0.0823,          +q[5],            0.0,                0.0)
    # relate end effector with base link
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    
    #print("T01: ",T01[0:3,3])
    #print("T02: ",T02[0:3,3])
    #print("T03: ",T03[0:3,3])
    #print("T04: ",T04[0:3,3])
    #print("T05: ",T05[0:3,3])
    #print("T06: ",T06[0:3,3])
    return T06


def jacobian_xyz_ur5(q, delta=0.0001):
    """
    Info: Analytic jacobian for cartesian position
    
    Inputs:
    ------
        - q: joint configuration [6x1 rad]
    
    Outputs:
    -------
        - J: analytic jacobian [3x6]
    """
    J = np.zeros((3,6))
    # Initial homogeneous transformation (using q)
    T = fkine_ur5(q)
    for i in range(6):
        dq      = copy(q)
        dq[i]   = dq[i] + delta
        dT      = fkine_ur5(dq)
        J[:,i]  = (dT[0:3,3] - T[0:3,3])/delta
    return J        

