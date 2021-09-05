# ===============================
#   Author  :   Jhon Charaja
#   Info    :   Basic functions
# ===============================

import numpy as np
import math
from copy import copy
import rbdl
import os

pi=np.pi

def dh(d, theta, a, alpha):
    """
    @ info  Computes homogeneous transformation matrix for Denavit-Hartenverg parameters of UR5 robot.
            The values d, theta, a, alpha are scalars.
    @param
            - theta [rad]
            - alpha [rad]
            - d     [m]
            - a     [m]
    """
    T = np.array(
        [[np.cos(theta),    -np.cos(alpha)*np.sin(theta),   +np.sin(alpha)*np.sin(theta),   a*np.cos(theta)],
         [np.sin(theta),    +np.cos(alpha)*np.cos(theta),   -np.sin(alpha)*np.cos(theta),   a*np.sin(theta)],
         [      0      ,            +np.sin(alpha)      ,           +np.cos(alpha)      ,           d      ],
         [      0      ,                    0           ,                   0           ,           1      ]])

    return T


def fkine_ur5(q):
    """
    @info   Computes forward kinematics of UR5 robot with current joint position.
            Lenghts in meters (m)

    @param q joint position [q1, q2, q3, q4, q5, q6]            
    """
    
    
    T1 = dh(0.08916,          +q[0],    0.0,     +pi/2)
    T2 = dh(    0.0,          +q[1], -0.425,       0.0)
    T3 = dh(    0.0,          +q[2], -0.392,       0.0)
    T4 = dh(0.10915,          +q[3],    0.0,     +pi/2)
    T5 = dh(0.09465,       +pi+q[4],    0.0,     +pi/2)
    T6 = dh( 0.0823,       +pi+q[5],    0.0,       0.0)
    
    # Efector final con respecto a la base
    T = np.dot(np.dot(np.dot(np.dot(np.dot(T1,T2),T3),T4),T5),T6)
    return T


def rot2quat(R):
    """
    @info Converts a rotation matrix to quaterion
    
    Input:
    ------
        - R: Rotation matrix
    Output:
    -------
        - Q: Quaternion [w, ex, ey, ez]
    """
    dEpsilon = 1e-6;
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)    


def quatError(Qdes, Q):
    """
    Compute difference between quaterions.

    Inputs:
    ------
        - Qdes:     Desired quaternion
        - Q   :     Current quaternion

    Output:
    -------
        - Qe  :     Error quaternion    
    """

    we = Qdes[0]*Q[0] + np.dot(Qdes[1:4].transpose(),Q[1:4]) - 1
    e  = -Qdes[0]*Q[1:4] + Q[0]*Qdes[1:4] - np.cross(np.transpose(Qdes[1:4]), np.transpose(Q[1:4]))
    Qe = np.array([ we, e[0], e[1], e[2] ])

    return Qe        


def jacobian_pose_ur5(q, delta=0.001):
    """
    @info Analytic jacobian for pose (orientation represented with quaternions)
    
    @param q: joint position [6x1]
    @param J: analytic jacobian [7x6]
    """
    J = np.zeros((7,6))
    # Initial homogeneous transformation (using q)
    T = fkine_ur5(q)
    Q = rot2quat(T[0:3,0:3])

    for i in range(6):
        dq      = copy(q)
        dq[i]   = dq[i] + delta
        dT      = fkine_ur5(dq)
        dQ      = rot2quat(dT[0:3,0:3])
        Jpos    = (dT[0:3,3] - T[0:3,3])/delta
        Jrot    = quatError(dQ, Q)/delta
        J[:,i] = np.concatenate((Jpos, Jrot), axis=0)
   
    return J    

def ikine_pose_ur5(xdes, dxdes, q0, dq0, ddq0):
    """
    @info   Computes inverse kinematics with the method of inverse jacobian.
            K values were sintonized.
    
    Inputs:
    -------
        - xdes  :   Desired position and orientation vector
        - dxdes :   Desired linear and angular velocity vector
        - ddxdes:   Desired linear and angular acceleration vector
        - q0    :   Initial joint configuration (It's very important)

    Outputs:
    --------        
        - qdes  :   Desired joint position
        - qdes  :   Desired joint velocity
        - ddqdes:   Desired joint acceleration
    """    
    k_p             = 150
    k_o             = 20
    k               = np.diag([k_p, k_p, k_p, k_o, k_o, k_o, k_o])
    best_norm_e1    = 1e-3 
    best_norm_e2    = 1e-3
    max_iter        = 10
    delta           = 0.01
    q               = copy(q0)

    for i in range(max_iter):
        T       = fkine_ur5(q)
        e1      = xdes[0:3] - T[0:3,3]                          # position error
        e2      = quatError(xdes[3:7], rot2quat(T[0:3,0:3]))    # orientation error
        e       = np.concatenate((e1,e2), axis=0)               # [pos, quaternion]
        de      = -np.dot(k,e)
        J       = jacobian_pose_ur5(q,delta)
        Jinv    = np.linalg.pinv(J)
        dq      = np.dot(Jinv, dxdes - de)
        q       = q + delta*dq

        #print("iter: ", i)
        #print("norma position: ",np.linalg.norm(e1))
        #print("norma orientation: ",np.linalg.norm(e2))

        if (np.linalg.norm(e2) < best_norm_e2) & (np.linalg.norm(e1)< best_norm_e1):

            best_norm_e2    =   np.linalg.norm(e2)
            best_norm_e1    =   np.linalg.norm(e1)
            q_best          =   q
            dq_best         =   dq
            #ddq_best        =   np.dot(Jinv, ( ddxdes - np.dot(dJ,dq_best) ))
            #print("iter: ", i)
            #print("norma position: ",best_norm_e1)
            #print("norma orientation: ",best_norm_e2)
            #print("q: ", q)#np.round(q,2))
            #print("dq: ", dq)#np.round(dq,2))
            #print("\n")

    ddq     = (dq_best - dq0)/delta  # angular acceleration 
    dddq    = (ddq- ddq0)/delta # angular jerk

    return q_best, dq_best, ddq, dddq



def ikine_pose_ur5_configuration(x_0, dx_0, q0, dq0, ddq0):
    """
    @info   computes values of q, dq, ddq that correspond to the first desired cartesian point
            Then, the ikine require less iteration With this previus configuraiton
    
    Inputs:
    -------
        - xdes  :   Desired position and orientation vector
        - dxdes :   Desired linear and angular velocity vector
        - ddxdes:   Desired linear and angular acceleration vector
        - q0    :   Initial joint configuration (It's very important)

    Outputs:
    --------        
        - qdes  :   Desired joint position
        - qdes  :   Desired joint velocity
        - ddqdes:   Desired joint acceleration
    """    
    k_p             = 400
    k_o             = 40
    k               = np.diag([k_p, k_p, k_p, k_o, k_o, k_o, k_o])
    best_norm_e1    = 1e-4 
    best_norm_e2    = 1e-4
    max_iter        = 1000
    delta           = 0.0001
    q               = copy(q0)

    #x_0     = np.zeros(7); x_0[3:7] = np.array([0.01676998,  0.99985616,  0.00251062,  0.00]) # fixed orientation
    #dx_0    = np.zeros(7) 
    #x_0[0:3], dx_0[0:3], _, _ = circular_trayectory_generator(0.0) # first point

    for i in range(max_iter):
        T       = fkine_ur5(q)
        e1      = x_0[0:3] - T[0:3,3]                          # position error
        e2      = quatError(x_0[3:7], rot2quat(T[0:3,0:3]))    # orientation error
        e       = np.concatenate((e1,e2), axis=0)               # [pos, quaternion]
        de      = -np.dot(k,e)
        J       = jacobian_pose_ur5(q,delta)
        Jinv    = np.linalg.pinv(J)
        dq      = np.dot(Jinv, dx_0 - de)
        q       = q + delta*dq

        #print("iter: ", i)
        #print("norma position: ",np.linalg.norm(e1))
        #print("norma orientation: ",np.linalg.norm(e2))
        if (np.linalg.norm(e2) < best_norm_e2) & (np.linalg.norm(e1)< best_norm_e1):

            best_norm_e2    =   np.linalg.norm(e2)
            best_norm_e1    =   np.linalg.norm(e1)
            q_best          =   q
            dq_best         =   dq
            #ddq_best        =   np.dot(Jinv, ( ddxdes - np.dot(dJ,dq_best) ))
            #print("iter: ", i)
            #print("norma position: ",best_norm_e1)
            #print("norma orientation: ",best_norm_e2)
            #print("q: ", q)#np.round(q,2))
            #print("dq: ", dq)#np.round(dq,2))
            #print("\n")
    print("\n\n")
    print("norma position: ",best_norm_e1)
    print("norma orientation: ",best_norm_e2)
    print("\n\n")
    ddq     = (dq_best - dq0)/delta  # angular acceleration 
    dddq    = (ddq- ddq0)/delta # angular jerk

    return q_best, dq_best, ddq, dddq

class Robot(object):
    """
    @info A robot with dynamics of UR5 robot
    """    
    def __init__(self, q0, dq0, ndof, dt):
        cwd = os.path.dirname(os.path.realpath(__file__))
        self.q  = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.ddq = np.zeros(ndof)
        self.M  = np.zeros([ndof, ndof])
        self.b  = np.zeros(ndof)
        self.z  = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel(os.path.join(cwd,'../../ur5_description/urdf/ur5_joint_limited_robot.urdf'))

    def send_command(self, tau):
        tau = np.squeeze(np.asarray(tau))
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)

        self.ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.dq = self.dq + self.dt*self.ddq
        self.q = self.q + self.dt*self.dq      

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

    def read_joint_accelerations(self):
        return self.ddq

    def get_M(self):
        return self.M

    def get_b(self):
        return self.b


def saturador_effort_control_UR5(u):
    # @ info Limit the value of control signal
    size0 = 12*0.9
    size1 = 28*0.9
    size2 = 56*0.9
    size3 = 150*0.9
    size4 = 330*0.9

    if abs(u[0]) >= size3:
        u[0] = np.sign(u[0])*size3

    if abs(u[1]) >= size3:
        u[1] = np.sign(u[1])*size3
    
    if abs(u[2]) >= size3:
        u[2] = np.sign(u[2])*size3

    if abs(u[3]) >= size1:
        u[3] = np.sign(u[3])*size1

    if abs(u[4]) >= size1:
        u[4] = np.sign(u[4])*size1

    if abs(u[5]) >= size1:
        u[5] = np.sign(u[5])*size1

    return u


def eight_trajectory_generator(t):
    """
    Generate points of a eight shape
    
    Inputs:
    -------
        - t     : time (s)
    Outpus:
    -------
        - x_tray    : x axis                (m)
        - y_tray    : y axis                (m)
        - dx_tray   : velocity on x axis    (m/s)
        - dy_tray   : velocity on y axis    (m/s)
    """
    # Pacient position
    x_paciente = +0.500     # m
    y_paciente = -0.275     # m

    # length of patient arm
    l       =   0.550       # m    

    # Safe parameters defined by therapist 
    r_max   = 0.70  # m     # No se modifica
    r_min   = 0.30  # m     # No se mofica
    y_max   = y_paciente + 0.80*l                   #   [m]
    y_min   = y_paciente + 0.20*l                   #   [m]
    r_circ  = 0.1                                  #   [m] 

    # Parameters of eight trayetory     
    f = 0.2                       # frecuency     [Hz]
    w = 2*np.pi*f                 # angular velocity [rad/s]

    x0_tray = (r_min + 0.5*(r_max - r_min))
    y0_tray = (y_min + 0.5*(y_max - y_min))    

    x_tray  = x0_tray + r_circ*np.cos(w*(t-1.242))
    y_tray  = y0_tray + r_circ*np.sin(w*(t-1.242))/2

    dx_tray = r_circ*( (-w)*np.sin(w*t) )
    dy_tray = r_circ*( (+w)*np.cos(w*t) )/2

    return [x_tray, y_tray, 0.0], [dx_tray, dy_tray, 0.0]


def circular_trayectory_generator(t):
    """
    Generate points of a circular trayectory.

    Inputs:
    -------
        -   t   : time [s]

    Outpus:
    -------
        -   x_circ_tray     : "x" position point of circular trayectory at time "t"
        -   y_circ_tray     : "y" position point of circular trayectory at time "t"
        -   dx_circ_tray    : "x" velocity point of circular trayectory at time "t"
        -   dy_circ_tray    : "y" velocity point of circular trayectory at time "t"
        -   ddx_circ_tray   : "x" acceleration point of circular trayectory at time "t"
        -   ddy_circ_tray   : "y" acceleration point of circular trayectory at time "t"        
    """
    # Pacient position
    x_paciente = +0.500     # m
    y_paciente = -0.275     # m

    # length of patient arm
    l       =   0.550       # m    

    # Safe parameters defined by therapist 
    r_max   = 0.70  # m     # No se modifica
    r_min   = 0.30  # m     # No se mofica
    y_max   = y_paciente + 0.80*l                   #   [m]
    y_min   = y_paciente + 0.20*l                   #   [m]
    r_circ  = 0.05#1                                   #   [m] 
    r_z     = 0.02

    # Parameters of circular trayetory     
    f           = 0.1                       # frecuency     [Hz]
    w           = 2*np.pi*f                 # angular velocity [rad/s]

    x0_tray = (r_min + 0.5*(r_max - r_min))
    y0_tray = (y_min + 0.5*(y_max - y_min))
    
    #phi = atan2(-1, 0)/ (2*pi) = -2.5
    phi = 0#-2.5
    
    # position points
    x_circ_tray  = x0_tray + r_circ*np.cos(w*(t+phi))
    y_circ_tray  = y0_tray + r_circ*np.sin(w*(t+phi))
    z_circ_tray  = r_z*np.sin(w*t)

    # velocity points
    dx_circ_tray = r_circ*( (-w)*np.sin(w*(t+phi)) )
    dy_circ_tray = r_circ*( (+w)*np.cos(w*(t+phi)) )
    dz_circ_tray = r_z*w*np.cos(w*t)

    # acceleration points
    ddx_circ_tray = r_circ*( (-w*w)*np.cos(w*(t+phi)) )
    ddy_circ_tray = r_circ*( (-w*w)*np.sin(w*(t+phi)) )    
    ddz_circ_tray = r_z*(-w*w)*np.sin(w*t)

    # jerk points
    dddx_circ_tray = r_circ*( (+w*w*w)*np.sin(w*(t+phi)) )
    dddy_circ_tray = r_circ*( (-w*w*w)*np.cos(w*(t+phi)) )
    dddz_circ_tray = r_z*(-w*w*w)*np.cos(w*t)

    # vectors
    pos   = [x_circ_tray, y_circ_tray, z_circ_tray]
    vel   = [dx_circ_tray, dy_circ_tray, dz_circ_tray]
    accel = [ddx_circ_tray, ddy_circ_tray, ddz_circ_tray]
    jerk  = [dddx_circ_tray, dddy_circ_tray, dddz_circ_tray] 

    return pos, vel, accel, jerk


def patient_force(max_force, min_force):
    """
    Generate force vectors with random direction and values.

    Inpunts:
    --------
        -   max_force:  maximum force value [N].
    Outputs:
    --------
        -   F:      random force value  [N]
        -   theta:  random direction    [rad]
    """
    
    alpha   = np.random.rand(1)
    if alpha>=0.5:
        betha = +1
    else:
        betha = -1
    F       = (max_force - min_force)*alpha + min_force
    F       = betha*F
    theta   = alpha*2*np.pi

    return F, theta


def star_trayectory_generator(t, dt, T):
    """
    Info: Generate points of star trayectory.

    Inputs:
    -------
        - t:        current time [ms]
        - dt:       sampling time [ms]
        - T:        Time for each side [s]
    Outputs:
    --------
        - pos:      number of sides
        - x:        desired position on x axis   
        - y:        desired position on y axis                          
    """

    # Pacient position
    x_paciente = +0.500     # m
    y_paciente = -0.275     # m

    # length of patient arm
    l       =   0.550       # m    

    # Safe parameters defined by therapist 
    r_max   = 0.70  # m     # No se modifica
    r_min   = 0.30  # m     # No se mofica
    y_max   = y_paciente + 0.80*l                   #   [m]
    y_min   = y_paciente + 0.20*l                   #   [m]
    r_circ  = 0.1                                   #   [m] 

    # Parameters of star trayetory     
    f           = 0.2                       # frecuency     [Hz]
    w           = 2*np.pi*f                 # angular velocity [rad/s]

    x0_tray = (r_min + 0.5*(r_max - r_min))         # 0.5 [m]
    y0_tray = (y_min + 0.5*(y_max - y_min))         # 0.0 [m]


    pos     = int(t/(2*dt*T)) + 1
    vueltas = int(t/(2*dt*T*8))
    aspas   = pos - 1 - vueltas*8
    tiempo  = t - (pos-1)*(2*dt*T)

    angulo = (aspas)*2*np.pi/8
    xf_tray = x0_tray + r_circ*np.cos(angulo - np.pi/2)
    yf_tray = y0_tray + r_circ*np.sin(angulo - np.pi/2)


    if (tiempo>=0) & (tiempo<(dt*T)):
        # llevar vaso
        modo = "alcanzar"      
        x = x0_tray + (xf_tray - x0_tray)/(dt*T)*(tiempo)
        y = y0_tray + (yf_tray - y0_tray)/(dt*T)*(tiempo)
        dx = (xf_tray - x0_tray)/(dt*T)
        dy = (yf_tray - y0_tray)/(dt*T)

    elif (tiempo>=(dt*T)) & (tiempo<=(2*dt*T)):
        # alcanzar vaso
        modo = "regresar"
        x = xf_tray + (x0_tray - xf_tray)/(dt*T)*(tiempo-(dt*T))
        y = yf_tray + (y0_tray - yf_tray)/(dt*T)*(tiempo-(dt*T))
        dx = (x0_tray - xf_tray)/(dt*T)
        dy = (y0_tray - yf_tray)/(dt*T)
         
    
    #print("time: ", t)
    #print("error: ", np.round(1000*e_p))
    #print("x: ", np.round(1000*x))
    #print("y: ", np.round(1000*y))
    #print("pos: ", pos)
    #print("aspas: ", aspas)
    #print("angulo: ", angulo)
    #print("tiempo: ", tiempo)
    #print("modo: ", modo)
    #print("vueltas:", vueltas)
    #print("mx: ", mx)
    #print("my: ", my)
    #print("f: ", np.round(f_mano[0:2],0))
    #print("flag: ", flag)
    print("-------")
    return pos, x, y, dx,dy #, m_x, m_y




def v(q):
    return np.expand_dims(q, axis = 1)

def tl(array):
    return array.tolist()



def get_current_pose(q):
    # @info Computes pose [x, y, z, w, ex, ey, ez]
    T_act       = fkine_ur5(q)
    Q_act       = rot2quat(T_act[0:3, 0:3])
    x_act       = np.array([T_act[0,3], T_act[1,3], T_act[2,3], Q_act[0], Q_act[1], Q_act[2], Q_act[3]])     
    return x_act

def get_current_dpose(q,dq):
    # @info Computes first derivative of pose with respect of time
    J = jacobian_pose_ur5(q)
    return np.dot(J, dq)


def compute_error_pose(x_des, x_act):
    # @info Computes pose error (cartesian and quaternions)
    x_error = np.zeros(7)
    x_error[0:3] = x_des[0:3] - x_act[0:3]
    x_error[3:7] = quatError(x_des[3:7], x_act[3:7])
    return x_error

def compute_error_dpose(dx_des, dx_act):
    # @info Computes first derivative of pose error with respect of time
    dx_error = np.zeros(7)
    dx_error[0:3] = dx_des[0:3] - dx_act[0:3]
    dx_error[3:7] = -dx_act[3:7]
    return dx_error     

def compute_error_ddpose(ddx_des, ddx_act):
    # @info Computes second derivative of pose error with respect of time
    ddx_error = np.zeros(7)
    ddx_error[0:3] = ddx_des[0:3] - ddx_act[0:3]
    ddx_error[3:7] = -ddx_act[3:7]
    return ddx_error    

def compute_error_dddpose(dddx_des, dddx_act):
    # @info Computes thrid derivative of pose error with respect of time
    dddx_error = np.zeros(7)
    dddx_error[0:3] = dddx_des[0:3] - dddx_act[0:3]
    dddx_error[3:7] = -dddx_act[3:7]
    return dddx_error 

class AdamOptimizer:
    def __init__(self, alpha=0.001, beta1=0.9, beta2=0.999, epsilon=1e-8):
        self.alpha = alpha
        self.beta1 = beta1
        self.beta2 = beta2
        self.epsilon = epsilon
        self.m = 0
        self.v = 0
        self.t = 0

    def backward_pass(self, k, gradient):
        self.t = self.t + 1
        self.m = self.beta1*self.m + (1 - self.beta1)*gradient
        self.v = self.beta2*self.v + (1 - self.beta2)*(gradient**2)
        m_hat = self.m/(1 - self.beta1**self.t)
        v_hat = self.v/(1 - self.beta2**self.t)
        k = k - self.alpha*(m_hat/(np.sqrt(v_hat) - self.epsilon))
        return k


def reference_trajectory(x_des, x_ref0, dx_ref0, dt):
    """
    Info: Generates a reference trajectory based on a desired trajectory.

    Inputs: 
    ------
        - x_des:  desired trajectory
        - x_ref0: initial conditions of x_ref
        - dt:     sampling time 
    """
    psi = 1 # damping factor
    wn  = 4 # natural frecuency

    k0 = wn*wn
    k1 = 2*psi*wn
    # compute ddx_ref
    ddx_ref = np.multiply(x_des,k0) -  np.multiply(dx_ref0,k1) - np.multiply(x_ref0,k0)
    # double integration 
    dx_ref = dx_ref0 + dt*ddx_ref
    x_ref  = x_ref0  + dt*dx_ref

    return x_ref, dx_ref, ddx_ref