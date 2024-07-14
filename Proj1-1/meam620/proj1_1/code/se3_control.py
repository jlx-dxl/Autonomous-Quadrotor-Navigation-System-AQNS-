import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        self.k_p_const = 5.65
        self.k_d_const = 3.63
        self.k_R_const = 180
        self.k_w_const = 18
        self.Kp = np.diag([self.k_p_const, self.k_p_const, self.k_p_const])
        self.Kd = np.diag([self.k_d_const, self.k_d_const, self.k_d_const])
        self.KR = self.k_R_const * np.eye(3)
        self.Kw = self.k_w_const * np.eye(3)
        
        # parameters for eq16
        l = self.arm_length
        gamma = self.k_drag / self.k_thrust
        self.linear_Eq = np.array([[1,1,1,1],
                                   [0,l,0,-l],
                                   [-l,0,l,0],
                                   [gamma,-gamma,gamma,-gamma]])


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        
        # calculate desired acceleration (eq31)
        desired_x_ddot = flat_output['x_ddot'] - np.matmul(self.Kd, (state['v'] - flat_output['x_dot'])) - np.matmul(self.Kp, (state['x']-flat_output['x']))
        
        # calculate desired force (eq33)
        desired_F = self.mass * desired_x_ddot.reshape([3,1]) + np.array([0, 0, self.mass * self.g]).reshape([3,1])

        # compute u_1 (eq34)
        R = Rotation.from_quat(state['q']).as_matrix()
        b3 = np.matmul(R, np.array([0,0,1])).reshape([3,1])        
        u_1 = b3.T @ desired_F

        # normalize b3 (eq35)
        desired_b3 = desired_F / np.linalg.norm(desired_F)
        
        # calculate a_phi (eq36)
        a_phi = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0]).reshape([3,1])
        
        # calculate desired b2 (eq37)
        desired_b2 = np.cross(desired_b3, a_phi, axis=0) / np.linalg.norm(np.cross(desired_b3, a_phi,axis=0))
        
        # calculate desired matrix (eq38)
        desired_R = np.hstack([np.cross(desired_b2,desired_b3,axis=0), desired_b2, desired_b3])
        
        # calculate e_R (eq39)
        S = (desired_R.T @ R - R.T @ desired_R) / 2
        e_R = np.array([S[2,1],S[0,2],S[1,0]])
        
        # get e_w
        e_w = state['w'] - flat_output['yaw_dot']
        
        # compute u_2 (eq40)
        u_2 = (self.inertia @(-self.KR @ e_R - self.Kw @ e_w)).reshape([3,1])
        
        # compute force using eq16
        u = np.vstack((u_1, u_2))
        F = np.matmul(np.linalg.inv(self.linear_Eq), u)
        F[F<0]=0
        w = np.sqrt(F / self.k_thrust)
    
        cmd_motor_speeds[:] = w.reshape([4])
        cmd_thrust = np.squeeze(np.squeeze(u_1))
        cmd_moment[:] = u_2.reshape([3])
        cmd_q = Rotation.from_matrix(desired_R).as_quat()

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input