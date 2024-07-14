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
        self.k_p_const = 12.00
        self.k_d_const = 3.00
        self.k_R_const = 2400
        self.k_w_const = 30
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

        # STUDENT CODE HERE
        
        control_input = {}
        
        R = Rotation.from_quat(state['q']).as_matrix()
        
        # Calculate errors
        position_diff = state['x'] - flat_output['x']
        velocity_diff = state['v'] - flat_output['x_dot']
        angular_velocity_error = state['w'] - flat_output['yaw_dot']

        # Compute controls
        velocity_control = np.dot(self.Kd, velocity_diff)
        position_control = np.dot(self.Kp, position_diff)

        # Adjusted desired acceleration
        corrected_accel = flat_output['x_ddot'] - velocity_control - position_control
        adjusted_accel = np.array([corrected_accel]).T

        # Gravity effect
        gravity_effect = np.array([0, 0, self.mass * self.g]).reshape([3,1])

        # Force calculation
        computed_force = self.mass * adjusted_accel + gravity_effect

        # Rotation matrix R
        unit_z = np.array([0, 0, 1]).reshape([3, 1])
        rotated_z = np.dot(R, unit_z).reshape([3, 1])
        thrust = np.dot(rotated_z.T, computed_force)

        # Force normalization
        normalized_force = computed_force / np.linalg.norm(computed_force)

        # Yaw handling
        yaw = flat_output['yaw']
        yaw_direction = np.array([np.cos(yaw), np.sin(yaw), 0]).reshape([3, 1])

        # Compute desired orientation
        cross_b3_phi = np.cross(normalized_force, yaw_direction, axis=0)
        normalized_b2 = cross_b3_phi / np.linalg.norm(cross_b3_phi)

        # Desired rotation matrix
        desired_rotation = np.hstack([np.cross(normalized_b2, normalized_force, axis=0), normalized_b2, normalized_force])

        # Compute rotation error
        rotation_skew = (desired_rotation.T @ R - R.T @ desired_rotation) / 2
        rotation_error = np.array([rotation_skew[2,1], rotation_skew[0,2], rotation_skew[1,0]])

        # Compute angular control
        angular_control = (self.inertia @ (-self.KR @ rotation_error - self.Kw @ angular_velocity_error)).reshape([3,1])

        # Final control output
        control_output = np.vstack((thrust, angular_control))
        force_output = np.matmul(np.linalg.inv(self.linear_Eq), control_output)
        force_output[force_output < 0] = 0

        # organize output
        control_input['cmd_motor_speeds'] = np.sqrt(force_output / self.k_thrust).reshape([4])
        control_input['cmd_thrust'] = thrust[0]
        control_input['cmd_moment'] = angular_control.reshape([3])
        control_input['cmd_q'] = Rotation.from_matrix(desired_rotation).as_quat()

        return control_input