import numpy as np

class WaypointTraj(object):
    """
    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        
        self.waypoints = points
        self.number_of_points = points.shape[0]
        self.linear_velocity = 1e4
        
        self.I_norm = np.zeros((points.shape[0], 3))
        self.distance = np.zeros(points.shape[0])
        self.duration = np.zeros(points.shape[0])
        self.time_stamp = np.zeros(points.shape[0])

        for i in range(self.number_of_points - 1):
            single_step = points[i+1] - points[i]
            # calculate norm I:
            self.distance[i] = np.linalg.norm(single_step)
            self.I_norm[i] = single_step / self.distance[i]
            # calculate duration:
            self.duration[i] = self.distance[i] / self.linear_velocity
            # calculate duration:
            self.time_stamp[i+1] = self.time_stamp[i] + self.duration[i]
            
    def find_indices(self, t):
        # iterate through the waypoints to find the indices
        if t > self.time_stamp[-1]:
            return self.number_of_points - 1
        else:
            for i in range(len(self.waypoints) - 1):
                if self.time_stamp[i] <= t < self.time_stamp[i+1]:
                    return i

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        
        # given a time t, calculate which way path currently in
        idx = self.find_indices(t)
        
        # calculate velocity:
        x_dot = self.linear_velocity * self.I_norm[idx]
        
        if t > self.time_stamp[-1]:
            x = self.waypoints[-1]
        else:
            x = self.waypoints[idx] + self.linear_velocity * self.I_norm[idx] * (t - self.time_stamp[idx])

        # output the result
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
