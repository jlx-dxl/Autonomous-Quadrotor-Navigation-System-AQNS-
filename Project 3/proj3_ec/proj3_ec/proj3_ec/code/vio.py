#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # YOUR CODE HERE
    
    # update position
    new_p = p + v*dt + 1/2 * (q.as_matrix() @ (a_m - a_b) + g) * dt**2

    # update velocity
    new_v = v + (q.as_matrix() @ (a_m - a_b) + g) * dt

    # update rotation 
    delta_q = Rotation.from_rotvec(((w_m - w_b)*dt).flatten())
    new_q = q * delta_q

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    _, _, q, a_b, w_b, _ = nominal_state

    # YOUR CODE HERE

    # Calculate the calibrated acceleration vector
    calibration_corrected_accel = (a_m - a_b).ravel()
    # Generate the skew-symmetric matrix for the acceleration vector
    skew_symmetric_acc_matrix = np.array([
        [0, -calibration_corrected_accel[2], calibration_corrected_accel[1]],
        [calibration_corrected_accel[2], 0, -calibration_corrected_accel[0]],
        [-calibration_corrected_accel[1], calibration_corrected_accel[0], 0]
    ])

    # linearization
    # Initialize the state transition matrix for the linearization process
    state_transition_matrix = np.eye(18)

    # Update different parts of the state transition matrix
    delta_t = dt  # Time step
    state_transition_matrix[:3, 3:6] = np.eye(3) * delta_t
    state_transition_matrix[3:6, 6:9] = - (q.as_matrix() @ skew_symmetric_acc_matrix) * delta_t
    state_transition_matrix[3:6, 9:12] = - q.as_matrix() * delta_t
    state_transition_matrix[3:6, 15:18] = np.eye(3) * delta_t

    # Adjust the rotation matrix component based on angular velocity correction
    rot_correction = Rotation.from_rotvec(((w_m - w_b) * delta_t).ravel())
    state_transition_matrix[6:9, 6:9] = rot_correction.as_matrix().T

    # Gyroscope bias error impact
    state_transition_matrix[6:9, 12:15] = - np.eye(3) * delta_t
    
    # processing noise
    # Define the sizes of the blocks in the noise processing matrix
    zero_block_1 = np.zeros((3, 12))
    identity_block = np.identity(12)
    zero_block_2 = np.zeros((3, 12))

    # Construct the processing noise matrix by vertically stacking the blocks
    noise_processing_matrix = np.concatenate((zero_block_1, identity_block, zero_block_2), axis=0)

    # construct Qi (noise covariance matrix )
    # Initialize the noise covariance matrix
    noise_covariance_matrix = np.zeros((12, 12))

    # Define constants for noise computation
    time_step_squared = dt ** 2
    time_step = dt

    # Set the sub-matrices related to accelerometer and gyroscope
    noise_covariance_matrix[:3, :3] = (accelerometer_noise_density ** 2 * time_step_squared) * np.eye(3)
    noise_covariance_matrix[3:6, 3:6] = (gyroscope_noise_density ** 2 * time_step_squared) * np.eye(3)
    noise_covariance_matrix[6:9, 6:9] = (accelerometer_random_walk ** 2 * time_step) * np.eye(3)
    noise_covariance_matrix[9:12, 9:12] = (gyroscope_random_walk ** 2 * time_step) * np.eye(3)


    new_P = state_transition_matrix @ error_state_covariance @ state_transition_matrix.T + noise_processing_matrix @ noise_covariance_matrix @ noise_processing_matrix.T

    # return an 18x18 covariance matrix
    return new_P


def measurement_update_step(nominal_state, error_state_covariance, image_measurement, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    delta = np.zeros((18,1))
    Jacobian_Matrix = np.zeros((2,18))
    
    # 解包
    position, velocity, quaternion, accel_bias, gyro_bias, gravity = nominal_state
    rotation_matrix = quaternion.as_matrix()
    
    # 预测校正
    predicted_correction = rotation_matrix.T @ (Pw - position)
    predicted_correction_xy = predicted_correction[0:2]/predicted_correction[2]
    
    # 计算预测校正与测量的差异
    measurement_innovation = image_measurement - predicted_correction_xy
    norm_of_innovation = np.linalg.norm(measurement_innovation)
    
    # 展平
    predicted_correction = predicted_correction.flatten()
    image_measurement = image_measurement.flatten()

    if norm_of_innovation < error_threshold:
        # 计算变换后的点相对于图像平面的雅可比矩阵
        correction_depth = predicted_correction[2]
        image_plane_jacobian = np.array([
            [1, 0, -predicted_correction_xy[0].item()],
            [0, 1, -predicted_correction_xy[1].item()]
        ]) / correction_depth

        # 计算旋转扰动对预测校正的影响
        delta_rotation_effect = np.array([
            [0, -correction_depth, predicted_correction[1]],
            [correction_depth, 0, -predicted_correction[0]],
            [-predicted_correction[1], predicted_correction[0], 0]
        ])

        # 通过组合影响来更新雅可比矩阵的相关部分
        image_plane_rotation_effect = image_plane_jacobian @ delta_rotation_effect
        image_plane_position_effect = image_plane_jacobian @ (-rotation_matrix.T)

        # 将计算结果填充到雅可比矩阵的相应部分
        Jacobian_Matrix[:, 0:3] = image_plane_position_effect
        Jacobian_Matrix[:, 6:9] = image_plane_rotation_effect

        # 计算卡尔曼增益
        Kalman_gain = error_state_covariance @ Jacobian_Matrix.T @ np.linalg.inv(
            Jacobian_Matrix @ error_state_covariance @ Jacobian_Matrix.T + Q
        )

        # 更新误差状态协方巨矩阵
        updated_covariance = np.eye(18) - Kalman_gain @ Jacobian_Matrix
        error_state_covariance = updated_covariance @ error_state_covariance @ updated_covariance.T + Kalman_gain @ Q @ Kalman_gain.T

        # 计算误差状态的变化
        delta = Kalman_gain @ measurement_innovation

    return (position + delta[0:3], velocity + delta[3:6], quaternion * Rotation.from_rotvec(delta[6:9].ravel()), accel_bias + delta[9:12], gyro_bias + delta[12:15], gravity + delta[15:18]), error_state_covariance, measurement_innovation

