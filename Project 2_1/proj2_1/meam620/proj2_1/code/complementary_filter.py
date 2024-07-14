# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


# %%

def complementary_filter_update(initial_rot: Rotation, gyro_data: list, accel_data: list, time_interval: float) -> Rotation:
    """
    Implements a complementary filter update for attitude estimation.

    :param initial_rot: Initial rotation estimate as a Rotation object
    :param gyro_data: Angular velocity in radians per second as a list [roll, pitch, yaw]
    :param accel_data: Linear acceleration in meters per second squared as a list [x, y, z]
    :param time_interval: Duration of the interval in seconds
    :return: Updated rotation estimate as a Rotation object
    """

    # Assuming gravity aligns with the IMU's x-axis
    gravity_direction = np.array([1, 0, 0])
    
    # Convert angular velocity (gyro_data) to rotation vector and create a Rotation object
    gyro_rotation = Rotation.from_rotvec(np.multiply(gyro_data, time_interval))
    
    # Apply gyro-derived rotation to the initial rotation estimate to get updated rotation
    updated_rotation_matrix = initial_rot * gyro_rotation
    
    # Normalize the acceleration vector after applying the updated rotation (to get its direction)
    normalized_accel = updated_rotation_matrix.apply(accel_data) / norm(updated_rotation_matrix.apply(accel_data))

    # Calculate the axis of needed correction by taking the cross product of the normalized acceleration and gravity direction
    axis_of_correction = np.cross(normalized_accel, gravity_direction)
    
    # Normalize the axis of correction if it is not a zero vector
    if norm(axis_of_correction) != 0:
        axis_of_correction = axis_of_correction / norm(axis_of_correction)

    # Calculate the angle of correction by taking the arccos of the dot product between normalized acceleration and gravity direction
    correction_angle = np.arccos(np.dot(normalized_accel, gravity_direction))
    
    # Convert the angle and axis of correction into a quaternion
    quaternion_correction = Rotation.from_rotvec(correction_angle * axis_of_correction).as_quat()

    # Calculate measurement error based on the deviation of acceleration magnitude from gravity
    error_measurement = abs(norm(accel_data) / 9.81 - 1)
    
    # Determine blending factor based on measurement error
    if error_measurement >= 0.2:
        alpha_blend = 0  # If error is large, rely solely on gyro data
    elif error_measurement <= 0.1:
        alpha_blend = 1  # If error is small, incorporate acceleration data fully
    else:
        alpha_blend = -10 * (error_measurement - 0.1) + 1  # Scale blending factor based on error

    # Blend the quaternion correction based on the calculated blending factor
    blended_quaternion_correction = ((1 - alpha_blend) * np.array([0, 0, 0, 1])) + (alpha_blend * quaternion_correction)
    
    # Apply the blended quaternion correction to the updated rotation
    final_rotation_estimate = Rotation.from_quat(blended_quaternion_correction) * updated_rotation_matrix

    # Return the final rotation estimate
    return final_rotation_estimate


