import numpy as np

# the gravity vector under robot frame at t=1
gv_r_1 = np.array([1.1, 0.9, -9.7])

# calculate the rotation matrix R(t) as a function of t
def calculate_rotation_matrix(t):
    R = np.array([
        [np.cos(t), -np.sin(t), 0],
        [np.sin(t), np.cos(t), 0],
        [0, 0, 1]
        ])
    return R

# calculate the rotation matrix at t=1 and t=5
R_1 = calculate_rotation_matrix(1)
R_5 = calculate_rotation_matrix(5)

# Since the Gravity vector under the world frame should always be the same, 
# we have: gv_w = R_5 @ gv_r_5 = R_1 @ gv_r_1, hence we have:
# gv_r_5 = R_5^T @ R_1 @ gv_r_1

gv_r_5 = R_5.T @ R_1 @ gv_r_1

print(gv_r_5)