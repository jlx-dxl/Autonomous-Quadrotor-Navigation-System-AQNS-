# Imports

import numpy as np
from scipy.spatial.transform import Rotation


def estimate_pose(uvd1, uvd2, pose_iterations, ransac_iterations, ransac_threshold):
    """
    Estimate Pose by repeatedly calling ransac

    :param uvd1:
    :param uvd2:
    :param pose_iterations:
    :param ransac_iterations:
    :param ransac_threshold:
    :return: Rotation, R; Translation, T; inliers, array of n booleans
    """

    R = Rotation.identity()

    for _ in range(0, pose_iterations):
        angular_velocity, translation, inliers = ransac_pose(uvd1, uvd2, R, ransac_iterations, ransac_threshold)
        R = Rotation.from_rotvec(angular_velocity.ravel()) * R

    return R, translation, inliers


def solve_w_t(uvd1, uvd2, R0):
    """
    solve_w_t core routine used to compute best fit w and t given a set of stereo correspondences

    :param uvd1: (3,n) ndarray : normailzed stereo results from frame 1
    :param uvd2: (3,n) ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :return: w, t : (3,) ndarray estimate for angular velocity, (3,) ndarray estimate for translation
    """

    # TODO Your code here replace the dummy return value with a value you compute
    angular_velocity = translation = np.zeros((3,1))

    rotation_matrix = R0.as_matrix()
    n_points = uvd1.shape[1]

    A = np.zeros((2 * n_points, 6))
    b = np.zeros((2 * n_points, 1))

    for i in range(n_points):
        projected_point = rotation_matrix @ np.array([uvd2[0, i], uvd2[1, i], 1])
        disparity = uvd2[2, i]

        skew_symmetric = np.array([[0, projected_point[2], -projected_point[1], disparity, 0, 0],
                              [-projected_point[2], 0, projected_point[0], 0, disparity, 0],
                              [projected_point[1], -projected_point[0], 0, 0, 0, disparity]])

        projection_matrix = np.array([[1, 0, -uvd1[0, i]],
                        [0, 1, -uvd1[1, i]]])

        A_sub = projection_matrix @ skew_symmetric
        b_sub = -projection_matrix @ projected_point

        A[2 * i:2 * i + 2, :] = A_sub
        b[2 * i:2 * i + 2, 0] = b_sub.ravel()

    solution = np.linalg.lstsq(A, b, rcond=None)[0]
    angular_velocity = solution[0:3]
    translation = solution[3:]
    
    return angular_velocity, translation


def find_inliers(w, t, uvd1, uvd2, R0, threshold):
    """

    find_inliers core routine used to detect which correspondences are inliers

    :param w: (3,) ndarray : angular velocity vector
    :param t: (3,) ndarray : translation vector
    :param uvd1: (3,n) ndarray : normailzed stereo results from frame 1
    :param uvd2: (3,n) ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param threshold: Threshold to use
    :return: ndarray with n boolean entries : Only True for correspondences that pass the test
    """


    n = uvd1.shape[1]
    inliers = np.zeros((n, 1))
    rotation_matrix = Rotation.as_matrix(R0)

    # TODO Your code here replace the dummy return value with a value you compute

    for i in range(n):
        point = rotation_matrix @ np.array([uvd2[0, i], uvd2[1, i], 1])
        disparity = uvd2[2, i]
        
        skew_symmetric = np.array([[0, float(point[2]), float(-point[1]), disparity, 0, 0],
                              [float(-point[2]), 0, float(point[0]), 0, disparity, 0],
                              [float(point[1]), float(-point[0]), 0, 0, 0, disparity]])

        projection_matrix = np.array([[1, 0, -uvd1[0, i]],
                        [0, 1, -uvd1[1, i]]])

        b_vector = ((-1 * projection_matrix) @ point).reshape(-1, 1)
        A_matrix = projection_matrix @ skew_symmetric

        pose_vector = np.vstack((w.reshape(3, 1), t.reshape(3, 1)))
        discrepancy = (A_matrix @ pose_vector) - b_vector
        
        if np.linalg.norm(discrepancy) < threshold:
            inliers[i] = True

    return inliers.flatten()


def ransac_pose(uvd1, uvd2, R0, ransac_iterations, ransac_threshold):
    """

    ransac_pose routine used to estimate pose from stereo correspondences

    :param uvd1: (3,n) ndarray : normailzed stereo results from frame 1
    :param uvd2: (3,n) ndarray : normailzed stereo results from frame 2
    :param R0: Rotation type - base rotation estimate
    :param ransac_iterations: Number of RANSAC iterations to perform
    :ransac_threshold: Threshold to apply to determine correspondence inliers
    :return: w, t : (3,) ndarray estimate for angular velocity, (3,) ndarray estimate for translation
    :return: (n,) boolean ndarray : Only True for correspondences that are inliers

    """
    
    n = uvd1.shape[1]

    # TODO Your code here replace the dummy return value with a value you compute
    angular_velocity = translation = np.zeros((3,1))

    n = uvd1.shape[1]

    if ransac_iterations == 0:
        angular_velocity, translation = solve_w_t(uvd1, uvd2, R0)
        inliers = find_inliers(angular_velocity, translation, uvd1, uvd2, R0, ransac_threshold)
    else:
        max_len = 0
        angular_velocity = translation = np.zeros((3, 1))
        inliers = np.zeros(n, dtype=bool)

        for _ in range(ransac_iterations):
            sample_indices = np.random.choice(n, size=10, replace=False)
            sampled_angular_velocity, sampled_tranlation = solve_w_t(uvd1[:, sample_indices], uvd2[:, sample_indices], R0)
            sampled_inliers = find_inliers(sampled_angular_velocity, sampled_tranlation, uvd1, uvd2, R0, ransac_threshold)

            if np.sum(sampled_inliers) > max_len:
                max_len = np.sum(sampled_inliers)
                angular_velocity, translation = sampled_angular_velocity, sampled_tranlation
                inliers = sampled_inliers

    return angular_velocity, translation, inliers
