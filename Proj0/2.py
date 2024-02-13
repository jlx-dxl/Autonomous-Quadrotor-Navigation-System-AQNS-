import numpy as np
from scipy.optimize import fsolve

print("1. solve possible values of x[1] and x[2]:")
# 1. to solve x[1]
x_1 = np.round(np.sqrt(1-0.892**2-0.423**2),3)
print("x[1] = ±", x_1)


# 2. to solve x[2]
x_2 = np.round(np.sqrt(1-x_1**2-(-0.186)**2),3)
print("x[2] = ±", x_2)


# # 3. to solve x[3] ~ x[6] using equations 2,3,8,9
# # 3.1 coefficient matrix
# A = np.array([
#     [1, 1, 0, 0],
#     [0, 0, 1, 1],
#     [1, 0, 1, 0],
#     [0, 1, 0, 1]
# ])
# # 3.2 constant matrix
# B = np.array([1 - 795664, 1 - 0.178928, 1 - 0.970**2, 1 - 0.034596])

# # 3.3 solve the linear equations
# print("The rank of the matrix is:", np.linalg.matrix_rank(A))
# solution = np.linalg.solve(A, B)
# print("The solution is:", solution)


# 4. solve unlinear equations:
print("\n2. solve possible values of x[3] and x[4]:")
x1 = [x_1, -x_1, x_1, -x_1]   # 0.159
x2 = [x_2, x_2, -x_2, -x_2]   # 0.970
x3 = []
x4 = []

def equations(vars):
    x3, x4 = vars
    eq1 = -0.9695 * x4 + 0.186 * x3 - 0.423
    eq2 = 0.892**2 + x3**2 + x4**2 - 1
    return [eq1, eq2]

initial_guess1 = [-1, -1]
solution1 = fsolve(equations, initial_guess1)
x_3, x_4 = solution1
x3.append(np.round(x_3,3))
x4.append(np.round(x_4,3))
initial_guess2 = [1, 1]
solution2 = fsolve(equations, initial_guess2)
x_3, x_4 = solution2
x3.append(np.round(x_3,3))
x4.append(np.round(x_4,3))

print("x3:",x3,"; x4:",x4)

# right-hand frame results
print("\nright-hand frame results:")
for x_1,x_2 in zip(x1,x2):
    for x_3,x_4 in zip(x3,x4):
        for c1,c2 in zip([1,-1,1,-1],[1,1,-1,-1]):
            x_3_ = x_3 * c1
            x_4_ = x_4 * c2
            _, x_5, x_6 = np.cross(np.array([x_1, x_2, -0.186]), np.array([0.892, x_3_, x_4_]))
            x_5 = np.round(x_5,3)
            x_6 = np.round(x_6,3)
            R = np.array([
                        [x_1, 0.892, 0.423],
                        [x_2, x_3_, x_5],
                        [-0.186, x_4_, x_6]
                        ])
            det = np.round(np.linalg.det(R),3)
            if np.abs(np.abs(det)-1)<0.005:
                print(x_1,x_2,x_3_,x_4_,x_5,x_6,";",np.round(np.linalg.det(R),3))

# left-hand frame results
print("left-hand frame results:")
for x_1,x_2 in zip(x1,x2):
    for x_3,x_4 in zip(x3,x4):
        for c1,c2 in zip([1,-1,1,-1],[1,1,-1,-1]):
            x_3_ = x_3 * c1
            x_4_ = x_4 * c2
            _, x_5, x_6 = -np.cross(np.array([x_1, x_2, -0.186]), np.array([0.892, x_3_, x_4_]))
            x_5 = np.round(x_5,3)
            x_6 = np.round(x_6,3)
            R = np.array([
                        [x_1, 0.892, 0.423],
                        [x_2, x_3_, x_5],
                        [-0.186, x_4_, x_6]
                        ])
            det = np.round(np.linalg.det(R),3)
            if np.abs(np.abs(det)-1)<0.005:
                print(x_1,x_2,x_3_,x_4_,x_5,x_6,";",np.round(np.linalg.det(R),3))