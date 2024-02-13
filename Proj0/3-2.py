import numpy as np
import matplotlib.pyplot as plt

# rotation matrix R
R = np.array([[-1, 0, 0], 
              [0, -np.cos(np.pi/6), np.sin(np.pi/6)], 
              [0, np.sin(np.pi/6), np.cos(np.pi/6)]])

# create 3d figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# axes before rotation
x_axis = np.array([1, 0, 0])
y_axis = np.array([0, 1, 0])
z_axis = np.array([0, 0, 1])
ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', label='Original X')
ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', label='Original Y')
ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', label='Original Z')

# axes after rotation
x_axis_rot = R @ x_axis
y_axis_rot = R @ y_axis
z_axis_rot = R @ z_axis
ax.quiver(0, 0, 0, x_axis_rot[0], x_axis_rot[1], x_axis_rot[2], color='r', linestyle='dashed', label='Rotated X')
ax.quiver(0, 0, 0, y_axis_rot[0], y_axis_rot[1], y_axis_rot[2], color='g', linestyle='dashed', label='Rotated Y')
ax.quiver(0, 0, 0, z_axis_rot[0], z_axis_rot[1], z_axis_rot[2], color='b', linestyle='dashed', label='Rotated Z')

# set attributes
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()

plt.show()