import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cam_params

from transforms3d import euler, affines

import matplotlib
import tf
matplotlib.use("TkAgg")


# def rotation_matrix_to_euler_angles(R):
#     sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#     singular = sy < 1e-6
#
#     if not singular:
#         x = np.arctan2(R[2, 1], R[2, 2])
#         y = np.arctan2(-R[2, 0], sy)
#         z = np.arctan2(R[1, 0], R[0, 0])
#     else:
#         x = np.arctan2(-R[1, 2], R[1, 1])
#         y = np.arctan2(-R[2, 0], sy)
#         z = 0
#
#     return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)

def rotation_matrix_to_euler_angles(R):
    sx = np.sqrt(R[0, 0] * R[0, 0] + R[0, 1] * R[0, 1])
    singular = sx < 1e-6

    if not singular:
        x = -np.arctan2(R[1, 2], R[2, 2])
        y = -np.arctan2(-R[0, 2], sx)
        z = -np.arctan2(R[0, 1], R[0, 0])
    else:
        x = -np.arctan2(-R[1, 2], R[1, 1])
        y = -np.arctan2(-R[0, 2], sx)
        z = 0

    return x,y,z

def rotation_matrix_to_euler_angles_ros(R):
    p = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
    if np.abs(p - np.pi / 2) > 1e-6:
        r = np.arctan2(R[2, 1] / np.cos(p), R[2, 2] / np.cos(p))
        y = np.arctan2(R[1, 0] / np.cos(p), R[0, 0] / np.cos(p))
    else:
        r = 0
        y = np.arctan2(R[0, 1], R[1, 1])

    return r,p,y

def rotmat_to_RPY_Ros(matrix):
    pitch = np.arcsin(matrix[0, 2])

    # Calculate yaw (rotation around X axis)
    if np.cos(pitch) != 0:
        roll = np.arctan2(-matrix[1, 2] / np.cos(pitch), matrix[2, 2] / np.cos(pitch))
    else:
        roll = 0

    # Calculate roll (rotation around Z axis)
    if np.cos(pitch) != 0:
        yaw = np.arctan2(-matrix[0, 1] / np.cos(pitch), matrix[0, 0] / np.cos(pitch))
    else:
        yaw = 0

    return roll, pitch, yaw

def rot2euler(R):
    r = np.arctan2(R[1, 2], R[2, 2])
    p = np.arctan2(-R[0, 2], np.sqrt(R[0, 0] * R[0, 0] + R[0, 1] * R[0, 1]))
    y = np.arctan2(R[0, 1], R[0, 0])

    return np.rad2deg(r), np.rad2deg(p), np.rad2deg(y)



# def rotation_matrix_to_euler_angles_ros(R):
#     y = np.arctan2(R[2, 1], R[2, 2])
#     p = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
#     r = np.arctan2(R[1, 0], R[0, 0])
#
#     return r,p,y


def plot_camera(ax, R, T, cam_idx, scale=0.1, add_labels=False):
    # Create a 3x4 projection matrix
    P = np.eye(4)
    P[:3, :3] = R
    P[:3, 3] = T

    # Define camera points in camera coordinate system
    points = np.array([
        [0, 0, 0, 1],
        [scale, 0, 0, 1],
        [0, scale, 0, 1],
        [0, 0, scale, 1]
    ]).T

    # Transform points to the world coordinate system
    world_points = np.linalg.inv(P) @ points
    world_points /= world_points[3, :]

    # Plot camera position and orientation
    ax.scatter(world_points[0, 0], world_points[1, 0], world_points[2, 0], marker="o", label=f"Camera {cam_idx}")
    ax.plot(world_points[0, :2], world_points[1, :2], world_points[2, :2], color="r", label="X" if add_labels else None)
    ax.plot(world_points[0, :3:2], world_points[1, :3:2], world_points[2, :3:2], color="g", label="Y" if add_labels else None)
    ax.plot(world_points[0, :4:3], world_points[1, :4:3], world_points[2, :4:3], color="b", label="Z" if add_labels else None)
    ax.axis('equal')

info_folder = "/home/sopho/camera_calibration_recs/Rec_1/calib/info"  # Replace with the path to your info folder

# Load extrinsics
extrinsics = cam_params.load_extrinsics(info_folder)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

#print
#print(extrinsics)

first_camera = True
for cam_idx, extrinsic_data in extrinsics.items():
    (R_l, T_l), (R_r, T_r) = extrinsic_data
    print(f"Camera {cam_idx} - Left Camera Rotation Matrix:\n{R_l}")
    print(f"Camera {cam_idx} - Left Camera Translation Vector:\n{T_l}")
    print(f"Camera {cam_idx} - Right Camera Rotation Matrix:\n{R_r}")
    print(f"Camera {cam_idx} - Right Camera Translation Vector:\n{T_r}")
    print()

    plot_camera(ax, R_l, T_l, cam_idx, add_labels=first_camera)
    plot_camera(ax, R_r, T_r, cam_idx, add_labels=False)
    first_camera = False


# Set plot labels and show the plot
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()



(R_l_1, T_l_1), (R_r_1, T_r_1) = extrinsics["1"]
(R_l_0, T_l_0), (R_r_0, T_r_0) = extrinsics["0"]

#transformation frames
A_r_0 = affines.compose(T_r_0, R_r_0,np.ones(3))
A_l_0= affines.compose(T_l_0, R_l_0,np.ones(3))
A_r_1= affines.compose(T_r_1, R_r_1,np.ones(3))
A_l_1= affines.compose(T_l_1, R_l_1,np.ones(3))

#take inverse because camera frame is wanted = inv(projection frame)
A_r_0 = np.linalg.inv(A_r_0)
A_l_0 = np.linalg.inv(A_l_0)
A_r_1 = np.linalg.inv(A_r_1)
A_l_1 = np.linalg.inv(A_l_1)

#extract new T and R
T_l_0= A_l_0[:3,3]
T_r_0= A_r_0[:3,3]
T_l_1= A_l_1[:3,3]
T_r_1= A_r_1[:3,3]
R_r_0= A_r_0[:3,:3]
R_l_0= A_l_0[:3,:3]
R_r_1= A_r_1[:3,:3]
R_l_1= A_l_1[:3,:3]
# left_lens2base_link = np.array([0.01, -0.06, -0.015])
left_lens2base_link = np.array([0.06, 0.015, 0.01])

#---------------------------------------------------------------
#FIND THE RIGHT TRANSFORM
# np.rad2deg()
# np.deg2rad()
T_b_0 = np.zeros(3)
T_b_1 = np.zeros(3)
# T_b_0[3]=1
# T_b_1[3]=1
T_b_0[:3] = T_l_0[:3]+(R_l_0 @ left_lens2base_link)
T_b_1[:3] = T_l_1[:3]+(R_l_1 @ left_lens2base_link)
A_b_0 = affines.compose(T_b_0, R_l_0,np.ones(3))
print(f"{A_b_0} \n")

# Given transformation from world frame to right lens for cam 1

A_b_1 = affines.compose(T_b_1, R_l_1,np.ones(3))
print(f"{A_b_1} \n")

R_ros = np.eye(3)
R_ros[0,:]=[0,0,1]
R_ros[1,:]=[-1,0,0]
R_ros[2,:]=[0,-1,0]
T_ros=np.zeros(3)
A_ros = affines.compose(T_ros, R_ros,np.ones(3))

A_r_01 = np.linalg.inv(A_b_0) @ A_b_1

# A_r_01 = np.linalg.inv(A_r_01)

R_ros_inv = np.linalg.inv(R_ros)
R_l_0_inv = np.linalg.inv(R_l_0)
R_l_1_inv= np.linalg.inv(R_l_1)

R_test = (R_ros_inv @ R_l_1_inv) @ np.linalg.inv(R_ros_inv @ R_l_0_inv)

print(f"{A_r_01} \n")

# np.linalg.inv(R_ros)
r1,p1,y1 = rotmat_to_RPY_Ros(R_test)
R_01 = A_r_01[:3,:3]

# R_01 = R_ros @ R_01
T_01 = R_ros @ A_r_01[:3,3]
RPY=np.array([r1,p1,y1])
# RPY = R_ros @ RPY
print(f"Translation: {T_01}\n")
print(f"RPY: {r1,p1,y1}\n")
print(f"RPY_deg: {np.rad2deg(RPY)}\n")

#---------------------------------------------------------------

plt.show()
