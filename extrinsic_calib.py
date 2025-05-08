import os
import pickle
import numpy as np
import pyzed.sl as sl
import cv2

import cam_params


#Create a folder called calib which contains all the data extracted from the svo files and the run the codey

def find_cb_corners(img, block_sizes):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # adaptiveThreshold can improve or deteriorate calibration
    #gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 2)y
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block_sizes[svo_idx], 2)
    ret, corners_2d = cv2.findChessboardCorners(gray, chessboard_size, None)
    #cb_width = corners_2d[:, 0, 0].max() - corners_2d[:, 0, 0].min()
    #cb_height = corners_2d[:, 0, 1].max() - corners_2d[:, 0, 1].min()
    #if(cb_width < cb_height):
    #    ret, corners_2d = cv2.findChessboardCorners(gray, chessboard_size_flipped, None)
    return corners_2d

# Chessboard
chessboard_size = (8, 11)
chessboard_size_flipped = (11, 8)
square_size = 0.030  # meters
# 3D CB points
cb_3d = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
cb_3d[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
cb_3d *= square_size

# iterate through 
path = rf'/home/chris/Videos/Rec_6'
# Info folder
info_folder = os.path.join(path, "calib", "info")
os.makedirs(info_folder, exist_ok=True)


calib = [os.path.join(path, filename) for filename in os.listdir(path) if filename == "calib"][0]
svo_files = [filename for filename in os.listdir(calib) if ".svo" in filename]

all_rvecs, all_tvecs = [], []
all_intrinsics, all_disto = [], []
img_shape = (0, 0)

# Adaptive Thresholding
block_sizes = [21, 61, 61]
test_vis = False
for svo_idx, svo_file in enumerate(reversed(svo_files)):
    svo_name = svo_file.replace('.svo', '')
    img_folder = os.path.join(path, "calib", "color", svo_name)
    img_paths = [os.path.join(path, "calib", "color", svo_name, img) for img in os.listdir(img_folder)]

    init_params = sl.InitParameters()
    init_params.set_from_svo_file(os.path.join(path, "calib", svo_file))
    zed = sl.Camera()
    err = zed.open(init_params)

    calib_params = zed.get_camera_information().camera_configuration.calibration_parameters
    fx_l, fy_l = calib_params.left_cam.fx, calib_params.left_cam.fy
    fx_r, fy_r = calib_params.right_cam.fx, calib_params.right_cam.fy
    cx_l, cy_l = calib_params.left_cam.cx, calib_params.left_cam.cy
    cx_r, cy_r = calib_params.right_cam.cx, calib_params.right_cam.cy

    intrinsics_l = np.array([[fx_l, 0, cx_l], [0, fy_l, cy_l], [0, 0, 1]], dtype=np.float32)
    intrinsics_r = np.array([[fx_r, 0, cx_r], [0, fy_r, cy_r], [0, 0, 1]], dtype=np.float32)
    disto_l = np.array([0.0, 0.0, 0.0, 0.0, 0.0])# calib_params.left_cam.disto
    disto_r = np.array([0.0, 0.0, 0.0, 0.0, 0.0])#calib_params.right_cam.disto

    all_intrinsics.append((intrinsics_l, intrinsics_r))
    all_disto.append((disto_l, disto_r))

    points_3d_l,  points_3d_r= [], []
    points_2d_l,  points_2d_r= [], []
    count = 0
    block_sizes = [21, 61, 61]
    for img_path in img_paths:
        full_img = cv2.imread(img_path)
        img_left = full_img[:, :full_img.shape[1]//2, :]
        img_right = full_img[:, full_img.shape[1]//2:, :]
        corners_2d_l = find_cb_corners(img_left, block_sizes)
        corners_2d_r = find_cb_corners(img_right, block_sizes)

        points_2d_l.append(corners_2d_l)
        points_2d_r.append(corners_2d_r)
        points_3d_l.append(cb_3d)
        points_3d_r.append(cb_3d)

        count += 1
        if count >= 10:
            break

    img_shape = img_left.shape[::-1][1:]

    ret_l, K_l, D_l, rvecs_l, tvecs_l = cv2.calibrateCamera(points_3d_l, points_2d_l, img_shape, intrinsics_l, disto_l,
                                            flags=cv2.CALIB_USE_INTRINSIC_GUESS+
                                                  cv2.CALIB_FIX_INTRINSIC +
                                                  cv2.CALIB_FIX_PRINCIPAL_POINT+cv2.CALIB_FIX_FOCAL_LENGTH)

    ret_r, K_r, D_r, rvecs_r, tvecs_r = cv2.calibrateCamera(points_3d_r, points_2d_r, img_shape, intrinsics_r, disto_r,
                                                            flags=cv2.CALIB_USE_INTRINSIC_GUESS+
                                                                  cv2.CALIB_FIX_INTRINSIC +
                                                                  cv2.CALIB_FIX_PRINCIPAL_POINT+cv2.CALIB_FIX_FOCAL_LENGTH)

    print(f"SVO_Left: {svo_name.split('_')[1]}, Loss: {ret_l}")
    print(f"SVO_Right: {svo_name.split('_')[1]}, Loss: {ret_r}")
    # Visual controll
    all_rvecs.append((rvecs_l, rvecs_r))
    all_tvecs.append((tvecs_l, tvecs_r))

    img_points_proj_i_l, _ = cv2.projectPoints(points_3d_l[-1], rvecs_l[-1], tvecs_l[-1], intrinsics_l, disto_l)
    img_points_proj_i_r, _ = cv2.projectPoints(points_3d_r[-1], rvecs_r[-1], tvecs_r[-1], intrinsics_r, disto_r)

    img_proj_l = img_left.copy()
    img_proj_r = img_right.copy()
    cv2.drawChessboardCorners(img_proj_l, chessboard_size, img_points_proj_i_l, True)
    cv2.drawFrameAxes(img_proj_l, K_l, D_l, rvecs_l[-1], tvecs_l[-1], 0.3)
    cv2.imshow("Projected points Left", img_proj_l)

    cv2.drawChessboardCorners(img_proj_r, chessboard_size, img_points_proj_i_r, True)
    cv2.drawFrameAxes(img_proj_r, K_r, D_r, rvecs_r[-1], tvecs_r[-1], 0.3)
    cv2.imshow("Projected points Right", img_proj_r)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# save intrinsics
cam_idxs = [svo_file.split('_')[1] for svo_file in svo_files]

cam_params.save_img_size(height=img_shape[1], width=img_shape[0], info_folder=info_folder)
cam_params.save_intrinsics(all_intrinsics, all_disto, cam_idxs, info_folder)

# save r_vecs, t_vecs
rvecs = [(cv2.Rodrigues(rvecs[0][-1])[0], cv2.Rodrigues(rvecs[1][-1])[0]) for rvecs in all_rvecs]
tvecs = [(np.squeeze(tvecs[0][-1], axis=1), np.squeeze(tvecs[1][-1], axis=1))for tvecs in all_tvecs]
is_ok = input("Do you want to save these extrinsics? Enter [y] or [n]")
if is_ok == "y":
    cam_params.save_extrinsics(rvecs, tvecs, cam_idxs, info_folder)

