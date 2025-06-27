import os
import pyzed.sl as sl
import cv2
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree


def extract_uint8_values(float32_arr):
    # Convert the float32 array to a uint32 array
    uint32_arr = np.frombuffer(float32_arr.tobytes(), dtype=np.uint32)

    # Extract each uint8 value using bitwise operations
    uint8_arr = np.zeros((uint32_arr.shape[0], 4), dtype=np.uint8)
    uint8_arr[:, 3] = (uint32_arr >> 24) & 0xFF
    uint8_arr[:, 2] = (uint32_arr >> 16) & 0xFF
    uint8_arr[:, 1] = (uint32_arr >> 8) & 0xFF
    uint8_arr[:, 0] = uint32_arr & 0xFF

    return uint8_arr


def filter_box(points: np.array):
    x_min, x_max = -1000.0, 1000.0
    z_min, z_max = 100, 1000
    y_min, y_max = -1000, 1000

    mask = np.logical_and(points[:, 0] >= x_min, points[:, 0] < x_max) & \
           np.logical_and(points[:, 1] >= y_min, points[:, 1] < y_max) & \
           np.logical_and(points[:, 2] >= z_min, points[:, 2] < z_max)

    return points[mask]


def extract_pc_from_svo(zed, rt_params, pc_path):
    exit_app = False
    point_cloud = sl.Mat()
    i = 0
    while not exit_app:
        if zed.grab(rt_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            pc_zed = point_cloud.get_data()
            pc_zed = pc_zed.reshape(-1, 4)
            pc_zed = pc_zed[(~np.isnan(pc_zed)).any(axis=1)]
            pc_zed = filter_box(pc_zed)
            points = pc_zed[:, :-1] / 1000
            points = points.astype(np.float64)
            color = pc_zed[:, [-1]]
            color = extract_uint8_values(color)
            pcd = load_o3d_colored_pc(points, color)
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=40, std_ratio=1.0)
            o3d.io.write_point_cloud(os.path.join(pc_path, f'{i}_{point_cloud.timestamp.data_ns}.ply'), cl, compressed=True)
            i += 1

        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("SVO end has been reached.")
            exit_app = True
    zed.set_svo_position(0)

def extract_rgbd_from_svo(zed, rt_params, color_path, depth_path, rec_path):
    exit_app = False
    img_timestamps = []
    color_img = sl.Mat()
    depth_img = sl.Mat()
    img_idx = 0
    while not exit_app:
        if zed.grab(rt_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(color_img, sl.VIEW.SIDE_BY_SIDE)
            zed.retrieve_measure(depth_img, sl.MEASURE.DEPTH)
            color = color_img.get_data()
            depth = depth_img.get_data().astype(np.uint16)
            img_timestamps.append((img_idx, color_img.timestamp.data_ns))

            cv2.imwrite(os.path.join(color_path, f'{img_idx}_{color_img.timestamp.data_ns}.jpg'), color)
            cv2.imwrite(os.path.join(depth_path, f'{img_idx}_{color_img.timestamp.data_ns}.png'), depth)

        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("SVO end has been reached.")
            exit_app = True
        img_idx += 1
    zed.set_svo_position(0)

    rec_path = rec_path.replace('.svo', '_timestamps.txt')
    with open(rec_path, 'w') as f:
        for img_idx, ts in img_timestamps:
            f.write(f"{img_idx}_{ts}\n")


def load_o3d_colored_pc(points, color):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    color = color[:, :3] / 256
    pcd.colors = o3d.utility.Vector3dVector(color)
    return pcd

def time_sync(max_dist_ms, ts_paths, delete_pc=False):

    max_dist_ns = max_dist_ms * 1000000
    # load timestamps
    filename_2_ts_2_idx = {}
    filename2timestamps = {}
    path, filenames = None, []
    for path in ts_paths:
        with open(path, 'r') as f:
            data = f.readlines()
        path, filename = os.path.split(path)
        timestamps_2_idx = {int(idx_and_ts.split('_')[1]): int(idx_and_ts.split('_')[0]) for idx_and_ts in data}
        filename = filename.replace('_timestamps.txt', '')
        filename_2_ts_2_idx[filename] = timestamps_2_idx
        filename2timestamps[filename] = [int(idx_and_ts.split('_')[1]) for idx_and_ts in data]
        filenames.append(filename)

    shortest_len = len(min(filename2timestamps.values(), key=len))
    filename2timestamps = {filename: rec_ts[:shortest_len] for filename, rec_ts in filename2timestamps.items()}
    filename_2_ts_2_idx = {filename: {ts: filename_2_ts_2_idx[filename][ts] for ts in timestamps} for filename, timestamps in filename2timestamps.items()}

    # time from ns to ms & expand dim into (n, 1) for use in KDTree
    filename2timestamps = {filename: np.array([[ts] for ts in rec_ts], dtype=np.int64) for filename, rec_ts in filename2timestamps.items()}
    timestamps = list(filename2timestamps.values())
    master = timestamps[0]
    distances = []
    indices = []
    for i, ts in enumerate(timestamps[1:], 1):
        tree = KDTree(ts)
        dist, idx = tree.query(master, k=1)
        timestamps[i] = timestamps[i][idx]  # nearest neighbor matching for master
        tree = KDTree(timestamps[i]) # recalculate nearest neighbors to get updated indices
        dist, idx = tree.query(master, k=1)
        indices.append(idx)
        distances.append(dist)
    distances = np.array(distances)
    indices = np.array(indices)
    timestamps = np.squeeze(np.array(timestamps, dtype=np.int64), axis=2)

    # delete logic
    # Problem: duplicate nearest neighbors for master ex: [0, 0, 1, 2, 3, ...],
    # where the first timestep (idx) is a NN for the first and second master timestep.
    # In this example we could delete this timestep due to high distance to second master timestep
    # We filter these values and set them to -1 to save them from deletion
    duplicates_all = []
    for i in range(indices.shape[0]):
        unique_vals, counts = np.unique(indices[0], return_counts=True)
        duplicates = unique_vals[counts > 1]
        duplicates_all.append(duplicates)
    duplicates = np.array(duplicates_all)
    dupl_idx_mask = np.isin(indices, duplicates)

    dist_mask = np.any(distances >= max_dist_ns, axis=0)
    dist_mask_multi = np.repeat(dist_mask[np.newaxis, :], dupl_idx_mask.shape[0], axis=0)
    timestamps[1:][np.logical_and(~dist_mask_multi, dupl_idx_mask)] = -1
    # set duplicate timestamps with dist < threshold to -1 to save them from deletion
    delete_timestamps = timestamps[:, dist_mask].astype(str)

    # delete files
    prev_ts = -1
    for rec_id, filename in enumerate(filenames):
        for ts in delete_timestamps[rec_id, :]:
            if ts == prev_ts or ts == "-1":
                continue

            img_idx = filename_2_ts_2_idx[filename][int(ts)]
            prev_ts = ts
            img_path = os.path.join(path, "color", filename, f"{img_idx}_{ts}.jpg")
            depth_path = os.path.join(path, "depth", filename, f"{img_idx}_{ts}.png")
            try:
                os.remove(img_path)
                os.remove(depth_path)
            except FileNotFoundError as e:
                print(f"Unable to remove images: {e}")
            if delete_pc:
                pc_path = os.path.join(path, "point_clouds", filename, f"{img_idx}_{ts}.ply")
                try:
                    os.remove(pc_path)
                except FileNotFoundError as e:
                    print(f"Unable to remove images: {e}")


visualize_pc = False
extract_recs = True
# Set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA

path = rf'/home/chris/Videos/Rec_83'
recs = [os.path.join(path, filename) for filename in os.listdir(path)]

for rec in recs:
    svo_files = [filename for filename in os.listdir(rec) if ".svo" in filename if filename]
    for svo_file in svo_files:
        if not extract_recs:
            break
        print(f"Starting with rec: {svo_file}")
        rec_path = os.path.join(path, rec, svo_file)
        pc_path = os.path.join(path, rec, "point_clouds", svo_file.replace('.svo', ''))
        color_path = os.path.join(path, rec, "color", svo_file.replace('.svo', ''))
        depth_path = os.path.join(path, rec, "depth", svo_file.replace('.svo', ''))
        svo_path = os.path.join(path, rec, svo_file)
        os.makedirs(pc_path, exist_ok=True)
        os.makedirs(color_path, exist_ok=True)
        os.makedirs(depth_path, exist_ok=True)
        init_params.set_from_svo_file(svo_path)
        zed = sl.Camera()
        err = zed.open(init_params)
        runtime_parameters = sl.RuntimeParameters()
        runtime_parameters.confidence_threshold = 80

        extract_rgbd_from_svo(zed, runtime_parameters, color_path, depth_path, rec_path)
        print(f"Extracted rgbd from rec: {svo_file}")
        #extract_pc_from_svo(zed, runtime_parameters, pc_path)
        #print(f"Extracted PCs from rec: {svo_file}")

    time_paths = [os.path.join(path, rec, svo_file).replace('.svo', '_timestamps.txt') for svo_file in svo_files]
    # Not necessary for calibration
    #time_sync(max_dist_ms=25.0, ts_paths=time_paths, delete_pc=False)
