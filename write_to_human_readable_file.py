import pprint
import numpy as np
import os
import pickle
# import yaml # Using ruamel.yaml instead
from ruamel.yaml import YAML as RuamelYAML
from ruamel.yaml.scalarstring import LiteralScalarString # For potential multiline string comments if needed
from ruamel.yaml.comments import CommentedMap, CommentedSeq

def load_extrinsics(info_folder):
    extrinsics_file_path = os.path.join(info_folder, "extrinsics.txt")
    if not os.path.exists(extrinsics_file_path):
        print(f"Error: extrinsics.txt not found in {info_folder}")
        return None
    try:
        with open(extrinsics_file_path, "rb") as f:
            data = f.read()
        extrinsics = pickle.loads(data)
        return extrinsics
    except Exception as e:
        print(f"Error loading or unpickling extrinsics.txt: {e}")
        return None

def inverse_transform(R, t):
    T_inv = np.eye(4, 4)
    T_inv[:3, :3] = np.transpose(R)
    t_col = np.array(t).reshape(3,1)
    T_inv[:3, 3] = (-1 * np.transpose(R) @ t_col).squeeze()
    return T_inv

# --- Helper for ruamel.yaml to represent lists in flow style (single line) ---
class FlowList(CommentedSeq): # Inherit from CommentedSeq for ruamel.yaml
    pass

def flow_style_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
# --- End Helper ---

def main():
    np.set_printoptions(precision=4, suppress=True)
    info_folder = "/home/chris/Videos/Rec_83/calib/info" 
    
    extrinsics = load_extrinsics(info_folder)

    if extrinsics is None:
        print("Failed to load extrinsics. Cannot proceed with YAML generation.")
        return
    if "0" not in extrinsics or "1" not in extrinsics:
        print("Error: Required keys '0' or '1' not found in extrinsics data.")
        return

    try:
        with open('extrinsics_hand_tracking.txt', 'w') as file:
            pprint.pprint(extrinsics, stream=file)
        print(f"Original extrinsics data saved to extrinsics_hand_tracking.txt")
    except Exception as e:
        print(f"Error saving extrinsics_hand_tracking.txt: {e}")

    (R_l_1, T_l_1), (R_r_1, T_r_1) = extrinsics["1"]
    (R_l_0, T_l_0), (R_r_0, T_r_0) = extrinsics["0"]

    print("\nall transforms in camera frame to checkerboard frame i.e. the translation is c_T_cs with c as camera frame and "
          "s as checkerboard frame")
    print("Translation l1 ", T_l_1)
    # ... (rest of your print statements) ...

    T_sc_l1 = inverse_transform(R_l_1, T_l_1) # Corresponds to T_chess_cam2
    T_sc_l0 = inverse_transform(R_l_0, T_l_0) # Corresponds to T_chess_cam1

    print("\ntransform of left camera of zed 0 (T_chess_cam1 candidate):")
    print(np.array2string(T_sc_l0, separator=', ', formatter={'float_kind': lambda x: "%.4f" % x}))
    print("\ntransform of left camera of zed 1 (T_chess_cam2 candidate):")
    print(np.array2string(T_sc_l1, separator=', ', formatter={'float_kind': lambda x: "%.4f" % x}))

    # --- Prepare data for YAML using ruamel.yaml ---
    yaml_data = CommentedMap()
    transforms_map = CommentedMap()
    
    # Creates a function to make our 4x4 matrices with proper formatting
    def make_matrix_with_flow_rows(matrix):
        result = []
        for row in matrix:
            flow_row = CommentedSeq(row)
            flow_row.fa.set_flow_style()  # Force this row to be on a single line
            result.append(flow_row)
        return result
    
    # T_robot_chess transform (previously T_0S)
    T_robot_chess_matrix = make_matrix_with_flow_rows([
        [-1.0, 0.0, 0.0, 0.358],
        [0.0, 1.0, 0.0, 0.03],
        [0.0, 0.0, -1.0, 0.006],
        [0.0, 0.0, 0.0, 1.0]
    ])
    transforms_map['T_robot_chess'] = T_robot_chess_matrix
    
    # Add a blank line and comment before T_chess_cam1
    transforms_map.yaml_set_comment_before_after_key('T_chess_cam1', before='\ncamera 33137761')
    
    # T_chess_cam1 transform (previously H1)
    T_chess_cam1_matrix_list = [[round(float(val), 4) for val in row] for row in T_sc_l0.tolist()]
    transforms_map['T_chess_cam1'] = make_matrix_with_flow_rows(T_chess_cam1_matrix_list)
    
    # Add a blank line and comment before T_chess_cam2
    transforms_map.yaml_set_comment_before_after_key('T_chess_cam2', before='\ncamera 36829049')
    
    # T_chess_cam2 transform (previously H2)
    T_chess_cam2_matrix_list = [[round(float(val), 4) for val in row] for row in T_sc_l1.tolist()]
    transforms_map['T_chess_cam2'] = make_matrix_with_flow_rows(T_chess_cam2_matrix_list)
    
    yaml_data['transforms'] = transforms_map
    
    # --- Save to YAML file ---
    output_yaml_path = "/home/chris/franka_ros2_ws/src/superquadric_grasp_system/config/transform.yaml"
    
    
    ryaml = RuamelYAML()
    ryaml.indent(mapping=2, sequence=4, offset=2)
    
    try:
        with open(output_yaml_path, 'w') as yaml_file:
            ryaml.dump(yaml_data, yaml_file)
        print(f"\nTransformation matrices saved to YAML: {output_yaml_path}")
    except Exception as e:
        print(f"Error saving YAML file to {output_yaml_path}: {e}")
    # --- End Save to YAML file ---

if __name__ == "__main__":
    main()