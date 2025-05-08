# Camera Calibration System

A comprehensive toolkit for automating ZED camera calibration workflows, featuring automatic recording, extraction, and calibration.

## Prerequisites

- Python 3.x
- ZED SDK installed
- Required Python packages (opencv-python, numpy, yaml)
- Connected ZED cameras

## Quick Start

The automated workflow script handles the entire calibration process:

```bash
python automate_calibration_workflow.py
```

This will:
1. Create a new recording directory (`Rec_N`)
2. Record short video sequences from connected cameras
3. Extract images from SVO files
4. Set up calibration directories
5. Run extrinsic calibration 
6. Generate human-readable transform files

## Individual Scripts

### `automate_calibration_workflow.py`

The main orchestration script that runs the entire calibration pipeline.

```bash
python automate_calibration_workflow.py
```

When prompted, you can specify a custom base path for recordings or use the default (`~/Videos`).

### `multi_camera.py` 

Records synchronized video from connected ZED cameras.

```bash
python multi_camera.py
```

### `svo_extract.py`

Extracts image frames from SVO video files.

```bash
python svo_extract.py
```

### `extrinsic_calib.py`

Performs extrinsic calibration using chessboard images.

```bash
python extrinsic_calib.py
```

### `write_to_human_readable_file.py`

Converts calibration data to human-readable format and can save transforms to YAML.

```bash
python write_to_human_readable_file.py
```

## Transform Files in YAML Format

### Saving Transforms Directly to YAML

The system can automatically save camera transform matrices to YAML format. The process happens during the final step of the calibration workflow when `write_to_human_readable_file.py` is executed.

#### Manual YAML Generation

To manually generate or update a YAML transform file:

1. Run the `write_to_human_readable_file.py` script independently:
   ```bash
   python write_to_human_readable_file.py
   ```

2. The script will:
   - Read calibration data from the most recent calibration
   - Generate transform matrices
   - Save transforms in human-readable format
   - Update the YAML file with new transforms

#### YAML File Format

The transforms are stored in the following format:

```yaml
transforms:
  T_0S:  # Transform matrix name
    - [x, x, x, x]  # Row 1 (4x4 matrix)
    - [x, x, x, x]  # Row 2
    - [x, x, x, x]  # Row 3
    - [0, 0, 0, 1]  # Row 4
    
  H1:    # Camera 1 transform
    - [x, x, x, x]
    - [x, x, x, x]
    - [x, x, x, x]
    - [0, 0, 0, 1]
```

#### Customizing YAML Output

To customize the YAML output format or file destination:
1. Open `write_to_human_readable_file.py`
2. Locate the YAML file path definition
3. Modify parameters as needed
4. Save changes before running the script

## Troubleshooting

- **No cameras detected**: Ensure ZED cameras are properly connected and recognized by the system
- **Calibration failures**: Use a clear chessboard pattern and ensure good lighting conditions
- **YAML file not updating**: Check file permissions and paths in the script

## Advanced Usage

For more detailed control of the calibration process, you can run each script individually in sequence with custom parameters.
