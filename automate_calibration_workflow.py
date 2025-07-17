import os
import re
import shutil
from pathlib import Path
import subprocess
import time
import threading

def get_next_rec_number(base_video_path: Path) -> int:
    """Finds the next Rec_N number."""
    rec_dirs = [d for d in base_video_path.iterdir() if d.is_dir() and d.name.startswith("Rec_")]
    if not rec_dirs:
        return 1
    max_num = 0
    for d in rec_dirs:
        try:
            num = int(d.name.split("_")[1])
            if num > max_num:
                max_num = num
        except (IndexError, ValueError):
            continue
    return max_num + 1

def update_script_content(script_path: Path, replacements: list[tuple[str, str]]) -> bool:
    """
    Updates the content of a script file using regex replacements.
    replacements is a list of (regex_pattern, replacement_string_template)
    """
    if not script_path.exists():
        print(f"Warning: Script {script_path} not found. Skipping update.")
        return False
    
    try:
        content = script_path.read_text()
        original_content = content
        
        for pattern, repl_template in replacements:
            content = re.sub(pattern, repl_template, content, flags=re.MULTILINE)

        if content != original_content:
            script_path.write_text(content)
            print(f"Updated paths in {script_path}")
            return True
        else:
            print(f"No changes needed in {script_path} (patterns might not have matched).")
            return False
    except Exception as e:
        print(f"Error updating {script_path}: {e}")
        return False

def run_python_script(script_path: Path, input_data: str | None = None, cwd: Path | None = None, timeout_duration: int | None = None):
    """Runs a python script using subprocess."""
    if not script_path.exists():
        print(f"Error: Script {script_path} not found. Cannot execute.")
        raise FileNotFoundError(f"Script {script_path} not found.")

    command = ['python3', str(script_path)]
    print(f"\nExecuting: {' '.join(command)}")
    
    try:
        if timeout_duration: # For scripts like multi_camera.py that need to be stopped
            process = subprocess.Popen(
                command,
                cwd=cwd,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,  # Line-buffered
                universal_newlines=True
            )
            
            script_name = script_path.name
            log_prefix = f"[{script_name} STDOUT] "
            err_log_prefix = f"[{script_name} STDERR] "
            ready_signal_msg = "CAMERAS_RECORDING_STARTED_SIGNAL"
            signal_received = False
            
            print(f"Waiting for {script_name} to signal readiness ('{ready_signal_msg}')...")
            # Timeout for waiting for the ready signal itself
            max_signal_wait_time = 60  # seconds (adjust as needed for camera init)
            start_wait_time = time.time()

            # Thread to continuously print stderr
            stderr_lines = []
            def log_stderr():
                for err_line in iter(process.stderr.readline, ''):
                    if err_line:
                        print(f"{err_log_prefix}{err_line.strip()}")
                        stderr_lines.append(err_line.strip())
            
            stderr_thread = threading.Thread(target=log_stderr)
            stderr_thread.daemon = True
            stderr_thread.start()

            while time.time() - start_wait_time < max_signal_wait_time:
                if process.poll() is not None: # Process terminated prematurely
                    print(f"{err_log_prefix}{script_name} exited prematurely with code {process.returncode} while waiting for signal.")
                    break
                
                line = process.stdout.readline()
                if not line: # EOF or process closed stdout
                    if process.poll() is not None:
                        break 
                    time.sleep(0.1) # Wait a bit if no line but process is running
                    continue

                line_stripped = line.strip()
                if line_stripped:
                    print(f"{log_prefix}{line_stripped}")

                if ready_signal_msg in line_stripped:
                    print(f"{log_prefix}{script_name} signaled ready. Starting {timeout_duration}s recording timer.")
                    signal_received = True
                    break
            
            if signal_received:
                print(f"Recording with {script_name} for {timeout_duration} seconds...")
                time.sleep(timeout_duration)
                print(f"Recording time for {script_name} elapsed. Terminating {script_name}...")
            elif process.poll() is None: 
                print(f"{err_log_prefix}Warning: {script_name} did not send ready signal within {max_signal_wait_time}s. Terminating.")
            else: 
                print(f"{err_log_prefix}Warning: {script_name} did not send ready signal and has already terminated (Code: {process.returncode}).")

            if process.poll() is None: # If still running
                print(f"Sending SIGTERM to {script_name} (PID: {process.pid}).")
                process.terminate()
                try:
                    process.wait(timeout=10) 
                except subprocess.TimeoutExpired:
                    print(f"{err_log_prefix}{script_name} did not terminate gracefully after SIGTERM, sending SIGKILL.")
                    process.kill()
                    process.wait(timeout=5) 
            
            # Wait for stderr thread to finish
            stderr_thread.join(timeout=2)

            # Capture any final stdout that might have been missed (less likely with readline)
            final_stdout = process.stdout.read()
            if final_stdout:
                print(f"{log_prefix}Final remaining STDOUT:\n{final_stdout.strip()}")
            
            print(f"{script_name} process stopped. Return code: {process.returncode}")

            if process.returncode not in [0, None, -15, -9]: # 0 (success), None (still running - should not happen), -15 (SIGTERM), -9 (SIGKILL)
                 print(f"Warning: {script_name} exited with a non-standard code: {process.returncode}")
            elif process.returncode == 0 and not signal_received and timeout_duration > 0: # Check if it exited cleanly before signal
                 print(f"Warning: {script_name} exited with code 0 but did not send the ready signal as expected.")

        else: # Original logic for non-timed scripts (subprocess.run)
            completed_process = subprocess.run(
                command,
                input=input_data,
                text=True,
                check=True, 
                capture_output=True,
                cwd=cwd
            )
            print(f"{script_path.name} stdout:\n{completed_process.stdout}")
            if completed_process.stderr:
                print(f"{script_path.name} stderr:\n{completed_process.stderr}")
        print(f"Successfully executed {script_path.name}")
    except subprocess.CalledProcessError as e:
        print(f"Error executing {script_path.name}: {e}")
        print(f"Stdout:\n{e.stdout}")
        print(f"Stderr:\n{e.stderr}")
        raise 
    except FileNotFoundError: 
        raise
    except Exception as e:
        print(f"An unexpected error occurred while running {script_path.name}: {e}")
        raise


def main():
    workspace_path = Path.cwd()
    
    default_videos_path = Path.home() / "Videos"
    videos_base_input = input(f"Enter the base path for Rec folders (default: {default_videos_path}): ")
    videos_base_path = Path(videos_base_input) if videos_base_input else default_videos_path

    if not videos_base_path.is_dir():
        print(f"Warning: The path {videos_base_path} is not a valid directory.")
        try:
            videos_base_path.mkdir(parents=True, exist_ok=True)
            print(f"Created base video directory: {videos_base_path}")
        except OSError as e:
            print(f"Error: Could not create directory {videos_base_path}: {e}")
            return

    # Original workflow approach
    next_rec_num = get_next_rec_number(videos_base_path)
    new_rec_folder_name = f"Rec_{next_rec_num}"
    new_rec_base_path = videos_base_path / new_rec_folder_name
    new_rec_rec_path = new_rec_base_path / "Rec"
    new_calib_folder_path = new_rec_base_path / "calib"
    new_calib_info_path_str = str(new_calib_folder_path / "info").replace('\\', '/')
    
    new_rec_base_path_str_rf = str(new_rec_base_path).replace('\\', '/')
    new_rec_rec_path_str_rf = str(new_rec_rec_path).replace('\\', '/')

    print(f"\nNext recording number: {next_rec_num}")
    print(f"Target recording base path: {new_rec_base_path}")
    print(f"Target SVO/image path: {new_rec_rec_path}")
    print(f"Target calibration path: {new_calib_folder_path}")

    # 1. Create Rec_X/Rec folder
    try:
        new_rec_rec_path.mkdir(parents=True, exist_ok=True)
        print(f"Successfully created directory: {new_rec_rec_path}")
    except OSError as e:
        print(f"Error creating directory {new_rec_rec_path}: {e}")
        return

    # 2. Adjust paths in scripts
    print("\n--- Updating script paths ---")
    multicam_script = workspace_path / "multi_camera.py"
    svo_extract_script = workspace_path / "svo_extract.py"
    extrinsic_calib_script = workspace_path / "extrinsic_calib.py"
    write_human_script = workspace_path / "write_to_human_readable_file.py"

    multicam_replacements = [
        (r"(path_create\s*=\s*rf')([^']+)(')", f"\g<1>{new_rec_rec_path_str_rf}\g<3>"),
        (r"(output_path\s*=\s*os\.path\.join\(rf')([^']+)(')", f"\g<1>{new_rec_rec_path_str_rf}\g<3>")
    ]
    update_script_content(multicam_script, multicam_replacements)

    svo_extract_replacements = [ (r"(path\s*=\s*rf')([^']+)(')", f"\g<1>{new_rec_base_path_str_rf}\g<3>") ]
    update_script_content(svo_extract_script, svo_extract_replacements)

    extrinsic_calib_replacements = [ (r"(path\s*=\s*rf')([^']+)(')", f"\g<1>{new_rec_base_path_str_rf}\g<3>") ]
    update_script_content(extrinsic_calib_script, extrinsic_calib_replacements)
    
    write_human_replacements = [ (r'(info_folder\s*=\s*")([^"]+)(")', f'\g<1>{new_calib_info_path_str}\g<3>') ]
    update_script_content(write_human_script, write_human_replacements)

    try:
        print("\n--- Starting Automated Workflow Execution ---")

        # 3. Run multicam.py (for ~2 seconds)
        print("Step 3: Running multicam.py to record SVO files...")
        print(f"Ensure {multicam_script} is configured to save to {new_rec_rec_path}")
        run_python_script(multicam_script, timeout_duration=2)

        # Run svo_extract.py
        print("\nStep 3 (cont.): Running svo_extract.py...")
        print(f"Ensure {svo_extract_script} processes data from {new_rec_base_path_str_rf} (expecting SVOs in {new_rec_rec_path})")
        run_python_script(svo_extract_script)

        # 4. Create 'calib' folder
        print(f"\nStep 4: Creating calibration folder: {new_calib_folder_path}")
        try:
            new_calib_folder_path.mkdir(parents=True, exist_ok=True)
            print(f"Successfully created directory: {new_calib_folder_path}")
        except OSError as e:
            print(f"Error creating directory {new_calib_folder_path}: {e}")
            raise

        # Copy recorded files from Rec_X/Rec into Rec_X/calib
        print(f"Copying files and folders from {new_rec_rec_path} to {new_calib_folder_path}...")
        if new_rec_rec_path.exists() and new_rec_rec_path.is_dir():
            copied_items_count = 0
            for item_name in os.listdir(new_rec_rec_path):
                source_item_path = new_rec_rec_path / item_name
                destination_item_path = new_calib_folder_path / item_name
                try:
                    if source_item_path.is_file():
                        shutil.copy2(source_item_path, destination_item_path)
                        print(f"Copied file: {source_item_path} to {destination_item_path}")
                        copied_items_count += 1
                    elif source_item_path.is_dir():
                        if destination_item_path.exists():
                             print(f"Warning: Destination directory {destination_item_path} already exists. Skipping copytree for this item to avoid errors. Manual check might be needed.")
                        else:
                            shutil.copytree(source_item_path, destination_item_path)
                            print(f"Copied directory: {source_item_path} to {destination_item_path}")
                            copied_items_count += 1
                except Exception as e_copy:
                    print(f"Error copying {source_item_path} to {destination_item_path}: {e_copy}")
            
            print(f"Copying complete. Copied {copied_items_count} files/folders.")
            if copied_items_count == 0:
                print(f"Warning: No files or folders were found in {new_rec_rec_path} to copy.")
        else:
            print(f"Warning: Source folder {new_rec_rec_path} for copying does not exist or is not a directory.")

        # 5. Run extrinsic_calib.py
        print("\nStep 5: Running extrinsic_calib.py...")
        print(f"Ensure chessboard_size and square_size are correctly set in {extrinsic_calib_script}!")
        print("Attempting to automatically answer 'y' to save extrinsics prompt.")
        run_python_script(extrinsic_calib_script, input_data='y\n')

        # 6. Run write_to_human_readable_file.py
        print("\nStep 6: Running write_to_human_readable_file.py...")
        print(f"This script will use 'info_folder = \"{new_calib_info_path_str}\"'")
        print(f"It will attempt to directly update yaml file (defined within the zed perception script).") 
        run_python_script(write_human_script)

        print("\n--- Automated Workflow Completed Successfully ---")

    except FileNotFoundError:
        print("Halting automation due to a missing script. Please ensure all required scripts are present.")
    except subprocess.CalledProcessError as e:
        error_output = getattr(e, 'stderr', '') or getattr(e, 'stdout', '')
        if "The function is not implemented" in str(error_output) and "cvShowImage" in str(error_output):
            print("\n--- OpenCV GUI Error Detected ---")
            print("The extrinsic_calib.py script failed due to missing OpenCV GUI support.")
            print("This is because cv2.imshow() requires GTK support which is not installed.")
            print("\nOptions to fix this:")
            print("1. Install GUI support: sudo apt install libgtk2.0-dev pkg-config && pip install opencv-python")
            print("2. Modify extrinsic_calib.py to run in headless mode (comment out cv2.imshow lines)")
            print("3. Set DISPLAY environment variable if running remotely")
            print("\nThe calibration calculation itself should work fine - it's just the visualization that failed.")
        else:
            print("Halting automation due to an error in one of the executed scripts. Check output above.")
    except Exception as e:
        print(f"An unexpected error occurred during workflow execution: {e}")
        print("Automation halted.")
        
if __name__ == "__main__":
    main()