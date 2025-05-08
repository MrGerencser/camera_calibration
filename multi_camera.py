########################################################################
#
# Copyright (c) 2020, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Multi cameras sample showing how to open multiple ZED in one program
"""

import pyzed.sl as sl
import os
import cv2
import numpy as np
import threading
import time
import signal

zed_list = []
left_list = []
depth_list = []
timestamp_list = []
thread_list = []
stop_signal = False


def signal_handler(signal_received_value, frame):
    global stop_signal
    print(f"Signal {signal_received_value} received in multi_camera.py. Stopping threads...")
    stop_signal = True


def grab_run(index):
    global stop_signal
    global zed_list

    runtime = sl.RuntimeParameters()
    print(f"Grab thread for camera {index} started.")
    while not stop_signal:
        if zed_list[index].grab(runtime) == sl.ERROR_CODE.SUCCESS:
            pass
        else:
            pass
        time.sleep(0.001)

    zed_list[index].disable_recording()
    zed_list[index].close()
    print(f"Grab thread for camera {index} finished and camera closed.")


def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("Running multi_camera.py...")
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 10

    name_list = []

    cameras = sl.Camera.get_device_list()
    if not cameras:
        print("No ZED cameras detected. Exiting.")
        return

    index = 0
    rec_timestamp = str(int(time.time()))
    for cam_info in cameras:
        init.set_from_serial_number(cam_info.serial_number)
        name_list.append(f"ZED {cam_info.serial_number}")
        print(f"Opening {name_list[index]}...")

        current_zed = sl.Camera()
        zed_list.append(current_zed)

        path_create = rf'/home/chris/Videos/Rec_6/Rec'
        output_path = os.path.join(path_create, f"rec_{index}_{rec_timestamp}.svo")

        status = current_zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening camera {cam_info.serial_number}: {status}")
            continue

        params = sl.RecordingParameters(video_filename=output_path, compression_mode=sl.SVO_COMPRESSION_MODE.LOSSLESS)
        err = current_zed.enable_recording(params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Error enabling recording for {cam_info.serial_number}: {err}")
            current_zed.close()
            continue

        print(f"Successfully opened and enabled recording for {name_list[index]} to {output_path}")
        index += 1

    if not zed_list:
        print("No cameras were successfully opened. Exiting.")
        return

    all_threads_started = True
    for i in range(len(zed_list)):
        if zed_list[i].is_opened():
            t = threading.Thread(target=grab_run, args=(i,))
            thread_list.append(t)
            t.start()
        else:
            all_threads_started = False
            print(f"Error: Camera {i} reported not opened before starting thread.")

    if all_threads_started and thread_list:
        print("CAMERAS_RECORDING_STARTED_SIGNAL", flush=True)
    else:
        print("Not all camera threads started or no cameras successfully initialized. Stopping.")
        stop_signal = True

    while not stop_signal:
        if not any(t.is_alive() for t in thread_list):
            print("All grab threads have unexpectedly stopped.")
            stop_signal = True
            break
        time.sleep(0.1)

    print("Stop signal received or threads finished. Joining threads in multi_camera.py...")
    for t in thread_list:
        if t.is_alive():
            t.join(timeout=5.0)
            if t.is_alive():
                print(f"Warning: Thread {t.name} did not join in time.")

    print("multi_camera.py finished.")


if __name__ == "__main__":
    main()
