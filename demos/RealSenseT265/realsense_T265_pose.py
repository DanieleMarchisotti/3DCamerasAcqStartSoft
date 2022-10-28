#!/usr/bin/python

# First import the library
import pyrealsense2 as rs
import numpy as np
import threading

stop_button = 0


def run_t265():
    global stop_button
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and request pose data
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)

    # Start streaming with requested config
    pipe.start(cfg)
    save_dir = ""
    data_to_save = []
    try:
        while True:
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()
            # Fetch pose frame
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                data_to_save.append([frames.get_frame_metadata(rs.frame_metadata_value.frame_timestamp),
                                     data.translation.x, data.translation.y, data.translation.z,
                                     data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
            if stop_button == 1:
                break
    finally:
        pipe.stop()
    data_to_save = np.array(data_to_save)
    np.savetxt(save_dir + "acq_t265.txt", data_to_save)


if __name__ == '__main__':
    run_sensor = threading.Thread(target=run_t265, args=())
    run_sensor.start()
    while True:
        key = input("Enter 'q' to stop: ")
        if key == "q":
            stop_button = 1
            break
