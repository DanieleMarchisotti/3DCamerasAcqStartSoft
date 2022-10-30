import pyrealsense2 as rs
import numpy as np
import cv2
import struct
import os
import json


def save_intrinsic_as_json(filename, frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0, 0, 0, intrinsics.fy, 0, intrinsics.ppx,
                    intrinsics.ppy, 1
                ]
            },
            outfile,
            indent=4)


def get_path_dataset(base_name):
    cwd = os.getcwd()
    cwd_split = cwd.split("\\")
    folder_base = ""
    for i in range(len(cwd.split("\\")) - 1):
        folder_base += cwd_split[i] + "\\"
    if not os.path.exists(folder_base + "data\\"):
        os.mkdir(folder_base + "data\\")
    dir_list = [folder_base + "data\\" + o for o in os.listdir(folder_base + "data\\")]
    possible_dir_list = [folder_base + "data\\" + base_name + "_" + str(i).zfill(4) for i in range(10000)]
    count = 0
    for i, curr_dir in enumerate(possible_dir_list):
        if curr_dir in dir_list:
            count += 1
            continue
        else:
            break
    path_output = possible_dir_list[count]
    if not os.path.exists(path_output):
        os.mkdir(path_output)
    if not os.path.exists(path_output + "\\depth"):
        os.mkdir(path_output + "\\depth")
    if not os.path.exists(path_output + "\\color"):
        os.mkdir(path_output + "\\color")
    return path_output


if __name__ == '__main__':
    basename = "Acquisition_RealSense_SR305"
    path_output = get_path_dataset(basename)
    path_depth = path_output+"\\depth"
    path_color = path_output+"\\color"
    pipe = rs.pipeline()
    realFPS = 0
    clipping_distance_in_meters = 10

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipe_profile = pipe.start(config)
    depth_sensor = pipe_profile.get_device().query_sensors()[0]
    depth_scale = depth_sensor.get_depth_scale()
    clipping_distance = clipping_distance_in_meters / depth_scale
    profile = pipe_profile.get_stream(rs.stream.depth)
    meas_unit_final = depth_sensor.get_option(rs.option.depth_units)
    acq_time = []
    i = 0
    while True:
        frames = pipe.wait_for_frames()
        depth_frame_raw = frames.get_depth_frame()
        color_frame_raw = frames.get_color_frame()
        depth_frame = np.asanyarray(depth_frame_raw.get_data())
        color_frame = np.asanyarray(color_frame_raw.get_data())
        if i == 0:
            save_intrinsic_as_json(os.path.join(path_output, "camera_intrinsic.json"), depth_frame)
        acq_time.append(frames.get_timestamp())
        cv2.imshow("Acquisition", depth_frame * meas_unit_final / 10)
        depth_image = np.where((depth_frame > clipping_distance) | \
                               (depth_frame <= 0), 0, depth_frame)
        cv2.imwrite("%s/%06d.png" % \
                    (path_depth, i), depth_image)
        cv2.imwrite("%s/%06d.png" % \
                    (path_color, i), color_frame)
        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        # depth image is 1 channel, color is 3 channels
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | \
                              (depth_image_3d <= 0), grey_color, color_frame)
        # Render images
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        cv2.namedWindow('Recorder Realsense', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Recorder Realsense', min(len(images[0]), 1280), min(len(images), 480))
        cv2.imshow('Recorder Realsense', images)
        k = cv2.waitKey(1)
        if k == 27:
            break
        i += 1
    acq_time_bin = struct.pack('f' * len(acq_time), *acq_time)
    file = open(path_output + "\\acqTime.bin", "wb")
    file.write(acq_time_bin)
    file.close()
    pipe.stop()
    config.disable_all_streams()
    cv2.destroyAllWindows()
