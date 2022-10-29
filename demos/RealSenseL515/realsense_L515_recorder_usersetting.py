
# pyrealsense2 is required.
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from os import makedirs
from os.path import exists, join
import shutil
import json
from enum import IntEnum
import setAllParameters
import writeCameraModes
import struct
import writeParameters

try:
    # Python 2 compatible
    input = raw_input
except NameError:
    pass


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


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        user_input = input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            makedirs(path_folder)
        else:
            exit()


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


if __name__ == '__main__':
    basename = "Acquisition_RealSense_D4xx"
    path_output = get_path_dataset(basename)
    path_depth = path_output+"\\depth"
    path_color = path_output+"\\color"

    # Create a pipeline
    ctx=rs.context()
    device_name=ctx.devices[0].get_info(rs.camera_info.name)
    print(device_name)

    pipeline = rs.pipeline()
    # depth_profiles=writeCameraModes.writeDepthModes(pipeline)
    depth_profiles,color_profiles,synch_depth_profiles_idx, synch_color_profiles_idx, is_RGBD_modes=writeCameraModes.writeRGBDModes(pipeline)
    if is_RGBD_modes:
        Mode=int(input("Enter the camera stream index mode: "))
        config = rs.config()
        config.enable_stream(rs.stream.depth,
                             depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().width(),
                             depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().height(),
                             rs.format.z16,
                             depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().fps())
        config.enable_stream(rs.stream.color,
                             color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().width(),
                             color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().height(),
                             rs.format.bgr8,
                             color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().fps())
        print(depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().width(), " x ",
              depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().height(), " @ ",
              depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().fps())
        print(color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().width(), " x ",
              color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().height(), " @ ",
              color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().fps())
    else:
        print("Cannot choose the RGBD module to set: set single depth and color modes")
        depth_profiles, color_profiles, mode_depth, mode_color = writeCameraModes.choose_RGB_and_depth_mode(pipeline)
        config = rs.config()
        config.enable_stream(rs.stream.depth,
                             depth_profiles[mode_depth].as_video_stream_profile().width(),
                             depth_profiles[mode_depth].as_video_stream_profile().height(),
                             rs.format.z16,
                             depth_profiles[mode_depth].as_video_stream_profile().fps())
        config.enable_stream(rs.stream.color,
                             color_profiles[mode_color].as_video_stream_profile().width(),
                             color_profiles[mode_color].as_video_stream_profile().height(),
                             rs.format.bgr8,
                             color_profiles[mode_color].as_video_stream_profile().fps())
        print(depth_profiles[mode_depth].as_video_stream_profile().width(), " x ",
              depth_profiles[mode_depth].as_video_stream_profile().height(), " @ ",
              depth_profiles[mode_depth].as_video_stream_profile().fps())
        print(color_profiles[mode_color].as_video_stream_profile().width(), " x ",
              color_profiles[mode_color].as_video_stream_profile().height(), " @ ",
              color_profiles[mode_color].as_video_stream_profile().fps())

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()
    postProcessStrs,dec_filter,spatial_filter,temp_filter,depth_sensor=setAllParameters.set_parameters(depth_sensor)
    profile_depth=profile.get_stream(rs.stream.depth)
    profile_color=profile.get_stream(rs.stream.color)
    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 10
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)
    if is_RGBD_modes:
        stringToWrite=writeParameters.to_file(profile_depth, profile_color,Mode, Mode,1000,depth_scale,depth_sensor,postProcessStrs,path_output,[dec_filter,spatial_filter,temp_filter])
    else:
        stringToWrite = writeParameters.to_file(profile_depth, profile_color, mode_depth, mode_color, 1000, depth_scale, depth_sensor,
                                                postProcessStrs, path_output, [dec_filter, spatial_filter, temp_filter])
    print("\nParameters used for the acquisition: ")
    print(stringToWrite)
    # Streaming loop
    frame_count = 0
    acq_time=[]

    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # if device_name=="Intel RealSense SR300":
            # depth_image=np.uint16(depth_image/8)
            # if args.record_imgs:
            if frame_count == 0:
                save_intrinsic_as_json(
                    join(path_output, "camera_intrinsic.json"),
                        color_frame)
            else:
                acq_time.append(frames.get_timestamp())
            depth_image=np.where((depth_image > clipping_distance) | \
                    (depth_image <= 0),0,depth_image)
            cv2.imwrite("%s/%06d.png" % \
                        (path_depth, frame_count), depth_image)
            cv2.imwrite("%s/%06d.png" % \
                        (path_color, frame_count), color_image)
            print("Saved color + depth image %06d" % frame_count)
            frame_count += 1
            print(frames.get_timestamp())
            print(frames.get_frame_metadata(rs.frame_metadata_value.frame_timestamp))
            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            # depth image is 1 channel, color is 3 channels
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | \
                    (depth_image_3d <= 0), grey_color, color_image)
            # Render images
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))
            cv2.namedWindow('Recorder Realsense', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Recorder Realsense', min(len(images[0]),1280), min(len(images),480))
            # cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Recorder Realsense', images)
            key = cv2.waitKey(1)

            # if 'esc' button pressed, escape loop and exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
        config.disable_all_streams()
        try:
            acq_time_bin = struct.pack('f' * len(acq_time), *acq_time)
            file = open(path_output + "\\acqTime.bin", "wb")
            file.write(acq_time_bin)
            file.close()
        except Exception:
            print("Error saving acquisition time file")
