import pyrealsense2 as rs
import numpy as np
import cv2
import struct
from setting_files import setAllParameters, writeCameraModes, writeParameters
import json
import os


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
    depth_profiles,color_profiles,synch_depth_profiles_idx, synch_color_profiles_idx = writeCameraModes.writeRGBDModes(pipe)
    Mode=int(input("Enter the camera stream index mode: "))
    config = rs.config()
    config.enable_stream(rs.stream.depth, depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().width(),
                         depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().height(), rs.format.z16,
                         depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().fps())
    config.enable_stream(rs.stream.color, color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().width(),
                         color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().height(), rs.format.bgr8,
                         color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().fps())
    print(depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().width()," x ",
          depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().height()," @ ",
          depth_profiles[synch_depth_profiles_idx[Mode]].as_video_stream_profile().fps())
    print(color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().width()," x ",
          color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().height()," @ ",
          color_profiles[synch_color_profiles_idx[Mode]].as_video_stream_profile().fps())
    pipe_profile=pipe.start(config)
    depth_sensor= pipe_profile.get_device().first_depth_sensor()
    profile=pipe_profile.get_stream(rs.stream.depth)
    profile_color = profile.get_stream(rs.stream.color)
    postProcessStrs,dec_filter,spatial_filter,temp_filter,depth_sensor, meas_unit_selected = setAllParameters.set_parameters(depth_sensor)
    acq_time=[]
    meas_unit_final=depth_sensor.get_option(rs.option.depth_units)
    stringToWrite = writeParameters.to_file(profile, profile_color, meas_unit_final, depth_sensor, postProcessStrs,
                                            path_output, [dec_filter, spatial_filter, temp_filter])
    print("\nParameters used for the acquisition: ")
    print(stringToWrite)
    align_to = rs.stream.color
    align = rs.align(align_to)
    clipping_distance_in_meters = 10
    depth_scale = depth_sensor.get_depth_scale()
    clipping_distance = clipping_distance_in_meters / depth_scale
    i = 0
    while True:
        # Get frameset of color and depth
        frames = pipe.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if postProcessStrs[0]=="Y":
            if postProcessStrs[1] == "Y":
                depth_frame=dec_filter.process(aligned_depth_frame)
            if postProcessStrs[2]=="Y":
                depth_frame=spatial_filter.process(aligned_depth_frame)
            if postProcessStrs[3]=="Y":
                depth_frame=temp_filter.process(aligned_depth_frame)
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if i == 0:
            save_intrinsic_as_json(os.path.join(path_output, "camera_intrinsic.json"), aligned_depth_frame)
        acq_time.append(frames.get_timestamp())
        depth_image = np.where((depth_image > clipping_distance) | \
                               (depth_image <= 0), 0, depth_image)
        cv2.imwrite("%s/%06d.png" % \
                    (path_depth, i), depth_image)
        cv2.imwrite("%s/%06d.png" % \
                    (path_color, i), color_image)
        i+=1
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
        cv2.resizeWindow('Recorder Realsense', min(len(images[0]), 1280), min(len(images), 480))
        cv2.imshow('Recorder Realsense', images)
        key = cv2.waitKey(1)
        if key==27:
            break
    acq_time_bin = struct.pack('f' * len(acq_time), *acq_time)
    file=open(path_output + "\\acqTime.bin","wb")
    file.write(acq_time_bin)
    file.close()
    pipe.stop()
    config.disable_all_streams()
    cv2.destroyAllWindows()
