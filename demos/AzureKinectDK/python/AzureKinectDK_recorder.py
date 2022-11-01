import open3d as o3d
import os
import json
import cv2
import numpy as np


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


class ViewerWithCallback:
    def __init__(self, config, device, align_depth_to_color):
        self.flag_exit = False
        self.align_depth_to_color = align_depth_to_color

        self.sensor = o3d.io.AzureKinectSensor(config)
        if not self.sensor.connect(device):
            raise RuntimeError('Failed to connect to sensor')

    def escape_callback(self, vis):
        self.flag_exit = True
        return False

    def run(self, acq_folder):
        glfw_key_escape = 256
        print("Sensor initialized. Press [ESC] to exit.")

        vis_geometry_added = False
        count = 0
        clipping_distance_in_meters = 10
        depth_scale = 1000
        clipping_distance = clipping_distance_in_meters / depth_scale
        while not self.flag_exit:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if rgbd is None:
                continue
            depth_image = np.asarray(rgbd.depth, dtype=np.uint16)
            color_image = np.asarray(rgbd.color, dtype=np.uint8)
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
            cv2.imwrite(acq_folder + "\\depth\\" + "%06d.png" % count, depth_image)
            cv2.imwrite(acq_folder + "\\color\\" + "%06d.png" % count, color_image)
            if key == 27:
                break
            count += 1


if __name__ == '__main__':
    basename = "Acquisition_AzureKinectDK"
    path_output = get_path_dataset(basename)
    with open(".\\config\\AzureKinectDK_config.json", 'r') as infile:
        config = json.load(infile)
    device = 0

    v = ViewerWithCallback(config, device,True)
    v.run(path_output)
