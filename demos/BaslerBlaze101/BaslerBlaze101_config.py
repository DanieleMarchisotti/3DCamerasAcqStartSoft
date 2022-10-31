"""
This sample illustrates how to get a range map from a blaze camera using the Python Harvester module.
"""

import os
import platform

# This is used for reshaping the image buffers.
import numpy as np
import json

# This is used for visualization
import cv2

# Use of Harvester to access the camera.
from harvesters.core import Harvester


def find_producer(name):
    ## Helper for the GenTL producers from the environment path.
    paths = os.environ['GENICAM_GENTL64_PATH'].split(os.pathsep)
    print(paths)
    for path in paths:
        path += os.path.sep + name
        if os.path.exists(path):
            return path
    return ""


def save_intrinsic_as_json(filename, f, cx, cy, width, height):
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'width':
                    width,
                'height':
                    height,
                'intrinsic_matrix': [
                    f, 0, 0, 0, f, 0, cx,
                    cy, 1
                ]
            },
            outfile,
            indent=4)


def write_uint16_array(folderPathToSave,depth_frame):
    with open(folderPathToSave,"wb") as f:
        f.write(bytearray(np.asarray(depth_frame,dtype=np.uint16)))


def write_float32_array(folderPathToSave,depth_frame):
    with open(folderPathToSave,"wb") as f:
        f.write(bytearray(np.float32(depth_frame)))


def write_uint8_array(folderPathToSave,depth_frame):
    with open(folderPathToSave,"wb") as f:
        f.write(bytearray(np.asarray(depth_frame,dtype=np.uint8)))


class ColorAndDepth:
    def __init__(self, path_output):
        # Create Harvester instances.
        self.h = Harvester()
        self.path_output = path_output
        self.frame_counter = 0
        self.timestamp_array = []
        # Location of the Basler blaze GenTL producer.
        if platform.system() == "Windows":
            path_to_blaze_cti = os.environ['BASLER_BLAZE_BIN64'] + "\\ProducerBlaze.cti"
            path_to_gev_cti = find_producer("ProducerGEV.cti")
        elif platform.system() == "Linux":
            path_to_blaze_cti = "/opt/Basler_blaze/lib/gentlproducer/gtl/ProducerBlaze.cti"
            path_to_gev_cti = find_producer("ProducerGEV.cti")
        else:
            print(f"{platform.system()} is not supported")
            assert False

        # Add producer to Harvester.
        print("path_to_blaze_cti: ", path_to_blaze_cti)
        print("path_to_gev_cti: ", path_to_gev_cti)
        assert os.path.exists(path_to_blaze_cti)
        self.h.add_cti_file(path_to_blaze_cti)

        # Update device list.
        self.h.update_device_info_list()

        # Print device list.
        print(self.h.device_info_list)

    def setup_blaze(self, path_output, config):
        ## Connect to first blaze-101 camera.
        dev_info = next((d for d in self.h.device_info_list if d.model == 'blaze-101'), None)
        print(dev_info)
        if dev_info is not None:
            self.ia_blaze = self.h.create_image_acquirer(model=dev_info.model)
        else:
            print("No blaze camera found.")
            exit()

        self.ia_blaze.remote_device.node_map.OperatingMode.value = config['OperatingMode']
        self.ia_blaze.remote_device.node_map.FastMode.value = config['FastMode']
        self.ia_blaze.remote_device.node_map.FilterSpatial.value = config["FilterSpatial"]
        self.ia_blaze.remote_device.node_map.FilterTemporal.value = config["FilterTemporal"]
        self.ia_blaze.remote_device.node_map.OutlierRemoval.value = config["OutlierRemoval"]
        self.ia_blaze.remote_device.node_map.ConfidenceThreshold.value = config["ConfidenceThreshold"]
        self.ia_blaze.remote_device.node_map.GammaCorrection.value = config["GammaCorrection"]
        self.ia_blaze.remote_device.node_map.DepthMin.value = config["DepthMin"]
        self.ia_blaze.remote_device.node_map.DepthMax.value = config["DepthMax"]
        self.ia_blaze.remote_device.node_map.ExposureTime.value = config["ExposureTime"]
        cx = self.ia_blaze.remote_device.node_map.Scan3dPrincipalPointU.value
        cy = self.ia_blaze.remote_device.node_map.Scan3dPrincipalPointV.value
        f = self.ia_blaze.remote_device.node_map.Scan3dFocalLength.value
        save_intrinsic_as_json(self.path_output + "\\camera_intrinsics.json", f, cx, cy, 640, 480)
        self.ia_blaze.remote_device.node_map.ComponentSelector.value = config["ComponentSelector"][0]
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = config["ComponentEnable"][0]
        self.ia_blaze.remote_device.node_map.PixelFormat.value = config["PixelFormat"][0]

        self.ia_blaze.remote_device.node_map.ComponentSelector.value = config["ComponentSelector"][1]
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = config["ComponentEnable"][1]
        self.ia_blaze.remote_device.node_map.PixelFormat.value = config["PixelFormat"][1]

        self.ia_blaze.remote_device.node_map.ComponentSelector.value = config["ComponentSelector"][2]
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = config["ComponentEnable"][2]
        self.ia_blaze.remote_device.node_map.PixelFormat.value = config["PixelFormat"][2]

        ## Configure the camera for software triggering.
        self.ia_blaze.remote_device.node_map.TriggerMode.value = config["TriggerMode"]
        self.ia_blaze.remote_device.node_map.TriggerSource.value = config["TriggerSource"]
        with open(path_output + os.sep + "sensor_params.json", 'w') as outfile:
            obj = json.dump(config, outfile, indent=4)

        ## Start image acquisition.
        self.ia_blaze.start_image_acquisition()
        # self.ia_blaze.start_acquisition()

    def close_blaze(self):
        ## Stop image acquisition.
        self.ia_blaze.stop_image_acquisition()
        self.ia_blaze.remote_device.node_map.TriggerMode.value = "Off"

        ## Disconnect from camera.
        self.ia_blaze.destroy()

    def close_harvesters(self):
        ## Remove the CTI file and reset Harvester.
        self.h.reset()

    def get_image_blaze(self):
        # self.ia_blaze.ExecuteCameraCommand("TimestampLatch")
        with self.ia_blaze.fetch_buffer() as buffer:
            self.timestamp_array.append(buffer.timestamp)

            ## Create an alias of the image components:
            pointcloud = buffer.payload.components[0]

            # intensity = buffer.payload.components[1]

            # confidence = buffer.payload.components[2]

            _3d = pointcloud.data.reshape(pointcloud.height, pointcloud.width,
                                          int(pointcloud.num_components_per_pixel))
            write_float32_array(path_output + "\\depth\\"+str(self.frame_counter).zfill(6)+".bin", _3d[:, :, 2])
            ## Reshape the intensity image into a 2D array:
            # _2d_intensity = intensity.data.reshape(intensity.height, intensity.width)
            ## Reshape the confidence image into a 2D array:
            # _2d_confidence = confidence.data.reshape(confidence.height, confidence.width)
            ## Show the captured images as grayscale. * 255.0 / self.ia_blaze.remote_device.node_map.DepthMax.value
            cv2.imshow('range', (_3d[:, :, 2]*10).astype(np.uint16))
            # cv2.imshow('_2d_intensity', _2d_intensity)
            # cv2.imshow('_2d_confidence', _2d_confidence)
            self.frame_counter += 1

    def run(self, path_output):
        # Set up the cameras.
        with open(".\\config\\BalserBlaze101_config.json", 'r') as infile:
            config = json.load(infile)
        self.setup_blaze(path_output, config)

        print('To exit, press ESC in one of the image windows')
        ## Grab the images.
        while True:
            self.ia_blaze.remote_device.node_map.TriggerSoftware.execute()
            self.get_image_blaze()
            # Break the endless loop by pressing ESC.
            k = cv2.waitKey(1) & 0xFF
            if k == 27 or self.frame_counter == config["max_frames"]:
                break

        # Close the camera and release the producers.
        cv2.destroyAllWindows()
        self.close_blaze()
        # self.close_2DCamera()
        self.close_harvesters()
        np.savetxt(path_output + os.sep + "acq_time_depth.txt", np.array(self.timestamp_array))


if __name__ == "__main__":
    cwd = os.getcwd()
    cwd_split = cwd.split("\\")
    folder_base = ""
    for i in range(len(cwd.split("\\")) - 1):
        folder_base += cwd_split[i] + "\\"
    if not os.path.exists(folder_base + "data\\"):
        os.mkdir(folder_base + "data\\")
    dir_list = [folder_base + "data\\" + o for o in os.listdir(folder_base + "data\\")]
    possible_dir_list = [folder_base + "data\\Acquisition_BaslerBlaze101_" + str(i).zfill(4) for i in range(10000)]
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
    Sample = ColorAndDepth(path_output)
    Sample.run(path_output)
