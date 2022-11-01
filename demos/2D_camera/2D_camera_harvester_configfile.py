
import os
import platform

# This is used for reshaping the image buffers.
import numpy as np
import json

# This is used for visualization and debayering.
import cv2
import time
from harvesters.core import Harvester


def write_uint8_array(folderPathToSave,depth_frame):
    with open(folderPathToSave,"wb") as f:
        f.write(bytearray(np.asarray(depth_frame,dtype=np.uint8)))


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
    return path_output


class RGBCamera(object):
    def __init__(self, path_output_color, path_output_ts):
        with open(".\\config\\FLIR_config.json", 'r') as infile:
            self.config = json.load(infile)
        self.path_output_color = path_output_color
        self.path_output_ts = path_output_ts
        self.frames_num = self.config["frames_num"]
        self.frames_count = 0
        self.h = Harvester()
        self.h.add_file(self.config["cti_file"])
        self.h.update()
        print(self.h.device_info_list)
        self.ia = self.h.create_image_acquirer(0)
        self.nodemap = None
        self.list_frames = []
        self.width = 0
        self.height = 0
        self.time_latch = None
        self.timestamp_array = np.array([])
        self.t1 = np.array([])
        self.t2 = np.array([])
        self.t_latch = np.array([])
        self.t_diff = np.array([])
        self.t_PC = np.array([])
        self.timestamp = np.array([])
        self.frames_pointer = 0
        self.save_frame = 1
        self.frames_name = 0
        self.saved_frames_count = 0
        self.f = None

    def setup_camera(self):
        self.nodemap = self.ia.remote_device.node_map
        # nodemap.PixelFormat.value = 'BayerRG16'
        self.nodemap.AcquisitionMode.value = self.config["AcquisitionMode"]
        self.nodemap.AcquisitionBurstFrameCount.value = self.config["AcquisitionBurstFrameCount"]
        self.nodemap.TriggerSelector.value = self.config["TriggerSelector"]
        self.nodemap.AcquisitionFrameRateEnable.value = self.config["AcquisitionFrameRateEnable"]
        if self.config["AcquisitionFrameRateEnable"] == 1:
            self.nodemap.AcquisitionFrameRate.value = self.config["AcquisitionFrameRate"]
        # nodemap.AcquisitionFrameRateEnable.value = True
        # nodemap.AcquisitionFrameRate.value = 200
        self.nodemap.TriggerSource.value = self.config["TriggerSource"]
        self.nodemap.TriggerActivation.value = self.config["TriggerActivation"]
        # nodemap.TriggerDelayEnabled.value = True
        self.nodemap.TriggerMode.value = self.config["TriggerMode"]
        # nodemap.TriggerOverlap.value = 'ReadOut'
        self.nodemap.ExposureMode.value = self.config["ExposureMode"]
        self.nodemap.ExposureAuto.value = self.config["ExposureAuto"]
        self.nodemap.GainAuto.value = self.config["GainAuto"]
        # nodemap.BalanceWhiteAuto.value = 'Off'
        # nodemap.pgrExposureCompensationAuto.value = 'Off'


        # nodemap.TriggerDelay.value = self.config['trigger_delay']
        self.nodemap.Gain.value = self.config["Gain"]
        self.nodemap.ExposureTime.value = self.config["ExposureTime"]
        self.width = self.nodemap.Width.value
        self.height = self.nodemap.Height.value

    def start_camera(self):
        self.ia.start_acquisition()

    def run(self):
        self.time_latch = self.nodemap.TimestampLatch
        t1_temp = time.time()
        self.time_latch.execute()
        t2_temp = time.time()
        self.t1 = np.append(self.t1, t1_temp)
        self.t2 = np.append(self.t2, t2_temp)
        t_latch = self.nodemap.TimestampLatchValue()
        self.t_latch = np.append(self.t_latch, t_latch)
        self.f = open(self.path_output_color + os.sep + str(0).zfill(6) + ".bin", "wb")
        while self.frames_count < self.frames_num:
            if self.frames_count >= 1:
                try:
                    with self.ia.fetch_buffer(timeout=1) as buffer:
                        t1_temp = time.time()
                        self.time_latch.execute()
                        t2_temp = time.time()
                        self.t1 = np.append(self.t1, t1_temp)
                        self.t2 = np.append(self.t2, t2_temp)
                        t_latch_temp = self.nodemap.TimestampLatchValue()
                        self.t_latch = np.append(self.t_latch, t_latch_temp)
                        self.timestamp_array = np.append(self.timestamp_array, buffer.timestamp)
                        I = buffer.payload.components[0]
                        self.f.write(bytearray(np.asarray(I.data.reshape(I.height, I.width).copy())))
                        cv2.namedWindow("color")
                        cv2.resizeWindow("color", int(I.width * 0.5), int(I.height * 0.5))
                        cv2.imshow("color", cv2.resize(np.asarray(I.data.reshape(I.height, I.width).copy()),
                                                       (int(I.width * 0.5), int(I.height * 0.5))))
                        key = cv2.waitKey(1)
                        if key == 27:
                            break
                        self.frames_count += 1
                except:
                    break
            else:
                with self.ia.fetch_buffer() as buffer:
                    t1_temp = time.time()
                    self.time_latch.execute()
                    t2_temp = time.time()
                    self.t1 = np.append(self.t1, t1_temp)
                    self.t2 = np.append(self.t2, t2_temp)
                    t_latch_temp = self.nodemap.TimestampLatchValue()
                    self.t_latch = np.append(self.t_latch, t_latch_temp)
                    self.timestamp_array = np.append(self.timestamp_array, buffer.timestamp)
                    I = buffer.payload.components[0]
                    self.f.write(bytearray(np.asarray(I.data.reshape(I.height, I.width).copy())))
                    cv2.namedWindow("color")
                    cv2.resizeWindow("color", int(I.width*0.5), int(I.height*0.5))
                    cv2.imshow("color", cv2.resize(np.asarray(I.data.reshape(I.height, I.width).copy()),
                                                   (int(I.width*0.5), int(I.height*0.5))))
                    cv2.waitKey(1)
                    self.frames_count += 1
            print(self.frames_count)
        self.f.close()
        t1_temp = time.time()
        self.time_latch.execute()
        t2_temp = time.time()
        self.t1 = np.append(self.t1, t1_temp)
        self.t2 = np.append(self.t2, t2_temp)
        t_latch = self.nodemap.TimestampLatchValue()
        self.t_latch = np.append(self.t_latch, t_latch)
        cv2.destroyAllWindows()

    def stop_camera(self):
        self.ia.stop_acquisition()
        self.ia.destroy()
        self.h.reset()

    def save_frames(self):
        self.t_diff = self.t2 - self.t1
        self.t_PC = (self.t1+self.t2)/2
        for t_samp in self.timestamp_array:
            t_camera_prev = self.t_latch[np.argmin(t_samp - self.t_latch[t_samp - self.t_latch > 0])]/1e9
            dim_argmin = len(self.t_latch[t_samp - self.t_latch > 0])
            t_camera_curr = self.t_latch[np.argmax(t_samp - self.t_latch[t_samp - self.t_latch < 0]) + dim_argmin]/1e9
            t_PC_prev = self.t_PC[np.argmin(t_samp - self.t_latch[t_samp - self.t_latch > 0])]
            t_PC_curr = self.t_PC[np.argmax(t_samp - self.t_latch[t_samp - self.t_latch < 0]) + dim_argmin]
            dPC = t_PC_curr - t_PC_prev
            d_camera = t_camera_curr - t_camera_prev
            self.timestamp = np.append(self.timestamp, t_PC_prev + dPC/d_camera * (t_samp/1e9 - t_camera_prev))
        np.savetxt(self.path_output_ts + os.sep + "acq_time_depth.txt", self.timestamp_array)
        np.savetxt(self.path_output_ts + os.sep + "t_1.txt", self.t1)
        np.savetxt(self.path_output_ts + os.sep + "t_2.txt", self.t2)
        np.savetxt(self.path_output_ts + os.sep + "t_diff.txt", self.t_diff)
        np.savetxt(self.path_output_ts + os.sep + "t_latch.txt", self.t_latch)
        np.savetxt(self.path_output_ts + os.sep + "timestamp.txt", self.timestamp)


if __name__ == '__main__':
    base_name = "Acquisition_FLIR"
    path_output = get_path_dataset(base_name)
    path_output_color = path_output + os.sep + "color_" + str(0).zfill(4)
    path_output_color_ts = path_output + os.sep + "color_ts_" + str(0).zfill(4)
    if not os.path.exists(path_output_color):
        os.mkdir(path_output_color)
    if not os.path.exists(path_output_color_ts):
        os.mkdir(path_output_color_ts)
    color_camera = RGBCamera(path_output_color, path_output_color_ts)
    color_camera.setup_camera()
    color_camera.start_camera()
    color_camera.run()
    color_camera.save_frames()
    color_camera.stop_camera()
