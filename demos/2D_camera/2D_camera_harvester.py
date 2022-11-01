from harvesters.core import Harvester
import cv2
import numpy as np
import time

if __name__ == '__main__':
    h = Harvester()

    h.add_file("C:/Program Files/FLIR Systems/Spinnaker/cti64/vs2015/FLIR_GenTL_v140.cti")

    h.update()

    print(h.device_info_list)

    ia = h.create_image_acquirer(0)
    nodemap = ia.remote_device.node_map
    nodemap.TriggerSource.value = 'Line0'
    nodemap.TriggerActivation.value = 'RisingEdge'
    nodemap.TriggerMode.value = 'Off'
    nodemap.ExposureMode.value = 'Timed'
    nodemap.ExposureAuto.value = 'Off'
    nodemap.GainAuto.value = 'Off'
    nodemap.AcquisitionMode.value = 'Continuous'
    nodemap.AcquisitionFrameRateEnable.value = True
    nodemap.AcquisitionFrameRate.value = 30
    nodemap.Gain.value = 10
    nodemap.ExposureTime.value = 2000

    ia.start_acquisition()
    frames_count = 0
    time_latch = nodemap.TimestampLatch
    t1 = np.array([])
    t2 = np.array([])
    t_latch = np.array([])
    t_diff = np.array([])

    while True:
        t1_temp = time.time()
        time_latch.execute()
        t2_temp = time.time()
        t1 = np.append(t1, t1_temp)
        t2 = np.append(t2, t2_temp)
        t_latch_temp = nodemap.TimestampLatchValue()
        t_latch = np.append(t_latch, t_latch_temp)
        t_diff = np.append(t_diff, t2_temp - t1_temp)
        with ia.fetch_buffer() as buffer:
            frame_time = time.time()
            I = buffer.payload.components[0]
            _2d = I.data.reshape(I.height, I.width)
            cv2.imshow("image", _2d)
            k = cv2.waitKey(1)
            if k == 27:
                break
            prev_time = frame_time
            frames_count += 1
    cv2.destroyAllWindows()
