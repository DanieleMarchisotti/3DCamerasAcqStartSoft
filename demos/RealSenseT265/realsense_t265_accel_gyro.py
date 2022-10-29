import pyrealsense2 as rs
import numpy as np


def initialize_camera():
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p


def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])


if __name__ == '__main__':
    p = initialize_camera()
    try:
        while True:
            f = p.wait_for_frames()
            gyro = gyro_data(f[0].as_motion_frame().get_motion_data())
            accel = accel_data(f[1].as_motion_frame().get_motion_data())
            print("accelerometer: ", accel)
            print("gyro: ", gyro)
    finally:
        p.stop()
