import pyrealsense2 as rs
import numpy as np


def initialize_camera():
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.pose)
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    conf.enable_stream(rs.stream.fisheye, 1)
    conf.enable_stream(rs.stream.fisheye, 2)
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
    profiles = p.get_active_profile()
    streams = {"pose": profiles.get_stream(rs.stream.pose),
               "imu": profiles.get_stream(rs.stream.accel),
               "fisheye": profiles.get_stream(rs.stream.fisheye)}
    (R, T) = get_extrinsics(streams["pose"], streams["imu"])
    print(R)
    print(T)
    temp = np.concatenate((R,np.expand_dims(T,1)),axis=1)
    transf = np.concatenate((temp,np.expand_dims(np.array([0,0,0,1]),axis=0)),axis=0)
    import transformations
    transformations.euler_from_matrix(transf)
    np.savetxt("tform_pose_to_imu.txt", transf)
