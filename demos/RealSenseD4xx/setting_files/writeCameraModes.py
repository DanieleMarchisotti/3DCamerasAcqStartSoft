
def writeDepthModes(pipe):
    pipe_for_profiles=pipe.start()
    depth_sensor_profiles = pipe_for_profiles.get_device().first_depth_sensor()
    depth_profiles=depth_sensor_profiles.get_stream_profiles()
    # color_profiles=rs.camera_info.get_stream_profiles(depth_sensor_profiles)
    pipe.stop()
    print("Camera streams indexes:")
    for i,item in enumerate(depth_profiles):
        if item.stream_name()=="Depth":
            print("Mode ",i,": ",item.stream_name(),": ",item.as_video_stream_profile().width()," x ", item.as_video_stream_profile().height()," @ ",item.as_video_stream_profile().fps())
    return depth_profiles


def writeRGBDModes(pipe):
    pipe_for_profiles=pipe.start()
    depth_sensor_profiles = pipe_for_profiles.get_device().first_depth_sensor()
    color_sensor_profiles = pipe_for_profiles.get_device().query_sensors()[1]
    depth_profiles=depth_sensor_profiles.get_stream_profiles()
    color_profiles=color_sensor_profiles.get_stream_profiles()
    pipe.stop()
    print("Camera streams indexes:")
    count=0
    synch_depth_profiles_idx=[]
    synch_color_profiles_idx=[]
    for i,depth_item in enumerate(depth_profiles):
        for j,color_item in enumerate(color_profiles):
            is_in_list=0
            for idx1,idx2 in zip(synch_depth_profiles_idx,synch_color_profiles_idx):
                is_in_list= depth_profiles[idx1].as_video_stream_profile().height()==depth_item.as_video_stream_profile().height()\
                and depth_profiles[idx1].as_video_stream_profile().width()==depth_item.as_video_stream_profile().width()\
                and depth_profiles[idx1].as_video_stream_profile().fps()==depth_item.as_video_stream_profile().fps()
                if is_in_list:
                    break
            if is_in_list==0 and depth_item.stream_name()=="Depth" and (depth_item.as_video_stream_profile().width()==color_item.as_video_stream_profile().width()) \
                    and (depth_item.as_video_stream_profile().height()==color_item.as_video_stream_profile().height())\
                    and (depth_item.as_video_stream_profile().fps()==color_item.as_video_stream_profile().fps()):
                print("Mode ",count,": ",depth_item.as_video_stream_profile().width()," x ", depth_item.as_video_stream_profile().height()," @ ",depth_item.as_video_stream_profile().fps())
                synch_depth_profiles_idx.append(i)
                synch_color_profiles_idx.append(j)
                count+=1
    if count==0:
        print("No matching RGBD video modes available")
        return depth_profiles, color_profiles, synch_depth_profiles_idx, synch_color_profiles_idx, 0
#         raise Exception("No RGBD video modes available")
    return depth_profiles,color_profiles,synch_depth_profiles_idx, synch_color_profiles_idx


def choose_RGB_and_depth_mode(pipe):
    pipe_for_profiles = pipe.start()
    depth_sensor_profiles = pipe_for_profiles.get_device().first_depth_sensor()
    color_sensor_profiles = pipe_for_profiles.get_device().query_sensors()[1]
    depth_profiles = depth_sensor_profiles.get_stream_profiles()
    color_profiles = color_sensor_profiles.get_stream_profiles()
    pipe.stop()
    for i, depth_item in enumerate(depth_profiles):
        print("Mode ", i, ": ", depth_item.as_video_stream_profile().width(), " x ",
              depth_item.as_video_stream_profile().height(), " @ ", depth_item.as_video_stream_profile().fps())
    mode_depth=int(input("Enter the depth mode to choose: "))
    for i, color_item in enumerate(color_profiles):
        print("Mode ", i, ": ", color_item.as_video_stream_profile().width(), " x ",
              color_item.as_video_stream_profile().height(), " @ ", color_item.as_video_stream_profile().fps())
    mode_color = int(input("Enter the depth mode to choose: "))
    return depth_profiles, color_profiles, mode_depth, mode_color
