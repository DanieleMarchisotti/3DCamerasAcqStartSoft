import pyrealsense2 as rs


def to_file(profile, profile_color, meas_unit_final,depth_sensor,postProcessStrs,folderPathToSave,filters):
    stringToWrite="Mode "
    stringToWrite+="Depth mode: " + str(profile.as_video_stream_profile().width())+" x " + \
                   str(profile.as_video_stream_profile().height())+" @ "+str(profile.as_video_stream_profile().fps())+"\n"
    stringToWrite += "Color mode: " + str(profile_color.as_video_stream_profile().width()) + " x " + str(
        profile_color.as_video_stream_profile().height()) + " @ " + str(profile_color.as_video_stream_profile().fps()) + "\n"
    stringToWrite+="Depth units: "+str(meas_unit_final)+"\n"
    preset_final=depth_sensor.get_option(rs.option.visual_preset)
    stringToWrite+="Preset: "+depth_sensor.get_option_value_description(rs.option.visual_preset,preset_final)+"\n"
    try:
        stringToWrite+="Accuracy: "+str(depth_sensor.get_option(rs.option.accuracy))+"\n"
    except RuntimeError:
        pass
    try:
        stringToWrite += "Motion range: " + str(depth_sensor.get_option(rs.option.motion_range)) + "\n"
    except RuntimeError:
        pass
    try:
        stringToWrite += "Filter option: " + str(depth_sensor.get_option(rs.option.filter_option)) + "\n"
    except RuntimeError:
        pass
    try:
        stringToWrite += "Confidence threshold: " + str(depth_sensor.get_option(rs.option.confidence_threshold)) + "\n"
    except RuntimeError:
        pass
    try:
        stringToWrite += "Max Distance: " + str(depth_sensor.get_option(rs.option.max_distance)) + "\n"
    except RuntimeError:
        pass
    try:
        stringToWrite += "Min Distance: " + str(depth_sensor.get_option(rs.option.min_distance)) + "\n"
    except RuntimeError:
        pass
    try:
        stringToWrite+="Enabled emitter: "+str(depth_sensor.get_option(rs.option.emitter_enabled))+"\n"
    except RuntimeError:
        print("")
    try:
        stringToWrite+="Laser power: "+str(depth_sensor.get_option(rs.option.laser_power))+"\n"
    except RuntimeError:
        print("")
    try:
        stringToWrite+="Autoexposure: "+str(depth_sensor.get_option(rs.option.enable_auto_exposure))+"\n"
    except RuntimeError:
        print("")
    try:
        stringToWrite+="Exposure: "+str(depth_sensor.get_option(rs.option.exposure))+"\n"
    except RuntimeError:
        print("")
    try:
        stringToWrite+="Gain: "+str(depth_sensor.get_option(rs.option.gain))+"\n"
    except RuntimeError:
        print("")
    if postProcessStrs[0]=="Y":
        stringToWrite+="Post-process enabled: "+str(1)+"\n"
        if postProcessStrs[1]=="Y":
            stringToWrite+="Decimation filter enabled: "+str(1)+"\n"
        else:
            stringToWrite += "Decimation filter enabled: " + str(0)+"\n"
        if postProcessStrs[2]=="Y":
            stringToWrite+="Spacial filter enabled: "+str(1)+"\n"
        else:
            stringToWrite+="Spacial filter enabled: "+str(0)+"\n"
        if postProcessStrs[3] == "Y":
            stringToWrite += "Temporal filter enabled: " + str(1)+"\n"
        else:
            stringToWrite += "Temporal filter enabled: " + str(0)+"\n"
    else:
        stringToWrite += "Post-process enabled: " + str(0)+"\n"
        stringToWrite += "Decimation filter enabled: " + str(0) + "\n"
        stringToWrite += "Spacial filter enabled: " + str(0) + "\n"
        stringToWrite += "Temporal filter enabled: " + str(0) + "\n"
    stringToWrite+="Decimation scale: "+str(filters[0].get_option(rs.option.filter_magnitude))+"\n"
    stringToWrite+="Spacial alpha: "+str(filters[1].get_option(rs.option.filter_smooth_alpha))+"\n"
    stringToWrite+="Spacial delta: "+str(filters[1].get_option(rs.option.filter_smooth_delta))+"\n"
    stringToWrite+="Spatial iteration: "+str(filters[1].get_option(rs.option.filter_magnitude))+"\n"
    stringToWrite+="Holes fill dimension: "+str(filters[1].get_option(rs.option.holes_fill))+"\n"
    stringToWrite+="Temporal alpha: "+str(filters[2].get_option(rs.option.filter_smooth_alpha))+"\n"
    stringToWrite+="Temporal delta: "+str(filters[2].get_option(rs.option.filter_smooth_delta))+"\n"
    stringToWrite+="Temporal persistence: "+str(filters[2].get_option(rs.option.holes_fill))+"\n"
    with open(folderPathToSave+"\\acqParameters.txt",mode="w") as file:
        file.write(stringToWrite)
    return stringToWrite