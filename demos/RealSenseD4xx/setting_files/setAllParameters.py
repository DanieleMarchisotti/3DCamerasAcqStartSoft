import pyrealsense2 as rs


def set_parameters(depth_sensor):
    meas_unit=depth_sensor.get_option_range(rs.option.depth_units)
    if meas_unit.min!=meas_unit.max:
        print("Depth units range (mm per unit) = ",meas_unit.min , "~", meas_unit.max)
        meas_unit_selected=float(input("Select the number of meters per unit: "))
        depth_sensor.set_option(rs.option.depth_units,meas_unit_selected)
    preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
    for i in range(int(preset_range.max)):
        visulpreset = depth_sensor.get_option_value_description(rs.option.visual_preset,i)
        print('%02d: %s'%(i,visulpreset))
    preset = input("Enter the preset to choose (es: Default): ")
    for i in range(int(preset_range.max)):
        visulpreset = depth_sensor.get_option_value_description(rs.option.visual_preset, i)
        if visulpreset == preset:
            depth_sensor.set_option(rs.option.visual_preset, i)
    try:
        depth_sensor.get_option(rs.option.laser_power)
        depth_sensor.set_option(rs.option.emitter_enabled, True)
        enable_emitter = input("Enable laser emitter (Y / N)? ")
        if enable_emitter=="Y":
            depth_sensor.set_option(rs.option.emitter_enabled,True)
        else:
            depth_sensor.set_option(rs.option.emitter_enabled, False)
    except RuntimeError:
        if depth_sensor.get_option(rs.option.laser_power)==1:
            print("Laser enabled")
    try:
        laser_range = depth_sensor.get_option_range(rs.option.laser_power)
        laser_pwr = depth_sensor.get_option(rs.option.laser_power)
        print("laser power range = ", laser_range.min, "~", laser_range.max, " (Default: ", laser_pwr, ")")
        laser_pwr_new = int(input("Enter laser power: "))
        if laser_pwr_new > laser_range.max:
            laser_pwr_new = laser_range.max
        elif laser_pwr_new < laser_range.min:
            laser_pwr_new = laser_range.min
        depth_sensor.set_option(rs.option.laser_power, laser_pwr_new)
    except RuntimeError:
        print("Laser emitter power cannot be set")
    try:
        depth_sensor.get_option(rs.option.enable_auto_exposure)
        enable_autoexp = input("Enable auto-exposure (Y / N)? ")
        if enable_autoexp=="Y":
            depth_sensor.set_option(rs.option.enable_auto_exposure,True)
        else:
            exposure_range=depth_sensor.get_option_range(rs.option.exposure)
            exp=depth_sensor.get_option(rs.option.exposure)
            print("Camera Exposure range = ",exposure_range.min, "~", exposure_range.max," (Default: ",exp,")")
            exp_value = int(input("Enter exposure value: "))
            if exp_value>exposure_range.max:
                exp_value=exposure_range.max
            elif exp_value<exposure_range.min:
                exp_value=exposure_range.min
            depth_sensor.set_option(rs.option.exposure,exp_value)
    except RuntimeError:
        print("Exposure cannot be set")
    try:
        gain_range = depth_sensor.get_option_range(rs.option.gain)
        set_gain=input("Set the Gain (Y / N): ")
        if set_gain=="Y":
            gain=depth_sensor.get_option(rs.option.gain)
            print("Gain range: ",gain_range.min, "~", gain_range.max," (Default: ",gain,")")
            new_gain=int(input("Enter the depth gain: "))
            depth_sensor.set_option(rs.option.gain,new_gain)
    except RuntimeError:
        print("Depth gain cannot be set")
    try:
        accuracy=depth_sensor.get_option(rs.option.accuracy)
        accuracy_range=depth_sensor.get_option_range(rs.option.accuracy)
        print("Accuracy range (number of patterns projected): ", accuracy_range.min, "~", accuracy_range.max, " (Default: ", accuracy, ")")
        new_accuracy = int(input("Enter the accuracy: "))
        depth_sensor.set_option(rs.option.accuracy, new_accuracy)
    except RuntimeError:
        print("Depth accuracy (number of patterns projected) cannot be set")
    try:
        motion=depth_sensor.get_option(rs.option.motion_range)
        motion_range=depth_sensor.get_option_range(rs.option.motion_range)
        print("Motion range (high values --> better depth range, low values --> high motion sensitivity): ", motion_range.min, "~", motion_range.max, " (Default: ", motion, ")")
        new_motion = int(input("Enter the motion range: "))
        depth_sensor.set_option(rs.option.motion_range, new_motion)
    except RuntimeError:
        print("Motion range cannot be set")
    try:
        filter_option=depth_sensor.get_option(rs.option.filter_option)
        filter_option_range=depth_sensor.get_option_range(rs.option.filter_option)
        print("Filter option: set the filter to apply for each depth frame. Each filter is optimized for the application requirements")
        print("Filter option: ",filter_option_range.min, "~", filter_option_range.max, " (Default: ", filter_option, ")")
        new_motion = int(input("Enter the filter option: "))
        depth_sensor.set_option(rs.option.filter_option, new_motion)
    except RuntimeError:
        print("Filter option cannot be set")
    set_post_process=input("Enable post-process (Y / N)?")
    dec_filter=rs.decimation_filter()
    spatial_filter=rs.spatial_filter()
    temp_filter=rs.temporal_filter()
    if set_post_process=="Y":
        enable_dec_filter = input("Enable decimation filter (Y / N)?")
        if enable_dec_filter =="Y":
            set_dec_params= input("Set decimation filter parameters (Y / N)?")
            if set_dec_params=="Y":
                dec_scale_range=dec_filter.get_option_range(rs.option.filter_magnitude)
                dec_scale=dec_filter.get_option(rs.option.filter_magnitude)
                print("Decimation filter scale range: ",dec_scale_range.min, "~", dec_scale_range.max," (Default: ",dec_scale,")")
                dec_value=int(input("Enter decimation scale: "))
                dec_filter.set_option(rs.option.filter_magnitude,dec_value)
        enable_spatial_filter = input("Enable spacial filter (Y / N)?")
        if enable_spatial_filter == "Y":
            set_spatial_params = input("Set spacial filter parameters (Y / N)?")
            if set_spatial_params == "Y":
                spat_alpha_range=spatial_filter.get_option_range(rs.option.filter_smooth_alpha)
                spat_alpha=spatial_filter.get_option(rs.option.filter_smooth_alpha)
                print("Spatial alpha range: ",spat_alpha_range.min, "~", spat_alpha_range.max," (Default: ",spat_alpha,")")
                spat_alpha_value = float(input("Enter spatial alpha: "))
                spatial_filter.set_option(rs.option.filter_smooth_alpha, spat_alpha_value)
                spat_delta_range = spatial_filter.get_option_range(rs.option.filter_smooth_delta)
                spat_delta = spatial_filter.get_option(rs.option.filter_smooth_delta)
                print("Spatial delta range: ", spat_delta_range.min, "~", spat_delta_range.max, " (Default: ", spat_delta,
                      ")")
                spat_delta_value = float(input("Enter spatial alpha: "))
                spatial_filter.set_option(rs.option.filter_smooth_delta, spat_delta_value)
                spat_iter_range = spatial_filter.get_option_range(rs.option.filter_magnitude)
                spat_iter = spatial_filter.get_option(rs.option.filter_magnitude)
                print("Spatial iteration range: ", spat_iter_range.min, "~", spat_iter_range.max, " (Default: ", spat_iter,
                      ")")
                spat_iter_value = int(input("Enter spatial iteration: "))
                spatial_filter.set_option(rs.option.filter_magnitude, spat_iter_value)
                spat_holes_range = spatial_filter.get_option_range(rs.option.holes_fill)
                spat_holes = spatial_filter.get_option(rs.option.holes_fill)
                print("Spatial holes fill dimension range: ", spat_holes_range.min, "~", spat_holes_range.max, " (Default: ", spat_holes,
                      ")")
                spat_holes_value = int(input("Enter holes fill dimension: "))
                spatial_filter.set_option(rs.option.holes_fill, spat_holes_value)
        enable_temp_filter = input("Enable temporal filter (Y / N)?")
        if enable_temp_filter == "Y":
            set_temp_params = input("Set temporal filter parameters (Y / N)?")
            if set_temp_params == "Y":
                temp_alpha_range = temp_filter.get_option_range(rs.option.filter_smooth_alpha)
                temp_alpha =temp_filter.get_option(rs.option.filter_smooth_alpha)
                print("Temporal alpha range: ", temp_alpha_range.min, "~", temp_alpha_range.max,
                      " (Default: ", temp_alpha,")")
                temp_alpha_value = float(input("Enter temporal alpha: "))
                temp_filter.set_option(rs.option.filter_smooth_alpha, temp_alpha_value)
                temp_delta_range = temp_filter.get_option_range(rs.option.filter_smooth_delta)
                temp_delta = temp_filter.get_option(rs.option.filter_smooth_delta)
                print("Temporal delta range: ", temp_delta_range.min, "~", temp_delta_range.max,
                      " (Default: ", temp_delta, ")")
                temp_delta_value = float(input("Enter temporal delta: "))
                temp_filter.set_option(rs.option.filter_smooth_delta, temp_delta_value)
                temp_holes_range = temp_filter.get_option_range(rs.option.holes_fill)
                temp_holes = temp_filter.get_option(rs.option.holes_fill)
                print("Temporal persistence range: ", temp_holes_range.min, "~", temp_holes_range.max,
                      " (Default: ", temp_holes, ")")
                temp_holes_value = int(input("Enter temporal persistence: "))
                temp_filter.set_option(rs.option.holes_fill, temp_holes_value)
    else:
        enable_dec_filter="N"
        enable_spatial_filter="N"
        enable_temp_filter="N"
    postProcessStrs=[set_post_process,enable_dec_filter,enable_spatial_filter,enable_temp_filter]
    # motion = depth_sensor.get_option(rs.option.motion_range)
    # print(motion)
    return postProcessStrs,dec_filter,spatial_filter,temp_filter,depth_sensor, meas_unit_selected