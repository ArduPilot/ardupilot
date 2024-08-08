 --[[
    Populate the fields of the CAMERA_INFORMATION message sent by the selected camera instance.
 --]]
 function set_camera_information()
    -- set the Camera Information data
    local cam_info = mavlink_camera_information_t()

    local INSTANCE = 0
    local vendor_name = 'Unknown'
    local model_name = 'Camera'
    local uri = ''

    -- "time_boot_ms" is populated automatically by the camera backend
    for i = 0, #vendor_name do
        cam_info:vendor_name(i, vendor_name:byte(i+1))
    end
    for i = 0, #model_name do
        cam_info:model_name(i, model_name:byte(i+1))
    end
    cam_info:firmware_version(0)
    cam_info:focal_length(1.6)
    cam_info:sensor_size_h(3840)
    cam_info:sensor_size_v(2160)
    cam_info:resolution_h(1920)
    cam_info:resolution_v(1080)
    -- "lens_id" is populated automatically by the camera backend
    cam_info:flags(256) -- CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM
    cam_info:cam_definition_version(0)
    for i = 0, #uri do
        cam_info:cam_definition_uri(i, uri:byte(i+1))
    end
    -- "gimbal_device_id" is populated automatically by the camera backend

    camera:set_camera_information(INSTANCE, cam_info)

end

return set_camera_information()
