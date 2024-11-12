 --[[
    Populate the fields of the VIDEO_STREAM_INFORMATION message sent by the selected camera instance.
 --]]
 function set_video_stream_information()
    -- set the Video Stream Information data
    local stream_info = mavlink_video_stream_information_t()

    local INSTANCE = 0
    local name = 'Video'
    local uri = '127.0.0.1:5600'

    stream_info:framerate(30)
    stream_info:bitrate(1500)
    stream_info:flags(1) -- VIDEO_STREAM_STATUS_FLAGS_RUNNING
    stream_info:resolution_h(1920)
    stream_info:resolution_v(1080)
    stream_info:rotation(0)
    stream_info:hfov(50)
    stream_info:stream_id(1)
    stream_info:count(1)
    stream_info:type(1) -- VIDEO_STREAM_TYPE_RTPUDP
    for i = 0, #name do
        stream_info:name(i, name:byte(i+1))
    end
    for i = 0, #uri do
        stream_info:uri(i, uri:byte(i+1))
    end

    camera:set_stream_information(INSTANCE, stream_info)

end

return set_video_stream_information()
