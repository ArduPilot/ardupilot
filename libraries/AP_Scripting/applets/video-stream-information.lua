 --[[
    Populate the VIDEO_STREAM_INFORMATION message based on user parameters
 --]]

local PARAM_TABLE_KEY = 90
local PARAM_TABLE_PREFIX = "VID1_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local VID_TYPE_ENUM = {RTSP=0, RTPUDP=1, TCP_MPEG=2, MPEG_TS=3}
local CAMMODEL_ENUM = {UNKNOWN=0, SIYI_A8=1, SIYI_ZR10=2, SIYI_ZR30=3, SIYI_ZT30_ZOOM=4, SIYI_ZT30_WIDE=5,
                       SIYI_ZT30_IR=6, SIYI_ZT6_RGB=7, SIYI_ZT6_IR=8, HERELINK_WIFIAP=9, HERELINK_USB_TETHERING=10,
                       TOPOTEK_1080P=11, TOPOTEK_480P=12, VIEWPRO=13}
local TEXT_PREFIX_STR = "video-stream-information:"
local SIYI_IP_DEFAULT = '192.168.144.25:8554'
local TOPOTEK_IP_DEFAULT = '192.168.144.108:554'
local VIEWPRO_IP_DEFAULT = '192.168.2.119:554'

 -- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: VID1_CAMMODEL
  // @DisplayName: Camera1 Video Stream Camera Model
  // @Description: Video stream camera model
  // @Values: 0:Unknown, 1:Siyi A8, 2:Siyi ZR10, 3:Siyi ZR30, 4:Siyi ZT30 Zoom, 5:Siyi ZT30 Wide, 6:Siyi ZT30 IR, 7:Siyi ZT6 RGB, 8:Siyi ZT6 IR, 9:Herelink WifiAP, 10:Herelink USB-tethering, 11:Topotek 1080p, 12:Topotek 480p, 13:Viewpro
  // @User: Standard
--]]
-- values should match CAMMODEL_ENUM
local VID1_CAMMODEL = bind_add_param('CAMMODEL', 1, 0)

--[[
  // @Param: VID1_ID
  // @DisplayName: Camera1 Video Stream Id
  // @Description: Video stream id
  // @Range: 0 50
  // @User: Standard
--]]
local VID1_ID = bind_add_param('ID', 2, 1)

--[[
  // @Param: VID1_TYPE
  // @DisplayName: Camera1 Video Stream Type
  // @Description: Video stream type
  // @Values: 0:RTSP, 1:RTPUDP, 2:TCP_MPEG, 3:MPEG_TS
  // @User: Standard
--]]
-- values should match VID_TYPE_ENUM
local VID1_TYPE = bind_add_param('TYPE', 3, 0)

--[[
  // @Param: VID1_FLAG
  // @DisplayName: Camera1 Video Stream Flags
  // @Description: Video stream flags
  // @Bitmask: 0:Running,1:Thermal,2:Thermal Range Enabled
  // @User: Standard
--]]
local VID1_FLAG = bind_add_param('FLAG', 4, 1)

--[[
  // @Param: VID1_FRAME_RATE
  // @DisplayName: Camera1 Video Stream Frame Rate
  // @Description: Video stream frame rate
  // @Range: 0 50
  // @User: Standard
--]]
local VID1_FR = bind_add_param('FRAME_RATE', 5, 30)

--[[
  // @Param: VID1_HRES
  // @DisplayName: Camera1 Video Stream Horizontal Resolution
  // @Description: Video stream horizontal resolution
  // @Range: 0 4096
  // @User: Standard
--]]
local VID1_HRES = bind_add_param('HRES', 6, 1920)

--[[
  // @Param: VID1_VRES
  // @DisplayName: Camera1 Video Stream Vertical Resolution
  // @Description: Video stream vertical resolution
  // @Range: 0 4096
  // @User: Standard
--]]
local VID1_VRES = bind_add_param('VRES', 7, 1080)

 --[[
  // @Param: VID1_BITRATE
  // @DisplayName: Camera1 Video Stream Bitrate
  // @Description: Video stream bitrate
  // @Range: 0 10000
  // @User: Standard
--]]
local VID1_BITR = bind_add_param('BITRATE', 8, 1500)

--[[
  // @Param: VID1_HFOV
  // @DisplayName: Camera1 Video Stream Horizontal FOV
  // @Description: Video stream horizontal FOV
  // @Range: 0 360
  // @User: Standard
--]]
local VID1_HFOV = bind_add_param('HFOV', 9, 0)

--[[
  // @Param: VID1_ENCODING
  // @DisplayName: Camera1 Video Stream Encoding
  // @Description: Video stream encoding
  // @Values: 0:Unknown, 1:H264, 2:H265
  // @User: Standard
--]]
local VID1_ENCODING = bind_add_param('ENCODING', 10, 1)

--[[
  // @Param: VID1_IPADDR0
  // @DisplayName: Camera1 Video Stream IP Address 0
  // @Description: Video stream IP Address first octet
  // @Range: 0 255
  // @User: Standard
--]]
local VID1_IPADDR0 = bind_add_param('IPADDR0', 11, -1)

--[[
  // @Param: VID1_IPADDR1
  // @DisplayName: Camera1 Video Stream IP Address 1
  // @Description: Video stream IP Address second octet
  // @Range: 0 255
  // @User: Standard
--]]
local VID1_IPADDR1 = bind_add_param('IPADDR1', 12, -1)

--[[
  // @Param: VID1_IPADDR2
  // @DisplayName: Camera1 Video Stream IP Address 2
  // @Description: Video stream IP Address third octet
  // @Range: 0 255
  // @User: Standard
--]]
local VID1_IPADDR2 = bind_add_param('IPADDR2', 13, -1)

--[[
  // @Param: VID1_IPADDR3
  // @DisplayName: Camera1 Video Stream IP Address 3
  // @Description: Video stream IP Address fourth octet
  // @Range: 0 255
  // @User: Standard
--]]
local VID1_IPADDR3 = bind_add_param('IPADDR3', 14, -1)

--[[
  // @Param: VID1_IPPORT
  // @DisplayName: Camera1 Video Stream IP Address Port
  // @Description: Video stream IP Address Port
  // @Range: 0 65535
  // @User: Standard
--]]
local VID1_IPPORT = bind_add_param('IPPORT', 15, -1)

function set_video_stream_information()
    local INSTANCE = 0
    local name = 'Video'

    -- set defaults by camera model
    local uri_ip = ''
    local uri_suffix = ''
    local hfov = 50
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_A8 then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/main.264'
        hfov = 81
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZR10 then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/main.264'
        hfov = 62
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZR30 then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/main.264'
        hfov = 58
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZT30_ZOOM then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/video2'
        hfov = 58
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZT30_WIDE then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/video2'
        hfov = 85
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZT30_IR then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/video1'
        hfov = 24
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZT6_RGB then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/video2'
        hfov = 85
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.SIYI_ZT6_IR then
        uri_ip = SIYI_IP_DEFAULT
        uri_suffix = '/video1'
        hfov = 32
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.HERELINK_WIFIAP then
        uri_ip = '192.168.42.1:8554'
        uri_suffix = '/fpv_stream'
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.HERELINK_USB_TETHERING then
        uri_ip = '192.168.42.129:8554'
        uri_suffix = '/fpv_stream'
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.TOPOTEK_1080P then
        uri_ip = TOPOTEK_IP_DEFAULT
        uri_suffix = '/stream=0'
        hfov = 50
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.TOPOTEK_480P then
        uri_ip = TOPOTEK_IP_DEFAULT
        uri_suffix = '/stream=1'
        hfov = 50
    end
    if VID1_CAMMODEL:get() == CAMMODEL_ENUM.VIEWPRO then
        uri_ip = VIEWPRO_IP_DEFAULT
    end

    -- calculate hfov
    if VID1_HFOV:get() ~= 0 then
        hfov = VID1_HFOV:get()
    end

    -- construct uri
    if VID1_IPADDR0:get() > 0 or VID1_IPADDR1:get() > 0 or
       VID1_IPADDR2:get() > 0 or VID1_IPADDR3:get() > 0 or
       VID1_IPPORT:get() > 0 then
        uri_ip = math.floor(VID1_IPADDR0:get()) .. '.' ..
                 math.floor(VID1_IPADDR1:get())  .. '.' ..
                 math.floor(VID1_IPADDR2:get())  .. '.' ..
                 math.floor(VID1_IPADDR3:get())  .. ':' ..
                 math.floor(VID1_IPPORT:get())
    end
    local uri
    if VID1_TYPE:get() == VID_TYPE_ENUM.RTPUDP or VID1_TYPE:get() == VID_TYPE_ENUM.MPEG_TS then
        -- sanity check port number
        if VID1_IPPORT:get() < 0 then
          gcs:send_text(MAV_SEVERITY.ERROR, TEXT_PREFIX_STR .. "check VID1_IPPORT")
          do return end
        end
        uri = tostring(math.floor(VID1_IPPORT:get()))
    elseif VID1_TYPE:get() == VID_TYPE_ENUM.RTSP then
        -- sanity check IP address
        if uri_ip == '' then
          gcs:send_text(MAV_SEVERITY.ERROR, TEXT_PREFIX_STR .. "check VID1_IPADDR params")
          do return end
        end
        uri = 'rtsp://' .. uri_ip .. uri_suffix
    else
        uri = uri_ip .. uri_suffix
    end

    -- create Video Stream Information message
    local stream_info = mavlink_video_stream_information_t()
    stream_info:stream_id(VID1_ID:get())
    stream_info:count(1)    -- hard coded to just a single stream
    stream_info:type(VID1_TYPE:get())
    stream_info:flags(VID1_FLAG:get())
    stream_info:framerate(VID1_FR:get())
    stream_info:resolution_h(VID1_HRES:get())
    stream_info:resolution_v(VID1_VRES:get())
    stream_info:bitrate(VID1_BITR:get())
    stream_info:rotation(0) -- video image rotation clockwise, hardcoded to zero
    stream_info:hfov(hfov)
    stream_info:encoding(VID1_ENCODING:get())

    for i = 0, #name do
        stream_info:name(i, name:byte(i+1))
    end
    for i = 0, #uri do
        stream_info:uri(i, uri:byte(i+1))
    end

    -- update camera library with the latest stream information
    camera:set_stream_information(INSTANCE, stream_info)
end

-- print welcome message
gcs:send_text(MAV_SEVERITY.INFO, "video-stream-information script loaded")

-- update function runs every 5 secs
function update()
  set_video_stream_information()
  return update, 5000
end

return update()
