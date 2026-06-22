--[[
 Control SIYI camera thermal functions
--]]

local PARAM_TABLE_KEY = 82
local PARAM_TABLE_PREFIX = "SIYI_"

local SIYI_IP = "192.168.144.25"
local SIYI_UDP_PORT = 37260

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: SIYI_ENABLE
  // @DisplayName: Enable SIYI control
  // @Description: Enable SIYI control
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
SIYI_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: DEBUG
  // @DisplayName: debug level
  // @Description: debug level
  // @Range: 0 1
  // @User: Standard
--]]
SIYI_DEBUG = bind_add_param("DEBUG", 2, 0)

--[[
  // @Param: ATT_RATE
  // @DisplayName: Attitude update rate
  // @Description: Attitude update rate
  // @Range: 0 50
  // @Units: Hz
  // @User: Standard
--]]
SIYI_ATT_RATE = bind_add_param("ATT_RATE", 3, 20)

--[[
  // @Param: THERM_RATE
  // @DisplayName: thermel update rate
  // @Description: thermal update rate
  // @Range: 0 50
  // @Units: Hz
  // @User: Standard
--]]
SIYI_THERM_RATE = bind_add_param("THERM_RATE", 4, 20)

--[[
  // @Param: TELEM_RATE
  // @DisplayName: telemetry rate
  // @Description: telemetry rate
  // @Range: 0 50
  // @Units: Hz
  // @User: Standard
--]]
SIYI_TELEM_RATE = bind_add_param("TELEM_RATE", 5, 10)

--[[
  // @Param: MAV_CHAN
  // @DisplayName: mavlink channel
  // @Description: mavlink channel
  // @Range: 0 10
  // @User: Standard
--]]
SIYI_MAV_CHAN = bind_add_param("MAV_CHAN", 6, 1)

local SIYI_HEADER1 = 0x55
local SIYI_HEADER2 = 0x66

--[[
   SIYI command codes
--]]
local READ_TEMP_FULL_SCREEN = 0x14
local ATTITUDE_EXTERNAL = 0x22
local REQUEST_CONTINUOUS_DATA = 0x25
local ACQUIRE_GIMBAL_ATTITUDE = 0x0D

local last_therm_send_ms = uint32_t(0)
local last_att_send_ms = uint32_t(0)
local last_att_recv_ms = uint32_t(0)

local last_fwd_source = uint32_t(0)
local last_fwd_port = 0

local send_sequence = 0
local siyi_attitude = nil

if SIYI_ENABLE:get() ~= 1 then
   return
end

--[[
   sock_udp_fwd is used for proxying requests from a GCS to the camera
--]]
local sock_udp_fwd = Socket(1)
if not sock_udp_fwd:bind("0.0.0.0", SIYI_UDP_PORT) then
   gcs:send_text(MAV_SEVERITY.ERROR, "SIYI: Failed to bind")
   return
end

--[[
   sock_udp_out is for our own requests to the camera
--]]
local sock_udp_out = Socket(1)
if not sock_udp_out:connect(SIYI_IP, SIYI_UDP_PORT) then
   gcs:send_text(MAV_SEVERITY.ERROR, "SIYI: Failed to connect")
   return
end

--[[
   CRC-16-CCITT
--]]
local function crc16(bytes)
   local crc = 0

   for i = 1, #bytes do
      local b = string.byte(string.sub(bytes, i, i))
      crc = crc ~ (b << 8)
      for _ = 1, 8 do
         if (crc & 0x8000) ~= 0 then
            crc = ((crc << 1) ~ 0x1021) & 0xFFFF
         else
            crc = (crc << 1) & 0xFFFF
         end
      end
   end
   return crc & 0xFFFF
end

-- Method to send packet
function send_packet(command_id, pkt)
    local plen = pkt and #pkt or 0
    local buf = string.pack("<BBBHHB", SIYI_HEADER1, SIYI_HEADER2, 1, plen, send_sequence, command_id)
    
    if pkt then
        buf = buf .. pkt
    end
    
    buf = buf .. string.pack("<H", crc16(buf))
    
    send_sequence = (send_sequence + 1) % 0xffff
    
    local success, err = pcall(function()
        sock_udp_out:send(buf, #buf)
    end)
    if err and SIYI_DEBUG:get() > 0 then
       gcs:send_text(MAV_SEVERITY.INFO, string.format("send_packet err:%s len=%u", err, #buf))
    end

    return success
end

-- Method to send packet with formatting
function send_packet_fmt(command_id, fmt, ...)
    local args = { ... }
    fmt = fmt or ""

    if SIYI_DEBUG:get() > 1 then
       gcs:send_text(MAV_SEVERITY.INFO, string.format("Sending cmd=0x%02x fmt=%s", command_id, fmt))
    end
    local success, err = pcall(function()
        local packed_pkt = string.pack(fmt, table.unpack(args))
        send_packet(command_id, packed_pkt)
    end)
    if err and SIYI_DEBUG:get() > 0 then
       gcs:send_text(MAV_SEVERITY.INFO, string.format("send_packet_fmt err:%s", err))
    end

    return success
end

-- wrap yaw angle in degrees to value between 0 and 360
local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

-- wrap yaw angle in degrees to value between -180 and +180
local function wrap_180(angle_deg)
  local res = wrap_360(angle_deg)
  if res > 180 then
    res = res - 360
  end
  return res
end

local function handle_gimbal_attitude(data)
   local z,y,x,sz,sy,sx = string.unpack("<hhhhhh", data)
   last_att_recv_ms = millis()
   local roll, pitch, yaw = x*0.1, y*0.1, wrap_180(-z*0.1)
   siyi_attitude = { roll, pitch, yaw }
   gcs:send_named_float("SIYI_PTCH", pitch)
   logger:write('SIAT', 'Y,P,R,Yr,Pr,Rr', 'ffffff',
                yaw, pitch, roll,
                -sz*0.1, sy*0.1, sx*0.1)
end

--[[
   handle full screen temperature
--]]
local function handle_temp_full_screen(data)
   local tmax, tmin, tmax_x, tmax_y, tmin_x, tmin_y = string.unpack("<HHHHHH", data)
   gcs:send_named_float("SIYI_X_TMAX", tmax)
   logger:write("SITM",'TMax,TMin,TMaxX,TMaxY,TMinX,TMinY','ffHHHH',
                tmax*0.01, tmin*0.01,
                tmax_x, tmax_y,
                tmin_x, tmin_y)

   if not siyi_attitude then
      return
   end

   local loc = ahrs:get_location()
   local gpsloc = gps:location(0)
   if not loc or not gpsloc then
      return
   end

   local data2 = string.pack("<iiifffHHffffff",
                             millis():toint(),
                             loc:lat(), loc:lng(), loc:alt()*0.01, gpsloc:alt()*0.01,
                             tmax*0.01, tmax_x, tmax_y,
                             siyi_attitude[1], siyi_attitude[2], siyi_attitude[3],
                             math.deg(ahrs:get_roll()), math.deg(ahrs:get_pitch()), math.deg(ahrs:get_yaw()))
   local data96_hdr = string.pack("<BB", 71, #data2)
   local data96 = data96_hdr .. data2 .. string.rep("\0", 96 - #data2)
   mavlink.send_chan(SIYI_MAV_CHAN:get(), 172, data96 )
end

-- Function to parse a single SIYI packet
local function parse_packet(pkt)
   local _, _, _, _, _, cmd = string.unpack("<BBBHHB", pkt:sub(1, 8))
   local data = pkt:sub(9, -3)
   local crc = string.unpack("<H", pkt:sub(-2))
   local crc2 = crc16(pkt:sub(1, -3))

   if crc ~= crc2 then
      --gcs:send_text(MAV_SEVERITY.INFO, "bad crc")
      return
   end

   if cmd == READ_TEMP_FULL_SCREEN then
      handle_temp_full_screen(data)
   elseif cmd == ACQUIRE_GIMBAL_ATTITUDE then
      handle_gimbal_attitude(data)
   end
end

-- Function to parse SIYI packet data
local function parse_data(pkt)
   while #pkt >= 10 do
      local h1, h2, _, plen, _, _ = string.unpack("<BBBHHB", pkt:sub(1, 8))
      if h1 ~= SIYI_HEADER1 or h2 ~= SIYI_HEADER2 then
         --gcs:send_text(MAV_SEVERITY.INFO, "bad header")
         break
      end
      if plen + 10 > #pkt then
         --gcs:send_text(MAV_SEVERITY.INFO, "bad len")
         break
      end
      parse_packet(pkt:sub(1, plen + 10))
      pkt = pkt:sub(plen + 11)
   end
end

--[[
   request full frame thermel data
--]]
local function thermal_request()
   send_packet_fmt(READ_TEMP_FULL_SCREEN, "<B", 2)
end

--[[
   send vehicle attitude to SIYI to aid in attitude estimation
--]]
local function send_attitude()
   local roll_rad = ahrs:get_roll()
   local pitch_rad = ahrs:get_pitch()
   local yaw_rad = ahrs:get_yaw()
   local gyro_rad = ahrs:get_gyro()
   send_packet_fmt(ATTITUDE_EXTERNAL, "<iffffff",
                   millis():toint(),
                   roll_rad, pitch_rad, yaw_rad,
                   gyro_rad:x(), gyro_rad:y(), gyro_rad:z())
   --gcs:send_text(MAV_SEVERITY.INFO, "send att")
end

--[[
   check for input packets (replies) or packets from a GCS
--]]
local function check_input()
   while true do
      local pkt, ip, port = sock_udp_fwd:recv(1024)
      if not pkt then
         break
      end
      if ip then
         last_fwd_source = ip
         last_fwd_port = port
      end
      -- forward packet to the camera from a GCS
      sock_udp_out:send(pkt, #pkt)
   end

   while true do
      local pkt = sock_udp_out:recv(1024)
      if not pkt then
         break
      end
      --[[
         forward all replies to the GCS, for logging and to allow GCS control
      --]]
      if last_fwd_port ~= 0 then
         sock_udp_fwd:sendto(pkt, #pkt, last_fwd_source, last_fwd_port)
      end
      parse_data(pkt)
   end
end

--[[
   main update function
--]]
local function update()
   local now_ms = millis()

   check_input()

   local therm_rate = SIYI_THERM_RATE:get()
   if therm_rate > 0 then
      local therm_period_ms = 1000.0 / therm_rate
      if now_ms - last_therm_send_ms >= therm_period_ms then
         last_therm_send_ms = now_ms
         thermal_request()
      end
   end

   local att_rate = SIYI_ATT_RATE:get()
   if att_rate > 0 then
      local att_period_ms = 1000.0 / att_rate
      if now_ms - last_att_send_ms >= att_period_ms then
         last_att_send_ms = now_ms
         send_attitude()
      end
   end

   now_ms = millis()
   if SIYI_TELEM_RATE:get() > 0 and now_ms - last_att_recv_ms > 1000 then
      gcs:send_text(MAV_SEVERITY.INFO, "SIYI request telem")
      send_packet_fmt(REQUEST_CONTINUOUS_DATA, "<BB", 1, SIYI_TELEM_RATE:get());
      last_att_recv_ms = millis()
   end
   
   return update, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "SIYI_control: loaded")

return update()
