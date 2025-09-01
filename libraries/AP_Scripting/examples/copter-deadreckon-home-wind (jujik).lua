-- create and initialise parameters
local PARAM_TABLE_KEY = 86  -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "DR_", 10), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add DR_ENABLE param')   -- 1 = enabled, 0 = disabled
assert(param:add_param(PARAM_TABLE_KEY, 2, 'ENAB_DIST', 50), 'could not add DR_ENAB_DIST param')   -- distance from home (in meters) beyond which the dead reckoning will be enabled
assert(param:add_param(PARAM_TABLE_KEY, 3, 'ENAB_TIMEOUT', 5000), 'could not add DR_ENAB_TIMEOUT param')   -- timeout ms after which the dead reckoning will be enabled
assert(param:add_param(PARAM_TABLE_KEY, 4, 'GPS_SACC_MAX', 0.8), 'could not add DR_GPS_SACC_MAX param') -- GPS speed accuracy max threshold
assert(param:add_param(PARAM_TABLE_KEY, 5, 'GPS_SAT_MIN', 6), 'could not add DR_GPS_SAT_MIN param')  -- GPS satellite count min threshold

assert(param:add_param(PARAM_TABLE_KEY, 6, 'FLY_ANGLE', 15), 'could not add DR_FLY_ANGLE param')                 -- lean angle (in degrees) during deadreckoning
assert(param:add_param(PARAM_TABLE_KEY, 7, 'FLY_ALT_MIN', 0), 'could not add DR_FLY_ALT_MIN param')              -- min alt above home (in meters) during deadreckoning. zero to return at current alt
assert(param:add_param(PARAM_TABLE_KEY, 8, 'FLY_TIMEOUT', 30), 'could not add DR_FLY_TIMEOUT param')             -- deadreckoning timeout (in seconds)
assert(param:add_param(PARAM_TABLE_KEY, 9, 'NEXT_MODE', 6), 'could not add DR_NEXT_MODE param')                  -- mode to switch to after GPS recovers or timeout elapses
assert(param:add_param(PARAM_TABLE_KEY, 10, 'LAST_GPS', 60), 'could not add DR_LAST_GPS param')   -- time without gps to use last known gps position for home yaw 

--[[
  // @Param: DR_ENABLE
  // @DisplayName: Deadreckoning Enable
  // @Description: Deadreckoning Enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local enable = Parameter("DR_ENABLE")                  -- 1 = enabled, 0 = disabled

--[[
  // @Param: DR_ENAB_DIST
  // @DisplayName: Deadreckoning Enable Distance
  // @Description: Distance from home (in meters) beyond which the dead reckoning will be enabled
  // @Units: m
  // @User: Standard
--]]
local enable_dist = Parameter("DR_ENAB_DIST")        -- distance from home (in meters) beyond which the dead reckoning will be enabled

--[[
  // @Param: DR_ENAB_TIMEOUT
  // @DisplayName: Deadreckoning Enable Timeout
  // @Description: Timeout (in ms) after which the dead reckoning will be enabled
  // @Units: ms
  // @User: Standard
--]]
local enable_timeout = Parameter("DR_ENAB_TIMEOUT")  -- timeout ms after which the dead reckoning will be enabled

--[[
  // @Param: DR_GPS_SACC_MAX
  // @DisplayName: Deadreckoning GPS speed accuracy maximum threshold
  // @Description: GPS speed accuracy maximum, above which deadreckoning home will begin (default is 0.8).  Lower values trigger with good GPS quality, higher values will allow poorer GPS before triggering. Set to 0 to disable use of GPS speed accuracy
  // @Range: 0 10
  // @User: Standard
--]]
local gps_speed_acc_max = Parameter("DR_GPS_SACC_MAX") -- GPS speed accuracy max threshold

--[[
  // @Param: DR_GPS_SAT_MIN
  // @DisplayName: Deadreckoning GPS satellite count min threshold
  // @Description: GPS satellite count threshold below which deadreckoning home will begin (default is 6).  Higher values trigger with good GPS quality, Lower values trigger with worse GPS quality. Set to 0 to disable use of GPS satellite count
  // @Range: 0 30
  // @User: Standard
--]]
local gps_sat_count_min = Parameter("DR_GPS_SAT_MIN")  -- GPS satellite count min threshold

--[[
  // @Param: DR_FLY_ANGLE
  // @DisplayName: Deadreckoning Lean Angle
  // @Description: lean angle (in degrees) during deadreckoning
  // @Units: deg
  // @Range: 0 45
  // @User: Standard
--]]
local fly_angle = Parameter("DR_FLY_ANGLE")            -- lean angle (in degrees) during deadreckoning

--[[
  // @Param: DR_FLY_ALT_MIN
  // @DisplayName: Deadreckoning Altitude Min
  // @Description: Copter will fly at at least this altitude (in meters) above home during deadreckoning
  // @Units: m
  // @Range: 0 1000
  // @User: Standard
--]]
-- TODO: add climb
local fly_alt_min = Parameter("DR_FLY_ALT_MIN")        -- min alt above home (in meters) during deadreckoning

--[[
  // @Param: DR_FLY_TIMEOUT
  // @DisplayName: Deadreckoning flight timeout
  // @Description: Copter will attempt to switch to NEXT_MODE after this many seconds of deadreckoning.  If it cannot switch modes it will continue in Guided_NoGPS.  Set to 0 to disable timeout
  // @Units: s
  // @User: Standard
--]]
local fly_timeoout = Parameter("DR_FLY_TIMEOUT")       -- deadreckoning timeout (in seconds)

--[[
  // @Param: DR_NEXT_MODE
  // @DisplayName: Deadreckoning Next Mode
  // @Description: Copter switch to this mode after GPS recovers or DR_FLY_TIMEOUT has elapsed.  Default is 6/RTL.  Set to -1 to return to mode used before deadreckoning was triggered
  // @Values: 2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,16:PosHold,17:Brake,20:Guided_NoGPS,21:Smart_RTL,27:Auto RTL
  // @User: Standard
--]]
local next_mode = Parameter("DR_NEXT_MODE")            -- mode to switch to after GPS recovers or timeout elapses

--[[
  // @Param: DR_LAST_GPS
  // @DisplayName: Deadreckoning Bad GPS Timeout
  // @Description: If last GPS was seen more than DR_LAST_GPS seconds ago Copter will use mission points to detect home target 
  // @Units: s
  // @User: Standard
--]]
local last_gps_timeout = Parameter("DR_LAST_GPS")            -- mode to switch to after GPS recovers or timeout elapses

-- local wpnav_speedup = Parameter("WPNAV_SPEED_UP")      -- maximum climb rate from WPNAV_SPEED_UP
-- local wpnav_accel_z = Parameter("WPNAV_ACCEL_Z")       -- maximum vertical acceleration from WPNAV_ACCEL_Z

local copter_guided_nogps_mode = 20 -- Guided_NoGPS is mode 20 on Copter
local copter_RTL_mode = 6           -- RTL is mode 6 on Copter
local recovery_delay_ms = 3000      -- switch to NEXT_MODE happens this many milliseconds after GPS and EKF failsafe recover

local gps_bad = false               -- true if GPS is failing checks
local rc_bad = false               -- true if RC Failsafe
local gps_and_rc_bad = true         -- true if GPS and/or EKF is bad, true once both have recovered
local last_gps_time_ms = nil

local flight_stage = 0  -- 0. wait for good-gps and dist-from-home, 1=wait for bad gps or ekf, 2=level vehicle, 3=deadreckon home
local recovery_start_time_ms = 0-- system time GPS quality and EKF failsafe recovered (0 if not recovered)

local home_dist = 0     -- distance to home in meters
local home_yaw = nil      -- direction to home in degrees

local target_yaw = 0    -- deg
local climb_rate = 0    -- m/s

local stage1_flight_mode = nil  -- flight mode vehicle was in during stage1 (may be used during recovery)
local stage2_start_time_ms  -- system time stage2 started (level vehicle)
local stage3_start_time_ms  -- system time stage3 started (deadreckon home)
local last_print_ms = 0     -- pilot update timer
local interval_ms = 1000     -- update at 1hz як часто буде запускатись апдейт скрипта
local stage3_recover_timeout = 0  -- system time when stage3 leveling started (level before change to next mode)

local TIMEOUT = 5000
local _last_seen_message_timestamp_ms = millis()

local EK3_DRAG_BCOEF_Y = Parameter()
EK3_DRAG_BCOEF_Y:init('EK3_DRAG_BCOEF_Y')
local C_y = EK3_DRAG_BCOEF_Y:get()

local EK3_DRAG_MCOEF = Parameter()
EK3_DRAG_MCOEF:init('EK3_DRAG_MCOEF')
local M = EK3_DRAG_MCOEF:get()

-- variables for logging
local stage4log = 1
local pitch4log = 2
local yaw4log = 3
local message4log = 4
local data4log = {}
-- ===================================================================================================================================
-- ===================================================================================================================================
-- ======================= MAVLINK MODULE (DO NOT UPDATE) ============================================================================

local mavlink_msg = {}
mavlink_msg.RC_CHANNELS_OVERRIDE = {}
mavlink_msg.RC_CHANNELS_OVERRIDE.id = 70
mavlink_msg.RC_CHANNELS_OVERRIDE.fields = {
             { "target_system", "<B" },
             { "target_component", "<B" },
             { "chan1_raw", "<I2" },
             { "chan2_raw", "<I2" },
             { "chan3_raw", "<I2" },
             { "chan4_raw", "<I2" },
             }

-- Auto generated MAVLink parsing script
local mavlink_msgs = {}

---Lookup the message id for a given message name
---@param msgname string
---@return integer -- message id
function mavlink_msgs.get_msgid(msgname)
  local message_map = mavlink_msg[msgname]
  if not message_map then
    error("Unknown MAVLink message " .. msgname)
  end
  return message_map.id
end

---Return a object containing everything that is not the payload
---@param message any -- encoded message
---@return table
function mavlink_msgs.decode_header(message)
  -- build up a map of the result
  local result = {}

  result.checksum = string.unpack("<H", message, 1)

  -- id the MAVLink version
  local magic = string.unpack("<B", message, 3)
  if (magic == 0xFE) then -- mavlink 1
    result.protocol_version = 1
  elseif (magic == 0XFD) then --mavlink 2
    result.protocol_version = 2
  else
    error("Invalid magic byte")
  end

  -- fetch payload length
  result.payload_length = string.unpack("<B", message, 4)

  -- fetch the incompat/compat flags
  result.incompat_flags, result.compat_flags = string.unpack("<BB", message, 5)

  -- fetch seq/sysid/compid
  result.seq, result.sysid, result.compid = string.unpack("<BBB", message, 7)

  -- fetch the message id
  result.msgid = string.unpack("<I3", message, 10)

  return result
end

-- generate the x25crc for a given buffer
---@param buffer string -- buffer to crc
---@return integer -- resulting crc 0 to 0xFFFF
function mavlink_msgs.generateCRC(buffer)
  -- generate the x25crc for a given buffer.
  local crc = 0xFFFF
  for i = 1, #buffer do
      local tmp = string.byte(buffer, i, i) ~ (crc & 0xFF)
      tmp = (tmp ~ (tmp << 4)) & 0xFF
      crc = (crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)
      crc = crc & 0xFFFF
  end
  return crc
end

-- Note that this does not parse the serial data, it parses the MAVLink 2 C structure `mavlink_message_t`
-- This structure is passed in by the ArduPilot bindings as a string
---@param message any -- encoded message
---@param msg_map table -- table containing message objects with keys of the message ID
---@return table|nil -- a table representing the contents of the message, or nill if decode failed
function mavlink_msgs.decode(message, msg_map)
  local result = mavlink_msgs.decode_header(message)
  local message_map = mavlink_msg[msg_map[result.msgid]]
  if not message_map then
    -- we don't know how to decode this message, bail on it
    return nil
  end

  -- If we have a crc extra for this message then check it
  -- This ensures compatibility with message definitions generated before the crc check was added
  if message_map.crc_extra then
    -- crc of payload and header values
    local crc_buffer
    if result.protocol_version == 2 then
      crc_buffer = string.sub(message, 4, 12 + result.payload_length)

    else
      -- V1 does not include all fields on the wire
      crc_buffer = string.char(result.payload_length)
      crc_buffer = crc_buffer .. string.char(result.seq)
      crc_buffer = crc_buffer .. string.char(result.sysid)
      crc_buffer = crc_buffer .. string.char(result.compid)
      crc_buffer = crc_buffer .. string.char(result.msgid)
      if result.payload_length > 0 then
        crc_buffer = crc_buffer .. string.sub(message, 13, 12 + result.payload_length)
      end

    end

    local crc = mavlink_msgs.generateCRC(crc_buffer .. string.char(message_map.crc_extra))

    if crc ~= result.checksum then
      -- crc failed
      return nil
    end
  end

  -- map all the fields out
  local offset = 13
  for _,v in ipairs(message_map.fields) do
    if v[3] then
      result[v[1]] = {}
      for j=1,v[3] do
        result[v[1]][j], offset = string.unpack(v[2], message, offset)
      end
    else
      result[v[1]], offset = string.unpack(v[2], message, offset)
      if string.sub(v[2],2,2) == 'c' then
        -- Got string, unpack includes 0 values to the set length
        -- this is annoying, so remove them
        result[v[1]] = string.gsub(result[v[1]], string.char(0), "")
      end
    end
  end

  return result
end

---Encode the payload section of a given message
---@param msgname string -- name of message to encode
---@param message table -- table containing key value pairs representing the data fields in the message
---@return integer -- message id
---@return string -- encoded payload
function mavlink_msgs.encode(msgname, message)
  local message_map = mavlink_msg[msgname]
  if not message_map then
    -- we don't know how to encode this message, bail on it
    error("Unknown MAVLink message " .. msgname)
  end

  local packString = "<"
  local packedTable = {}
  local packedIndex = 1
  for i,v in ipairs(message_map.fields) do
    if v[3] then
      packString = (packString .. string.rep(string.sub(v[2], 2), v[3]))
      for j = 1, v[3] do
        packedTable[packedIndex] = message[message_map.fields[i][1]][j]
        if packedTable[packedIndex] == nil then
          packedTable[packedIndex] = 0
        end
        packedIndex = packedIndex + 1
      end
    else
      packString = (packString .. string.sub(v[2], 2))
      packedTable[packedIndex] = message[message_map.fields[i][1]]
      packedIndex = packedIndex + 1
    end
  end
  return message_map.id, string.pack(packString, table.unpack(packedTable))
end

local function write_to_dataflash(stage,message)
  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- lua automatically adds a timestamp in micro seconds
  if (message) then
    logger:write('DRM','message','Z','-','-',message)
  else
    data4log[stage4log] = stage or data4log[stage4log]
    logger:write('DR','stage,pitch,yaw','bff','-dd','---',data4log[stage4log],data4log[pitch4log],data4log[yaw4log])
  end
end

-- ===================================================================================================================================
-- ===================================================================================================================================

-- ===================================================================
-- ===============   RC OVERRIDES CONFIG    ==========================
local ROLL_RC = 1
local PITCH_RC = 2
local THROTTLE_RC = 3
local YAW_RC = 4
local MODE_RC = 5
local GIMBAL_RC = 6
local GPS_RC = 11
local DROP_RC = 13
local DROP_ARM_RC = 15

local RC_CONFIG = {
    [ROLL_RC] = param:get('RC1_TRIM'),
    [PITCH_RC] = param:get('RC2_TRIM'),
    [THROTTLE_RC] = 1500,
    [YAW_RC] = param:get('RC4_TRIM'),
    [MODE_RC] = 1800,
    [GIMBAL_RC] = 1500,
-- TODO: UNCOMMENT FOR REAL DRONE?
--    [GPS_RC] = 1000,
    [DROP_RC] = 1000,
    [DROP_ARM_RC] = 1007
}
-- ===================================================================
-- ===================================================================

local RC_CHANNELS_OVERRIDE = mavlink_msgs.get_msgid("RC_CHANNELS_OVERRIDE")
local _last_seen_message_timestamp_ms = millis()

local msg_map = {
    [RC_CHANNELS_OVERRIDE] = "RC_CHANNELS_OVERRIDE"
}

function update_rc()
  for k, v in pairs(RC_CONFIG) do
      local ch = rc:get_channel(k)
      ch:set_override(v)
  end
end

mavlink:init(1, 10)
mavlink:register_rx_msgid(RC_CHANNELS_OVERRIDE)

function getRoll()
  local wind = ahrs:wind_estimate()
  local body_wind = ahrs:earth_to_body(wind)
  local v = body_wind:y()
  if (not v or not C_y or not M) then
    gcs:send_text(2, "rollcalc: 0")
    return 0
  end
  local density_ratio = 1 / ahrs:get_EAS2TAS()
  local rho = math.max(1.225 * density_ratio, 0.1)
  local drag_force_sign = v > 0 and -1 or (v == 0 and 0 or 1)
  gcs:send_text(2,  "rollcalc w:" .. tostring(wind:y()))
  gcs:send_text(2,  "rollcalc v:" .. tostring(v))
  -- https://github.com/ArduPilot/ardupilot/blob/2e516475dd2d21acc42f185a1d994c428377e904/libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp#L531
  local a = (0.5 / C_y) * rho * v * v * drag_force_sign - v * M * density_ratio
  local roll = math.deg(a / 9.8)
  if (roll > 20) then
    return 20
  end
  if (roll < -20) then
    return -20
  end
  return roll
end

function update()
  -- exit immediately if not enabled
  if (enable:get() < 1) then
    return update, 1000
  end
  data4log[stage4log]=flight_stage
  data4log[pitch4log]=0
  data4log[yaw4log]=0
  -- determine if progress update should be sent to user
  local now_ms = millis()
  local update_user = false --as I understood it's flag for unable to spam GS
  if (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    update_user = true
  end

  -- check GPS
  local gps_speed_acc = gps:speed_accuracy(gps:primary_sensor())
  if gps_speed_acc == nil then
    gps_speed_acc = 99
  end
  local gps_speed_acc_bad = ((gps_speed_acc_max:get() > 0) and (gps_speed_acc > gps_speed_acc_max:get()))
  local gps_num_sat = gps:num_sats(gps:primary_sensor())
  local gps_num_sat_bad = (gps_sat_count_min:get() > 0) and ((gps_num_sat == nil) or (gps:num_sats(gps:primary_sensor()) < gps_sat_count_min:get()))
  local gps_status = gps:status(gps:primary_sensor())
  local gps_bad_status = gps_status == gps.NO_GPS or gps_status == gps.NO_FIX
  if gps_bad then
    -- GPS is bad, check for recovery
    if (not gps_speed_acc_bad and not gps_num_sat_bad and not gps_bad_status) then
      gps_bad = false
    end
  else
    last_gps_time_ms = now_ms
    -- GPS is good, check for GPS going bad
    if (gps_speed_acc_bad or gps_num_sat_bad or gps_bad_status) then
      gps_bad = true
    end
  end

  -- check RC
  -- TODO: UNCOMMENT FOR REAL DRONE
  -- local msg, _, timestamp_ms = mavlink.receive_chan()

  -- if msg then
  --   local parsed_msg = mavlink_msgs.decode(msg, msg_map)
  --   if parsed_msg.msgid == RC_CHANNELS_OVERRIDE then
  --     if rc_bad then
  --       rc_bad = false
  --       gcs:send_text(2, "rc failsafe cleared")
  --     end
  --     _last_seen_message_timestamp_ms = timestamp_ms
  --   end
  -- end
  local parsed_msg = rc:get_pwm(12)

  if parsed_msg < 1500 then
    if rc_bad then
      rc_bad = false
      gcs:send_text(2, "rc failsafe cleared")
      write_to_dataflash(nil,"rc failsafe cleared")
    end
    _last_seen_message_timestamp_ms = millis()
  end

  if millis() - _last_seen_message_timestamp_ms > TIMEOUT then
    if not rc_bad then
        rc_bad = true
    end
  end

  -- check for GPS and RC going bad
  if gps_bad and rc_bad then
    gps_and_rc_bad = true
    -- TODO: UNCOMMENT FOR REAL DRONE
    -- not sure if it's needed (zz)
    -- turn on gps rc
    -- local ch = rc:get_channel(11)
    -- ch:set_override(1000)
    if (update_user) then
      gcs:send_text(1, "DR: GPS bad and RC Failsafe")
      write_to_dataflash(nil,"GPS bad and RC Failsafe")
    end
  end

  -- check for GPS and RC recovery
  if (gps_and_rc_bad and (not gps_bad or not rc_bad)) then
    -- start recovery timer
    if recovery_start_time_ms == 0 then
      recovery_start_time_ms = now_ms
    end
    if (now_ms - recovery_start_time_ms > recovery_delay_ms) then
      gps_and_rc_bad = false
      recovery_start_time_ms = 0
      gcs:send_text(1, "DR: GPS or RC recovered")
      write_to_dataflash(nil,"GPS or RC recovered")
    end
  end

  -- update distance and direction home while GPS and EKF are good
  if (not gps_bad) then
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_location()
    if home and curr_loc then
      home_dist = curr_loc:get_distance(home)
      home_yaw = math.deg(curr_loc:get_bearing(home))
    end
  end
  
  -- if last gps is old use mission WP
  if (last_gps_time_ms and (now_ms - last_gps_time_ms) > (last_gps_timeout:get() * 100)) then
    local m = mission:get_item(1)
    if (m) then
      local home = ahrs:get_home()
      local curr_loc = Location()
      curr_loc:lat(m:x())
      curr_loc:lng(m:y())
      curr_loc:relative_alt(true)
      curr_loc:terrain_alt(false)
      curr_loc:origin_alt(false)
      curr_loc:alt(math.floor(m:z()*100))
      home_yaw = math.deg(curr_loc:get_bearing(home))
    end
  end

  -- reset flight_stage when disarmed
  if not arming:is_armed() then 
    flight_stage = 0
    transition_start_time_ms = 0
    return update, interval_ms
  end

  -- ===================================================
  -- RC FAILSAFE

  if (rc_bad) then
    if (update_user) then
      gcs:send_text(1, "DR RC FAILSAFE")
      write_to_dataflash(nil,"DR RC FAILSAFE")
    end
    -- TODO: uncomment 
    update_rc()
    return update, interval_ms
  end
  -- ===================================================
  -- ===================================================
  data4log[stage4log]=flight_stage
  data4log[pitch4log]=0
  data4log[yaw4log]=0

  -- flight_stage 0: wait for dist-from-home or timeout
  if (flight_stage == 0) then
    stage3_recover_timeout = 0 -- reset timeout stage3
    -- wait for distance from home to pass DR_ENAB_DIST or timeout after 
    if ((home_dist > 0) and (home_dist >= enable_dist:get())) then
      flight_stage = 1
      gcs:send_text(5, "DR armed (changed to 1 stage)")
      write_to_dataflash(nil,"DR armed (changed to 1 stage)")
    end
    return update, interval_ms
  end

  -- flight_stage 1: wait for bad gps and bad rc
  if (flight_stage == 1) then
    if (gps_and_rc_bad) then
      -- change to Guided_NoGPS and initialise stage2
      if (vehicle:set_mode(copter_guided_nogps_mode)) then
        flight_stage = 2
        target_yaw = math.deg(ahrs:get_yaw())
        stage2_start_time_ms = now_ms
      else
        -- warn user of unexpected failure
        if (update_user) then
          gcs:send_text(0, "DR: failed to change to Guided_NoGPS mode")
          write_to_dataflash(nil,"DR: failed to change to Guided_NoGPS mode")
        end
      end
    else
      -- store flight mode (may be used during recovery)
      stage1_flight_mode = vehicle:get_mode()
    end
    write_to_dataflash(flight_stage,nil)
    return update, interval_ms
  end

  -- flight_stage 2: level vehicle for 5 seconds
  if (flight_stage == 2) then
    -- if gps recovered fallback to RC Failsafe
    if (rc_bad and not gps_bad) then
      gcs:send_text(4, "DR: GPS Recovered. Switching to RTL mode")
      write_to_dataflash(nil,"DR: GPS Recovered. Switching to RTL mode")
      flight_stage = 1
      write_to_dataflash(flight_stage,nil)
      return update, interval_ms
    end
    -- allow pilot to retake control
    if (vehicle:get_mode() ~= copter_guided_nogps_mode) then
      gcs:send_text(5, "DR: pilot retook control")
      write_to_dataflash(nil,"DR: pilot retook control")
      flight_stage = 1
      write_to_dataflash(flight_stage,nil)
      return update, interval_ms
    end

    -- level vehicle for 5 seconds, yaw towards home
    climb_rate = 0
    if (home_yaw) then
      target_yaw = math.fmod(home_yaw, 360)
    end
    gcs:send_text(5, "DR: rotating home")
    write_to_dataflash(nil,"DR: rotating home")
    data4log[pitch4log]=0
    data4log[yaw4log]=target_yaw
    -- TODO мені здається краще крутитись по напряму дому вже після того, як буде висіти 5 секунд
    vehicle:set_target_angle_and_climbrate(0, 0, target_yaw, climb_rate, true, 1 ) --5 параметр bool use_yaw_rate (boolean): Якщо true, то yaw_deg інтерпретується як кутова швидкість рискання (градуси за секунду). Якщо false, то це абсолютний кут. 6 параметр це  float yaw_rate_degs. Схоже ці параметри не використовуються. Принаймні в однойменній функції в Copter.cpp вони взагалі ніяк не використовуються
    if ((now_ms - stage2_start_time_ms) >= 5000) then
      flight_stage = 3
      stage3_start_time_ms = now_ms
      gcs:send_text(5, "DR: flying home")
      write_to_dataflash(nil,"DR: flying home")
    end
    if (update_user) then
      gcs:send_text(5, "DR: leveling vehicle")
    end
    write_to_dataflash(flight_stage)
    return update, interval_ms
  end

  -- flight_stage 3: deadreckon towards home
  if (flight_stage == 3) then
    -- if gps recovered fallback to RC Failsafe
    if (rc_bad and not gps_bad) then
      gcs:send_text(5, "DR: GPS Recovered. Switching to RTL mode")
      write_to_dataflash(nil,"DR: GPS Recovered. Switching to RTL mode")
      flight_stage = 1
      write_to_dataflash(flight_stage,nil)
      return update, interval_ms
    end

    -- allow pilot to retake control
    if (vehicle:get_mode() ~= copter_guided_nogps_mode) then
      gcs:send_text(5, "DR: pilot retook control")
      write_to_dataflash(nil,"DR: pilot retook control")
      flight_stage = 1
      write_to_dataflash(flight_stage,nil)
      return update, interval_ms
    end

    -- check for timeout
    local time_elapsed_ms = now_ms - stage3_start_time_ms
    local timeout = (fly_timeoout:get() > 0) and (time_elapsed_ms >= (fly_timeoout:get() * 1000))

    -- set angle target to roll 0, pitch to lean angle (note: negative is forward), yaw towards home

    local roll = getRoll()
    local targetPitch = math.abs(fly_angle:get())
    if (home_yaw) then
      targetPitch = targetPitch * -1
    end
    data4log[pitch4log]=targetPitch
    data4log[yaw4log]=target_yaw
    if (vehicle:set_target_angle_and_climbrate(roll, targetPitch, target_yaw, climb_rate, false, 0)) then
      if (update_user) then
        local time_left_str = ""
        if (not timeout and (fly_timeoout:get() > 0)) then
          time_left_str = " t:" .. tostring(math.max(0, ((fly_timeoout:get() * 1000) - time_elapsed_ms) / 1000))
        end
        gcs:send_text(5, "DR: fly home yaw:" .. tostring(math.floor(target_yaw)) .. " pit:" .. tostring(math.floor(fly_angle:get())) .. " roll:" .. tostring(math.floor(roll)) .. time_left_str)
        write_to_dataflash(nil,"DR: fly home yaw:" .. tostring(math.floor(target_yaw)) .. " pit:" .. tostring(math.floor(fly_angle:get())) .. " roll:" .. tostring(math.floor(roll)) .. time_left_str)
      end
    elseif (update_user) then
      gcs:send_text(0, "DR: failed to set attitude target")
      write_to_dataflash(nil,"DR: failed to set attitude target")
    end

    -- if GPS and EKF recover or timeout switch to next mode
    if (not gps_and_rc_bad) or timeout then
      -- level vehicle for some time before changing to next mode
      if (stage3_recover_timeout == 0) then
        stage3_recover_timeout = now_ms
      end
      if (now_ms - stage3_recover_timeout > 2000) then
        -- exit from deadreckoning
        local recovery_mode = stage1_flight_mode
        if (next_mode:get() >= 0) then
          recovery_mode = next_mode:get()
        end
        if (recovery_mode == nil) then
          recovery_mode = copter_RTL_mode
          gcs:send_text(4, "DR: NEXT_MODE=-1 but fallingback to RTL")
          write_to_dataflash(nil,"DR: NEXT_MODE=-1 but fallingback to RTL")
        end
        -- change to DR_NEXT_MODE
        if (vehicle:set_mode(recovery_mode)) then
          flight_stage = 0
        elseif (update_user) then
          -- warn user of unexpected failure        
          gcs:send_text(0, "DR: failed to change to mode " .. tostring(recovery_mode))
          write_to_dataflash(nil,"DR: failed to change to mode " .. tostring(recovery_mode))
        end
      end
    end
    write_to_dataflash(flight_stage,nil)
    return update, interval_ms
  end
  write_to_dataflash(10,nil)
  -- we should never get here but just in case
  return update, interval_ms
end

gcs:send_text(5, "DR script init")

return update()