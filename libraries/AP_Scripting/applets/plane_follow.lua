--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Follow in Plane
   Support follow "mode" in plane. This will actually use GUIDED mode with 
   a scripting switch to allow guided to track the vehicle id in FOLL_SYSID
   Uses the AP_Follow library so all of the existing FOLL_* parameters are used
   as documented for Copter, + add 3 more for this script
   ZPF_EXIT_MODE - the mode to switch to when follow is turned of using the switch
   ZPF_FAIL_MODE - the mode to switch to if the target is lost
   ZPF_TIMEOUT - number of seconds to try to reaquire the target after losing it before failing
   ZPF_OVRSHT_DEG - if the target is more than this many degrees left or right, assume an overshoot
   ZPR_TURN_DEG - if the target is more than this many degrees left or right, assume it's turning
--]]

SCRIPT_VERSION = "4.7.0-059"
SCRIPT_NAME = "Plane Follow"
SCRIPT_NAME_SHORT = "PFollow"

-- FOLL_ALT_TYPE and Mavlink FRAME use different values 
ALT_FRAME = { GLOBAL = 0, RELATIVE = 1, TERRAIN = 3}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = {GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3,  GLOBAL_TERRAIN_ALT = 10}
MAV_CMD_INT = { ATTITUDE = 30, GLOBAL_POSITION_INT = 33, REQUEST_DATA_STREAM = 66,
                  DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192,
                  CMD_SET_MESSAGE_INTERVAL = 511, CMD_REQUEST_MESSAGE = 512,
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1, DEFAULT = 2} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

MAV_DATA_STREAM = { MAV_DATA_STREAM_ALL=0, MAV_DATA_STREAM_RAW_SENSORS=1, MAV_DATA_STREAM_EXTENDED_STATUS=2, MAV_DATA_STREAM_RC_CHANNELS=3,
                     MAV_DATA_STREAM_RAW_CONTROLLER=4, MAV_DATA_STREAM_POSITION=6, MAV_DATA_STREAM_EXTRA1=10, MAV_DATA_STREAM_EXTRA2=11 }

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

local ahrs_eas2tas = ahrs:get_EAS2TAS()
local windspeed_vector = ahrs:wind_estimate()

local now = millis():tofloat() * 0.001
local now_target_heading = now
local now_telemetry_request = now
local now_follow_lost = now
local follow_enabled = false
local too_close_follow_up = 0
local save_target_heading1 = -400.0
local save_target_heading2 = -400.0
local save_target_altitude
local tight_turn = false

local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "ZPF_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end
-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 25), 'could not add param table')

-- add a parameter and bind it to a variable
--local function bind_add_param2(name, idx, default_value)
--   assert(param:add_param(PARAM_TABLE_KEY2, idx, name, default_value), string.format('could not add param %s', name))
--   return Parameter(PARAM_TABLE_PREFIX2 .. name)
--end
-- setup follow mode specific parameters- second tranche
-- assert(param:add_table(PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2, 10), 'could not add param table')

-- This uses the exisitng FOLL_* parameters and just adds a couple specific to this script
-- but because most of the logic is already in AP_Follow (called by binding to follow:) there
-- is no need to access them in the scriot

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = Parameter('FOLL_ALT_TYPE')
FOLL_SYSID = Parameter('FOLL_SYSID')
FOLL_OFS_Y = Parameter('FOLL_OFS_Y')
local foll_sysid = FOLL_SYSID:get() or -1
local foll_ofs_y = FOLL_OFS_Y:get() or 0.0
local foll_alt_type = FOLL_ALT_TYPE:get() or ALT_FRAME.GLOBAL

-- Add these ZPF_ parameters specifically for this script
--[[
  // @Param: ZPF_FAIL_MODE
  // @DisplayName: Plane Follow lost target mode
  // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX). 
  // @User: Standard
--]]
ZPF_FAIL_MODE = bind_add_param('FAIL_MODE', 1, FLIGHT_MODE.RTL)

--[[
  // @Param: ZPF_EXIT_MODE
  // @DisplayName: Plane Follow exit mode
  // @Description: Mode to switch to when follow mode is exited normally
  // @User: Standard
--]]
ZPF_EXIT_MODE = bind_add_param('EXIT_MODE', 2, FLIGHT_MODE.LOITER)

--[[
    // @Param: ZPF_ACT_FN
    // @DisplayName: Plane Follow Scripting ActivationFunction
    // @Description: Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable
    // @Range: 300 307
--]]
ZPF_ACT_FN = bind_add_param("ACT_FN", 3, 301)

--[[
    // @Param: ZPF_TIMEOUT
    // @DisplayName: Plane Follow Telemetry Timeout
    // @Description: How long to try reaquire a target if telemetry from the lead vehicle is lost.
    // @Range: 0 30
    // @Units: s
--]]
ZPF_TIMEOUT = bind_add_param("TIMEOUT", 4, 10)

--[[
    // @Param: ZPF_OVRSHT_DEG
    // @DisplayName: Plane Follow Scripting Overshoot Angle
    // @Description: If the target is greater than this many degrees left or right, assume an overshoot 
    // @Range: 0 180
    // @Units: deg
--]]
ZPF_OVRSHT_DEG = bind_add_param("OVRSHT_DEG", 5, 75)

--[[
    // @Param: ZPF_TURN_DEG
    // @DisplayName: Plane Follow Scripting Turn Angle
    // @Description: If the target is greater than this many degrees left or right, assume it's turning
    // @Range: 0 180
    // @Units: deg
--]]
ZPF_TURN_DEG = bind_add_param("TURN_DEG", 6, 15)

--[[
    // @Param: ZPF_DIST_CLOSE
    // @DisplayName: Plane Follow Scripting Close Distance
    // @Description: When closer than this distance assume we track by heading
    // @Range: 0 100
    // @Units: m
--]]
ZPF_DIST_CLOSE = bind_add_param("DIST_CLOSE", 7, 50)

--[[
    // @Param: ZPF_WIDE_TURNS
    // @DisplayName: Plane Follow Scripting Wide Turns
    // @Description: Use wide turns when following a turning target. Alternative is "cutting the corner"
    // @Range: 0 1
--]]
ZPF_WIDE_TURNS = bind_add_param("WIDE_TURNS", 8, 1)

--[[
    // @Param: ZPF_ALT
    // @DisplayName: Plane Follow Scripting Altitude Override
    // @Description: When non zero, this altitude value (in FOLL_ALT_TYPE frame) overrides the value sent by the target vehicle
    // @Range: 0 1000
    // @Units: m
--]]
ZPF_ALT_OVR = bind_add_param("ALT_OVR", 9, 0)

--[[
    // @Param: ZPF_D_P
    // @DisplayName: Plane Follow Scripting distance P gain
    // @Description: P gain for the speed PID controller distance component
    // @Range: 0 1
--]]
ZPF_D_P = bind_add_param("D_P", 11, 0.0005)

--[[
    // @Param: ZPF_D_I
    // @DisplayName: Plane Follow Scripting distance I gain
    // @Description: I gain for the speed PID  distance component
    // @Range: 0 1
--]]
ZPF_D_I = bind_add_param("D_I", 12, 0.0005)

--[[
    // @Param: ZPF_D_D
    // @DisplayName: Plane Follow Scripting distance D gain
    // @Description: D gain for the speed PID controller distance component
    // @Range: 0 1
--]]
ZPF_D_D = bind_add_param("D_D", 13, 0.00025)

--[[
    // @Param: ZPF_V_P
    // @DisplayName: Plane Follow Scripting speed P gain
    // @Description: P gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
ZPF_V_P = bind_add_param("V_P", 14, 0.0005)

--[[
    // @Param: ZPF_V_I
    // @DisplayName: Plane Follow Scripting speed I gain
    // @Description: I gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
ZPF_V_I = bind_add_param("V_I", 15, 0.0005)

--[[
    // @Param: ZPF_V_D
    // @DisplayName: Plane Follow Scripting speed D gain
    // @Description: D gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
ZPF_V_D = bind_add_param("V_D", 16, 0.00025)

--[[
    // @Param: ZPF_LkAHD
    // @DisplayName: Plane Follow Lookahead seconds
    // @Description: Time to "lookahead" when calculating distance errors
    // @Units: s
--]]
ZPF_LKAHD = bind_add_param("LKAHD", 17, 6)

--[[
    // @Param: ZPF_DIST_FUDGE
    // @DisplayName: Plane Follow distance fudge factor
    // @Description: THe distance returned by the AP_FOLLOW library might be off by about this factor of airspeed
    // @Units: s
--]]
ZPF_DIST_FUDGE = bind_add_param("DIST_FUDGE", 18, 0.92)

--[[
    // @Param: ZPF_SIM_TELF_FN
    // @DisplayName: Plane Follow Simulate Telemetry fail function
    // @Description: Set this switch to simulate losing telemetry from the other vehicle
    // @Range: 300 307
--]]
ZPF_SIM_TELF_FN = bind_add_param("SIM_TELF_FN", 19, 302)

--[[
    // @Param: ZPF_SR_CH
    // @DisplayName: Plane Follow Stream Rate Serial Channel
    // @Description: This is the serial/channel where mavlink messages will go to the target vehicle
    // @Range: 0 9
--]]
ZPF_SR_CH = bind_add_param("SR_CH", 20, -1)

--[[
    // @Param: ZPF_SR_INT
    // @DisplayName: Plane Follow Stream Rate Interval
    // @Description: This is the stream rate (milliseconds between messages) that the chase plane will request from the lead plane for GLOBAL_POSITION_INT and ATTITUDE telemetry. Set to -1 to disable.
    // @Range: 25 500
    // @Units: ms
--]]
ZPF_SR_INT = bind_add_param("SR_INT", 21, 50)

REFRESH_RATE = 0.05   -- in seconds, so 20Hz
LOST_TARGET_TIMEOUT = (ZPF_TIMEOUT:get() or 10) / REFRESH_RATE
OVERSHOOT_ANGLE = ZPF_OVRSHT_DEG:get() or 75.0
TURNING_ANGLE = ZPF_TURN_DEG:get() or 20.0
DISTANCE_LOOKAHEAD_SECONDS = ZPF_LKAHD:get() or 5.0

local lost_target_countdown = LOST_TARGET_TIMEOUT

local fail_mode = ZPF_FAIL_MODE:get() or FLIGHT_MODE.QRTL
local exit_mode = ZPF_EXIT_MODE:get() or FLIGHT_MODE.LOITER

local use_wide_turns = ZPF_WIDE_TURNS:get() or 1

local distance_fudge = ZPF_DIST_FUDGE:get() or 0.92

local target_serial_channel = ZPF_SR_CH:get() or -1

local simulate_telemetry_failed = false

AIRSPEED_MIN = Parameter('AIRSPEED_MIN')
AIRSPEED_MAX = Parameter('AIRSPEED_MAX')
AIRSPEED_CRUISE = Parameter('AIRSPEED_CRUISE')
WP_LOITER_RAD = Parameter('WP_LOITER_RAD')
WINDSPEED_MAX = Parameter('AHRS_WIND_MAX')

local airspeed_max = AIRSPEED_MAX:get() or 25.0
local airspeed_min = AIRSPEED_MIN:get() or 12.0
local airspeed_cruise = AIRSPEED_CRUISE:get() or 18.0
local windspeed_max = WINDSPEED_MAX:get() or 100.0

local function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

local speedpid = require("speedpid")
local pid_controller_distance = speedpid.speed_controller(ZPF_D_P:get() or 0.01,
                                                            ZPF_D_I:get() or 0.01,
                                                            ZPF_D_D:get() or 0.005,
                                                            0.5, airspeed_min - airspeed_max, airspeed_max - airspeed_min)

local pid_controller_velocity = speedpid.speed_controller(ZPF_V_P:get() or 0.01,
                                                            ZPF_V_I:get() or 0.01,
                                                            ZPF_V_D:get() or 0.005,
                                                            0.5, airspeed_min, airspeed_max)


local mavlink_attitude = require("mavlink_attitude")
local mavlink_attitude_receiver = mavlink_attitude.mavlink_attitude_receiver()

local function follow_frame_to_mavlink(follow_frame)
   local mavlink_frame = MAV_FRAME.GLOBAL;
   if (follow_frame == ALT_FRAME.TERRAIN) then
      mavlink_frame = MAV_FRAME.GLOBAL_TERRAIN_ALT
   end
   if (follow_frame == ALT_FRAME.RELATIVE) then
      mavlink_frame = MAV_FRAME.GLOBAL_RELATIVE_ALT
   end
   return mavlink_frame
end

local mavlink_command_int = require("mavlink_command_int")

-- set_vehicle_target_altitude() Parameters
-- target.alt = new target altitude in meters
-- target.frame = Altitude frame MAV_FRAME, it's very important to get this right!
-- target.alt = altitude in meters to acheive
-- target.speed = z speed of change to altitude (1000.0 = max)
local function set_vehicle_target_altitude(target)
   local speed = target.speed or 1000.0 -- default to maximum z acceleration
   if target.alt == nil then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_target_altitude no altitude")
      return
   end
   -- GUIDED_CHANGE_ALTITUDE takes altitude in meters
   if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_ALTITUDE, {
                              frame = follow_frame_to_mavlink(target.frame),
                              p3 = speed,
                              z = target.alt }) then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink CHANGE_ALTITUDE returned false")
   end
end

-- set_vehicle_heading() Parameters
-- heading.heading_type = MAV_HEADING_TYPE (defaults to HEADING)
-- heading.heading = the target heading in degrees
-- heading.accel = rate/acceleration to acheive the heading 0 = max
local function set_vehicle_heading(heading)
   local heading_type = heading.type or MAV_HEADING_TYPE.HEADING
   local heading_heading = heading.heading or 0
   local heading_accel = heading.accel or 10.0

   if heading_heading == nil or heading_heading <= -400 or heading_heading > 360 then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_heading no heading")
      return
   end

   if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_HEADING, { frame = MAV_FRAME.GLOBAL,
                                 p1 = heading_type, 
                                 p2 = heading_heading,
                                 p3 = heading_accel }) then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink GUIDED_CHANGE_HEADING failed")
   end
end

-- set_vehicle_speed() Parameters
-- speed.speed - new target speed 
-- speed.type - speed type MAV_SPEED_TYPE
-- speed.throttle - new target throttle (used if speed = 0)
-- speed.slew - specified slew rate to hit the target speed, rather than jumping to it immediately 0 = maximum acceleration
local function set_vehicle_speed(speed)
   local new_speed = speed.speed or 0.0
   local speed_type = speed.type or MAV_SPEED_TYPE.AIRSPEED
   local throttle = speed.throttle or 0.0
   local slew = speed.slew or 0.0
   local mode = vehicle:get_mode()

   if speed_type == MAV_SPEED_TYPE.AIRSPEED and mode == FLIGHT_MODE.GUIDED then
      if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_SPEED, { frame = MAV_FRAME.GLOBAL,
                                 p1 = speed_type,
                                 p2 = new_speed,
                                 p3 = slew }) then
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink GUIDED_CHANGE_SPEED failed")
      end
   else
      if not gcs:run_command_int(MAV_CMD_INT.DO_CHANGE_SPEED, { frame = MAV_FRAME.GLOBAL,
                                 p1 = speed_type,
                                 p2 = new_speed,
                                 p3 = throttle }) then
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink DO_CHANGE_SPEED failed")
      end
   end
end

-- set_vehicle_target_location() Parameters
-- target.groundspeed (-1 for ignore)
-- target.radius (defaults to 2m)
-- target.yaw - not really yaw - it's the loiter direction 1 = CCW, -1 = CW NaN = default
-- target.lat - latitude in decimal degrees
-- target.lng - longitude in decimal degrees
-- target.alt - target alitude in meters
local function set_vehicle_target_location(target)
   local radius = target.radius or 2.0
   local yaw = target.yaw or 1
   -- If we are on the right side of the vehicle make sure any loitering is CCW (moves away from the other plane)
   -- yaw > 0 - CCW = turn to the right of the target point
   -- yaw < 0 - Clockwise = turn to the left of the target point
   -- if following direct we turn on the "outside"

   -- if we were in HEADING mode, need to switch out of it so that REPOSITION will work
   -- Note that MAVLink DO_REPOSITION requires altitude in meters
   set_vehicle_heading({type = MAV_HEADING_TYPE.DEFAULT})
   if not gcs:run_command_int(MAV_CMD_INT.DO_REPOSITION, { frame = follow_frame_to_mavlink(target.frame),
                              p1 = target.groundspeed or -1,
                              p2 = 1,
                              p3 = radius,
                              p4 = yaw,
                              x = target.lat,
                              y = target.lng,
                              z = target.alt }) then  -- altitude in m
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink DO_REPOSITION returned false")
   end
end

--[[
   return true if we are in a state where follow can apply
--]]
local reported_target = true
local lost_target_now = now
local function follow_active()
   local mode = vehicle:get_mode()

   if mode == FLIGHT_MODE.GUIDED then
      if follow_enabled then
        if follow:have_target() then
            reported_target = true
            lost_target_now = now
         else
            if reported_target then -- i.e. if we previously reported a target but lost it
               if (now - lost_target_now) > 5 then
                  gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": lost prior target: " .. follow:get_target_sysid())
                  lost_target_now = now
               end
            end
            reported_target = false
        end
      end
   else
      reported_target = false
   end

   return reported_target
end

--[[
   check for user activating follow using an RC switch set HIGH
--]]
local last_follow_active_state = rc:get_aux_cached(ZPF_ACT_FN:get())
local last_tel_fail_state = rc:get_aux_cached(ZPF_SIM_TELF_FN:get())

local function follow_check()
   if ZPF_ACT_FN == nil then
      return
   end
   local foll_act_fn = ZPF_ACT_FN:get()
   if foll_act_fn == nil then
      return
   end
   local active_state = rc:get_aux_cached(foll_act_fn)
   if (active_state ~= last_follow_active_state) then
      if( active_state == 0) then
         if follow_enabled then
            -- Follow disabled - return to EXIT mode
            vehicle:set_mode(exit_mode)
            follow_enabled = false
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": disabled")
         end
      elseif (active_state == 2) then
         if not (arming:is_armed()) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": must be armed")
         end
         -- Follow enabled - switch to guided mode
         vehicle:set_mode(FLIGHT_MODE.GUIDED)
         follow_enabled = true
         lost_target_countdown = LOST_TARGET_TIMEOUT
         --speed_controller_pid.reset()
         pid_controller_distance.reset()
         pid_controller_velocity.reset()
         gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": enabled")
      end
      -- Don't know what to do with the 3rd switch position right now.
      last_follow_active_state = active_state
   end
   local sim_tel_fail = ZPF_SIM_TELF_FN:get()
   local tel_fail_state = rc:get_aux_cached(sim_tel_fail)
   if tel_fail_state ~= last_tel_fail_state then
      if tel_fail_state == 0 then
         simulate_telemetry_failed = false
      else
         simulate_telemetry_failed = true
      end
      last_tel_fail_state = tel_fail_state
   end
end

local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

local function calculate_airspeed_from_groundspeed(velocity_vector)
   --[[ 
   This is the code from AHRS.cpp    
   Vector3f true_airspeed_vec = nav_vel - wind_vel;

   This is the c++ code from AP_AHRS_DCM.cpp and also from AP_AHRS.cpp
   float true_airspeed = airspeed_ret * get_EAS2TAS();
        true_airspeed = constrain_float(true_airspeed,
                                        gnd_speed - _wind_max,
                                        gnd_speed + _wind_max);
        airspeed_ret = true_airspeed / get_EAS2TAS(
   --]]

   local airspeed_vector = velocity_vector - windspeed_vector
   local airspeed = airspeed_vector:length()
   airspeed = airspeed * ahrs_eas2tas
   airspeed = constrain(airspeed, airspeed - windspeed_max, airspeed + windspeed_max)
   airspeed = airspeed / ahrs_eas2tas

   return airspeed
end

-- main update function
local function update()
   now = millis():tofloat() * 0.001
   ahrs_eas2tas = ahrs:get_EAS2TAS()
   windspeed_vector = ahrs:wind_estimate()

   follow_check()
   if not follow_active() then
      return
   end

   -- set the target frame as per user set parameter - this is fundamental to this working correctly
   local close_distance = ZPF_DIST_CLOSE:get() or airspeed_cruise * 2.0
   local long_distance = close_distance * 4.0
   local altitude_override = ZPF_ALT_OVR:get() or 0

   LOST_TARGET_TIMEOUT = (ZPF_TIMEOUT:get() or 10) / REFRESH_RATE
   OVERSHOOT_ANGLE = ZPF_OVRSHT_DEG:get() or 75.0
   TURNING_ANGLE = ZPF_TURN_DEG:get() or 20.0
   foll_ofs_y = FOLL_OFS_Y:get() or 0.0
   foll_alt_type = FOLL_ALT_TYPE:get() or ALT_FRAME.GLOBAL
   use_wide_turns = ZPF_WIDE_TURNS:get() or 1
   distance_fudge = ZPF_DIST_FUDGE:get() or 0.92
   target_serial_channel = ZPF_SR_CH:get() or 0

   -- Need to request that the follow vehicle sends telemetry at a reasonable rate
   -- we send a new request every 10 seconds, just to make sure the message gets through
   if (now - now_telemetry_request) > 10 then
      local stream_interval = ZPF_SR_INT:get() or 50
      if stream_interval > 0 then
         -- we'd like to get GLOBAL_POSITION_INT and ATTITUDE messages from the target vehicle at 20Hz = every 50ms
         mavlink_command_int.request_message_interval(target_serial_channel, {sysid = foll_sysid, message_id = MAV_CMD_INT.ATTITUDE, interval_ms = stream_interval})
         mavlink_command_int.request_message_interval(target_serial_channel, {sysid = foll_sysid, message_id = MAV_CMD_INT.GLO, interval_ms = stream_interval})
         now_telemetry_request = now
      end
   end

   --[[
      get the current navigation target. 
   --]]
   local target_location                     -- = Location()     of the target
   local target_location_offset              -- Location  of the target with FOLL_OFS_* offsets applied
   local target_velocity -- = Vector3f()     -- current velocity of lead vehicle
   local target_velocity_offset -- Vector3f  -- velocity to the offset target_location_offset
   local target_distance -- = Vector3f()     -- vector to lead vehicle
   local target_distance_offset              -- vector to the target taking offsets into account
   local xy_dist                             -- distance to target with offsets in meters
   local target_heading                      -- heading of the target vehicle

   local current_location = ahrs:get_location()
   if current_location == nil then
      return
   end
   ---@cast current_location -nil
   local current_altitude = current_location:alt() * 0.01

   local vehicle_airspeed = ahrs:airspeed_estimate()
   local current_target = vehicle:get_target_location()

   -- because of the methods available on AP_Follow, need to call these multiple methods get_target_dist_and_vel_ned() MUST BE FIRST
   -- to get target_location, target_velocity, target distance and target 
   -- and yes target_offsets (hopefully the same value) is returned by both methods
   -- even worse - both internally call get_target_location_and_Velocity, but making a single method
   -- in AP_Follow is probably a high flash cost, so we just take the runtime hit
   --[[
   target_distance, target_distance_offsets, target_velocity = follow:get_target_dist_and_vel_ned() -- THIS HAS TO BE FIRST
   target_location, target_velocity = follow:get_target_location_and_velocity()
   target_location_offset, target_velocity = follow:get_target_location_and_velocity_ofs()
   local xy_dist = follow:get_distance_to_target() -- this value is set by get_target_dist_and_vel_ned() - why do I have to know this?
   local target_heading = follow:get_target_heading_deg()

   target_distance, target_distance_offset,
      target_velocity, target_velocity_offset,
      target_location, target_location_offset,
      xy_dist = follow:get_target_info()

   // The order here is VERY important. This needs to called first because it updates a lot of internal variables
    if(!get_target_dist_and_vel_ned(dist_ned, dist_with_offs, target_vel_ned)) {
        return false;
    }
    if(!get_target_location_and_velocity(target_loc,target_vel_ned)) {
        return false;
    }
    if(!get_target_location_and_velocity_ofs(target_loc_ofs, target_vel_ned_ofs)) {
        return false;
    }
    target_dist_ofs = _dist_to_target;

   --]]
   target_distance, target_distance_offset, _ = follow:get_target_dist_and_vel_ned() -- THIS HAS TO BE FIRST
   target_location, target_velocity = follow:get_target_location_and_velocity()
   target_location_offset, target_velocity_offset = follow:get_target_location_and_velocity_ofs()
   xy_dist = follow:get_distance_to_target() -- this value is set by get_target_dist_and_vel_ned() - why do I have to know this? 
   target_heading = follow:get_target_heading_deg() or -400

   -- if we lose the target wait for LOST_TARGET_TIMEOUT seconds to try to reaquire it
   if target_location == nil or target_location_offset == nil or
      target_velocity == nil or target_velocity_offset == nil or
      target_distance_offset == nil or current_target == nil or target_distance == nil or xy_dist == nil or
      simulate_telemetry_failed then
      lost_target_countdown = lost_target_countdown - 1
      if lost_target_countdown <= 0 then
         follow_enabled = false
         vehicle:set_mode(fail_mode)
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(": follow: %.0f FAILED", foll_sysid))
         return
      end

      -- maintain the current heading until we re-establish telemetry from the target vehicle
      if (now - now_follow_lost) > 2 then
         gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": follow: lost %.0f set hdg: %.0f", foll_sysid, save_target_heading1))
         now_follow_lost = now
         set_vehicle_heading({heading = save_target_heading1})
         set_vehicle_target_altitude({alt = save_target_altitude, frame = foll_alt_type}) -- pass altitude in meters (location has it in cm)
      end
      return
   else
      -- have a good target so reset the countdown 
      lost_target_countdown = LOST_TARGET_TIMEOUT
      now_follow_lost = now
   end

   -- target_velocity from MAVLink (via AP_Follow) is groundspeed, need to convert to airspeed, 
   -- we can only assume the windspeed for the target is the same as the chase plane
   local target_airspeed = calculate_airspeed_from_groundspeed(target_velocity_offset)

   local vehicle_heading = math.abs(wrap_360(math.deg(ahrs:get_yaw())))
   local heading_to_target_offset = math.deg(current_location:get_bearing(target_location_offset))

   -- offset_angle is the difference between the current heading of the follow vehicle and the target_location (with offsets)
   local offset_angle = wrap_180(vehicle_heading - heading_to_target_offset)

   -- rotate the target_distance_offsets in NED to the same direction has the follow vehicle, we use this below
   local target_distance_rotated = target_distance_offset:copy()
   target_distance_rotated:rotate_xy(math.rad(vehicle_heading))

   -- default the desired heading to the target heading (adjusted for projected turns) - we might change this below
   local airspeed_difference = vehicle_airspeed - target_airspeed

   -- distance seem to be out by about 0.92s at approximately current airspeed just eyeballing it.
   xy_dist = math.abs(xy_dist) - vehicle_airspeed * distance_fudge
   -- xy_dist will always be a positive value. To get -v to represent overshoot, use the offset_angle
   -- to decide if the target is behind
   if (math.abs(xy_dist) < long_distance) and (math.abs(offset_angle) > OVERSHOOT_ANGLE) then
      xy_dist = -xy_dist
   end

   -- projected_distance is how far off the target we expect to be if we maintain the current airspeed for DISTANCE_LOOKAHEAD_SECONDS
   local projected_distance = xy_dist - airspeed_difference * DISTANCE_LOOKAHEAD_SECONDS

   -- target angle is the difference between the heading of the target and what its heading was 2 seconds ago
   local target_angle = 0.0
   if (target_heading ~= nil and target_heading > -400) then
      -- want the greatest angle of we might have turned
      local angle_diff1 = wrap_180(math.abs(save_target_heading1 - target_heading))
      local angle_diff2 = wrap_180(math.abs(save_target_heading2 - target_heading))
      if angle_diff2 > angle_diff1 then
         target_angle = angle_diff2
      else
         target_angle = angle_diff1
      end
      -- remember the target heading from 2 seconds ago, so we can tell if it is turning or not
      if (now - now_target_heading) > 1 then
         save_target_altitude = current_altitude
         save_target_heading2 = save_target_heading1
         save_target_heading1 = target_heading
         now_target_heading = now
      end
   end

   -- if the target vehicle is starting to roll we need to pre-empt a turn is coming
   -- this is fairly simplistic and could probably be improved
   -- got the code from Mission Planner - this is how MP calculates the turn radius in c#
   --[[
   public float radius
        {
            get
            {
                if (_groundspeed <= 1) return 0;
                return (float)toDistDisplayUnit(_groundspeed * _groundspeed / (9.80665 * Math.Tan(roll * MathHelper.deg2rad)));
            }
        }
   --]]
   local turning = (math.abs(projected_distance) < long_distance and math.abs(target_angle) > TURNING_ANGLE)
   local turn_starting = false
   local target_attitude = mavlink_attitude_receiver.get_attitude(foll_sysid)
   local pre_roll_target_heading = target_heading
   local desired_heading = target_heading
   local angle_adjustment
   local turning_airspeed_ratio = 1.0
   tight_turn = false
   if target_attitude ~= nil and target_airspeed ~= nil and target_airspeed > airspeed_min then
      if (now - (target_attitude.timestamp_ms * 0.001) < LOST_TARGET_TIMEOUT) and
         math.abs(target_attitude.roll) > 0.1 or math.abs(target_attitude.rollspeed) > 1 then
         -- the roll and rollspeed are delayed values from the target plane, try to extrapolate them to at least "now" + 1 second
         local rollspeed_impact = target_attitude.rollspeed * (target_attitude.delay_ms + 1000)
         local target_turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * math.tan(target_attitude.roll + rollspeed_impact))

         turning = true
         -- predict the roll in 1s from now and use that based on rollspeed
         -- need some more maths to convert a roll angle into a turn angle - from Mission Planner:
         -- target_turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * math.tan(target_attitude.roll + target_attitude.rollspeed))
         local target_tangent_angle = wrap_360(math.deg(math.pi/2.0 - vehicle_airspeed / target_turn_radius))

         angle_adjustment = target_tangent_angle * 0.6
         -- if the roll direction is the same as the y offset, then we are turning on the "inside" (a tight turn) 
         if (target_attitude.roll < 0 and foll_ofs_y < 0) or
            (target_attitude.roll > 0 and foll_ofs_y > 0) then
            tight_turn = true
         end

         -- if the roll direction is the same as the rollspeed then we are heading into a turn, otherwise we are finishing a turn
         if foll_ofs_y == 0 or
            (target_attitude.roll < 0 and target_attitude.rollspeed < 0) or
            (target_attitude.roll > 0 and target_attitude.rollspeed > 0) then
            turn_starting = true
            target_angle = wrap_360(target_angle - angle_adjustment)
            desired_heading = wrap_360(target_heading - angle_adjustment)
            -- push the target heading back because it hasn't figured out we are turning yet
            save_target_heading1 = save_target_heading2
         end

         -- calculate the path/distance around the turn for the lead and follow vehicle so we can slow down or speed up
         -- depending on which is flying the shorter path
         local turn_radius = target_turn_radius + foll_ofs_y
         if tight_turn then
            turn_radius = target_turn_radius - foll_ofs_y
         end
         -- theta = l/r - i.e. the angle of the arc is the length of the arc divided by the radius
         local theta = target_airspeed / target_turn_radius
         -- now calculate what my airspeed would need to be to match the target, given I'm flying an arc with a different radius
         local my_airspeed = theta * turn_radius
         if my_airspeed > 0 and my_airspeed > airspeed_min and target_airspeed > 0 then 
            turning_airspeed_ratio = constrain(my_airspeed / target_airspeed, 0.0, 2.0)
         end
      end
   end

   -- don't count it as close if the heading is diverging (i.e. the target plane has overshot the target so extremely that it's pointing in the wrong direction)
   local too_close = (projected_distance > 0) and (math.abs(projected_distance) < close_distance) and (offset_angle < OVERSHOOT_ANGLE)

   -- we've overshot if 
   -- the distance to the target location is not too_close but will be hit in DISTANCE_LOOKAHEAD_SECONDS (projected_distance)
   -- or the distance to the target location is already negative AND the target is very close OR 
   -- the angle to the target plane is effectively backwards 
   local overshot = not too_close and (
                     (projected_distance < 0 or xy_dist < 0) and
                     (math.abs(xy_dist or 0.0) < close_distance)
                                       or offset_angle > OVERSHOOT_ANGLE
                                       )

   if overshot or too_close or (too_close_follow_up > 0) then
      if too_close_follow_up > 0 then
         too_close = true
         too_close_follow_up = too_close_follow_up - 1
      else
         too_close_follow_up = 10
      end
   else
      too_close_follow_up = 0
   end

   local target_altitude = 0.0
   local frame_type_log = foll_alt_type

   if altitude_override ~= 0 then
      target_altitude = altitude_override
      frame_type_log = -1
   elseif target_location_offset ~= nil then
      -- change the incoming altitude frame from the target_vehicle to the frame this vehicle wants
      target_location_offset:change_alt_frame(foll_alt_type)
      target_altitude = target_location_offset:alt() * 0.01
   end

   local mechanism = 0 -- for logging 1: position/location 2:heading
   local close = (math.abs(projected_distance) < close_distance)
   local too_wide = (math.abs(target_distance_rotated:y()) > (close_distance/4) and not turning)

   -- xy_dist < 3.0 is a special case because mode_guided will try to loiter around the target location if within 2m
   -- target_heading - vehicle_heading catches the circumstance where the target vehicle is heaidng in completely the opposite direction
   if math.abs(xy_dist or 0.0) < 10.0 or
         ((turning and ((tight_turn and turn_starting) or use_wide_turns or foll_ofs_y == 0)) or -- turning 
         ((close or overshot) and not too_wide) -- we are very close to the target
         ) then
      set_vehicle_heading({heading = desired_heading})
      set_vehicle_target_altitude({alt = target_altitude, frame = foll_alt_type}) -- pass altitude in meters (location has it in cm)
      mechanism = 2 -- heading - for logging
   elseif target_location_offset ~= nil then
      set_vehicle_target_location({lat = target_location_offset:lat(),
                                    lng = target_location_offset:lng(),
                                    alt = target_altitude,
                                    frame = foll_alt_type,
                                    yaw = foll_ofs_y})
      mechanism = 1  -- position/location - for logging
   end

   -- dv = interim delta velocity based on the pid controller using projected_distance per loop as the error (we want distance == 0)
   local dv_error = projected_distance * REFRESH_RATE
   -- re-project the distance error based on the turning angle
   -- if (turning and (tight_turn and turn_starting)) and math.abs(offset_angle) > TURNING_ANGLE  and turning_airspeed_ratio > 0 then
   if turning_airspeed_ratio > 0 and turning_airspeed_ratio < 2.0 then
      dv_error = dv_error * turning_airspeed_ratio
   end
   local airspeed_new = vehicle_airspeed
   local airspeed_error = (target_airspeed - vehicle_airspeed)
   local dv = 0.0
   if dv_error ~= nil then
      dv = pid_controller_distance.update(airspeed_error, dv_error)
      airspeed_new = pid_controller_velocity.update(vehicle_airspeed, dv)

      set_vehicle_speed({speed = constrain(airspeed_new, airspeed_min, airspeed_max)})
   end

   local log_too_close = 0
   local log_too_close_follow_up = 0
   local log_overshot = 0
   if too_close then
         log_too_close = 1
   end
   if too_close_follow_up then 
      log_too_close_follow_up = 1
   end
   if overshot then
      log_overshot = 1
   end
   logger:write("ZPF1",'Dst,DstP,DstE,AspT,Asp,AspD,AspE,AspO,TrnR,Mech,Cls,CF,OS','fffffffffBBBB','mmmnnnnnn----','-------------',
                  xy_dist,
                  projected_distance,
                  dv_error,
                  target_airspeed,
                  vehicle_airspeed,
                  dv,
                  airspeed_error,
                  airspeed_new,
                  turning_airspeed_ratio,
                  mechanism, log_too_close, log_too_close_follow_up, log_overshot
               )
   logger:write("ZPF2",'AngT,AngO,Alt,AltT,AltFrm,HdgT,Hdg,HdgP,HdgO','ffffbffff','ddmm-dddd','---------',
                  target_angle,
                  offset_angle,
                  current_altitude,
                  target_altitude,
                  frame_type_log,
                  target_heading,
                  vehicle_heading,
                  pre_roll_target_heading,
                  desired_heading
               )
end

-- wrapper around update(). This calls update() at 1/REFRESH_RATE Hz
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
   local success, err = pcall(update)

   if not success then
      gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
   end
   return protected_wrapper, 1000 * REFRESH_RATE
end

local function delayed_startup()
   gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
   return protected_wrapper()
end

-- start running update loop - waiting 20s for the AP to initialize
if FWVersion:type() == 3 then
   if arming:is_armed() then
      return delayed_startup, 1000
   else
      return delayed_startup, 20000
   end
else
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
