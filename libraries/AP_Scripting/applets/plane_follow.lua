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
   Support follow "mode" in plane. Theis will actually use GUIDED mode with 
   a scripting switch to allow guided to track the vehicle id in FOLL_SYSID
   Uses the AP_Follow library so all of the existing FOLL_* parameters are used
   as documented for Copter, + add 3 more for this script
   ZPF_EXIT_MODE - the mode to switch to when follow is turned of using the switch
   ZPF_FAIL_MODE - the mode to switch to if the target is lost
   ZPF_TIMEOUT - number of seconds to try to reaquire the target after losing it before failing
   ZPF_OVRSHT_DEG - if the target is more than this many degrees left or right, assume an overshoot
   ZPR_TURN_DEG - if the target is more than this many degrees left or right, assume it's turning
--]]

SCRIPT_VERSION = "4.6.0-033"
SCRIPT_NAME = "Plane Follow"
SCRIPT_NAME_SHORT = "PFollow"

-- FOLL_ALT_TYPE and Mavlink FRAME use different values 
ALT_FRAME = { GLOBAL = 0, RELATIVE = 1, TERRAIN = 3}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = {GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3,  GLOBAL_TERRAIN_ALT = 10}
MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1, DEFAULT = 2} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

DISTANCE_LOOKAHEAD_SECONDS = 5

local current_location = ahrs:get_location()
local ahrs_eas2tas = ahrs:get_EAS2TAS()
local windspeed_vector = ahrs:wind_estimate()

local now = millis():tofloat() * 0.001
local now_too_close = now
local now_overshot = now
local now_target_distance = now
local now_distance = now
local now_results = now
local now_airspeed = now
local now_target_airspeed = now
local now_lost_target = now
local now_target_heading = now
local follow_enabled = false
local too_close_follow_up = 0
local lost_target_countdown = LOST_TARGET_TIMEOUT
local save_target_heading = {0.0, 0.0, 0.0}
local save_target_heading1 = -400.0
local save_target_heading2 = -400.0
local tight_turn = false

local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "ZPF_"
local PARAM_TABLE_KEY2 = 121
local PARAM_TABLE_PREFIX2 = "ZPF2_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end
-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param2(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY2, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX2 .. name)
end
-- setup follow mode specific parameters- second tranche
assert(param:add_table(PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2, 10), 'could not add param table')

-- This uses the exisitng FOLL_* parameters and just adds a couple specific to this script
-- but because most of the logic is already in AP_Follow (called by binding to follow:) there
-- is no need to access them in the scriot

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = Parameter('FOLL_ALT_TYPE')
FOLL_SYSID = Parameter('FOLL_SYSID')
FOLL_OFS_Y = Parameter('FOLL_OFS_Y')
local foll_sysid = FOLL_SYSID:get() or -1
local foll_ofs_y = FOLL_OFS_Y:get() or 0
local foll_alt_type = FOLL_ALT_TYPE:get() or ALT_FRAME.GLOBAL


-- Add these ZPF_ parameters specifically for this script
--[[
  // @Param: ZPF_FAIL_MODE
  // @DisplayName: Plane Follow lost target mode
  // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX). 
  // @User: Standard
--]]
ZPF_FAIL_MODE = bind_add_param('FAIL_MODE', 1, FLIGHT_MODE.LOITER)

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
    // @DisplayName: Plane Follow Scripting Timeout
    // @Description: How long to try re-aquire a target if lost
    // @Range: 0 30
    // @Units: seconds
--]]
ZPF_TIMEOUT = bind_add_param("TIMEOUT", 4, 10)

--[[
    // @Param: ZPF_OVRSHT_DEG
    // @DisplayName: Plane Follow Scripting Overshoot Angle
    // @Description: If the target is greater than this many degrees left or right, assume an overshoot 
    // @Range: 0 180
    // @Units: degrees
--]]
ZPF_OVRSHT_DEG = bind_add_param("OVRSHT_DEG", 5, 75)

--[[
    // @Param: ZPF_TURN_DEG
    // @DisplayName: Plane Follow Scripting Turn Angle
    // @Description: If the target is greater than this many degrees left or right, assume it's turning
    // @Range: 0 180
    // @Units: degrees
--]]
ZPF_TURN_DEG = bind_add_param("TURN_DEG", 6, 15)

--[[
    // @Param: ZPF_DIST_CLOSE
    // @DisplayName: Plane Follow Scripting Close Distance
    // @Description: When closer than this distance assume we track by heading
    // @Range: 0 100
    // @Units: meters
--]]
ZPF_DIST_CLOSE = bind_add_param("DIST_CLOSE", 7, 50)

--[[
    // @Param: ZPF_WIDE_TURNS
    // @DisplayName: Plane Follow Scripting Wide Turns
    // @Description: Use wide turns when following a turning target. Alternative is "cutting the corner"
    // @Range: 0 1
    // @Units: boolean
--]]
ZPF_WIDE_TURNS = bind_add_param("WIDE_TURNS", 8, 1)

--[[
    // @Param: ZPF_ALT
    // @DisplayName: Plane Follow Scripting Altitude Override
    // @Description: When non zero, this altitude value (in FOLL_ALT_TYPE frame) overrides the value sent by the target vehicle
    // @Range: 0 1000
    // @Units: meters
--]]
ZPF_ALT_OVR = bind_add_param("ALT_OVR", 9, 0)

--[[
    // @Param: ZPF_SPD_P
    // @DisplayName: Plane Follow Scripting speed P gain
    // @Description: P gain for the speed PID controller
    // @Range: 0 1
--]]
ZPF2_SPD_P = bind_add_param2("SPD_P", 1, 0.2)

--[[
    // @Param: ZPF_SPD_I
    // @DisplayName: Plane Follow Scripting speed I gain
    // @Description: I gain for the speed PID controller
    // @Range: 0 1
--]]
ZPF2_SPD_I = bind_add_param2("SPD_I", 2, 0.2)

--[[
    // @Param: ZPF_SPD_D
    // @DisplayName: Plane Follow Scripting speed D gain
    // @Description: D gain for the speed PID controller
    // @Range: 0 1
--]]
ZPF2_SPD_D = bind_add_param2("SPD_D", 3, 0.05)

REFRESH_RATE = 0.05   -- in seconds, so 20Hz
LOST_TARGET_TIMEOUT = (ZPF_TIMEOUT:get() or 10) / REFRESH_RATE
OVERSHOOT_ANGLE = ZPF_OVRSHT_DEG:get() or 75.0
TURNING_ANGLE = ZPF_TURN_DEG:get() or 20.0

AIRSPEED_MIN = Parameter('AIRSPEED_MIN')
AIRSPEED_MAX = Parameter('AIRSPEED_MAX')
AIRSPEED_CRUISE = Parameter('AIRSPEED_CRUISE')
WP_LOITER_RAD = Parameter('WP_LOITER_RAD')
WINDSPEED_MAX = Parameter('AHRS_WIND_MAX')

local airspeed_max = AIRSPEED_MAX:get() or 25.0
local airspeed_min = AIRSPEED_MIN:get() or 12.0
local airspeed_cruise = AIRSPEED_CRUISE:get() or 18.0
local windspeed_max = WINDSPEED_MAX:get() or 100.0

--[[
create a NaN value
--]]
local function NaN()
   return 0/0
end
local function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

--local speedpi = require("speedpi")
--local speed_controller = speedpi.speed_controller(0.15, 0.15, 2.5, airspeed_min, airspeed_max)
local speedpid = require("speedpid")
--[[
local speed_controller_pid = speedpid.speed_controller(ZPF2_SPD_P:get() or 0.2,
                                          ZPF2_SPD_I:get() or 0.2,
                                          ZPF2_SPD_D:get() or 0.05,
                                          1.5, airspeed_min, airspeed_max)
--]]
local pid_controller_distance = speedpid.speed_controller(ZPF2_SPD_P:get() or 0.2,
                                                            ZPF2_SPD_I:get() or 0.2,
                                                            ZPF2_SPD_D:get() or 0.05,
                                                            5.0, airspeed_min - airspeed_max, airspeed_max - airspeed_min)

local pid_controller_velocity = speedpid.speed_controller(ZPF2_SPD_P:get() or 0.2,
                                                            ZPF2_SPD_I:get() or 0.2,
                                                            ZPF2_SPD_D:get() or 0.05,
                                                            5.0, airspeed_min, airspeed_max)


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

-- set_vehicle_target_altitude() Parameters
-- target.alt = new target altitude in meters
-- target.frame = Altitude frame MAV_FRAME, it's very important to get this right!
-- target.alt = altitude in meters to acheive
-- target.speed = z speed of change to altitude (1000.0 = max)
local function set_vehicle_target_altitude(target)
   local home_location = ahrs:get_home()
   local speed = target.speed or 1000.0 -- default to maximum z acceleration
   if target.alt == nil then
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": set_vehicle_target_altitude no altiude")
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
   local heading_accel = heading.accel or 0.0

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
-- target.yaw - not really yaw - it's the loiter direction -1 = CCW, 1 = CW NaN = default
-- target.lat - latitude in decimal degrees
-- target.lng - longitude in decimal degrees
-- target.alt - target alitude in meters
local function set_vehicle_target_location(target)
   local home_location = ahrs:get_home()
   local radius = target.radius or 2.0
   local angle = target.turning_angle or 0
   local yaw = target.yaw or 1
   -- If we are on the right side of the vehicle make sure any loitering is CCW (moves away from the other plane)
   if foll_ofs_y > 0 or angle < 0 then
      yaw = -yaw
   end

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

local last_follow_active_state = rc:get_aux_cached(ZPF_ACT_FN:get())

--[[
   return true if we are in a state where follow can apply
--]]
local reported_target = true
local function follow_active()
   local mode = vehicle:get_mode()

   if mode == FLIGHT_MODE.GUIDED then
      if follow_enabled then
        if follow:have_target() then
            reported_target = true
        else
            if reported_target then
               gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": no target: " .. follow:get_target_sysid())
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
            vehicle:set_mode(ZPF_EXIT_MODE:get())
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

--[[ probably not required
-- calculate difference between the target heading and the following vehicle heading
local function follow_target_angle(target_heading_follow, target_location_follow)

   -- find the current location of the vehicle and calculate the bearing to its current target
   local follow_heading = math.deg(current_location:get_bearing(target_location_follow))

   local angle_target_return = wrap_180(target_heading_follow - follow_heading)
   
   --if follow_heading > vehicle_heading then
   --   angle_target_return = target_heading - vehicle_heading
   --end
   --angle_target_return = wrap_180(angle_target_return)
   
   return angle_target_return
end
--]]

local function calculate_airspeed_from_groundspeed(velocity_vector)
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
   current_location = ahrs:get_location()
   ahrs_eas2tas = ahrs:get_EAS2TAS()
   windspeed_vector = ahrs:wind_estimate()

   follow_check()
   if not follow_active() then
    return
   end

   -- set the target frame as per user set parameter - this is fundamental to this working correctly
   local wp_loiter_rad = WP_LOITER_RAD:get()
   local close_distance = ZPF_DIST_CLOSE:get() or airspeed_cruise * 2.0
   local long_distance = close_distance * 4.0
   local fail_mode = ZPF_FAIL_MODE:get() or FLIGHT_MODE.QRTL
   local exit_mode = ZPF_EXIT_MODE:get() or FLIGHT_MODE.LOITER
   local use_wide_turns = ZPF_WIDE_TURNS:get() or 1
   local altitude_override = ZPF_ALT_OVR:get() or 0

   LOST_TARGET_TIMEOUT = (ZPF_TIMEOUT:get() or 10) / REFRESH_RATE
   OVERSHOOT_ANGLE = ZPF_OVRSHT_DEG:get() or 75.0
   TURNING_ANGLE = ZPF_TURN_DEG:get() or 20.0
   foll_ofs_y = FOLL_OFS_Y:get() or 0
   foll_alt_type = FOLL_ALT_TYPE:get() or ALT_FRAME.GLOBAL

   --[[
      get the current navigation target. 
   --]]
   local target_location -- = Location()     of the target
   local target_location_offset -- Location  of the target with FOLL_OFS_* offsets applied
   local target_velocity -- = Vector3f()     -- current velocity of lead vehicle
   local target_velocity_offset -- Vector3f  -- velocity to the offset target_location_offset
   local target_distance -- = Vector3f()     -- vector to lead vehicle
   local target_distance_offsets             -- vector to the target taking offsets into account
   local xy_dist = 0.0                       -- distance to target with offsets in meters
   local target_heading = 0.0                -- heading of the target vehicle
   local heading_to_target                   -- heading of the follow vehicle to the target with offsets

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
   --]]

   target_distance, target_distance_offsets,
      target_velocity, target_velocity_offset,
      target_location, target_location_offset, 
      xy_dist, heading_to_target = follow:get_target_info()
   target_heading = follow:get_target_heading_deg() or -400

   -- if we lose the target wait for LOST_TARGET_TIMEOUT seconds to try to reaquire it
   if target_location == nil or target_velocity == nil or target_distance_offsets == nil or current_target == nil then
      lost_target_countdown = lost_target_countdown - 1
      if lost_target_countdown <= 0 then
         follow_enabled = false
         vehicle:set_mode(fail_mode)
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": follow: " .. FOLL_SYSID:get() .. " FAILED")
         return
      end
      if math.floor(now) ~= math.floor(now_lost_target) then
         now_lost_target = millis():tofloat() * 0.001
         gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": lost target: " .. FOLL_SYSID:get() .. " count: " .. lost_target_countdown)
      end
      return
   else
      -- have a good target so reset the countdown 
      lost_target_countdown = LOST_TARGET_TIMEOUT
   end

   -- target_velocity from MAVLink (via AP_Follow) is groundspeed, need to convert to airspeed, 
   -- we can only assume the windspeed for the target is the same as the chase plane
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
   local target_airspeed = calculate_airspeed_from_groundspeed(target_velocity)

   --[[
   if math.floor(now) ~= math.floor(now_target_airspeed) then
      now_target_airspeed = millis():tofloat() * 0.001
      gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": wind %.1f target vel: %.1f asp: %.1f", windspeed_vector:length(), target_velocity:length(), target_airspeed) )
   end
   --]]

   -- target_angle is the difference between the current heading of the target vehicle and the follow vehicle heading to the target_location
   -- local target_angle = follow_target_angle(target_heading, target_location)
   -- offset_angle is the difference between the current heading of the follow vehicle and the target_location (with offsets)
   --local vehicle_heading = math.deg(current_location:get_bearing(target_location))
   local vehicle_heading = math.abs(wrap_360(math.deg(ahrs:get_yaw())))
   local follow_heading = heading_to_target -- math.deg(current_location:get_bearing(target_location_offset))
   local offset_angle = wrap_180(vehicle_heading - follow_heading)

   -- rotate the target_distance_offsets in NED to the same direction has the follow vehicle, we use this below
   local target_distance_rotated = target_distance_offsets:copy()
   target_distance_rotated:rotate_xy(math.rad(vehicle_heading))
   --[[
   if math.floor(now_target_distance) ~= math.floor(now) then
      gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": distance %.0f y %.0f ", target_distance_offsets:length(), target_distance_offsets:y()))
      now_target_distance = millis():tofloat() * 0.001
   end
   --]]
   

   -- target angle is the difference between the heading of the target and what its heading was 2 seconds ago
   local target_angle = 0.0
   if target_heading ~= nil and target_heading > -400 then
      -- want the greatest angle of we might have turned
      local angle_diff1 = wrap_180(math.abs(save_target_heading1 - target_heading))
      local angle_diff2 = wrap_180(math.abs(save_target_heading2 - target_heading))
      if angle_diff2 > angle_diff1 then
         target_angle = angle_diff2
      else
         target_angle = angle_diff1
      end
      if math.abs(save_target_heading1 - target_heading ) > 10.0 or math.abs( save_target_heading2- target_heading ) > 10.0  then
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": DIFFERENT tgt hdg:%.0fangle: %.0f", target_heading, target_angle))
      end
      -- remember the target heading from 2 seconds ago, so we can tell if it is turning or not
      if (now - now_target_heading) > 1 then
         save_target_heading2 = save_target_heading1
         save_target_heading1 = target_heading
         now_target_heading = now
      end
   end

   -- if the target vehicle is starting to roll we need to pre-empt a turn is coming
   -- this is fairly simplistic and could probably be improved
   -- got the code from Mission Planner - this is how it calculates the turn radius
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
   local turn_starting = false
   local target_attitude = mavlink_attitude_receiver.get_attitude(foll_sysid)
   if target_attitude ~= nil then
      if math.abs(target_attitude.roll) > 0.1 or math.abs(target_attitude.rollspeed) > 1 then
         local turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * math.tan(target_attitude.roll))
         local angle_adjustment = 60 * target_attitude.roll
         -- predict the roll in 2s from now and use that.
         -- need some more maths to convert a roll angle into a turn angle
         turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * math.tan(target_attitude.roll + target_attitude.rollspeed*2.0))
         local tangent_angle = wrap_360(math.deg(math.pi/2.0 - vehicle_airspeed / turn_radius))
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": Att roll:%.2f rollspd: %.2f rad %.2f tan %.2f", target_attitude.roll, target_attitude.rollspeed, turn_radius, tangent_angle))

         angle_adjustment = tangent_angle
         target_angle = wrap_360(target_angle + angle_adjustment)
         target_heading = wrap_360(target_heading + angle_adjustment)
         -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": adjust:%.2f angle: %.2f heading %.2f", angle_adjustment, target_angle, target_heading))
         -- push the target heading back because it hasn't figured out we are turning yet
         save_target_heading1 = save_target_heading2
      end
      -- if the roll direction is the same as the y offset, then we are turning on the "inside" (a tight turn) 
      if (target_attitude.roll < 0 and foll_ofs_y < 0) or
         (target_attitude.roll > 0 and foll_ofs_y > 0) then
            tight_turn = true
         else
            tight_turn = false
      end

      -- if the roll direction is the same as the rollspeed then we are heading into a turn, otherwise we are finishing a turn
      if foll_ofs_y == 0 or
         (target_attitude.roll < 0 and target_attitude.rollspeed < 0) or
         (target_attitude.roll > 0 and target_attitude.rollspeed > 0) then
         turn_starting = true
      end
   end

   -- default the desired heading to the target heading (adjusted for projected turns) - we might change this below
   local desired_heading = target_heading

   local desired_airspeed = target_airspeed
   local airspeed_difference = vehicle_airspeed - target_airspeed

   -- distance seem to be out by about 1s at approximately current airspeed just eyeballing it.
   xy_dist = math.abs(xy_dist) - vehicle_airspeed * 0.92
   -- xy_dist will always be a positive value. To get -v to represent overshoot, use the offset_angle
   -- to decide if the target is behind
   if (math.abs(xy_dist) < long_distance) and (math.abs(offset_angle) > OVERSHOOT_ANGLE) then
      xy_dist = -xy_dist
   end

   -- projected_distance is how far off the target we expect to be if we maintain the current airspeed for DISTANCE_LOOKAHEAD_SECONDS
   local projected_distance = xy_dist - airspeed_difference * DISTANCE_LOOKAHEAD_SECONDS

   -- don't count it as close if the heading is diverging (i.e. the target plane has overshot the target so extremely that it's pointing in the wrong direction)
   local too_close = (projected_distance > 0) and (math.abs(projected_distance) < close_distance) and (offset_angle < OVERSHOOT_ANGLE)
   if too_close then
      if math.floor(now_too_close) ~= math.floor(now) then
         now_too_close = millis():tofloat() * 0.001
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT )
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format("TOO CLOSE: xy_dist: %.0f close dist %.0f target angle %.0f offset %.0f", xy_dist, close_distance, target_angle, offset_angle))
      end
   end

   -- we've overshot if 
   -- the distance to the target location is not too_close but will be hit in DISTANCE_LOOKAHEAD_SECONDS (projected_distance)
   -- or the distance to the target location is already negative AND the target is very close OR 
   -- the angle to the target plane is effectively backwards 
   local overshot = not too_close and ((projected_distance < 0 or xy_dist < 0) and (math.abs(xy_dist) < close_distance) 
                                       or offset_angle > OVERSHOOT_ANGLE
                                       )

   --[[
   if overshot then
      if math.floor(now_overshot) ~= math.floor(now) then
         now_overshot = millis():tofloat() * 0.001
         if projected_distance < 0 or xy_dist < 0 then
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": OVERSHOT reason xy_dist: %.0f projected: %.1f", xy_dist, projected_distance))
         end
         if target_angle >= OVERSHOOT_ANGLE then
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": OVERSHOT reason angle: %.0f overshoot: %.1f", target_angle, OVERSHOOT_ANGLE))
         end
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": OVERSHOT reason distance: %.0f project %0.f close: %.0f", xy_dist, projected_distance, close_distance * 2))
      end
   end
   --]]

   local distance_error = constrain(math.abs(projected_distance) / (close_distance * DISTANCE_LOOKAHEAD_SECONDS), 0.1, 1.0)
   local speed_factor = 0.0 -- how agressively to scale the speed 0 - 1 least to most 0 = maintain current speed -ve to slow down
   if overshot or too_close or too_close_follow_up > 0 then
      desired_heading = target_heading

      -- now calculate what airspeed we will need to fly for a few seconds to catch up the projected distance
      -- need to slow down dramatically. First we try to match the TARGET airspeed unless special cases kick in
      if overshot then
         -- either we have actually overshot or we are likely to overshot (projected) very soon
         -- but if we have overshot actual xy_dist, but projected is not overshot then we are already recovering
         --[[if xy_dist < 0 and projected_distance < 0 then -- actual and projected overshoot
            speed_factor = -0.12
         elseif xy_dist > 0 and projected_distance < 0 then -- actual ok, but projected overshoot
            speed_factor = -0.07
         elseif xy_dist < 0 and projected_distance > 0 then -- we are probably recovering
            speed_factor = -0.02
         else -- we get here if we used the OVERSHOOT_ANGLE to decide we overshot. The distances are fine, so don't change speed
            speed_factor = 0.0
         end
         if math.floor(now_airspeed) ~= math.floor(now) then
            now_airspeed = millis():tofloat() * 0.001
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": OVERSHOT airspeed projected %.0f desired speed: %.1f error: %.3f", projected_distance, desired_airspeed, distance_error) )
         end
         --]]
      else
         too_close = true
      
         --[[if xy_dist < 0 and projected_distance > 0 then
            speed_factor = 0.07 -- speed up we are going too slow
         elseif xy_dist > 0 and projected_distance < 0 then 
            speed_factor = -0.04 -- slow down so we don't overshoot
         elseif xy_dist < projected_distance then -- getting further away need to speed up a bit
               speed_factor = 0.10
         elseif math.abs(xy_dist) < close_distance/4 and math.abs(projected_distance) < close_distance/4 then
            speed_factor = 0.0
         else -- both xy_dist and projected_distance must bhe > 0 so too far away, need to speed up            
            speed_factor = 0.11
         end
         if math.floor(now_airspeed) ~= math.floor(now) then
            now_airspeed = millis():tofloat() * 0.001
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": TOOCLOSE dst %.0f prj %0.f asp new: %.1f err: %.3f", xy_dist, projected_distance, desired_airspeed, distance_error) )
         end
         --]]
      end
      if too_close_follow_up > 0 then
         too_close_follow_up = too_close_follow_up - 1
      else
         too_close_follow_up = 10
      end
      if math.floor(now_distance) ~= math.floor(now) then
         local where = "CLOSE"
         if not too_close then
            where = "OVERSHOT"
         end
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": %s distance %.0f projected %.0f ", where, xy_dist, projected_distance))
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": %s target %.1f desired airspeed %.1f", where, target_airspeed, desired_airspeed))
         now_distance = millis():tofloat() * 0.001
      end
      --[[
      if speed_factor == 0 then
         desired_airspeed = vehicle_airspeed
      elseif speed_factor < 0 then -- go slower - down to airspeed_min 
         desired_airspeed = vehicle_airspeed + (((vehicle_airspeed - airspeed_min) * distance_error) * speed_factor)
      else -- go faster - up to airspeed_max
         desired_airspeed = vehicle_airspeed + (((airspeed_max - vehicle_airspeed) * distance_error) * speed_factor)
      end
      --]]
   else
      too_close_follow_up = 0
      -- AP_Follow doesn't speed up enough if wa are a long way from the target
      -- what we want is a. to figure out if we are a long way from the target. Basically if our current airspeed will not catch up in DISTANCE_LOOKAHEAD_SECONDS
      -- be calculate an increasing target speed up to AIRSPEED_MAX based on the projected distance from the target.
      --[[
      if projected_distance > 0 then
         local incremental_speed = projected_distance / DISTANCE_LOOKAHEAD_SECONDS
         desired_airspeed = constrain(target_airspeed + incremental_speed, airspeed_min, airspeed_max)
      else
         desired_airspeed = target_airspeed + 0.0
      end
      ]]--
      --desired_airspeed = vehicle_airspeed + ((airspeed_max - vehicle_airspeed) * distance_error)
      --[[
      if math.floor(now_distance) ~= math.floor(now) then
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": LONG distance %.0f projected %.0f close %.0f", xy_dist, projected_distance, close_distance))
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": LONG target %.1f desired airspeed %.1f", target_airspeed, desired_airspeed))
         now_distance = millis():tofloat() * 0.001
      end
      --]]
   end

   local old_location = ahrs:get_location()
   local current_altitude = 0.0
   if old_location ~= nil then
      current_altitude = old_location:alt() * 0.01
   end

   local new_target_location = old_location:copy()
   local target_altitude = 0.0
   local frame_type_log = foll_alt_type

   if altitude_override ~= 0 then
      target_altitude = altitude_override
      frame_type_log = -1
   else
      -- change the incoming altitude frame from the target_vehicle to the frame this vehicle wants
      target_location_offset:change_alt_frame(foll_alt_type)
      target_altitude = target_location_offset:alt() * 0.01
   end
   new_target_location:lat(target_location_offset:lat())
   new_target_location:lng(target_location_offset:lng())
   new_target_location:alt(target_location_offset:alt()) -- location uses cm for altitude

   local mechanism = 1 -- for logging 1: position/location 2:heading
   local normalized_distance = math.abs(projected_distance)
   local close = (normalized_distance < close_distance)
   local turning = (normalized_distance < long_distance and math.abs(target_angle) > TURNING_ANGLE) -- and turn_starting
   local too_wide = (math.abs(target_distance_rotated:y()) > (close_distance/5) and not turning)
   local wide_turn = not tight_turn

   -- xy_dist < 3.0 is a special case because mode_guided will try to loiter around the target location if within 2m
   -- target_heading - vehicle_heading catches the circumstance where the target vehicle is heaidng in completely the opposite direction
   if math.abs(xy_dist) < 3.0 or
         ((turning and ((tight_turn and turn_starting) or use_wide_turns or foll_ofs_y == 0)) or -- turning 
         ((close or overshot) and not too_wide) -- we are very close to the target
         --math.abs(target_heading - vehicle_heading) > 135) -- the target is going the other way
         ) then
      set_vehicle_heading({heading = desired_heading})
      set_vehicle_target_altitude({alt = target_altitude, frame = foll_alt_type}) -- pass altitude in meters (location has it in cm)
      mechanism = 2 -- heading - for logging
   else
      set_vehicle_target_location({lat = target_location_offset:lat(),
                                    lng = target_location_offset:lng(),
                                    alt = target_altitude,
                                    frame = foll_alt_type,
                                    turning_angle = target_angle})
      mechanism = 1  -- position/location - for logging
   end

   --[[
   if too_wide or
      (not turning and
         (normalized_distance > close_distance and 
            (turning or
            normalized_distance > wp_loiter_rad or 
            math.abs(offset_angle) < OVERSHOOT_ANGLE))) then
--   (math.abs(xy_dist) <= wp_loiter_rad and math.abs(target_angle) < TURNING_ANGLE)
--      or math.abs(xy_dist) > close_distance then
      set_vehicle_target_location({lat = target_location_offset:lat(),
                                    lng = target_location_offset:lng(),
                                    alt = target_altitude,
                                    frame = altitude_frame,
                                    turning_angle = target_angle})
      --if normalized_distance > wp_loiter_rad then
      --   gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" normalized: %.0f projected %.0f angle target %.1f offset %.1f", normalized_distance, projected_distance, target_angle, offset_angle))
      --end
      --if math.abs(target_angle) > TURNING_ANGLE then
      --   gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" TURNING position: %.0f save heading %.0f angle target %.1f offset %.1f", target_heading, save_target_heading1, target_angle, offset_angle))
      --end
      mechanism = 1  -- position/location - for logging
   else
      --if math.abs(target_angle) > TURNING_ANGLE then
      --   desired_heading = target_heading
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" TURNING heading: %.0f save heading %.0f angle target %.1f offset %.1f", target_heading, save_target_heading1, target_angle, offset_angle))
      --end
      set_vehicle_heading({heading = desired_heading})
      set_vehicle_target_altitude({alt = target_altitude, frame = altitude_frame}) -- pass altitude in meters (location has it in cm)
      mechanism = 2 -- heading - for logging
   end
   ]]--
   --local airspeed_new = speed_controller.update(vehicle_airspeed, desired_airspeed - vehicle_airspeed)
   --local airspeed_new = speed_controller_pid.update(vehicle_airspeed, desired_airspeed - vehicle_airspeed)
   local dv = pid_controller_distance.update(target_airspeed - vehicle_airspeed, projected_distance)
   local airspeed_new = pid_controller_velocity.update(vehicle_airspeed, dv)
   if math.floor(now_results) ~= math.floor(now) then
      gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": dst %.1f prj %.1f asp %.1f NEW %.1f dv %.2f diff %.2f", xy_dist, projected_distance, vehicle_airspeed, airspeed_new, dv, target_airspeed - vehicle_airspeed))
      --gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format("vehicle x %.1f y %.1f length %.1f", vehicle_vector:x(), vehicle_vector:y(), vehicle_vector:length()))
      --gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format("target x %.1f y %.1f length %.1f", target_vector:x(), target_vector:y(), target_vector:length()))
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format("angle target %.1f offset %.1f hdg diff %.1f", target_angle, offset_angle, math.abs(wrap_180(target_heading - vehicle_heading))))
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": dst %.1f prj %.1f diff %.0f err %.3f ", xy_dist, projected_distance, distance, distance_error))
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": dst %.1f prj %.1f asp %.1f NEW %.1f fac %.2f err %.2f", xy_dist, projected_distance, desired_airspeed, airspeed_new, speed_factor, distance_error))
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": tgt hdg:%.0f veh hdg: %.0f des hdg %.0f ang tar %.0f", target_heading, vehicle_heading, desired_heading, target_angle ))
      --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": target alt: %.1f desired alt %.1f frame %d", target_location:alt() * 0.01, target_altitude, foll_alt_type ))
      --[[if turning then
         if tight_turn then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": TURNING TIGHT proj %.1f xy_dist %.0f rotated x %.1f y %.1f z %.1f ", projected_distance, xy_dist,  target_distance_rotated:x(), target_distance_rotated:y(), target_distance_rotated:z()))
         else
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": TURNING proj %.1f xy_dist %.0f rotated x %.1f y %.1f z %.1f ", projected_distance, xy_dist,  target_distance_rotated:x(), target_distance_rotated:y(), target_distance_rotated:z()))
         end
      else
         gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(":         proj %.1f xy_dist %.0f rotated x %.1f y %.1f z %.1f ", projected_distance, xy_dist,  target_distance_rotated:x(), target_distance_rotated:y(), target_distance_rotated:z()))
      end
      --]]
      if math.abs(xy_dist) < wp_loiter_rad and (math.abs(offset_angle) > OVERSHOOT_ANGLE) then
         --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" REVERSE: distance %.1f offset_angle %.1f ", xy_dist, offset_angle ))
      end
      now_results = millis():tofloat() * 0.001
   end

   set_vehicle_speed({speed = constrain(airspeed_new, airspeed_min, airspeed_max)})

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
   logger.write("ZPF1",'Dst,DstP,DstE,AspT,Asp,AspO,Mech,Cls,ClsF,OSht','ffffffBBBB','mmmnnn----','----------',
                  xy_dist,
                  projected_distance,
                  distance_error,
                  target_airspeed,
                  vehicle_airspeed,
                  airspeed_new,
                  mechanism, log_too_close, log_too_close_follow_up, log_overshot
               )
   logger.write("ZPF2",'AngT,AngO,Alt,AltT,AltFrm,HdgT,Hdg,HdgO','ffffbfff','ddmm-ddd','--------',
                  target_angle,
                  offset_angle,
                  current_altitude,
                  target_altitude,
                  frame_type_log,
                  target_heading,
                  vehicle_heading,
                  desired_heading
               )
end

-- wrapper around update(). This calls update() at 20Hz,
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

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )

-- start running update loop
return protected_wrapper()

