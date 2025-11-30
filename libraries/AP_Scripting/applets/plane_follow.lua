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
   FOLLP_EXIT_MODE - the mode to switch to when follow is turned of using the switch
   FOLLP_FAIL_MODE - the mode to switch to if the target is lost
   FOLLP_TIMEOUT - number of seconds to try to reaquire the target after losing it before failing
   FOLLP_OVRSHT_DEG - if the target is more than this many degrees left or right, assume an overshoot
   FOLLP_TURN_DEG - if the target is more than this many degrees left or right, assume it's turning
--]]

SCRIPT_VERSION = "4.7.0-073"
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

local now_ms = millis()
local now_target_heading_ms = now_ms
local now_follow_lost_ms = now_ms
local now_heading_ms = now_ms
local too_close_follow_up = 0
local save_target_heading1 = -400.0
local save_target_heading2 = -400.0
local tight_turn = false

local PARAM_TABLE_KEY = 123
local PARAM_TABLE_PREFIX = "FOLLP_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end
-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

-- This uses the exisitng FOLL_* parameters and just adds a couple specific to this script
-- but because most of the logic is already in AP_Follow (called by binding to follow:) there
-- is no need to access them in the scriot

-- we need these existing FOLL_ parametrs
FOLL_ALT_TYPE = Parameter('FOLL_ALT_TYPE')
FOLL_SYSID = Parameter('FOLL_SYSID')
FOLL_OFS_Y = Parameter('FOLL_OFS_Y')
local foll_sysid = FOLL_SYSID:get()
local foll_ofs_y = FOLL_OFS_Y:get()
local foll_alt_type = FOLL_ALT_TYPE:get()

-- Add these FOLLP_ parameters specifically for this script
--[[
  // @Param: FOLLP_FAIL_MODE
  // @DisplayName: Plane Follow lost target mode
  // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX). 
  // @User: Standard
--]]
FOLLP_FAIL_MODE = bind_add_param('FAIL_MODE', 1, FLIGHT_MODE.RTL)

--[[
  // @Param: FOLLP_EXIT_MODE
  // @DisplayName: Plane Follow exit mode
  // @Description: Mode to switch to when follow mode is exited normally
  // @User: Standard
--]]
FOLLP_EXIT_MODE = bind_add_param('EXIT_MODE', 2, FLIGHT_MODE.LOITER)

--[[
    // @Param: FOLLP_ACT_FN
    // @DisplayName: Plane Follow Scripting ActivationFunction
    // @Description: Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable
    // @Range: 300 307
--]]
FOLLP_ACT_FN = bind_add_param("ACT_FN", 3, 301)

--[[
    // @Param: FOLLP_TIMEOUT
    // @DisplayName: Plane Follow Telemetry Timeout
    // @Description: How long to try reaquire a target if telemetry from the lead vehicle is lost.
    // @Range: 0 30
    // @Units: s
--]]
FOLLP_TIMEOUT = bind_add_param("TIMEOUT", 4, 10)

--[[
    // @Param: FOLLP_OVRSHT_DEG
    // @DisplayName: Plane Follow Scripting Overshoot Angle
    // @Description: If the target is greater than this many degrees left or right, assume an overshoot 
    // @Range: 0 180
    // @Units: deg
--]]
FOLLP_OVRSHT_DEG = bind_add_param("OVRSHT_DEG", 5, 75)

--[[
    // @Param: FOLLP_TURN_DEG
    // @DisplayName: Plane Follow Scripting Turn Angle
    // @Description: If the target is greater than this many degrees left or right, assume it's turning
    // @Range: 0 180
    // @Units: deg
--]]
FOLLP_TURN_DEG = bind_add_param("TURN_DEG", 6, 15)

--[[
    // @Param: FOLLP_DIST_CLOSE
    // @DisplayName: Plane Follow Scripting Close Distance
    // @Description: When closer than this distance assume we track by heading
    // @Range: 0 100
    // @Units: m
--]]
FOLLP_DIST_CLOSE = bind_add_param("DIST_CLOSE", 7, 75)

--[[
    // @Param: FOLLP_WIDE_TURNS
    // @DisplayName: Plane Follow Scripting Wide Turns
    // @Description: Use wide turns when following a turning target. Alternative is "cutting the corner"
    // @Range: 0 1
--]]
FOLLP_WIDE_TURNS = bind_add_param("WIDE_TURNS", 8, 1)

--[[
    // @Param: FOLLP_ALT
    // @DisplayName: Plane Follow Scripting Altitude Override
    // @Description: When non zero, this altitude value (in FOLL_ALT_TYPE frame) overrides the value sent by the target vehicle
    // @Range: 0 1000
    // @Units: m
--]]
FOLLP_ALT_OVR = bind_add_param("ALT_OVR", 9, 0)

--[[
    // @Param: FOLLP_D_P
    // @DisplayName: Plane Follow Scripting distance P gain
    // @Description: P gain for the speed PID controller distance component
    // @Range: 0 1
--]]
FOLLP_D_P = bind_add_param("D_P", 10, 0.00025)

--[[
    // @Param: FOLLP_D_I
    // @DisplayName: Plane Follow Scripting distance I gain
    // @Description: I gain for the speed PID  distance component
    // @Range: 0 1
--]]
FOLLP_D_I = bind_add_param("D_I", 11, 0.00025)

--[[
    // @Param: FOLLP_D_D
    // @DisplayName: Plane Follow Scripting distance D gain
    // @Description: D gain for the speed PID controller distance component
    // @Range: 0 1
--]]
FOLLP_D_D = bind_add_param("D_D", 12, 0.00013)

--[[
    // @Param: FOLLP_V_P
    // @DisplayName: Plane Follow Scripting speed P gain
    // @Description: P gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
FOLLP_V_P = bind_add_param("V_P", 13, 0.00025)

--[[
    // @Param: FOLLP_V_I
    // @DisplayName: Plane Follow Scripting speed I gain
    // @Description: I gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
FOLLP_V_I = bind_add_param("V_I", 14, 0.00025)

--[[
    // @Param: FOLLP_V_D
    // @DisplayName: Plane Follow Scripting speed D gain
    // @Description: D gain for the speed PID controller velocity component
    // @Range: 0 1
--]]
FOLLP_V_D = bind_add_param("V_D", 15, 0.0005)

--[[
    // @Param: FOLLP_LkAHD
    // @DisplayName: Plane Follow Lookahead seconds
    // @Description: Time to "lookahead" when calculating distance errors
    // @Units: s
--]]
FOLLP_LKAHD = bind_add_param("LKAHD", 16, 3)

--[[
    // @Param: FOLLP_SIM_TLF_FN
    // @DisplayName: Plane Follow Simulate Telemetry fail function
    // @Description: Set this switch to simulate losing telemetry from the other vehicle
    // @Range: 300 307
--]]
FOLLP_SIM_TLF_FN = bind_add_param("SIM_TlF_FN", 17, 302)

--[[
    // @Param: FOLLP_XT_P
    // @DisplayName: Plane Follow crosstrack PID controller P term
    // @Description: P term for the crosstrack/heading PID controller
    // @Range: 0 1
--]]
FOLLP_XT_P = bind_add_param("XT_P", 20, 0.8)

--[[
    // @Param: FOLLP_XT_I
    // @DisplayName: Plane Follow crosstrack PID controller I term
    // @Description: I term for the crosstrack/heading PID controller
    // @Range: 0 1
--]]
FOLLP_XT_I = bind_add_param("XT_I", 21, 0.01)

--[[
    // @Param: FOLLP_XT_D
    // @DisplayName: Plane Follow crosstrack PID controller D term
    // @Description: D term for the crosstrack/heading PID controller
    // @Range: 0 1
--]]
FOLLP_XT_D = bind_add_param("XT_D", 22, 0.5)

--[[
    // @Param: FOLLP_XT_MAX
    // @DisplayName: Plane Follow crosstrack PID controller maximum correction
    // @Description: maximum correction retured by the crosstrack/heading PID controller
    // @Range: 0 1
    // @Units: deg
--]]
FOLLP_XT_MAX = bind_add_param("XT_MAX", 23, 45)

--[[
    // @Param: FOLLP_XT_I_MAX
    // @DisplayName: Plane Follow crosstrack PID controller maximum integral
    // @Description: maximum I applied the crosstrack/heading PID controller
    // @Range: 0 100
    // @Units: ms
--]]
FOLLP_XT_I_MAX = bind_add_param("XT_I_MAX", 24, 100)

--[[
    // @Param: FOLLP_REFRESH
    // @DisplayName: Plane Follow refresh rate 
    // @Description: refresh rate for Plane Follow updates
    // @Range: 0 0.2
    // @Units: s
--]]
FOLLP_REFRESH = bind_add_param("REFRESH", 25, 0.05)

local refresh_rate = FOLLP_REFRESH:get()
LOST_TARGET_TIMEOUT = FOLLP_TIMEOUT:get() / refresh_rate
OVERSHOOT_ANGLE = FOLLP_OVRSHT_DEG:get()
TURNING_ANGLE = FOLLP_TURN_DEG:get()
DISTANCE_LOOKAHEAD_SECONDS = FOLLP_LKAHD:get()

local lost_target_countdown = LOST_TARGET_TIMEOUT

local fail_mode = FOLLP_FAIL_MODE:get()
local exit_mode = FOLLP_EXIT_MODE:get()

local use_wide_turns = FOLLP_WIDE_TURNS:get()

--local simulate_telemetry_failed = false

AIRSPEED_MIN = Parameter('AIRSPEED_MIN')
AIRSPEED_MAX = Parameter('AIRSPEED_MAX')
WP_LOITER_RAD = Parameter('WP_LOITER_RAD')
WINDSPEED_MAX = Parameter('AHRS_WIND_MAX')

local airspeed_max = AIRSPEED_MAX:get()
local airspeed_min = AIRSPEED_MIN:get()
local windspeed_max = WINDSPEED_MAX:get()

-------------------------------------------------------------------------------
--- Utility Functions
-------------------------------------------------------------------------------

---@diagnostic disable-next-line:lowercase-global
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

---@diagnostic disable-next-line:lowercase-global
function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

---@diagnostic disable-next-line:lowercase-global
function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end


-------------------------------------------------------------------------------
--- PID Controllers
-------------------------------------------------------------------------------

local pid = require("pid")
local pid_controller_distance = pid.speed_controller(FOLLP_D_P:get(),
                                                            FOLLP_D_I:get(),
                                                            FOLLP_D_D:get(),
                                                            0.5, airspeed_min - airspeed_max, airspeed_max - airspeed_min)

local pid_controller_velocity = pid.speed_controller(FOLLP_V_P:get(),
                                                            FOLLP_V_I:get(),
                                                            FOLLP_V_D:get(),
                                                            0.5, airspeed_min, airspeed_max)

-- We need a PID controller to manage cross track errors
-- start of crosstrackpid {} class definition
local crosstrackpid = {}
crosstrackpid.__index = crosstrackpid

function crosstrackpid.new(kp, ki, kd, max_correction, integral_limit)
   local self = {}
   setmetatable(self, { __index = crosstrackpid })

   self.kp = kp or 0.8
   self.ki = ki or 0.01
   self.kd = kd or 0.5
   self.max_correction = max_correction -- degrees
   self.integral_limit = integral_limit -- m·s

   self.integral = 0
   self.last_error = 0

   return self
end

-- reset integrator to an initial value
function crosstrackpid.reset(self)
   if self == nil then
      self = setmetatable({}, { __index = crosstrackpid })
   end
   self.integral = 0
   self.last_error = 0
end

function crosstrackpid:compute(desired_track_heading, cross_track_error, dt)
   -- Derivative
   local error_rate = (cross_track_error - self.last_error) / dt

   -- Integral with clamp
   self.integral = self.integral + cross_track_error * dt
   if self.integral > self.integral_limit then self.integral = self.integral_limit end
   if self.integral < -self.integral_limit then self.integral = -self.integral_limit end

   -- PID correction (heading offset in degrees)
   local correction = self.kp * cross_track_error +
                     self.kd * error_rate +
                     self.ki * self.integral

   -- Clamp heading correction
   if correction > self.max_correction then correction = self.max_correction end
   if correction < -self.max_correction then correction = -self.max_correction end

   -- Apply correction (subtract because +error = left of track)
   local corrected_heading = desired_track_heading - correction
   corrected_heading = (corrected_heading + 360) % 360

   -- Save state
   self.last_error = cross_track_error

   return corrected_heading
end
-- end of crosstrackpid {} class definition

-- Instantiate the crosstrack/heading PID controller (outside update loop) 
local xt_pid = crosstrackpid.new(FOLLP_XT_P:get(),
                                 FOLLP_XT_I:get(),
                                 FOLLP_XT_D:get(),
                                 FOLLP_XT_MAX:get(),
                                 FOLLP_XT_I_MAX:get())

-------------------------------------------------------------------------------
--- MAVLink utility objects and functions
-------------------------------------------------------------------------------

local mavlink_attitude = require("mavlink_attitude")
local mavlink_attitude_receiver = mavlink_attitude.mavlink_attitude_receiver()

---@diagnostic disable-next-line:lowercase-global
function follow_frame_to_mavlink(follow_frame)
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
---@diagnostic disable-next-line:lowercase-global
function set_vehicle_target_altitude(target)
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
---@diagnostic disable-next-line:lowercase-global
function set_vehicle_heading(heading)
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
---@diagnostic disable-next-line:lowercase-global
function set_vehicle_speed(speed)
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

-------------------------------------------------------------------------------
--- Allow for simulation of telemetry fail for testing
-------------------------------------------------------------------------------

--[[
      -- checks for user simulating telemetry fail using FOLLP_SIM_TLF_FN
         - enables (HIGH)/disables (LOW) 
--]]
local simulate_failure = {
   telemetry = false,
}
(function ()
   local last_tel_fail_state = rc:get_aux_cached(FOLLP_SIM_TLF_FN:get())
   function simulate_failure.check()
      local sim_tel_fail = FOLLP_SIM_TLF_FN:get()
      local tel_fail_state = rc:get_aux_cached(sim_tel_fail)
      if tel_fail_state ~= last_tel_fail_state then
         if tel_fail_state == 0 then
            --simulate_telemetry_failed = false
            simulate_failure.telemetry = false
         else
            --simulate_telemetry_failed = true
            simulate_failure.telemetry = true
         end
         last_tel_fail_state = tel_fail_state
      end
   end
end) ()

-------------------------------------------------------------------------------
--- FOLLOW mode managmenet
-------------------------------------------------------------------------------
local follow_mode = {
   enabled = false,
}
(function ()
   local reported_target = true
   local lost_target_now_ms = now_ms
   --[[
      follow_active() - return true if we are in a state where follow can apply
   --]]
   function follow_mode.active()
      local mode = vehicle:get_mode()

      if not follow_mode.enabled then
         return false
      end

      if mode ~= FLIGHT_MODE.GUIDED then
         reported_target = false
         return reported_target
      end
      if follow:have_target() then
            reported_target = true
            lost_target_now_ms = now_ms
      else
         if reported_target then -- i.e. if we previously reported a target but lost it
            if (now_ms - lost_target_now_ms) > 5000 then
               gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": lost prior target: " .. follow:get_target_sysid())
               lost_target_now_ms = now_ms
            end
         end
         reported_target = false
         gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": no target: " .. follow:get_target_sysid())
      end

      return reported_target
   end
   function follow_mode.enable()
      if follow_mode.enabled then
         return
      end
      if not arming:is_armed() then
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": must be armed")
         return
      end
      -- Follow enabled - switch to guided mode but only if armed 
      vehicle:set_mode(FLIGHT_MODE.GUIDED)
      follow_mode.enabled = true
      lost_target_countdown = LOST_TARGET_TIMEOUT
      pid_controller_distance.reset()
      pid_controller_velocity.reset()
      xt_pid.reset()
      gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": enabled")
   end
   function follow_mode.disable()
      if not follow_mode.enabled then
         return
      end
      -- Follow switched to disabled - return to EXIT mode - but not if disarmed
      if not arming:is_armed() then
         vehicle:set_mode(exit_mode)
      end
      follow_mode.enabled = false
      gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": disabled")
   end

   --[[
      follow_check() - check for user activating follow using an RC switch 
         - set HIGH and if so
            - switches to GUIDED
            - enables follow
            - resets the PID controllers
         -- set LOW and if so
            - exits from GUIDED to the FOLLP_EXIT_MODE
            - disables follow
   --]]
   local last_follow_active_state = rc:get_aux_cached(FOLLP_ACT_FN:get())
   function follow_mode.check()
      if FOLLP_ACT_FN == nil then
         return
      end
      local foll_act_fn = FOLLP_ACT_FN:get()
      local active_state = rc:get_aux_cached(foll_act_fn)
      if (active_state == last_follow_active_state) then
         return
      end
      if( active_state == 0) then
         follow_mode.disable()
      elseif (active_state == 2) then
         follow_mode.enable()
      end
      last_follow_active_state = active_state
   end
end) ()

-- convert a groundspeed to airspeed using windspeed and EAS2TAS (from AHRS)
---@diagnostic disable-next-line:lowercase-global
function calculate_airspeed_from_groundspeed(velocity_vector)
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

-- ChatGPT code to calculate the distance to the target along_track and cross_track
---@diagnostic disable-next-line:lowercase-global
function calculate_track_distance(P_f, P_l)
    -- Get the follower's ground-relative velocity
    local V_f = ahrs:get_velocity_NED()
    if V_f == nil or (V_f:x() == 0 and V_f:y() == 0) then
        -- No valid velocity, return zeros
        return 0, 0
    end

    -- Create horizontal velocity vector
    local D_f = Vector3f()
    D_f:x(V_f:x())
    D_f:y(V_f:y())
    D_f:z(0)         -- need to do a dot product with 3d vector R below

    -- Normalize in-place
    D_f:normalize()

    -- Get relative position vector
    local R = P_f:get_distance_NED(P_l)

    -- Along-track distance: projection of R onto D_f
    local along_track_distance = R:dot(D_f)

    -- Cross-track distance: projection onto perpendicular to D_f
    local perp_D_f = Vector3f()
    perp_D_f:x(-D_f:y())
    perp_D_f:y(D_f:x())
    perp_D_f:z(0)    -- need to do a dot product with 3d vector R below
    perp_D_f:normalize()

    local cross_track_distance = R:dot(perp_D_f)

    -- -ve cross_track_distance to align to the sign of FOLL_OFS_Y where +ve is to the right
    return along_track_distance, -cross_track_distance
end

---@diagnostic disable-next-line:lowercase-global
function calculate_target_angle(heading)
   local new_target_angle = 0.0
   if (heading ~= nil and heading > -400) then
      -- want the greatest angle of we might have turned
      local angle_diff1 = wrap_180(math.abs(save_target_heading1 - heading))
      local angle_diff2 = wrap_180(math.abs(save_target_heading2 - heading))
      if angle_diff2 > angle_diff1 then
         new_target_angle = angle_diff2
      else
         new_target_angle = angle_diff1
      end
      -- remember the target heading from 2 seconds ago, so we can tell if it is turning or not
      if (now_ms - now_target_heading_ms) > 1000 then
         save_target_heading2 = save_target_heading1
         save_target_heading1 = heading
         now_target_heading_ms = now_ms
      end
   end
   return new_target_angle
end

-- main update function
function Update()
   now_ms = millis()
   ahrs_eas2tas = ahrs:get_EAS2TAS()
   windspeed_vector = ahrs:wind_estimate()

   simulate_failure.check()
   follow_mode.check()
   if not follow_mode.active() then
      return
   end

   -- set the target frame as per user set parameter - this is fundamental to this working correctly
   local close_distance = FOLLP_DIST_CLOSE:get()
   local long_distance = close_distance * 4.0
   local altitude_override = FOLLP_ALT_OVR:get()

   LOST_TARGET_TIMEOUT = FOLLP_TIMEOUT:get() / refresh_rate
   OVERSHOOT_ANGLE = FOLLP_OVRSHT_DEG:get()
   TURNING_ANGLE = FOLLP_TURN_DEG:get()
   foll_ofs_y = FOLL_OFS_Y:get()
   foll_alt_type = FOLL_ALT_TYPE:get()
   use_wide_turns = FOLLP_WIDE_TURNS:get()

   --[[
      get the current navigation target. 
   --]]
   local target_location                     -- = Location()     of the target
   local target_location_offset              -- Location  of the target with FOLL_OFS_* offsets applied
   local target_velocity -- = Vector3f()     -- current velocity of lead vehicle
   local target_velocity_offset -- Vector3f  -- velocity to the offset target_location_offset
   local target_heading                      -- heading of the target vehicle

   local current_location = ahrs:get_location()
   if current_location == nil then
      return
   end
   local current_altitude = current_location:alt() * 0.01

   local vehicle_airspeed = ahrs:airspeed_EAS()
   local current_target = vehicle:get_target_location()

   target_location, target_velocity = follow:get_target_location_and_velocity()
   target_location_offset, target_velocity_offset = follow:get_target_location_and_velocity_ofs()

   target_heading = follow:get_target_heading_deg() or -400

   -- if we lose the target wait for LOST_TARGET_TIMEOUT seconds to try to re-aquire it
   if target_location == nil or target_location_offset == nil or
      target_velocity == nil or target_velocity_offset == nil or current_target == nil or
      simulate_failure.telemetry then
      lost_target_countdown = lost_target_countdown - 1
      if lost_target_countdown <= 0 then
         follow_mode.enabled = false
         vehicle:set_mode(fail_mode)
         gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(": follow: %.0f FAILED", foll_sysid))
         return
      end

      -- maintain the current heading for 2 seconds until we re-establish telemetry from the target vehicle
      if (now_ms - now_follow_lost_ms) > 2000 then
         gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": follow: lost %.0f set hdg: %.0f", foll_sysid, save_target_heading1))
         now_follow_lost_ms = now_ms
         set_vehicle_heading({heading = save_target_heading1})
      end
      return
   else
      -- have a good target so reset the countdown 
      lost_target_countdown = LOST_TARGET_TIMEOUT
      now_follow_lost_ms = now_ms
   end

   -- calculate the target_distance from target distance offset so we don't need to call get_target_dist_and_vel_NED
   local new_target_distance = current_location:get_distance_NED(target_location_offset)
   local along_track_distance, cross_track_distance = calculate_track_distance(current_location, target_location_offset)

   -- target_velocity from MAVLink (via AP_Follow) is groundspeed, need to convert to airspeed, 
   -- we can only assume the windspeed for the target is the same as the chase plane
   local target_airspeed = calculate_airspeed_from_groundspeed(target_velocity)

   local vehicle_heading = math.abs(wrap_360(math.deg(ahrs:get_yaw_rad())))
   local heading_to_target_offset = math.deg(current_location:get_bearing(target_location_offset))

   -- offset_angle is the difference between the current heading of the follow vehicle and the heading to the target_location (with offsets)
   local offset_angle = wrap_180(vehicle_heading - heading_to_target_offset)

   -- rotate the target_distance_offsets in NED to the same direction has the follow vehicle, we use this below
   -- local target_distance_rotated = target_distance_offset:copy()
   local target_distance_rotated = new_target_distance:copy()
   target_distance_rotated:rotate_xy(math.rad(vehicle_heading))

   -- default the desired heading to the target heading (adjusted for projected turns) - we might change this below
   local airspeed_difference = vehicle_airspeed - target_airspeed

   -- projected_distance is how far off the target we expect to be if we maintain the current airspeed for DISTANCE_LOOKAHEAD_SECONDS
   local projected_distance = along_track_distance - airspeed_difference * DISTANCE_LOOKAHEAD_SECONDS

   -- target angle is the difference between the heading of the target and what its heading was 2 seconds ago
   local target_angle = calculate_target_angle(target_heading)

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
   local turning_airspeed_ratio = 1.0
   tight_turn = false
   if target_attitude ~= nil and target_airspeed ~= nil and target_airspeed > airspeed_min then
      if ((now_ms - target_attitude.timestamp_ms) < LOST_TARGET_TIMEOUT) and
         math.abs(target_attitude.roll) > 0.1 or math.abs(target_attitude.rollspeed) > 1 then
         -- the roll and rollspeed are delayed values from the target plane, try to extrapolate them to at least "now" + 1 second
         local rollspeed_impact = target_attitude.rollspeed * (target_attitude.delay_ms + 1000)
         local target_turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * math.tan(target_attitude.roll + rollspeed_impact))

         turning = true

         -- if the roll direction is the same as the y offset, then we are turning on the "inside" (a tight turn) 
         if (target_attitude.roll < 0 and foll_ofs_y < 0) or
            (target_attitude.roll > 0 and foll_ofs_y > 0) then
            tight_turn = true
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
                     (projected_distance < 0 or along_track_distance < 0) and
                     (math.abs(along_track_distance) < close_distance)
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
   local desired_heading = target_heading

   -- target_heading - vehicle_heading catches the circumstance where the target vehicle is heading in completely the opposite direction
   if (math.abs(along_track_distance) < airspeed_max * 0.75 or (math.abs(cross_track_distance) < airspeed_max * 0.25)) or
         ((turning and ((tight_turn and turn_starting) or use_wide_turns or foll_ofs_y == 0)) or -- turning 
         ((close or overshot) and not too_wide) -- we are very close to the target
         ) then
      -- set the desired heading to the targt heading
      mechanism = 2 -- heading - for logging
   elseif target_location_offset ~= nil then
      -- override the heading to point directly to the target location with offsets.
      desired_heading = heading_to_target_offset
      mechanism = 1  -- position/location - for logging
   end

   -- The desired heading needs a PID controller, especially when it gets close.
   desired_heading = xt_pid:compute(desired_heading, cross_track_distance, (now_ms - now_heading_ms):tofloat() * 0.001)

   -- dv = interim delta velocity based on the pid controller using projected_distance per loop as the error (we want distance == 0)
   local dv_error = along_track_distance * refresh_rate * 2.0
   if dv_error < 0 then
      dv_error = dv_error * 5.0  -- fudge factor to deal with it being harder to slow down from overshoot
   end
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
   end

   -- Finally after all the calculations - send the target heading, altitude and airspeed to AP
   set_vehicle_heading({heading = desired_heading})
   set_vehicle_target_altitude({alt = target_altitude, frame = foll_alt_type}) -- pass altitude in meters (location has it in cm)
   set_vehicle_speed({speed = constrain(airspeed_new, airspeed_min, airspeed_max)})

   -- now log everything
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
   logger:write("PF1",'Dst,DstP,DstE,AspT,Asp,AspD,AspE,AspO,TrnR,Mech,Cls,CF,OS','fffffffffBBBB','mmmnnnnnn----','-------------',
                  along_track_distance,
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
   logger:write("PF2",'AngT,AngO,Alt,AltT,AltFrm,HdgT,Hdg,HdgP,HdgO,XTD','ffffbfffff','ddmm-ddddm','----------',
                  target_angle,
                  offset_angle,
                  current_altitude,
                  target_altitude,
                  frame_type_log,
                  target_heading,
                  vehicle_heading,
                  pre_roll_target_heading,
                  desired_heading,
                  cross_track_distance
               )
end

-- wrapper around update(). This calls update() at 1/REFRESH_RATE Hz
-- and if update faults then an error is displayed, but the script is not
-- stopped
function Protected_Wrapper()
   local success, err = pcall(Update)

   if not success then
      gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. " Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return Protected_Wrapper, 1000
   end
   return Protected_Wrapper, 1000 * refresh_rate
end

function Delayed_Startup()
   gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
   return Protected_Wrapper()
end

-- start running update loop - waiting 25s for the AP to initialize if not armed
if FWVersion:type() == 3 then
   if arming:is_armed() then
      return Delayed_Startup()
   else
      return Delayed_Startup, 25000
   end
else
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
