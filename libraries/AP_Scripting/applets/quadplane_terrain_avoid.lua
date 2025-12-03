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

 Terrain Avoidance in QuadPlane.

 This code will detect if a quadplane following an Auto mission is likely to hit elevated terrain, such as
 a small hill, cliff edge, high trees or other obstacles that might not show up in the OOTB STRM terrain model.
 The code will attempt to avoid the impact by:-
 - Pitching up if the plane can safely fly over the obstacle
 - Otherwise switching to QuadPlane Qloiter mode (Quading) and gaining altitude using VTOL motors
 This code requires long range rangefinders such as the LightWare long range lidars that can measure
   distances up to 90-95 meters away.
 The terrain avoidance will be on by default but will not function at "home" or within TA_HOME_DIST meters
 of home. The scripting function TA_ACT_FN can be used to disable terrain folling at any time
 Terrain following will operate in modes Auto, Guided, RTL and QRTL.

The "Can't make that climb" (CMTC) feature will prevent ArduPlane from flying into terrain it does know about
by calculating the required pitch to avoid terrain between the current location and the next waypoint including
all points in between. If the pitch required is > PTCH_TRIM_MAX_DEG / 2 then the code will perform a loiter to
altitude to achieve a safe AMSL altitude (terrain + TA_CMTC_HGT) to avoid the terrain before continuing the mission.
CMTC can be disabled by setting TA_CMTC_ENABLE = 0. The loiter radius will be TA_CMTC_RAD if set or otherwise WP_LOITER_RAD.
--]]

SCRIPT_NAME = "Terrain Avoid"
SCRIPT_NAME_SHORT = "TerrAvoid"
SCRIPT_VERSION = "4.6.0-023"

STARTUP_DELAY = 25  -- wait this many seconds for the FC to come up before starting the script

FLIGHT_MODE = {FBWA=5, AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QLAND=20, QRTL=21}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3}
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points

RANGEFINDER_ORIENT = {DOWNWARD = 25, FORWARD = 0}
RANGEFINDER_STATUS = {NOTCONNECTED = 0, NODATA = 1, OUTOFRANGELOW = 2, OUTOFRANGEHIGH = 3, GOOD = 4}

PARAM_TABLE_KEY = 106
PARAM_TABLE_PREFIX = "TA_"

-- add a parameter and bind it to a variable
---@diagnostic disable-next-line:lowercase-global
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 19), SCRIPT_NAME_SHORT .. 'could not add param table: ' .. PARAM_TABLE_PREFIX)

--[[
    // @Param: TA_ACT_FN
    // @DisplayName: Activation Function for Terrain Avoidance
    // @Description: Setting an RC channel's _OPTION to this value will use it for Terrain Avoidance enable/disable
    // @Range: 300 307
--]]
TA_ACT_FN = bind_add_param("ACT_FN", 1, 305)

--[[
    // @Param: TA_PTCH_DWN_MIN
    // @DisplayName: down distance minimum for Pitching
    // @Description: If the downward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
--]]
TA_PTCH_DWN_MIN = bind_add_param("PTCH_DWN_MIN", 2, 46)

--[[
    // @Param: TA_PTCH_FWD_MIN
    // @DisplayName: forward distance minimum for Pitching
    // @Description: If the farwardward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
--]]
TA_PTCH_FWD_MIN = bind_add_param("PTCH_FWD_MIN", 3, 80)

--[[
    // @Param: TA_QUAD_DWN_MIN
    // @DisplayName: Downward distance minimum Quading
    // @Description: If the downward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
--]]
TA_QUAD_DWN_MIN = bind_add_param("QUAD_DWN_MIN", 4, 35)

--[[
    // @Param: TA_QUAD_FWD_MIN
    // @DisplayName: minimum forward distance for Quading
    // @Description: If the farwardward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
--]]
TA_QUAD_FWD_MIN = bind_add_param("QUAD_FWD_MIN", 5, 20)

--[[
    // @Param: TA_PTCH_GSP_MIN
    // @DisplayName: minimum ground speed for Pitching
    // @Description: Minimum Groundspeed (not airspeed) to be flying for Pitching to be used.
    // @Units: m/s
--]]
TA_PTCH_GSP_MIN = bind_add_param("PTCH_GSP_MIN", 6, 17)

--[[
    // @Param: TA_PTCH_TIMEOUT
    // @DisplayName: timeout Pitching
    // @Description: Minimum down or forward distance must be triggered for more than this many seconds to start Pitching
    // @Units: s
--]]
TA_PTCH_TIMEOUT = bind_add_param("PTCH_TIMEOUT", 7, 2)

--[[
    // @Param: TA_HOME_DIST
    // @DisplayName: safe distance around home
    // @Description: Terrain avoidance will not be applied if the vehicle is less than this distance from home
    // @Units: m
--]]
TA_HOME_DIST = bind_add_param("HOME_DIST", 8, 100)

--[[
    // @Param: TA_ALT_MAX
    // @DisplayName: ceiling for pitching/quading
    // @Description: This is a limit on how high the terrain avoidane will take the vehicle. It acts a failsafe to prevent vertical flyaways.
    // @Range: 20 1000
    // @Units: m
--]]
TA_ALT_MAX = bind_add_param("ALT_MAX", 9, 120)

--[[
    // @Param: TA_GSP_MAX
    // @DisplayName: Maximum Groundspeed
    // @Description: This is a limit on how fast in groundspeeed terrain avoidance will take the vehicle. This is to allow for reliable sensor readings. -1 for disabled.
    // @Range: 10 40
    // @Units: m/s
--]]
TA_GSP_MAX = bind_add_param("GSP_MAX", 10, -1)

--[[
    // @Param: TA_GSP_AIRBRAKE
    // @DisplayName: Groudspeed Airbrake limt
    // @Description: This is the limit for triggering airbrake to slow groundspeed as a difference between the airspeed and groundspeed. -1 for disabled.
    // @Range: -1 -10
    // @Units: m/s
--]]
TA_GSP_AIRBRAKE = bind_add_param("GSP_AIRBRAKE", 11, 0)

--[[
    // @Param: TA_CMTC_HGT
    // @DisplayName: CMTC Height
    // @Description: The minimum Height above terrain to maintain when following an AUTO mission or RTL. If zero(0) use TA_PTCH_DOW_MIN.
    // @Units: m
--]]
TA_CMTC_HGT = bind_add_param("CMTC_HGT", 12, 50)

--[[
    // @Param: TA_CMTC_ENABLE
    // @DisplayName: CMTC Enable
    // @Description: Whether to enable Can't Make That Climb while running Terrain Avoidance
    // @Range: 0 1
--]]
TA_CMTC_ENABLE = bind_add_param("CMTC_ENABLE", 13, 0)

--[[
    // @Param: TA_UPDATE_RATE
    // @DisplayName: Frequency to process avoidance
    // @Description: Avoidance processing rate
    // @Units: Hz
--]]
TA_UPDATE_RATE = bind_add_param("UPDATE_RATE", 14, 10)

--[[
    // @Param: TA_CMTC_RAD
    // @DisplayName: CMTC loiter radius
    // @Description: Use this radius for the loiter when trying to gain altitude. If not set or <=0 use WP_LOITER_RAD
    // @Units: m
--]]
TA_CMTC_RAD = bind_add_param("CMTC_RAD", 15, 0)

local pitch_down_min = TA_PTCH_DWN_MIN:get()
local pitch_forward_min = TA_PTCH_FWD_MIN:get()
local pitch_timeout = TA_PTCH_TIMEOUT:get()
local home_distance_max = TA_HOME_DIST:get()

local quad_down_min = TA_QUAD_DWN_MIN:get()
local quad_forward_min = TA_QUAD_FWD_MIN:get()

local altitude_max = TA_ALT_MAX:get()

local groundspeed_max = TA_GSP_MAX:get()
local groundspeed_airbrake_limit = TA_GSP_AIRBRAKE:get()

local cmtc_height_m = TA_CMTC_HGT:get()
if cmtc_height_m == 0 then
    cmtc_height_m = pitch_down_min
end
local cmtc_enable = TA_CMTC_ENABLE:get()

MIN_ALT_MAX = 20

REFRESH_RATE = 1.0 / TA_UPDATE_RATE:get()

local rangefinder_down_value = 0.0
local rangefinder_forward_value = 0.0
MAX_RANGEFINDER_VALUE = 90

Q_ENABLE = Parameter('Q_ENABLE')
Q_WVANE_ENABLE = Parameter('Q_WVANE_ENABLE')
Q_TILT_ENABLE = Parameter('Q_TILT_ENABLE')
THR_MAX = Parameter('THR_MAX')
AIRSPEED_MIN = Parameter("AIRSPEED_MIN")
AIRSPEED_CRUISE = Parameter("AIRSPEED_CRUISE")
AIRSPEED_MAX = Parameter("AIRSPEED_MAX")
WP_LOITER_RAD = Parameter("WP_LOITER_RAD")
TERRAIN_ENABLE = Parameter("TERRAIN_ENABLE")
TERRAIN_SPACING = Parameter("TERRAIN_SPACING")
PTCH_LIM_MAX_DEG = Parameter("PTCH_LIM_MAX_DEG")
TECS_CLMB_MAX = Parameter("TECS_CLMB_MAX")
local airspeed_min = AIRSPEED_MIN:get()
local airspeed_cruise = AIRSPEED_CRUISE:get()
local airspeed_max = AIRSPEED_MAX:get()
local wp_loiter_rad_m = WP_LOITER_RAD:get()
local cmtc_rad_m = TA_CMTC_RAD:get()
local terrain_spacing = TERRAIN_SPACING:get()
local ptch_lim_max_deg = PTCH_LIM_MAX_DEG:get()
local tecs_climb_max = TECS_CLMB_MAX:get()

if cmtc_rad_m <= 0 then
    cmtc_rad_m = wp_loiter_rad_m
end

THROTTLE_CHANNEL_NO = Parameter('RCMAP_THROTTLE'):get()
THROTTLE_CHANNEL = rc:get_channel(THROTTLE_CHANNEL_NO) -- The RC channel used for throttle

THROTTLE_CHANNEL_PREFIX = string.format("RC%.0f_", THROTTLE_CHANNEL_NO)
THROTTLE_CHANNEL_MIN = Parameter(THROTTLE_CHANNEL_PREFIX .. "MIN"):get()
THROTTLE_CHANNEL_MAX = Parameter(THROTTLE_CHANNEL_PREFIX .. "MAX"):get()
-- Throttle up will be 75% 
THROTTLE_UP_PWM = (THROTTLE_CHANNEL_MAX - THROTTLE_CHANNEL_MIN) * 0.80 + THROTTLE_CHANNEL_MIN
-- Hover will be midpoint between MIN and MAX
THROTTLE_HOVER_PWM = (THROTTLE_CHANNEL_MAX - THROTTLE_CHANNEL_MIN) * 0.5 + THROTTLE_CHANNEL_MIN

PITCH_TOLERANCE = 1.1
QUAD_TOLERANCE = 1.5

local vehicle_mode = vehicle:get_mode()

local current_location = ahrs:get_location()
local current_wp_bearing_deg = 0.0
local previous_location
local current_altitude_m = 0.0
local current_location_target
local current_heading_deg = -1
local new_location_target
local terrain_altitude = terrain:height_above_terrain(true)
local terrain_max_exceeded = false
local groundspeed_vector = ahrs:groundspeed_vector()
local groundspeed_current = groundspeed_vector:length()
local airspeed_current = ahrs:airspeed_EAS()
local airspeed_desired = airspeed_current

local now_ms = millis()
local old_now_ms = now_ms

local pitch_last_good_timestamp_ms = now_ms
local pitch_last_bad_timestamp_ms = now_ms
local pitch_bad_timer = -10

local slowdown_quading = false

local q_wvane_enable_save = Q_WVANE_ENABLE:get()
local avoid_enter_mode = -1

-- function forward declarations
local avoid_terrain
local terrain_approaching
local pitch_obstacle_detected
local pitch_obstacle_down
local pitch_obstacle_forward
local quading

local mavlink = require("mavlink_wrappers")

local location_tracker  -- forward declaration. See below for definition and instantiation.

-------------------------------------------------
-- deal with deprecations in 4.7
-------------------------------------------------
local version47orhigher = true
Get_Yaw_Function = ahrs.get_yaw_rad
if Get_Yaw_Function == nil then
    ---@diagnostic disable-next-line: deprecated
    Get_Yaw_Function = ahrs.get_yaw
    version47orhigher = false
end
Distance_Orient_Function = rangefinder.distance_orient
if Distance_Orient_Function == nil then
    ---@diagnostic disable-next-line:deprecated
    Distance_Orient_Function = rangefinder.distance_cm_orient
end
function Rangefinder_Distance_Orient_m(orientation)
    if version47orhigher then
        return Distance_Orient_Function(rangefinder,orientation)
    else
        -- the 4.6 function is in cm
        return Distance_Orient_Function(rangefinder,orientation) * 0.01
    end
end

-- Roll and Pitch functions change in 4.7, making it clearer that they return radians
Get_Roll_Function = ahrs.get_roll_rad
if Get_Roll_Function == nil then
    ---@diagnostic disable-next-line:deprecated
    Get_Roll_Function = ahrs.get_roll
end
function Get_Roll_Deg()
    return math.deg(Get_Roll_Function(ahrs))
end

Get_Pitch_Function = ahrs.get_pitch_rad
if Get_Pitch_Function == nil then
    ---@diagnostic disable-next-line:deprecated
    Get_Pitch_Function = ahrs.get_pitch
end
function Get_Pitch_Deg()
    return math.deg(Get_Pitch_Function(ahrs))
end


-- constrain a value between limits
local function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
end

-------------------------------------------------
-- Mode functions
-------------------------------------------------

-------------------------------------------------
-- wrap vehicle:set_mode to prevent accidentally setting invalid modes or manual mode
-- and also keep track of "prior" mode
-------------------------------------------------
---@diagnostic disable-next-line:lowercase-global
function set_vehicle_mode(new_mode)
    if new_mode <= 0 or vehicle_mode == new_mode then
        return
    end
    vehicle:set_mode(new_mode)
    vehicle_mode = new_mode
end

-- save the current mode and then enter the new mode
---@diagnostic disable-next-line:lowercase-global
function set_avoid_mode(new_mode)
    if vehicle:get_mode() ~= new_mode then
        avoid_enter_mode = vehicle_mode
        set_vehicle_mode(new_mode)
    end
end

-- reset the flight mode to the previously saved flight mode (if valid) or AUTO
---@diagnostic disable-next-line:lowercase-global
function reset_avoid_mode()
    local new_mode = FLIGHT_MODE.AUTO
    if avoid_enter_mode == FLIGHT_MODE.AUTO or avoid_enter_mode == FLIGHT_MODE.GUIDED or avoid_enter_mode == FLIGHT_MODE.LOITER or
        avoid_enter_mode == FLIGHT_MODE.QHOVER or avoid_enter_mode == FLIGHT_MODE.QLOITER or avoid_enter_mode == FLIGHT_MODE.QRTL or
        avoid_enter_mode == FLIGHT_MODE.RTL then
       new_mode= avoid_enter_mode
    end

    set_vehicle_mode(new_mode)
    avoid_enter_mode = -1

    if new_mode == FLIGHT_MODE.AUTO then
        airspeed_desired = airspeed_cruise

        previous_location = location_tracker.get_saved_location()
        if previous_location ~= nil then
            vehicle:set_crosstrack_start(previous_location)
        end
    end
end

local function disable_wvane()
    q_wvane_enable_save = Q_WVANE_ENABLE:get()
    Q_WVANE_ENABLE:set(0)
end

local function enable_wvane()
    if q_wvane_enable_save >=0 then
        Q_WVANE_ENABLE:set(q_wvane_enable_save)
    else
        Q_WVANE_ENABLE:set(0)
    end
    q_wvane_enable_save = -1
end

local airbrake_trigger = false
local airbrake_on = false
local airbrake_triggered_now_ms = millis()

-- This method terminates airbraking if it had been acive
local function slowdown_airbrake_end()
    airbrake_trigger = false
    if not airbrake_on then
        return
    end
    airbrake_on = false
    enable_wvane()
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": airbraking stopping")
    if vehicle_mode ~= FLIGHT_MODE.QLOITER then
        -- user or a failesafe has exited QLoiter, so we don't want to mess with that
        return
    end
    reset_avoid_mode()
end

local function slow_down(groundspeed)
    local groundspeed_error = groundspeed.error or 0.0
    local airspeed_new = constrain(airspeed_current + groundspeed_error, airspeed_min, airspeed_max)

    if groundspeed_airbrake_limit ~= 0 and not airbrake_on and groundspeed_error < groundspeed_airbrake_limit then
        if not airbrake_trigger then
            airbrake_trigger = true
            airbrake_triggered_now_ms = millis()
        end

        if (now_ms - airbrake_triggered_now_ms):tofloat() * 0.001 > pitch_timeout then
            airbrake_on = true
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": airbrake current %f error %f new %f", airspeed_current,groundspeed_error,airspeed_new) )
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": airbraking Starting")
            disable_wvane()
            set_avoid_mode(FLIGHT_MODE.QLOITER)
        end
    elseif airbrake_trigger then
        airbrake_trigger = false
    end
    if airbrake_on then
        if groundspeed_error >= 0 or (now_ms - airbrake_triggered_now_ms):tofloat() * 0.001 > 10 then -- it's not working, give up
            slowdown_airbrake_end()
        else
            -- we want to hover in place. 
            -- There is currently (2025-08-03) no better way to do this this so using RC overrride.
            THROTTLE_CHANNEL:set_override(THROTTLE_HOVER_PWM)
        end
    end
    return airspeed_new
end

local function set_altitude_target(new_altitude)
    current_location_target = vehicle:get_target_location()
    if current_location_target ~= nil then
        new_location_target = current_location_target:copy()
        if new_location_target ~= nil and new_altitude ~= nil then
            new_location_target:alt(math.floor(new_altitude * 100))
            -- can't use MAVLink for this because we might not be in GUIDED mode
            if not vehicle:update_target_location(current_location_target,new_location_target) then
                gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT ..
                    string.format(": failed to set alt: %.0f frame: %d current frame: %d", new_altitude, new_location_target:get_alt_frame(), current_location_target:get_alt_frame()))
            end
        else
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": failed to set alt: " .. new_altitude)
        end
    end
    return new_altitude
end

-- This method forces the plane to target a very high altitude to force it to gain altitude quicky (fixed wing only)
local function set_altitude_high()
    local old_target_altitude = 0.0
    if current_location_target == nil and current_location ~= nil then
        old_target_altitude = current_location:alt() * 0.01
    elseif current_location_target ~= nil then
        old_target_altitude = current_location_target:alt() * 0.01
    end
    set_altitude_target(current_altitude_m + 2000)
    return old_target_altitude
end

-- Attempts to duplicate the code that updates the prev_WP_loc variable in the c++ code
function LocationTracker()
    local self = {}

    -- to get this to work, need to keep 2 prior generations of "target_location"
    local target_location
    local previous_target_location          -- the target prior to the current one
    local previous_previous_target_location -- the target prior to that - this is the one we want

    function self.same_loc_as(A, B)
       if A == nil or B == nil then
          return false
       end
       if (A:lat() ~= B:lat()) or (A:lng() ~= B:lng()) then
          return false
       end
       return (A:alt() == B:alt()) and (A:get_alt_frame() == B:get_alt_frame())
    end

    function self.update()
       target_location = vehicle:get_target_location()
       if target_location ~= nil then
          if not self.same_loc_as(previous_target_location, target_location) then
            -- maintain three generations of location
            if previous_target_location ~= nil and not self.same_loc_as(previous_target_location, previous_previous_target_location)  then
                previous_previous_target_location = previous_target_location:copy()
            end
            previous_target_location = target_location:copy()
            end
            if previous_previous_target_location == nil then
                previous_previous_target_location = previous_target_location:copy()
            end
        end
    end

    function self.get_saved_location()
        return previous_previous_target_location
    end

    return self
end
location_tracker = LocationTracker()    -- instantiate previously declared instance

-------------------------------------------------------------------------------
-- CMTC - Can't make that climb. If the path isn't acheivable loiter to gain altitude
-------------------------------------------------------------------------------
local cmtc = {
    active = false,
    target_alt_amsl = -1,
}
(function ()
    local pre_cmtc_heading_deg = 0.0

    function cmtc.start(target_alt_amsl)
        if cmtc.active then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": CMTC to ALREADY ACTIVE: " .. cmtc.target_alt_amsl)
            return
        end
        if terrain_altitude > altitude_max then
            return -- already too high
        end
        cmtc.active = true
        pre_cmtc_heading_deg = current_heading_deg -- math.deg(ahrs:get_yaw_rad() or 0)

        -- AP libraries use a 0.5m "near enough" buffer for matching altitude, we'll use 3 meters
        cmtc.target_alt_amsl = target_alt_amsl - 3.0

        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC loiter %s to %.0f alt",
                avoid_terrain(target_alt_amsl),
                cmtc.target_alt_amsl) )

        airspeed_desired = airspeed_max
    end

    function cmtc.stop()
        if current_location ~= nil and cmtc.active then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC Done alt: %.0f", current_altitude_m) )
        end
        cmtc.active = false
        cmtc.target_alt_amsl = -1
        reset_avoid_mode()
    end

    function cmtc.update()
        if current_location == nil then
            return
        end
        local current_location_amsl = current_location:copy()
        current_location_amsl:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

        -- if we are above TA_ALT_MAX exit immediately
        -- if we are above the cmtc.target_alt_amsl then wait till we are pointing to the next WP
        if (terrain_altitude > altitude_max) or
                ((current_altitude_m > cmtc.target_alt_amsl) and
                math.abs(pre_cmtc_heading_deg - current_heading_deg) < 45.0) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC STOP alt curr %.0f trg %.0f max %0.1f",
                        current_altitude_m, (current_location_amsl:alt() * 0.01)
                        , altitude_max) )
            cmtc.stop()
        else
            airspeed_desired = airspeed_cruise
            mavlink.set_vehicle_speed({speed=airspeed_desired})
        end
    end
end)()

-----------------------------------------------------------
-- Pitching (aka quicky gain altiude in fixed wing mode)
-----------------------------------------------------------
local pitching = {
    active = false,
}
(function ()
    local pre_pitch_mode = vehicle_mode
    local pre_pitch_target_altitude= 0.0
    local pre_pitch_target_location

    function pitching.start()
        if slowdown_quading then
            quading.start() -- we are already in multirotor mode, so go straight to quading
            return
        end

        pitching.active = true
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Pitching started")

        -- save the current target location so we can track if we pass a waypoint while pitching
        pre_pitch_mode = vehicle_mode
        pre_pitch_target_altitude = set_altitude_high()
        pre_pitch_target_location = current_location_target:copy()

        -- pitch up by setting a very high altitude and high speed. TECS will make it so.
        pitch_last_bad_timestamp_ms = millis()
    end

    function pitching.check()
        if groundspeed_current ~= nil and (groundspeed_current < TA_PTCH_GSP_MIN:get()) then
            return false
        end

        if pitch_obstacle_detected(1.0) then
            if not pitching.active then
                -- we don't jump into pitching right away, we give it a TA_PTCH_TIMEOUT seconds to be sure
                local time_since_good = (now_ms - pitch_last_good_timestamp_ms):tofloat() * 0.001
                if time_since_good > pitch_timeout then
                    pitching.start()
                    if pitch_obstacle_down(1.0) then
                        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle down: %.2f m", rangefinder_down_value) )
                    end
                    if pitch_obstacle_forward(1.0) then
                        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle forward: %.2f m", rangefinder_forward_value) )
                    end
                else
                    if time_since_good > pitch_bad_timer + 1 then
                        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": high terrain detected")
                        pitch_bad_timer = math.floor(time_since_good + 0.5)
                    end
                end
            end
            return true
        else
            if pitch_bad_timer >= 0 and not terrain_max_exceeded then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": terrain Ok")
            end
            pitch_last_good_timestamp_ms = now_ms
            pitch_bad_timer = -1
        end
        return false
    end

    function pitching.stop()
        if pitching.active then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Pitching DONE")
        end
        pitching.active = false
        if pre_pitch_mode ~= vehicle_mode then
            return  -- user or maybe a failsafe changed mdoes. Don't interfere
        end
        if pre_pitch_target_altitude ~= nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": reset Alt: " .. pre_pitch_target_altitude)
            set_altitude_target(pre_pitch_target_altitude)
            airspeed_desired = airspeed_cruise
            previous_location = location_tracker.get_saved_location()
            if previous_location ~= nil then
                vehicle:set_crosstrack_start(previous_location)
            end
        else
            -- don't know where to go, so lets go home (like a good kid)
            set_vehicle_mode(FLIGHT_MODE.RTL)
        end
    end

    --local old_airspeed_desired = airspeed_current
    function pitching.update()
        -- quading takes precedence over pitching
        if quading.active then
            pitching.active = false
            return
        end

        -- if we are above the pitching altitude (with a margin) we might want to stop pitching
        -- we don't stop pitching right away, we give it a TA_PTCH_TIMEOUT seconds * 2 to be sure
        if not pitch_obstacle_detected(PITCH_TOLERANCE) and
            (now_ms - pitch_last_bad_timestamp_ms):tofloat() * 0.001 > (pitch_timeout * 2) or
            not (pre_pitch_target_location:lat() == current_location_target:lat() and pre_pitch_target_location:lng() == current_location_target:lng() ) then
            pitching.stop()
        else
            set_altitude_high()
            if pitch_obstacle_detected(PITCH_TOLERANCE) then
                pitch_last_bad_timestamp_ms = millis()
            end
            airspeed_desired = airspeed_cruise
        end
    end
end)()  -- pitching

-------------------------------------------------------------------------------
-- Quading - aka switch to QLOITER mode and quickly gain altitude
-------------------------------------------------------------------------------
-- This function decides if there is an obstacle for quading based on
-- 1. if terrain height is < quad_down_min or
-- 2. if there is a valid rangefinder down and rangefinder_down_value < pitch_quad_min or
-- 3. if there is a valid rangefinder forward and rangefinder_forward_value < quad_forward_min
--
quading = {
    active = false,
}
(function ()
    local last_quading_location
    local quad_tolerance = QUAD_TOLERANCE

    function quading.quad_obstacle_forward()
        if rangefinder_forward_value > 0 and rangefinder_forward_value < MAX_RANGEFINDER_VALUE
        and rangefinder_forward_value < quad_forward_min then
            return true
        end
    end

    function quading.quad_obstacle_down()
        if rangefinder_down_value > 0 and rangefinder_down_value < MAX_RANGEFINDER_VALUE
        and rangefinder_down_value < quad_down_min then
            return true
        end
        return false
    end

    function quading.quad_obstacle_detected()
        -- prevent quading if we are in a flyaway situation
        if terrain_max_exceeded then
            if quading.active then
                gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Quading " .. terrain_altitude .. " above: " .. altitude_max)
            end
            return false
        end

        if quading.quad_obstacle_down() then
            return true
        end
        if quading.quad_obstacle_forward() then
            return true
        end

        -- no obstacle, we are good
        return false
    end

    function quading.start() -- forward declaration above
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Quading started")
        if slowdown_quading then
            slowdown_quading = false -- already in multi-motor mode, so just switch to quading
        else
            if cmtc.active then
                gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": Quading overrides CMTC: " .. cmtc.target_alt_amsl)
                set_vehicle_mode(FLIGHT_MODE.QLOITER)
            else
                set_avoid_mode(FLIGHT_MODE.QLOITER)
            end
            disable_wvane()
        end
        quading.active = true
        pitching.active = false
        cmtc.active = false
        THROTTLE_CHANNEL:set_override(THROTTLE_UP_PWM)

        if current_location then
            if last_quading_location then
                -- if we seem to be repeatedly quading at the same location, try going a little higher this time
                if current_location:get_distance(last_quading_location) < wp_loiter_rad_m then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Quading repeat: " .. quad_tolerance)
                    quad_tolerance = quad_tolerance * 1.5
                else 
                    quad_tolerance = QUAD_TOLERANCE
                end
            end
            -- remeber for next time
            last_quading_location = current_location:copy()
        end
    end

    function quading.check()
        if quading.quad_obstacle_detected() then
            if not quading.active then
                if quading.quad_obstacle_down() then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle down: %.2f m", rangefinder_down_value) )
                end
                if quading.quad_obstacle_forward() then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle forward: %.2f m", rangefinder_forward_value) )
                end
                if pitching.active then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Stop Pitching")
                    pitching.stop()
                end
                quading.start()
                return true
            end
        end
        return false
    end

    function quading.stop()
        if quading.active then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Quading DONE")
        end
        quading.active = false
        airbrake_on = false
        enable_wvane()
        if vehicle_mode ~= FLIGHT_MODE.QLOITER then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": exit Quading NOT QLoiter")
            avoid_enter_mode = 0
            return -- user or failsafe has changed modes. Don't interfere
        end
        reset_avoid_mode()
    end

    function quading.update()
        -- not a typo, if we kick into quading we want to go higher than would trigger pitching when when we are done
        if not pitch_obstacle_detected(quad_tolerance) or terrain_max_exceeded then
            -- we are done quadding
            quading.stop()
        else
            -- continually override the throttle to keep going up
            THROTTLE_CHANNEL:set_override(THROTTLE_UP_PWM)
        end
    end
end)()  -- Quading

-------------------------------------------------------------------------------
-- Rangefinder functions
-------------------------------------------------------------------------------

-- This function decides if there is an obstacle for pitching based on
-- 1. if terrain height is < pitch_down_min or
-- 2. if there is a valid rangefinder down and rangefinder_down_value < pitch_down_min or
-- 3. if there is a valid rangefinder forward and rangefinder_forward_value < pitch_forward_min
--
pitch_obstacle_down = function(multiplier)
    if rangefinder_down_value > 0 and
            rangefinder_down_value < MAX_RANGEFINDER_VALUE and
            rangefinder_down_value < (pitch_down_min * multiplier) then
        return true
    end
    return false
end

pitch_obstacle_forward = function(multiplier)
    if rangefinder_forward_value > 0 and 
            rangefinder_forward_value < MAX_RANGEFINDER_VALUE and
            rangefinder_forward_value < (pitch_forward_min * multiplier) then
        return true
    end
    return false
end

pitch_obstacle_detected = function(multiplier)
    -- prevent pitching if we are in a flyaway situation
    if terrain_max_exceeded then
        if pitching.active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Pitching " .. terrain_altitude .. " above: " .. altitude_max)
        end
        if quading.active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Quading " .. terrain_altitude .. " above: " .. altitude_max)
        end
        if cmtc.active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": CMTC " .. terrain_altitude .. " above: " .. altitude_max)
        end
        return false
    end

    if pitch_obstacle_down(multiplier) then
        return true
    end
    if pitch_obstacle_forward(multiplier) then
        return true
    end

    -- no obstacle, we are good
    return false
end

-- this method checks the distance down and forward.
-- and this uses RC8 to simulate forward rangefinder and RC5 to simulate downward
local function populate_rangefinder_values()
    -- Get the new values of the range finders every update cycle
    -- We'll probably want some kind of certainty check for the range finders
    -- So a small error won't cause it to freakout.

    if rangefinder:has_data_orient(RANGEFINDER_ORIENT.DOWNWARD)
        and rangefinder:status_orient(RANGEFINDER_ORIENT.DOWNWARD) == RANGEFINDER_STATUS.GOOD then
        rangefinder_down_value = Rangefinder_Distance_Orient_m(RANGEFINDER_ORIENT.DOWNWARD) -- rangefinder:distance_cm_orient(RANGEFINDER_ORIENT.DOWNWARD) / 100.0
    else
        -- if we don't have a downward rangefinder revert to terrain altitude
        rangefinder_down_value = terrain:height_above_terrain(true) or 0.0
    end
    if rangefinder:has_data_orient(RANGEFINDER_ORIENT.FORWARD)
        and rangefinder:status_orient(RANGEFINDER_ORIENT.FORWARD) == RANGEFINDER_STATUS.GOOD then
        rangefinder_down_value = Rangefinder_Distance_Orient_m(RANGEFINDER_ORIENT.FORWARD) --rangefinder_forward_value = rangefinder:distance_cm_orient(RANGEFINDER_ORIENT.FORWARD) / 100.0
    else
        rangefinder_forward_value = 0.0
    end

    terrain_altitude = terrain:height_above_terrain(true)
    terrain_max_exceeded = false
    if altitude_max ~= nil and terrain_altitude ~= nil then
        terrain_max_exceeded = (altitude_max > MIN_ALT_MAX and terrain_altitude > altitude_max)
    end

    if rangefinder_down_value == nil or rangefinder_down_value <= 0 or rangefinder_down_value > MAX_RANGEFINDER_VALUE then
        rangefinder_down_value = terrain_altitude or 0
    end
    if rangefinder_forward_value == nil or rangefinder_forward_value <= 0 then
        rangefinder_forward_value = 0
    end
    if rangefinder_forward_value > MAX_RANGEFINDER_VALUE then
        rangefinder_forward_value = MAX_RANGEFINDER_VALUE
    end
end

local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

--[[
c++_code from AP_Terrain.cpp used as reference - i.e. this is not commented out code, it's the c++ origin of the Lua code below
// check for terrain at grid spacing intervals
while (distance > 0) {
    gcs().send_text(MAV_SEVERITY_INFO, "lookahead distance %.1f", distance);
    loc.offset_bearing(bearing, grid_spacing);
    climb += climb_ratio * grid_spacing;
    distance -= grid_spacing;
    float height;
    if (height_amsl(loc, height)) {
        float rise = (height - base_height) - climb;
        //if(rise > 0)
            gcs().send_text(MAV_SEVERITY_INFO, "lookahead alt %.1f climb %.1f rise %.1f", height, climb, rise);
        if (rise > lookahead_estimate) {
            lookahead_estimate = rise;
            loc_highest = loc;
            gcs().send_text(MAV_SEVERITY_INFO, "lookahead estimate %.1f", lookahead_estimate);
        }
    }
}
--]]

-------------------------------------------------------------------------------
-- Lookahead functions - replaces the c++ functions in AP_Terrain
-------------------------------------------------------------------------------
function Terrain_Lookahead(start_location, search_bearing, search_distance, search_ratio)
    local highest_location = nil
    local climb = 0.0
    local highest_rise = 0.0
    local height
    local search_location = start_location:copy()
    search_location:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

    local base_height = terrain:height_amsl(search_location, true)

    while search_distance > 0 do
        search_location:offset_bearing(search_bearing, terrain_spacing)
        climb = climb + search_ratio * terrain_spacing

        height = terrain:height_amsl(search_location, true)
        if height ~= nil then
            local rise = (height - base_height) - climb
            if rise > highest_rise then
                highest_rise = rise
                highest_location = search_location:copy()
                highest_location:alt( math.floor(height * 100) )
            end
        end

        search_distance = search_distance - terrain_spacing
    end

    return highest_location
end

-- returns required pitch to avoid hitting something between here and the next waypoint or other destination such as
-- home location for RTL
-- returns pitch required, altitude required, maximum alitude error
terrain_approaching = function(clearance)
    if current_location == nil then
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": current_location NIL") )
        return
    end
    local wp_distance = current_location:get_distance(current_location_target)
    local wp_bearing = math.deg(current_location:get_bearing(current_location_target))
    local pitch_required = 0
    local alt_required_amsl = -1
    local highest_location
    local highest_alt_difference = 0.0
    local current_location_amsl = current_location:copy()
    current_location_amsl:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

    if wp_distance == nil then
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": wp_distance NIL") )
        return 0.0, -1.0, 0.0
    end

    highest_location = Terrain_Lookahead(current_location, wp_bearing, wp_distance, 0.5 * tecs_climb_max / groundspeed_current)
    if highest_location == nil then
        return 0.0, -1.0, 0.0
    end

    -- need to know how far ahead the highest location is and how much higher than the current
    -- altitude to calculate a minimum required glide slope (which TECS already does)
    highest_location:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)
    local altitude_difference =  (highest_location:alt() * 0.01) + clearance - (current_location_amsl:alt() * 0.01)
    if altitude_difference > 0 then
        -- what is the pitch up required to acheive that altitude?
        local highest_distance = current_location_amsl:get_distance(highest_location)
        pitch_required = math.deg(math.atan(altitude_difference,highest_distance))
        -- the target location we need to hit needs to be AMSL to ensure we fly above terrain
        alt_required_amsl = highest_location:alt() * 0.01 + clearance * 1.5

        highest_alt_difference = altitude_difference
    end

    return pitch_required, alt_required_amsl, highest_alt_difference
end

-- search around an arch from loiter_center for the highest/lowest terrain altitude around the arc
function Arc_Terrain_Altitude(loiter_center, bearing_start, bearing_step, arc_max, loiter_rad_m)
    local next_increment = bearing_step
    local highest_terrain_m = 0.0
    while math.abs(next_increment) < arc_max do
        local test_bearing = wrap_360(bearing_start + next_increment)

        local loiter_edge = loiter_center:copy()
        loiter_edge:offset_bearing(test_bearing, loiter_rad_m)
        local terrain_height_m = terrain:height_amsl(loiter_edge, true)
        if terrain_height_m > highest_terrain_m then
            highest_terrain_m = terrain_height_m or 0.0
        end
        next_increment = next_increment + bearing_step
    end

    return highest_terrain_m --, lowest_terrain_m
end

-- avoids upcoming terrain entering a loiter to altitude
-- loiters either left or right depending on which is less likely to hit terrain
avoid_terrain = function(target_alt_amsl) -- forward declaration above
    -- calculate the highest location in an arc assuming that we loiter first right and then left
    -- then we choose the one that has the lowest terrain either way
    if current_location == nil then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": avoid_terrain no current_location")
        return
    end
    local direction
    local loiter_center_left = current_location:copy()
    local loiter_center_right = current_location:copy()
    local radius_scaled_m = cmtc_rad_m * (1.0 + current_altitude_m/3000.0)
    -- calculate a loiter location (center of the circle) 90 degrees left and 90 degrees right
    -- note we are using the bearing to the current waypoint, not the current heading which might be different
    -- the fudge factor on the radius is becuase AP scales he loiter radius internally based on altitude
    loiter_center_left:offset_bearing(wrap_360(current_wp_bearing_deg - 90), radius_scaled_m)
    loiter_center_right:offset_bearing(wrap_360(current_wp_bearing_deg + 90), radius_scaled_m)
    -- at a radius of radius_scaled_m how many degrees is TERRAIN_SPACING
    -- Central Angle = Arc length(AB) / Radius(OA) = (s × 360°) / 2πr
    local spacing_degrees = (terrain_spacing * 360.0) / (2.0 * math.pi * radius_scaled_m)

    -- find the highest terrain we are likely to hit if we loiter left vs right
    local highest_left_terrain = Arc_Terrain_Altitude(loiter_center_left, current_wp_bearing_deg, -spacing_degrees, 180, radius_scaled_m)
    local highest_right_terrain = Arc_Terrain_Altitude(loiter_center_right, current_wp_bearing_deg, spacing_degrees, 180, radius_scaled_m)

    set_avoid_mode(FLIGHT_MODE.GUIDED)

    -- loiter up to the requjired AMSL height, in the direction of lowest terrain
    -- except if the vehicle is already rolling > 30 degrees left then it won't try to "reverse" the roll
    -- 30 degrees is a bit arbitrary, with further testing it could be adjusted/tweaked
    if highest_left_terrain < highest_right_terrain or Get_Roll_Deg() > 30 then
        mavlink.set_vehicle_target_location({lat = loiter_center_left:lat(),
            lng = loiter_center_left:lng(),
            alt = target_alt_amsl + 50.0,
            alt_frame = mavlink.ALT_FRAME.ABSOLUTE,
            radius = cmtc_rad_m,
            yaw = 1 })
        direction = "left"
    else
        mavlink.set_vehicle_target_location({lat = loiter_center_right:lat(),
            lng = loiter_center_right:lng(),
            alt = target_alt_amsl + 50.0,
            alt_frame = mavlink.ALT_FRAME.ABSOLUTE,
            radius = cmtc_rad_m,
            yaw = 0 })
        direction = "right"
    end

    -- Set an extra height altitude to ensure the plane tries to climb as fast as possible, because set_vehicle_target_location doesn't always stick
    mavlink.set_vehicle_target_altitude({alt = target_alt_amsl + 50.0, alt_frame = mavlink.ALT_FRAME.ABSOLUTE})
    airspeed_desired = airspeed_cruise

    return direction
end

-- make the pitching and quading parameters updatable in the air (refresh every second)
function GetParameterValues()
    if now_ms - old_now_ms > 1000 then
        pitch_down_min = TA_PTCH_DWN_MIN:get()
        pitch_forward_min = TA_PTCH_FWD_MIN:get()
        pitch_timeout = TA_PTCH_TIMEOUT:get()
        home_distance_max = TA_HOME_DIST:get()

        quad_down_min = TA_QUAD_DWN_MIN:get()
        quad_forward_min = TA_QUAD_FWD_MIN:get()

        altitude_max = TA_ALT_MAX:get()

        groundspeed_max = TA_GSP_MAX:get()
        groundspeed_airbrake_limit = TA_GSP_AIRBRAKE:get()
        airspeed_min = AIRSPEED_MIN:get()
        airspeed_cruise = AIRSPEED_CRUISE:get()
        airspeed_max = AIRSPEED_MAX:get()

        cmtc_enable = TA_CMTC_ENABLE:get()
        if cmtc_enable then
            cmtc_height_m = TA_CMTC_HGT:get()
            if cmtc_height_m <= 0 then
                cmtc_height_m = pitch_down_min
            end
            cmtc_rad_m = TA_CMTC_RAD:get()
            if cmtc_rad_m <= 0 then
                cmtc_rad_m = wp_loiter_rad_m
            end
        end

        old_now_ms = now_ms
    end
end

local switch_on = true
local last_switch_state = -1

-- check if the activation switch has been newly enabled or disabled
local function check_activation_switch()
    local switch_function = TA_ACT_FN:get()
    if switch_function == nil then
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
        if switch_state == 0 then -- switch Low to activate - so defaults to on
            -- activate Terrain Avoidance
            switch_on = true
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT ..": activated")
            pitch_last_good_timestamp_ms = now_ms
        elseif switch_state == 2 then -- switch High to turn off
            -- deactivate Terrain Avoidance
            switch_on = false
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT ..": deactivated")
        end
        -- Don't know what to do with the 3rd switch position right now.
        last_switch_state = switch_state
    end
end

-- check that all the conditions for terrain avoidance are met
local close_to_home = false
local function terravoid_active()
    if not switch_on then
        return false
    end
    if not (arming:is_armed()) then
        return false
    end
    if not(vehicle_mode == FLIGHT_MODE.AUTO or vehicle_mode == FLIGHT_MODE.GUIDED or
        vehicle_mode == FLIGHT_MODE.RTL or vehicle_mode == FLIGHT_MODE.QRTL or vehicle_mode == FLIGHT_MODE.QLAND or
        ((quading.active or airbrake_on) and (vehicle_mode == FLIGHT_MODE.QLOITER or vehicle_mode == FLIGHT_MODE.QHOVER))) then
        return false
    end

    local home = ahrs:get_home()
    local home_distance = 0.0
    if home ~= nil and current_location ~= nil then
        home_distance = home:get_distance(current_location) or 0.0
    end
    if home_distance ~= nil and home_distance_max ~= nil and home_distance < home_distance_max then
        if not close_to_home then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": close to home")
            close_to_home = true
        end
        return false
    end
    if close_to_home then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": away from home")
    end
    close_to_home = false

    return true
end

-- main update function called by protected_wrapper REFRESH_RATE times per second
function Update()
    now_ms = millis()
    current_location = ahrs:get_location()
    if current_location ~= nil then
        current_altitude_m = current_location:alt() * 0.01
    end
    vehicle_mode = vehicle:get_mode()
    current_location_target = vehicle:get_target_location()
    current_wp_bearing_deg = vehicle:get_wp_bearing_deg() or 0
    current_heading_deg = math.deg(Get_Yaw_Function(ahrs) or 0)
    groundspeed_vector = ahrs:groundspeed_vector()
    groundspeed_current = groundspeed_vector:length()
    airspeed_current = ahrs:airspeed_EAS()

    -- save the previous target location only if in auto mode, if restoring it in AUTO mode
    -- don't update it if already pitching or quading because the altitude change will mess up the history
    if (vehicle_mode == FLIGHT_MODE.AUTO or vehicle_mode == FLIGHT_MODE.GUIDED) and
            location_tracker ~= nil and
            not (pitching.active or quading.active) then
        location_tracker.update()
    end

    GetParameterValues()
    check_activation_switch()
    if not terravoid_active() then
        if cmtc.active then
            cmtc.stop()
        end
        if quading.active then
            quading.stop()
        end
        if pitching.active then
            pitching.stop()
        end
        return
    end

    populate_rangefinder_values()

    -- first decide if we are seriously close to the terrain and need to start quading
    if not quading.active and not quading.check() then
        if cmtc_enable == 1 and not cmtc.active then
            -- lets check if our current flight path is likely to hit terrain 
            -- sometime soon, and if so we need to avoid it.
            local pitch_required_deg, alt_required_amsl, _ = terrain_approaching(cmtc_height_m)
            if pitch_required_deg > (ptch_lim_max_deg * 0.5) then
                -- need to fly OVER the highest point - with TA_CMTC_HGT clearance
                cmtc.start(alt_required_amsl)
            end
        end
        if not cmtc.active then
            -- otherwise - lets see if we are close enough to need to start pitching
            pitching.check()
        end
    end

    if terrain_max_exceeded and not cmtc.active then
        if quading.active then
            quading.stop()
        end
        if pitching.active then
            pitching.stop()
        end
    end

    -- quading is the priority. If we are quading, do that, otherwise if we are pitching do that
    if quading.active then
        quading.update()
        -- return here becasue we don't want to set airspeed
        return
    elseif pitching.active then
        pitching.update()
    elseif cmtc.active then
        cmtc.update()
    end

    if groundspeed_vector ~= nil then
        if groundspeed_max > 0 and groundspeed_current > groundspeed_max then
            -- if the groundspeed is too high we need to slow down
            airspeed_desired = slow_down({error = (groundspeed_max - groundspeed_current)})
        elseif groundspeed_airbrake_limit ~= 0 then
            slowdown_airbrake_end()
        end
    end
    mavlink.set_vehicle_speed({speed=airspeed_desired})
end

-- wrapper around update(). This calls update() at 1/REFRESHRATE Hz,
-- and if update faults then an error is displayed, but the script is not stopped
function Protected_Wrapper()
    local success, err = pcall(Update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, 1000 * REFRESH_RATE
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return Protected_Wrapper()
end

-- wait a bit for AP to come up then start running update loop
if FWVersion:type() == 3 and Q_ENABLE:get() == 1 and TERRAIN_ENABLE:get() == 1 then
    if arming:is_armed() then -- no delay if armed
        return Delayed_Startup()
    else
        return Delayed_Startup, 1000 * STARTUP_DELAY
    end
else
    gcs:send_text(MAV_SEVERITY.NOTICE,string.format("%s: Must run on QuadPlane with terrain follow", SCRIPT_NAME_SHORT))
end
