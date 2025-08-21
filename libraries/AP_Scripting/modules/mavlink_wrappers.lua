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

   This ArduPilot lua module should be in stalled in a "modules" subdirectory under the scripts
   directory on your SD card or execution directory in SITL.

   This module provides wrapper functions for some commonly used Lua bindings 
   for MAVLink MAV_CMD command_int commands.

--]]

local MAVLink = {}

MAVLink.SCRIPT_VERSION = "4.6.0-003"
MAVLink.SCRIPT_NAME = "MAVLink wrappers"
MAVLink.SCRIPT_NAME_SHORT = "MAVwrappers"

MAVLink.SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
-- these are the MAVLink altitude frames
MAVLink.FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3, GLOBAL_RELATIVE_ALT_INT = 6,
                    GLOBAL_TERRAIN_ALT = 10, GLOBAL_TERRAIN_ALT_INT = 11}
-- these are the ArduPilot Location::ALtFrame frames
MAVLink.ALT_FRAME = { ABSOLUTE = 0, ABOVE_HOME = 1, ABOVE_ORIGIN = 2, ABOVE_TERRAIN = 3 }
MAVLink.CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192,
                    GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAVLink.SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAVLink.HEADING_TYPE = { COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

MAVLink.PLANE_FLIGHT_MODE = {MANUAL=0, CIRCLE=1, STABILIZE=2, TRAINING=3, ACRO=4,
                                FLY_BY_WIRE_A = 5, FLY_BY_WIRE_B = 6, CRUISE        = 7, AUTOTUNE      = 8,
                                AUTO          = 10,RTL           = 11,LOITER        = 12,TAKEOFF       = 13,
                                AVOID_ADSB    = 14,GUIDED        = 15,INITIALISING  = 16,
                                QSTABILIZE    = 17,QHOVER        = 18,QLOITER       = 19,QLAND         = 20,
                                QRTL          = 21,QAUTOTUNE     = 22,QACRO         = 23,
                                THERMAL       = 24,LOITER_ALT_QLAND = 25,AUTOLAND      = 26}

function MAVLink.NaN()
    return 0/0
end

MAVLink.previous_speed = -1

function MAVLink.set_vehicle_speed(speed)
    local new_speed = speed.speed or -2.0   -- special airspeed value meaning "default"
    local speed_type = speed.type or MAVLink.SPEED_TYPE.AIRSPEED
    local throttle = speed.throttle or 0.0
    local slew = speed.slew or 0.0
    local vehicle_mode = vehicle:get_mode()

    if new_speed == MAVLink.previous_speed or new_speed <= 0 then
        return
    end
    MAVLink.previous_speed = new_speed
    if FWVersion:type() == 3 and vehicle_mode == MAVLink.PLANE_FLIGHT_MODE.GUIDED and
       speed_type == MAVLink.SPEED_TYPE.AIRSPEED and speed.slew ~= 0 then
        local mavlink_result = gcs:run_command_int(MAVLink.CMD_INT.GUIDED_CHANGE_SPEED, { frame = MAVLink.FRAME.GLOBAL,
                                  p1 = speed_type,
                                  p2 = new_speed,
                                  p3 = slew })
        if mavlink_result > 0 then
            return false
        end
    else
        local mavlink_result = gcs:run_command_int(MAVLink.CMD_INT.DO_CHANGE_SPEED, { frame = MAVLink.FRAME.GLOBAL,
                                  p1 = speed_type,
                                  p2 = new_speed,
                                  p3 = throttle })
        if mavlink_result > 0 then
            return false
        end
    end
    return true
end

function MAVLink.set_vehicle_target_location(target)
    local alt_frame = target.frame or MAVLink.ALT_FRAME.ABSOLUTE
    local groundspeed = target.speed or -1
    local bitmask = target.bitmask or 0
    local radius = target.radius or 0
    local yaw = target.yaw or MAVLink.NaN()
    local mavlink_result = gcs:run_command_int(MAVLink.CMD_INT.DO_REPOSITION, { 
            frame = MAVLink.alt_frame_to_mavlink(alt_frame),
            p1 = groundspeed,
            p2 = bitmask,
            p3 = radius,
            p4 = yaw,
            x = target.lat, y = target.lng, z = target.alt})

    if mavlink_result > 0 then
        gcs:send_text(MAVLink.SEVERITY.ERROR, MAVLink.SCRIPT_NAME_SHORT .. string.format(": MAVLink DO_REPOSITION returned %d", mavlink_result))
        return false
    end
    return true
end

function MAVLink.alt_frame_to_mavlink(alt_frame)
    local mavlink_frame = MAVLink.FRAME.GLOBAL
    if (alt_frame == MAVLink.ALT_FRAME.ABOVE_TERRAIN) then
        mavlink_frame = MAVLink.FRAME.GLOBAL_TERRAIN_ALT
    elseif (alt_frame == MAVLink.ALT_FRAME.ABOVE_HOME) then
       mavlink_frame = MAVLink.FRAME.GLOBAL_RELATIVE_ALT
    end
    return mavlink_frame
end

function MAVLink.mavlink_frame_to_alt(mavlink_frame)
    local alt_frame = MAVLink.ALT_FRAME.ABSOLUTE
    if (mavlink_frame == MAVLink.FRAME.GLOBAL_RELATIVE_ALT or
       mavlink_frame == MAVLink.FRAME.GLOBAL_RELATIVE_ALT_INT) then
        alt_frame = MAVLink.ALT_FRAME.ABOVE_HOME
    elseif (mavlink_frame == MAVLink.FRAME.GLOBAL_TERRAIN_ALT or
        mavlink_frame == MAVLink.FRAME.GLOBAL_TERRAIN_ALT_INT) then
         alt_frame = MAVLink.ALT_FRAME.ABOVE_TERRAIN
    end
    return alt_frame
end

function MAVLink.set_vehicle_target_altitude(target)
    local velocity = target.velocity or 1000.0 -- default to maximum z acceleration
    local alt = target.alt or nil
    local alt_frame = target.alt_frame or MAVLink.ALT_FRAME.ABOVE_HOME
    if alt == nil then
       gcs:send_text(MAVLink.SEVERITY.ERROR, MAVLink.SCRIPT_NAME_SHORT .. ": set_vehicle_target_altitude no altiude")
       return
    end
    -- GUIDED_CHANGE_ALTITUDE takes altitude in meters
    local mavlink_result = gcs:run_command_int(MAVLink.CMD_INT.GUIDED_CHANGE_ALTITUDE, {
                               frame = MAVLink.alt_frame_to_mavlink(alt_frame),
                               p3 = velocity,
                               z = alt })
    if mavlink_result > 0 then
         gcs:send_text(MAVLink.SEVERITY.ERROR, MAVLink.SCRIPT_NAME_SHORT .. string.format(": MAVLink GUIDED_CHANGE_ALTITUDE returned %d", mavlink_result))
         return false
    end
    return true
end

return MAVLink

