--[[----------------------------------------------------------------------------

SaveTurns ArduPilot Lua script

Saves the locations of vehicle turns as waypoints in the current nav mission.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Nov 2021, revised Apr 2022

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]
-- luacheck: only 0

local SCRIPT_NAME = 'SaveTurns'

--------  USER EDITABLE GLOBALS  --------
local RC_OPTION       = 300  -- RC option number for switched control (300, 301, etc)
local MIN_DIST        = 2.0  -- (m)   min distance between waypoints
local HDG_DELTA       = 8.0  -- (deg) save wp after heading change of this magnitude
local RUN_INTERVAL_MS = 200  -- (ms)  how often to run this script
local VERBOSE_MODE    =   2  -- 0 to suppress all GCS messages,
                             -- 1 for normal status messages
                             -- 2 for additional GPS/debug messages

--------  MAVLINK/AUTOPILOT 'CONSTANTS'  --------
local ROVER_MODE_AUTO = 10
local STANDBY         =  0
local SAVE_WPS        =  1
local CLEAR_WPS       =  2
local WAYPOINT        = 16   -- waypoint command
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO    = 6
local MSG_NORMAL           = 1
local MSG_DEBUG            = 2

local RC_CHAN = rc:find_channel_for_option(RC_OPTION)
local last_wp = Location()
local last_yaw = 999.0

-- wrapper for gcs:send_text()
local function gcs_msg(msg_type, severity, txt)
    if type(msg_type) == 'string' then
    -- allow just a string to be passed for simple/routine messages
        txt      = msg_type
        msg_type = MSG_NORMAL
        severity = MAV_SEVERITY_INFO
    end
    if msg_type <= VERBOSE_MODE then
        gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
    end
end

local function yaw_diff(yaw1, yaw2)
    --https://stackoverflow.com/questions/5024375/getting-the-difference-between-two-headings
    return math.abs((yaw2 - yaw1 + 540.0) % 360.0 - 180.0)
end

local function new_mission()
    local home = ahrs:get_home()
    local item = mavlink_mission_item_int_t()

    mission:clear()

    item:command(WAYPOINT)
    item:x(home:lat())
    item:y(home:lng())
    item:z(home:alt())

    if not mission:set_item(0, item) then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Failed to create new mission')
        return false
    end

    return true
end

local function save_wp(position, index)
    local item = mavlink_mission_item_int_t()

    if (not position) then
		gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, string.format('WP %d - invalid position', index))
        return false
    end

    item:command(WAYPOINT)
    item:x(position:lat())
    item:y(position:lng())
    item:z(0)

    if not mission:set_item(index, item) then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, string.format('WP %d - failed to save', index))
        return false
    end

    return true
end

function collect_breadcrumbs()
    if vehicle:get_mode() == ROVER_MODE_AUTO then
        gcs_msg('AUTO mode - saving canceled')
        return standby, RUN_INTERVAL_MS
    end
    local sw_pos = RC_CHAN:get_aux_switch_pos()
    if sw_pos ~= SAVE_WPS then return standby, RUN_INTERVAL_MS end

    if not ahrs:healthy() then return collect_breadcrumbs, RUN_INTERVAL_MS end

    local cur_pos = ahrs:get_location()
    local cur_yaw = ahrs:get_yaw() * 180.0 / math.pi

    if cur_pos:get_distance(last_wp) < MIN_DIST then
        last_yaw = cur_yaw
        return collect_breadcrumbs, RUN_INTERVAL_MS
    end

    if yaw_diff(cur_yaw, last_yaw) > HDG_DELTA then
        local idx = mission:num_commands()
        if save_wp(cur_pos, idx) then
            gcs_msg(string.format('WP %d -  saved', idx))
            last_wp = cur_pos
            last_yaw = cur_yaw
        end
    end

    return collect_breadcrumbs, RUN_INTERVAL_MS
end

function await_switch_change()
    local sw_pos = RC_CHAN:get_aux_switch_pos()
    if sw_pos == STANDBY then return standby, RUN_INTERVAL_MS end
    if sw_pos == SAVE_WPS then return collect_breadcrumbs, RUN_INTERVAL_MS end
    return await_switch_change, RUN_INTERVAL_MS
end

function standby()
    if vehicle:get_mode() == ROVER_MODE_AUTO then return standby, RUN_INTERVAL_MS end
    local sw_pos = RC_CHAN:get_aux_switch_pos()
    if sw_pos == SAVE_WPS then
        last_wp = Location()
        last_yaw = 999.0
        return collect_breadcrumbs, RUN_INTERVAL_MS
    end
    if sw_pos == CLEAR_WPS then
        if new_mission() then
            gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Mission cleared')
        end
        return await_switch_change, RUN_INTERVAL_MS
    end
    return standby, RUN_INTERVAL_MS
end

function initialize()
    if not ahrs:healthy() then return initialize, RUN_INTERVAL_MS end
    gcs_msg('Ready')
    return standby, RUN_INTERVAL_MS
end

gcs_msg('Awaiting AHRS initialization...')

return initialize()
