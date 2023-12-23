--[[----------------------------------------------------------------------------

MinFixType ArduPilot Lua script

Checks for mission running and commands a hold/pause if the GPS fix type is less
than a threshold value.

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Aug 2021, revised Apr 2022

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local SCRIPT_NAME = 'MinFixType'

--------  MAVLINK/AUTOPILOT 'CONSTANTS'  --------
local ROVER_MODE_MANUAL       =  0
local ROVER_MODE_HOLD         =  4
local ROVER_MODE_AUTO         = 10
local MAV_SEVERITY_WARNING    =  4
local MAV_SEVERITY_INFO       =  6

--------  USER EDITABLE GLOBALS  --------
local GPS_INSTANCE = 0                 -- GPS to monitor (moving base, most likely)
local MIN_FIX_TYPE = 6                 -- see table below
local MSN_PAUSE_MODE     = ROVER_MODE_MANUAL  -- mode to command when GPS fix is inadequate
local THR_SAFEGUARD_MODE = ROVER_MODE_HOLD    -- mode to command if mission paused with non-zero throttle
local BAD_FIX_TIMEOUT  = 1600          -- how long a bad fix type must be present before pausing the mission
local GOOD_FIX_TIMEOUT =  600          -- how long a good fix type must be present before resuming the mission
local RUN_INTERVAL_MS  =  200          -- (ms) how often to run this script (50-250 should work fine)
local VERBOSE_MODE     =    2  -- 0 to suppress all GCS messages,
                               -- 1 for normal status messages
                               -- 2 for additional debug messages
local MSG_NORMAL = 1
local MSG_DEBUG  = 2

local FIX_TYPES = {
    [0] = 'No GPS',  -- Lua arrays are 1 based unless you specify discrete indices like this
    [1] = 'No Fix',
    [2] = '2D Fix',
    [3] = '3D Fix',
    [4] = 'DGPS Fix',
    [5] = 'RTK Float',
    [6] = 'RTK Fixed',
    [7] = 'Static Fixed',
    [8] = 'PPP, 3D'}

local MODE_THRESHOLDS = {1231, 1361, 1491, 1621, 1750, 2050}

local USER_MODES = {
    param:get('MODE1'),
    param:get('MODE2'),
    param:get('MODE3'),
    param:get('MODE4'),
    param:get('MODE5'),
    param:get('MODE6')
}

local MODE_CH  = param:get('MODE_CH')
local THR_CH   = param:get('RCMAP_THROTTLE')
local THR_TRIM = param:get(string.format('RC%d_TRIM', THR_CH))
local THR_DZ   = param:get(string.format('RC%d_DZ', THR_CH))

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

-- return RC transmitter selected mode
local function get_user_mode()
    local pwm = rc:get_pwm(MODE_CH)
    local mode_num = 6
    for i, threshold in pairs(MODE_THRESHOLDS) do
        if (pwm < threshold) then
            mode_num = i
            break
        end
    end
    return USER_MODES[mode_num]
end

local function get_pause_mode()
    if math.abs(rc:get_pwm(THR_CH) - THR_TRIM) > THR_DZ then
        return THR_SAFEGUARD_MODE
    end
    return MSN_PAUSE_MODE
end

function resume_mission()
    if get_user_mode() ~= ROVER_MODE_AUTO then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Pause Canceled - Mode Change')
        return standby, RUN_INTERVAL_MS
    end
    if not arming:is_armed() then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Pause Canceled - Disarmed')
        return standby, RUN_INTERVAL_MS
    end

    if gps:status(GPS_INSTANCE) < MIN_FIX_TYPE then
        return mission_paused, RUN_INTERVAL_MS
    end

    vehicle:set_mode(ROVER_MODE_AUTO)
    gcs_msg('Mission Resumed')
    return monitor, RUN_INTERVAL_MS
end

function mission_paused()
    if get_user_mode() ~= ROVER_MODE_AUTO then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Pause Canceled - Mode Change')
        return standby, RUN_INTERVAL_MS
    end
    if not arming:is_armed() then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Pause Canceled - Disarmed')
        return standby, RUN_INTERVAL_MS
    end

    local fix_type = gps:status(GPS_INSTANCE)
    if fix_type >= MIN_FIX_TYPE then
        return resume_mission, GOOD_FIX_TIMEOUT
    end

    if vehicle:get_mode() == THR_SAFEGUARD_MODE then
        local mode = get_pause_mode()
        if mode == MSN_PAUSE_MODE then
            gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Throttle neutral')
            vehicle:set_mode(mode)
        end
        return mission_paused, RUN_INTERVAL_MS
    end

    -- for edge cases where a non-RC command to resume was issued but fix type is still unsuitable
    if vehicle:get_mode() ~= MSN_PAUSE_MODE then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Cannot resume - ' .. FIX_TYPES[fix_type])
        vehicle:set_mode(get_pause_mode())
    end
    return mission_paused, RUN_INTERVAL_MS
end

function transition_to_pause()
    if mission:state() ~= mission.MISSION_RUNNING then
        return standby, RUN_INTERVAL_MS
    end

    local fix_type = gps:status(GPS_INSTANCE)

    if fix_type >= MIN_FIX_TYPE then
        return monitor, RUN_INTERVAL_MS
    end

    local mode = get_pause_mode()
    if mode == THR_SAFEGUARD_MODE then
        gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Throttle not neutral!')
    end

    vehicle:set_mode(mode)
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Mission Paused - ' .. FIX_TYPES[fix_type])

    return mission_paused, RUN_INTERVAL_MS
end

function monitor()  -- monitor for reduced GPS fix state during auto mission
    if mission:state() ~= mission.MISSION_RUNNING then
        return standby, RUN_INTERVAL_MS
    end

    local fix_type = gps:status(GPS_INSTANCE)

    if fix_type < MIN_FIX_TYPE then
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_WARNING, 'GPS ' .. (GPS_INSTANCE + 1) .. ' - ' .. FIX_TYPES[fix_type])
        return transition_to_pause, BAD_FIX_TIMEOUT
    end

    return monitor, RUN_INTERVAL_MS
end

function standby()  -- wait here until an auto mission is active
    if mission:state() == mission.MISSION_RUNNING then
        return monitor, RUN_INTERVAL_MS
    end
    return standby, RUN_INTERVAL_MS
end

gcs_msg('Script active')

return standby()
