--[[----------------------------------------------------------------------------

TerrainDetector ArduPilot Lua script

Uses gyro and accelerometer data to detect rough terrain and slow the
vehicle's speed accordingly during auto missions.

Provides a set of custom tuning parameters:
ROUGH_SPEED     : (m/s) rough terrain desired speed, set to -1 to disable detection
ROUGH_TIMEOUT_MS: (ms) min time to remain slowed down for rough terrain
ROUGH_GZ_MAX    : (G) slow down on Gz impulses greater than this value
ROUGH_RATE_MAX  : (deg/s) slow down on rate transients greater than this value

These gain values are used for smooth terrain detection.
Smaller values (less than 1.0) are more restrictive about resuming normal speed.
ROUGH_GZ_GAIN   : Gz impulse gain
ROUGH_RATE_GAIN : gyro transient rate gain

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Jul 2022

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html

Concept first presented during this live stream:
https://www.youtube.com/watch?v=UdXGXjigxAo&t=7155s
Credit to @ktrussell for the idea and discussion!
------------------------------------------------------------------------------]]

local SCRIPT_NAME     = 'TerrainDetector'
local RUN_INTERVAL_MS =  25  -- needs to be pretty fast for good detection
local SBY_INTERVAL_MS = 500  -- slower interval when detection is disabled
local PARAM_TABLE_KEY = 117  -- unique index value between 0 and 200

-- GCS messages
local VERBOSE_MODE = 1 -- 0 to suppress all GCS messages,
                       -- 1 for normal status messages
                       -- 2 for additional debug messages
local MSG_NORMAL   = 1
local MSG_DEBUG    = 2

-- MAVLink values
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO    = 6

-- mathematical/physical constants
local G          = -9.81 -- m/s/s

-- create custom parameter set
local function add_params(key, prefix, tbl)
    assert(param:add_table(key, prefix, #tbl), string.format('Could not add %s param table.', prefix))
    for num = 1, #tbl do
        assert(param:add_param(key, num,  tbl[num][1], tbl[num][2]), string.format('Could not add %s%s.', prefix, tbl[num][1]))
    end
end

add_params(PARAM_TABLE_KEY, 'ROUGH_', {
    --  { name, default value },
        { 'SPEED',        0.7 },
        { 'GZ_MAX',      1.33 },
        { 'RATE_MAX',      28 },
        { 'GZ_GAIN',      0.9 },
        { 'RATE_GAIN',    0.8 },
        { 'TIMEOUT_MS',  7500 }
    })

-- wrapper for gcs:send_text()
local function gcs_msg(msg_type, severity, txt)
    if type(msg_type) == 'string' then
    -- allow just a string to be passed for simple/routine messages
        txt      = msg_type
        msg_type = MSG_NORMAL
        severity = MAV_SEVERITY_INFO
    end
    if type(severity) == 'string' then
    -- allow just severity and string to be passed for normal messages
        txt = severity
        severity = msg_type
        msg_type = MSG_NORMAL
    end
    if msg_type <= VERBOSE_MODE then
        gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
    end
end

local last_g_z   = 0  -- to calculate impulse Gz
local timeout_ms = 0

-- debug values
local max_g_z       = 0
local max_gyro_rate = 0

local wp_speed_normal = nil
local wp_speed_rough = nil
local impulse_gz_threshold = nil
local gyro_rate_threshold = nil
local rough_terrain_timeout_ms = nil
local impulse_gain = nil
local gyro_gain = nil

function standby()
    wp_speed_rough = param:get('ROUGH_SPEED')
    if wp_speed_rough < 0 then return standby, SBY_INTERVAL_MS end

    if mission:state() == mission.MISSION_RUNNING then
        -- only poll remaining parameters at start of mission
        wp_speed_normal          = param:get('WP_SPEED')
        impulse_gz_threshold     = param:get('ROUGH_GZ_MAX')
        gyro_rate_threshold      = param:get('ROUGH_RATE_MAX')
        rough_terrain_timeout_ms = param:get('ROUGH_TIMEOUT_MS')
        impulse_gain             = param:get('ROUGH_GZ_GAIN')
        gyro_gain                = param:get('ROUGH_RATE_GAIN')
        -- if ROUGH_SPEED param not set or invalid, use half of WP_SPEED
        if wp_speed_rough == 0 or wp_speed_rough > wp_speed_normal then
            wp_speed_rough = wp_speed_normal / 2
            gcs_msg(MAV_SEVERITY_WARNING, 'ROUGH_SPEED invalid, using half WP_SPEED')
        end

        last_g_z = 0
        return do_normal_speed, RUN_INTERVAL_MS
    end
    return standby, SBY_INTERVAL_MS
end

function initiate_rough_speed()
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Slowing for rough terrain')
    vehicle:set_desired_speed(wp_speed_rough)
    last_g_z   = 0
    timeout_ms = millis() + rough_terrain_timeout_ms
    max_g_z       = 0
    max_gyro_rate = 0
    return do_rough_speed, RUN_INTERVAL_MS
end

function initiate_normal_speed()
    gcs_msg(MSG_NORMAL, MAV_SEVERITY_WARNING, 'Resuming normal speed')
    vehicle:set_desired_speed(wp_speed_normal)
    last_g_z = 0
    return do_normal_speed, RUN_INTERVAL_MS
end

function do_normal_speed()
    if mission:state() ~= mission.MISSION_RUNNING then
        return standby, SBY_INTERVAL_MS
    end

    local g_z         = ahrs:get_accel():z() / G  -- convert to G from m/s/s
    local gyro        = ahrs:get_gyro()
    -- sample max of roll/pitch rates
    local gyro_rate   = math.deg(math.max(math.abs(gyro:x()), math.abs(gyro:y())))
    local impulse_g_z = math.abs(g_z - last_g_z)  -- sample delta is the measured impulse
    last_g_z = g_z

    if impulse_g_z > impulse_gz_threshold then  -- slow down
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Impulse - %.2f Gz', impulse_g_z))
        return initiate_rough_speed, RUN_INTERVAL_MS
    end

    if gyro_rate > gyro_rate_threshold then  -- slow down
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Transient - %.2f deg/s', gyro_rate))
        return initiate_rough_speed, RUN_INTERVAL_MS
    end

    return do_normal_speed, RUN_INTERVAL_MS
end

function do_rough_speed()
    if mission:state() ~= mission.MISSION_RUNNING then
        return standby, SBY_INTERVAL_MS
    end

    local g_z         = ahrs:get_accel():z() / G  -- convert to G from m/s/s
    local gyro        = ahrs:get_gyro()
    -- sample max of roll/pitch rates
    local gyro_rate   = math.deg(math.max(math.abs(gyro:x()), math.abs(gyro:y())))
    local impulse_g_z = math.abs(g_z - last_g_z) -- sample delta is the measured impulse
    last_g_z = g_z

    max_g_z = math.max(g_z, max_g_z)
    max_gyro_rate = math.max(gyro_rate, max_gyro_rate)

    local now = millis()

    if impulse_g_z > impulse_gz_threshold * impulse_gain then  -- stay slow
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Timer reset - %.2fGz', impulse_g_z))
        max_g_z = 0
        timeout_ms = now + rough_terrain_timeout_ms
        return do_rough_speed, RUN_INTERVAL_MS
    end

    if gyro_rate > gyro_rate_threshold * gyro_gain then  -- stay slow
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Timer reset - %.2f deg/s', gyro_rate))
        max_gyro_rate = 0
        timeout_ms = now + rough_terrain_timeout_ms
        return do_rough_speed, RUN_INTERVAL_MS
    end

    if now > timeout_ms then  -- no longer in rough terrain
        gcs_msg(MSG_DEBUG, MAV_SEVERITY_INFO, string.format('Max - %.2fGz ... %.2f deg/s', max_g_z, max_gyro_rate))
        return initiate_normal_speed, RUN_INTERVAL_MS
    end

    return do_rough_speed, RUN_INTERVAL_MS
end

gcs_msg('Script active')

return standby, SBY_INTERVAL_MS
