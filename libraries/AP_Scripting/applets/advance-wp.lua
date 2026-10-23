--[[----------------------------------------------------------------------------

advance-wp ArduPilot Lua script

Set WAYPT_ADVANCE to an aux function number (e.g. 300).
Set RCx_OPTION to the chosen aux function number (preferably on a momentary switch).

When the RC switch is activated, waypoint index will be advanced to the next waypoint
(wraps to WP1 after the last waypoint).

Mission Planner's Aux Function tab can be used in lieu of dedicating RC channels.

Optionally:
    Set WAYPT_ANNOUNCE to another aux function number (e.g. 301).
    Set WAYPT_ANNOUNCE_S to desired interval (s) between waypoint announcements (0 to disable).
    Set WAYPT_BUZ_ENABLE to 1 to enable buzzer feedback vs waypoint distance.
    Set RCx_OPTION to the chosen aux function number.

    When the announce switch is activated, the current waypoint index, bearing, and distance
    will be broadcast as a GCS message every WAYPT_ANNOUNCE_S seconds (useful when using a
    telemetry link like "Yaapu" where named float values are not always readily displayed).

    If WAYPT_BUZ_ENABLE is set, the buzzer will increase in frequency and pitch as distance
    to the selected waypoint decreases (useful if no telemetry source is readily available).

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- Apr 2024

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

local RUN_INTERVAL_MS = 100
local PARAM_TABLE_KEY = 193
local PARAM_TABLE_PREFIX = 'WAYPT_'
local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

-- borrowed from Rover QuikTune
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("Advance WP: Could not find %s parameter", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
        string.format('Could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

local function get_wp_location(item)
    local loc = Location()
    loc:lat(item:x())
    loc:lng(item:y())
    loc:alt(math.floor(item:z() * 100))
    return loc
end

local function get_pitch_by_distance(distance)
    local max_distance = 300
    local min_distance = 0.01
    local total_notes = 73

    distance = math.max(min_distance, math.min(distance, max_distance))
    local scale_factor = 105                                    -- scale factor adjusted for a good spread over the distance range
    local log_ratio = math.log(max_distance / distance)
    local max_log_ratio = math.log(max_distance / min_distance) -- max possible value of log_ratio
    local log_distance_scaled = log_ratio / max_log_ratio * total_notes * scale_factor / 100
    local note_index = math.min(math.floor(log_distance_scaled), total_notes - 1)

    return 'N' .. math.max(1, note_index)
end

local function get_buzz_interval(distance)
    local max_distance = 100
    local min_distance = 0.01
    local max_interval = 2000
    local min_interval = 250

    distance = math.max(min_distance, math.min(distance, max_distance))
    local interval = max_interval - (max_interval - min_interval) * ((max_distance - distance) / max_distance)
    return math.floor(interval)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'Advance WP: Could not add param table')

local WAYPT_ADVANCE = bind_add_param('ADVANCE', 1, 300)
local WAYPT_ANNOUNCE = bind_add_param('ANNOUNCE', 2, 301)
local WAYPT_ANNOUNCE_S = bind_add_param('ANNOUNCE_S', 3, 0)
local WAYPT_BUZ_ENABLE = bind_add_param('BUZ_ENABLE', 4, 1)

local last_advance_sw_pos = -1
local last_announce_ms = uint32_t(0)
local last_buzz_ms = uint32_t(0)
function update()
    ---- WAYPT_ADVANCE ----
    local advance_opt = WAYPT_ADVANCE:get()
    if not advance_opt then return update, RUN_INTERVAL_MS end

    local adv_sw_pos = rc:get_aux_cached(advance_opt)
    if not adv_sw_pos then return update, RUN_INTERVAL_MS end

    local num_commands = mission:num_commands()
    if num_commands < 1 then return update, RUN_INTERVAL_MS end

    if adv_sw_pos > 0 and adv_sw_pos ~= last_advance_sw_pos then
        local nav_index = mission:get_current_nav_index()
        local new_index = (nav_index + 1) % mission:num_commands()
        mission:set_current_cmd(new_index)
        gcs:send_text(MAV_SEVERITY.NOTICE, ('Advance WP -> %d'):format(mission:get_current_nav_index()))
    end
    last_advance_sw_pos = adv_sw_pos or 0

    ---- WAYPT_ANNOUNCE ----
    local announce_s = WAYPT_ANNOUNCE_S:get()
    if not announce_s then return update, RUN_INTERVAL_MS end
    if announce_s <= 0 then return update, RUN_INTERVAL_MS end

    local announce_opt = WAYPT_ANNOUNCE:get()
    if not announce_opt then return update, RUN_INTERVAL_MS end

    local ann_sw_pos = rc:get_aux_cached(announce_opt)
    if not ann_sw_pos then return update, RUN_INTERVAL_MS end

    local now = millis()

    if ann_sw_pos > 0 then
        -- to work when mission is inactive, need to convert current nav item to location
        local nav_index = mission:get_current_nav_index()
        local item = mission:get_item(nav_index)
        local wp_loc = get_wp_location(item)
        local cur_loc = ahrs:get_location()
        if cur_loc then
            local bearing = math.deg(cur_loc:get_bearing(wp_loc))
            local distance = cur_loc:get_distance(wp_loc)

            local buzz_enable = WAYPT_BUZ_ENABLE:get()
            if buzz_enable and buzz_enable > 0 and now - last_buzz_ms > get_buzz_interval(distance) then
                notify:play_tune('MFT240MSL8' .. get_pitch_by_distance(distance))
                last_buzz_ms = now
            end

            if now - last_announce_ms > announce_s * 1000 then
                gcs:send_text(MAV_SEVERITY.NOTICE, ('WP %d: %03.0f° / %.3fm'):format(nav_index, bearing, distance))
                last_announce_ms = now
            end
        elseif now - last_announce_ms > announce_s * 1000 then
            gcs:send_text(MAV_SEVERITY.WARNING, 'Advance WP: Invalid AHRS location')
            last_announce_ms = now
        end
    end

    return update, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY.INFO, 'Advance WP Script Active')

return update, RUN_INTERVAL_MS
