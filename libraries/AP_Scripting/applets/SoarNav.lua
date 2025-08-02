--[[
SoarNav: an advanced feature for hunting thermals by Marco Robustini.
-- Version 0.9.5 - 2025/08/02

In early versions of SOAR mode in Ardupilot, which I collaborated on with
the author, when the glider exited THERMAL mode the heading pointed
approximately to home. This feature that I wanted was later removed
because it was considered unnecessary by some. I therefore created a LUA
that reimplements it but in an advanced way.
By activating SoarNav when the autopilot has SOAR active and Cruise or
FBWB is used as the flight mode the script continuously monitors a
predefined radius from the home or a custom polygon area. But it doesn't
just fly randomly. SoarNav uses a sophisticated set of strategies to
maximize flight time.

================================================================================
Key Features
================================================================================

- **Systematic Exploration (Virtual Grid)**:
  The script divides the flight area into a virtual grid and systematically
  explores it. It generates new waypoints by choosing primarily from cells
  that have not yet been visited. Once all valid cells have been explored,
  the grid's visited status is reset, and the exploration cycle begins anew.
  The grid is automatically re-initialized if the `SNAV_MAX_DIST` parameter changes.

- **Advanced Thermal Analysis (Strength & Quality)**:
  The script doesn't just remember *where* a thermal was, but also *how good*
  it was. It analyzes the climb rate during thermal mode to save its average
  and peak strength, and also assesses its **consistency** (steady vs. variable lift).

- **Intelligent Thermal Memory with Boundary Checks**:
  When the flight controller enters THERMAL mode, the script saves a "hotspot"
  corrected for wind drift. It intelligently **ignores and discards thermals**
  detected outside the defined operational area (polygon or radius). Wind
  compensation uses an adaptive factor based on the thermal's measured strength
  (e.g., stronger thermals lead to longer compensation times for more accurate drift prediction),
  enhancing the accuracy of return-to-thermal navigation.

- **Energy-Aware & Dual-Strategy Decision Making**:
  The script's core logic is based on energy management, utilizing an altitude hysteresis
  to ensure stable transitions between energy states.
    - **Low Energy**: When low on altitude, it prioritizes safety by navigating
      deterministically to the **strongest known thermal** to regain height.
    - **Normal Energy**: With ample altitude, it uses a probabilistic "tournament"
      selection, picking two random hotspots and targeting the stronger of the pair.
      This balances exploiting known lift with re-evaluating other good areas,
      and intelligently avoids immediately re-targeting the last used hotspot if others are available.

- **Adaptive Strategy (Dynamic Memory Chance)**:
  The script dynamically adjusts its strategy. Based on the number of recent
  thermals found within a history window, it increases or decreases the probability of returning to a
  known hotspot versus exploring a new, unknown grid cell.

- **Dynamic Thermal Drift Prediction**:
  When targeting a known hotspot, the script uses the current wind vector and
  the hotspot's age to predict its new, drifted position, increasing the
  chances of a successful intercept.

- **Safety Pre-flight Checks (Parameter Validation)**:
  Before starting, the script validates key `SNAV_*` parameters
  to ensure they are within logical ranges. This prevents errors from
  critical misconfigurations, and the script will disable itself if an
  unsafe value (e.g., a negative radius) is detected.

- **Tactical Anti-Stuck Navigation (Upwind Repositioning)**:
  If the script detects that the aircraft is not making consistent progress towards a
  waypoint (i.e., is "stuck") due to headwind or other factors (using an adaptive "stuck counter" based on wind speed),
  it initiates an automatic maneuver. It temporarily flies
  to an upwind position to then approach the original target from a more
  favorable trajectory.
  
- **Mid-flight Tactical Re-route**:
  To introduce strategic unpredictability, the script can change target mid-flight.
  When halfway to a destination, it checks if the path is long (e.g., over half the
  area's max diameter) and the grid is not almost fully explored. If these conditions
  are met, there is a random chance the script will abandon the current
  target (returning it to the unvisited list) and navigate to a new unexplored cell.

- **Adaptive Waypoint Timeout**:
  The script intelligently adjusts the maximum time allowed to reach a waypoint
  based on current wind conditions, allowing more time in strong headwinds (up to 2.5x base timeout).

- **Intuitive Pilot Override & Stick Gestures**:
  - **Temporary Override**: Moving the Pitch or Yaw sticks instantly pauses the
    script's autonomous navigation. It resumes a few seconds after sticks are centered.
  - **Persistent Override (Roll Gesture)**: A rapid sequence of roll stick movements
    toggles a persistent manual override, allowing the pilot to fly freely until the
    gesture is repeated.
  - **Dynamic Area Re-centering (Pitch Gesture)**: A rapid sequence of pitch stick
    movements re-centers the circular search area to the aircraft's current location,
    also resetting the exploration grid.

- **Dual Area System (Radius or Polygon)**:
  The search area can be defined as a circular radius from a dynamically chosen
  center point (which can be re-centered by gesture) or a custom polygon loaded from the SD card,
  with automatic polygon closing if needed.

- **Advanced Logging System**:
  A multi-level logging system (controlled by SNAV_LOG_LVL) provides clear
  operational feedback, from key events to detailed real-time status for debugging.

  Example of a Level 2 status message sent to the GCS:

  27/07/2025 07:31:37 : SoarNav: Thermal mem: 0 active
  27/07/2025 07:31:37 : SoarNav: Grid: 7/116 expl. 6% | Curr. Cell: 157
  27/07/2025 07:31:37 : SoarNav: WP: D:263m, Hdg Err:+0, Roll:+0.2
  27/07/2025 07:31:37 : SoarNav: Nav to: Cell: 173


--------------------------------------------------------------------------------
Script Parameters (SNAV_*)
--------------------------------------------------------------------------------
- SNAV_ENABLE: Master switch to enable (1) or disable (0) the script.
- SNAV_LOG_LVL: Sets the verbosity level of messages sent to the Ground Control Station.
    - **0 (Silent)**: Only critical script errors are reported.
    - **1 (Events)**: [Default] Reports key events: script start/stop, waypoint
      reached, new waypoint generation, and thermal memory recording. Ideal for standard use.
    - **2 (Detailed Status)**: Includes all Level 1 events PLUS a periodic
      status report for in-depth, real-time debugging.
- SNAV_MAX_DIST: Defines the radius in meters of the circular flight area around the home
  point. If set to 0, the script will look for a polygon file (snav.poly) on the SD card.
- SNAV_ROLL_LIMIT: The maximum roll (bank) angle, in degrees, that the script will
  command during autonomous navigation.
- SNAV_WP_RADIUS: The acceptance radius in meters. Defines the distance to a waypoint
  at which it is considered 'reached'.
- SNAV_NAV_P / SNAV_NAV_D: Gains for the PD (Proportional-Derivative) navigation controller.
  SNAV_NAV_P controls responsiveness, while SNAV_NAV_D dampens the response for smoother control.
  A roll smoothing factor is also applied for fluid control.
- SNAV_TMEM_ENABLE: Enables (1) or disables (0) the Thermal Memory feature, which allows
  the script to remember and return to found lift areas.
- SNAV_TMEM_LIFE: The lifetime in seconds of a 'hotspot' (a stored thermal). After this
  time, the point is considered expired and is removed from memory.
- SNAV_WIND_COMP: A compensation factor used to estimate the upwind position of a thermal
  relative to where it was detected, improving memory accuracy.

================================================================================
CHANGELOG / DEVELOPER NOTES
================================================================================
- v0.9.5 (2025/08/02): Added Mid-flight Tactical Re-route feature, shortened all
  GCS log messages, minor cosmetic fix.
- v0.9.41 (2025/08/01): Fixed a critical logic bug in the anti-stuck feature
  where the script would not re-engage the original waypoint after completing
  an upwind repositioning maneuver.
- v0.9.4 (2025/07/31): Major refactoring to leverage native ArduPilot APIs
  (e.g., Location objects, get_distance, offset_bearing) instead of manual
  Lua calculations. This drastically improves precision, performance, and
  code robustness. Thanks to Peter Hall for guidance on the correct API
  implementation.
- v0.9.2 (2025/07/29): Initial version published with manual geospatial calculations.

]]

math.randomseed(millis():toint())

-- Mavlink severity levels for GCS messages
local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
}

-- Script operational states
local SCRIPT_STATE = {
    IDLE = 0,
    NAVIGATING = 1,
    PILOT_OVERRIDE = 2,
    THERMAL_PAUSE = 3,
    ERROR = 4
}

--[[
 // @Param: SNAV_ENABLE
 // @DisplayName: SoarNav Enable
 // @Description: Enable script
 // @Values: 0:Disabled,1:Enabled
]]
--[[
 // @Param: SNAV_LOG_LVL
 // @DisplayName: SoarNav Log Verbosity
 // @Description: GCS log verbosity
 // @Values: 0:Silent,1:Normal,2:Detailed
]]
--[[
 // @Param: SNAV_MAX_DIST
 // @DisplayName: Maximum Distance
 // @Description: Maximum distance (meters) from home or 0 for polygon area
]]
--[[
 // @Param: SNAV_ROLL_LIMIT
 // @DisplayName: Roll Limit
 // @Description: Maximum roll angle (degrees) for navigation
]]
--[[
 // @Param: SNAV_WP_RADIUS
 // @DisplayName: Waypoint Radius
 // @Description: Acceptance radius for virtual waypoints (meters)
]]
--[[
 // @Param: SNAV_NAV_P
 // @DisplayName: Navigation P Gain
 // @Description: Navigation P-gain for roll controller (higher is more responsive)
]]
--[[
 // @Param: SNAV_NAV_D
 // @DisplayName: Navigation D Gain
 // @Description: Navigation D-gain for roll controller (dampens response)
]]
--[[
 // @Param: SNAV_TMEM_ENABLE
 // @DisplayName: Thermal Memory Enable
 // @Description: Enable Thermal Memory
 // @Values: 0:Disabled,1:Enabled
]]
--[[
 // @Param: SNAV_TMEM_LIFE
 // @DisplayName: Thermal Hotspot Lifetime
 // @Description: Lifetime of a thermal hotspot (seconds)
]]
--[[
 // @Param: SNAV_WIND_COMP
 // @DisplayName: Wind Compensation Factor
 // @Description: Upwind compensation factor for thermal memory
]]

local PARAM_TABLE_KEY = 151
local PARAM_PREFIX    = "SNAV_"

local param_list = {
    {name = "ENABLE",      default = 0},
    {name = "LOG_LVL",     default = 1},
    {name = "MAX_DIST",    default = 500},
    {name = "ROLL_LIMIT",  default = 30},
    {name = "WP_RADIUS",   default = 30},
    {name = "NAV_P",       default = 0.6},
    {name = "NAV_D",       default = 0.05},
    {name = "TMEM_ENABLE", default = 1},
    {name = "TMEM_LIFE",   default = 1200},
    {name = "WIND_COMP",   default = 60}
}

-- Creates the script's parameter table in ArduPilot
local function add_params()
    assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, #param_list),
           string.format("CRITICAL ERROR: SoarNav - Could not add param table '%s'.", PARAM_PREFIX))
    for i, p in ipairs(param_list) do
        assert(param:add_param(PARAM_TABLE_KEY, i, p.name, p.default),
               string.format("CRITICAL ERROR: SoarNav - Could not add param %s%s.", PARAM_PREFIX, p.name))
    end
end

add_params()

-- Bind script parameters to local variables for efficient access
local p_enable         = Parameter(PARAM_PREFIX .. "ENABLE")
local p_log_lvl        = Parameter(PARAM_PREFIX .. "LOG_LVL")
local p_max_dist       = Parameter(PARAM_PREFIX .. "MAX_DIST")
local p_roll_limit     = Parameter(PARAM_PREFIX .. "ROLL_LIMIT")
local p_wp_radius      = Parameter(PARAM_PREFIX .. "WP_RADIUS")
local p_nav_p          = Parameter(PARAM_PREFIX .. "NAV_P")
local p_nav_d          = Parameter(PARAM_PREFIX .. "NAV_D")
local p_tmem_enable    = Parameter(PARAM_PREFIX .. "TMEM_ENABLE")
local p_tmem_life      = Parameter(PARAM_PREFIX .. "TMEM_LIFE")
local p_wind_comp      = Parameter(PARAM_PREFIX .. "WIND_COMP")
local p_soar_alt_min   = Parameter("SOAR_ALT_MIN")
local p_soar_alt_max   = Parameter("SOAR_ALT_MAX")
local p_sys_roll_limit = Parameter("ROLL_LIMIT_DEG")
local p_rcmap_roll     = Parameter("RCMAP_ROLL")
local p_rcmap_pitch    = Parameter("RCMAP_PITCH")
local p_rcmap_yaw      = Parameter("RCMAP_YAW")

-- Static constants used throughout the script
local SoarNavConstants = {
    pilot_resume_delay_ms = 5000,
    wp_timeout_ms = 300000,
    grid_update_interval_ms = 2000,
    rc_search_interval_ms = 5000,
    thermal_history_window_ms = 900000,

    STUCK_GRACE_PERIOD_MS = 30000,
    STUCK_PROGRESS_CHECK_INTERVAL_MS = 20000,
    STUCK_MIN_PROGRESS_M = 10,

    roll_smoothing_factor = 0.1,
    HYSTERESIS_MARGIN = 5,

    polygon_filename = "snav.poly",
    min_cell_size_m = 30,
    max_total_grid_cells = 128,
    grid_init_cells_per_call = 20,
    MAX_GRID_SEARCH_ATTEMPTS_PER_CALL = 10,

    max_hotspots = 10,
    THERMAL_CONSISTENCY_VARIANCE_THRESHOLD = 0.5,
    MAX_DRIFT_AGE_S = 600,
    MAX_DRIFT_FACTOR = 1.5,

    rc_deadzone = 30,
    PITCH_GESTURE_COUNT_TARGET = 4,
    PITCH_GESTURE_THRESHOLD = 0.5,
    PITCH_GESTURE_TIMEOUT_MS = 2000,
    ROLL_GESTURE_COUNT_TARGET = 4,
    ROLL_GESTURE_THRESHOLD = 0.5,
    ROLL_GESTURE_TIMEOUT_MS = 2000,
    
    mid_flight_reroute_chance = 33,

    rc_thresh_high = 1500,
    rc_opt_soaring_active = 88,
    mode_fbwb = 6,
    mode_cruise = 7,
    mode_thermal = 24,

    max_location_errors = 10
}

-- Global table for managing the script's state
local SoarNavGlobals = {
    script_state = SCRIPT_STATE.IDLE,
    rc_roll_channel = rc:get_channel(1),
    polygon_points = {},
    polygon_bounds = nil,
    last_cell_index = nil,
    use_polygon_area = false,
    target_loc = nil,
    waypoint_start_time_ms = 0,
    last_pilot_input_ms = 0,
    last_heading_error = 0,
    last_commanded_roll_deg = 0,
    thermal_hotspots = {},
    location_error_count = 0,
    grid_cells = {},
    grid_rows = 0,
    grid_cols = 0,
    grid_bounds = nil,
    grid_initialized = false,
    is_initializing = false,
    grid_init_step = 0,
    grid_init_row = 1,
    grid_init_col = 1,
    grid_populate_index = 1,
    last_grid_update_ms = 0,
    rc1_min = 1000, rc1_max = 2000, rc1_trim = 1500,
    rc2_min = 1000, rc2_max = 2000, rc2_trim = 1500,
    rc4_min = 1000, rc4_max = 2000, rc4_trim = 1500,
    rc_limits_read = false,
    thermal_entry_timestamps = {},
    g_waypoint_source_info = "N/A",
    energy_state = "NORMAL",
    was_in_thermal_mode = false,
    is_monitoring_thermal = false,
    last_thermal_sample_ms = 0,
    current_thermal_stats = {},
    last_used_hotspot_timestamp = 0,
    waypoint_search_in_progress = false,
    logged_ignore_message = false,
    last_snav_max_dist_value = -1,
    valid_cell_indices = {},
    unvisited_cell_indices = {},
    distance_to_wp = -1,
    is_repositioning = false,
    original_target_loc = nil,
    last_progress_check_ms = 0,
    distance_at_last_check = -1,
    stuck_counter = 0,
    last_grid_reset_ms = 0,
    dynamic_center_location = nil,
    pitch_gesture_state = "idle",
    pitch_gesture_count = 0,
    pitch_gesture_triggered_this_override = false,
    pitch_gesture_start_ms = 0,
    roll_gesture_state = "idle",
    roll_gesture_count = 0,
    roll_gesture_start_ms = 0,
    manual_override_active = false,
    force_grid_after_reset = false,
    cached_wind = nil,
    wind_cache_time = 0,
    max_operational_distance = 0,
    initial_distance_to_wp = -1,
    reroute_check_armed = false
}

-- Checks if a Location object is inside the defined polygon using a ray-casting algorithm.
---@param loc Location_ud
---@return boolean
local function is_point_in_polygon(loc)
    local poly = SoarNavGlobals.polygon_xy
    local origin = SoarNavGlobals.polygon_origin
    if not poly or not origin or #poly < 3 then return false end

    local xy = origin:get_distance_NE(loc)
    local px = xy:x()
    local py = xy:y()

    local inside = false
    local j = #poly
    for i = 1, #poly do
        local xi, yi = poly[i].x, poly[i].y
        local xj, yj = poly[j].x, poly[j].y
        if ((yi > py) ~= (yj > py)) and
           (px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi) then
            inside = not inside
        end
        j = i
    end
    return inside
end

-- Calculates the initial bearing from one location to another.
local function calculate_bearing(lat1, lon1, lat2, lon2)
    lat1 = tonumber(lat1); lon1 = tonumber(lon1)
    lat2 = tonumber(lat2); lon2 = tonumber(lon2)
    if not lat1 or not lon1 or not lat2 or not lon2 then
        log_gcs(MAV_SEVERITY.NOTICE, 0, string.format("Bearing invalid params: %s %s %s %s.", tostring(lat1), tostring(lon1), tostring(lat2), tostring(lon2)))
        return 0
    end
    lat1 = math.rad(lat1)
    lon1 = math.rad(lon1)
    lat2 = math.rad(lat2)
    lon2 = math.rad(lon2)

    local dLon = lon2 - lon1
    local y = math.sin(dLon) * math.cos(lat2)
    local x = math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    local function atan2(_y, _x)
        if _x == 0 and _y == 0 then return 0 end
        local t = math.atan(_y/_x)
        if _x < 0 then
            if _y >= 0 then
                t = t + math.pi
            else
                t = t - math.pi
            end
        elseif _x == 0 and _y ~= 0 then
            t = (_y > 0) and (math.pi/2) or (-math.pi/2)
        end
        return t
    end
    
    local brng = math.deg(atan2(y, x))
    if brng < 0 then brng = brng + 360 end
    return brng
end

-- Returns the active center for navigation (either Home or a dynamic, pilot-set location).
local function get_active_center_location()
    if SoarNavGlobals.dynamic_center_location then
        return SoarNavGlobals.dynamic_center_location
    else
        return ahrs:get_home()
    end
end

-- Safely decrements a number, ensuring it does not go below zero.
local function safe_decrement(n)
    return math.max(0, n - 1)
end

-- Centralized function for sending formatted GCS messages.
local function log_gcs(severity, level, message)
    if (p_log_lvl:get()) >= level then
        gcs:send_text(severity, "SoarNav: " .. message)
    end
end

-- Returns a cached wind vector, refreshing it periodically to save resources.
local function get_wind_vector()
    local now_ms = millis():toint()
    if not SoarNavGlobals.cached_wind or (now_ms - SoarNavGlobals.wind_cache_time) > 5000 then
        SoarNavGlobals.cached_wind = ahrs:wind_estimate()
        SoarNavGlobals.wind_cache_time = millis():toint()
    end
    return SoarNavGlobals.cached_wind
end

-- Validates key SNAV_* parameters to ensure they are within safe and logical ranges.
local function validate_params()
    local all_valid = true
    local max_dist = p_max_dist:get()
    if max_dist < 0 then
        log_gcs(MAV_SEVERITY.ERROR, 0, "SNAV_MAX_DIST cannot be negative.")
        all_valid = false
    end
    local roll_limit = p_roll_limit:get()
    if roll_limit < 10 or roll_limit > 50 then
        log_gcs(MAV_SEVERITY.WARNING, 1, "SNAV_ROLL_LIMIT out of range (10-50).")
    end
    return all_valid
end

-- Determines the grid cell index for a given geographical location.
---@param loc Location_ud
---@return integer|nil
local function get_cell_index_from_location(loc)
    local lat = loc:lat()
    local lon = loc:lng()
    if not SoarNavGlobals.grid_initialized or not SoarNavGlobals.grid_bounds then return nil end
    if lat < SoarNavGlobals.grid_bounds.min_lat or lat > SoarNavGlobals.grid_bounds.max_lat or lon < SoarNavGlobals.grid_bounds.min_lon or lon > SoarNavGlobals.grid_bounds.max_lon then
        return nil
    end
    local lat_fraction = (lat - SoarNavGlobals.grid_bounds.min_lat) / (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat)
    local lon_fraction = (lon - SoarNavGlobals.grid_bounds.min_lon) / (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon)
    local row = math.floor(lat_fraction * SoarNavGlobals.grid_rows) + 1
    local col = math.floor(lon_fraction * SoarNavGlobals.grid_cols) + 1
    row = math.max(1, math.min(SoarNavGlobals.grid_rows, row))
    col = math.max(1, math.min(SoarNavGlobals.grid_cols, col))
    return (row - 1) * SoarNavGlobals.grid_cols + col
end

-- Assesses the aircraft's energy state (NORMAL, LOW, CRITICAL) based on altitude.
local function assess_energy_state()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    if not dist_from_home_3d then return "UNKNOWN" end
    local current_alt = -dist_from_home_3d:z()
    local min_alt = p_soar_alt_min:get()
    local max_alt = p_soar_alt_max:get()
    local margin = (max_alt - min_alt) * 0.3
    local lower_threshold = min_alt + margin - SoarNavConstants.HYSTERESIS_MARGIN
    local upper_threshold = min_alt + margin + SoarNavConstants.HYSTERESIS_MARGIN

    if current_alt < min_alt then
        SoarNavGlobals.energy_state = "CRITICAL"
    elseif SoarNavGlobals.energy_state == "NORMAL" and current_alt < lower_threshold then
        SoarNavGlobals.energy_state = "LOW"
    elseif SoarNavGlobals.energy_state == "LOW" and current_alt > upper_threshold then
        SoarNavGlobals.energy_state = "NORMAL"
    elseif SoarNavGlobals.energy_state ~= "CRITICAL" and current_alt >= lower_threshold and SoarNavGlobals.energy_state ~= "LOW" then
        SoarNavGlobals.energy_state = "NORMAL"
    end
    return SoarNavGlobals.energy_state
end

-- Formats and logs the details of a recorded thermal event.
local function log_thermal_event(hotspot)
    local wind = get_wind_vector()
    if wind then
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal: %.1fm/s (max %.1f, %s) W:%.1f@%d.", hotspot.avg_strength, hotspot.max_strength, hotspot.consistency, wind:length(), math.floor((math.deg(math.atan(wind:y(), wind:x())) + 360) % 360)))
    else
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal: %.1fm/s (max %.1f, %s) W:N/A.", hotspot.avg_strength, hotspot.max_strength, hotspot.consistency))
    end
end

-- Calculates the statistical variance of climb rate samples to determine thermal consistency.
local function calculate_thermal_variance(samples)
    if not samples or #samples < 2 then return 0 end
    local sum = 0
    for _, val in ipairs(samples) do sum = sum + val end
    local mean = sum / #samples
    local variance_sum = 0
    for _, val in ipairs(samples) do variance_sum = variance_sum + (val - mean)^2 end
    return variance_sum / #samples
end

-- Manages script state transitions and associated cleanup actions.
local function set_script_state(new_state, reason)
    if SoarNavGlobals.script_state ~= new_state then
        if new_state == SCRIPT_STATE.IDLE and SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
            SoarNavGlobals.pitch_gesture_state = "idle"
            SoarNavGlobals.pitch_gesture_count = 0
            SoarNavGlobals.pitch_gesture_start_ms = 0
            SoarNavGlobals.pitch_gesture_triggered_this_override = false
            SoarNavGlobals.roll_gesture_state = "idle"
            SoarNavGlobals.roll_gesture_count = 0
            SoarNavGlobals.roll_gesture_start_ms = 0
            SoarNavGlobals.manual_override_active = false
        end
        SoarNavGlobals.script_state = new_state
        log_gcs(MAV_SEVERITY.NOTICE, 1, reason)
        if new_state == SCRIPT_STATE.IDLE or new_state == SCRIPT_STATE.PILOT_OVERRIDE or new_state == SCRIPT_STATE.THERMAL_PAUSE or new_state == SCRIPT_STATE.ERROR then
            if SoarNavGlobals.rc_roll_channel then
                SoarNavGlobals.rc_roll_channel:set_override(0)
            end
            SoarNavGlobals.last_heading_error = 0
            SoarNavGlobals.last_commanded_roll_deg = 0
        end
        if new_state == SCRIPT_STATE.IDLE or new_state == SCRIPT_STATE.THERMAL_PAUSE or new_state == SCRIPT_STATE.ERROR then
            SoarNavGlobals.target_loc = nil
        end
    end
end

-- Detects a rapid pitch stick gesture to re-center the search area.
local function check_pitch_gesture()
    if (p_max_dist:get()) <= 0 or SoarNavGlobals.pitch_gesture_triggered_this_override then
        return
    end

    if SoarNavGlobals.pitch_gesture_state ~= "idle" and (millis():toint() - SoarNavGlobals.pitch_gesture_start_ms) > SoarNavConstants.PITCH_GESTURE_TIMEOUT_MS then
        SoarNavGlobals.pitch_gesture_state = "idle"
        SoarNavGlobals.pitch_gesture_count = 0
    end

    local pitch_chan_obj = rc:get_channel(p_rcmap_pitch:get() or 2)
    if not pitch_chan_obj then return end
    local normalized_pitch = pitch_chan_obj:norm_input()

    if SoarNavGlobals.pitch_gesture_state == "idle" then
        if math.abs(normalized_pitch) > SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_start_ms = millis():toint()
            SoarNavGlobals.pitch_gesture_count = 1
            if normalized_pitch > 0 then
                SoarNavGlobals.pitch_gesture_state = "waiting_for_down"
            else
                SoarNavGlobals.pitch_gesture_state = "waiting_for_up"
            end
        end
    elseif SoarNavGlobals.pitch_gesture_state == "waiting_for_down" then
        if normalized_pitch < -SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_count = SoarNavGlobals.pitch_gesture_count + 1
            SoarNavGlobals.pitch_gesture_state = "waiting_for_up"
        end
    elseif SoarNavGlobals.pitch_gesture_state == "waiting_for_up" then
        if normalized_pitch > SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_count = SoarNavGlobals.pitch_gesture_count + 1
            SoarNavGlobals.pitch_gesture_state = "waiting_for_down"
        end
    end

    if SoarNavGlobals.pitch_gesture_count >= SoarNavConstants.PITCH_GESTURE_COUNT_TARGET then
        log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: Area re-centered.")
        SoarNavGlobals.dynamic_center_location = ahrs:get_location():copy()
        SoarNavGlobals.target_loc = nil
        SoarNavGlobals.grid_initialized = false
        SoarNavGlobals.pitch_gesture_triggered_this_override = true
    end
end

-- Detects a rapid roll stick gesture to toggle a persistent manual override.
local function check_roll_gesture()
    if SoarNavGlobals.roll_gesture_state ~= "idle" and (millis():toint() - SoarNavGlobals.roll_gesture_start_ms) > SoarNavConstants.ROLL_GESTURE_TIMEOUT_MS then
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
    end

    local roll_chan = p_rcmap_roll:get()
    local chan1 = rc:get_channel(roll_chan)
    if not chan1 then return end
    local normalized_roll = chan1:norm_input()

    if SoarNavGlobals.roll_gesture_state == "idle" then
        if math.abs(normalized_roll) > SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_start_ms = millis():toint()
            SoarNavGlobals.roll_gesture_count = 1
            if normalized_roll > 0 then
                SoarNavGlobals.roll_gesture_state = "waiting_for_left"
            else
                SoarNavGlobals.roll_gesture_state = "waiting_for_right"
            end
        end
    elseif SoarNavGlobals.roll_gesture_state == "waiting_for_left" then
        if normalized_roll < -SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_count = SoarNavGlobals.roll_gesture_count + 1
            SoarNavGlobals.roll_gesture_state = "waiting_for_right"
        end
    elseif SoarNavGlobals.roll_gesture_state == "waiting_for_right" then
        if normalized_roll > SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_count = SoarNavGlobals.roll_gesture_count + 1
            SoarNavGlobals.roll_gesture_state = "waiting_for_left"
        end
    end

    if SoarNavGlobals.roll_gesture_count >= SoarNavConstants.ROLL_GESTURE_COUNT_TARGET then
        SoarNavGlobals.manual_override_active = not SoarNavGlobals.manual_override_active
        if SoarNavGlobals.manual_override_active then
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: Manual override ON.")
        else
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: Manual override OFF.")
        end
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
        SoarNavGlobals.roll_gesture_start_ms = 0
    end
end

-- Kicks off the grid generation process.
local function initialize_grid()
    SoarNavGlobals.is_initializing = true
    SoarNavGlobals.grid_initialized = false
    SoarNavGlobals.grid_init_step = 1
    SoarNavGlobals.grid_cells = {}
    SoarNavGlobals.valid_cell_indices = {}
    SoarNavGlobals.unvisited_cell_indices = {}
    SoarNavGlobals.last_cell_index = nil
    SoarNavGlobals.grid_populate_index = 1
    SoarNavGlobals.grid_rows = 0
    SoarNavGlobals.grid_cols = 0
    SoarNavGlobals.grid_bounds = nil
    SoarNavGlobals.max_operational_distance = 0
    log_gcs(MAV_SEVERITY.INFO, 1, "Grid init started...")
end

-- Manages the multi-step, non-blocking grid generation and validation.
local function manage_grid_initialization()
    if not SoarNavGlobals.is_initializing then return end
    local active_center = get_active_center_location()
    if not active_center then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Grid init failed: no active center.")
        SoarNavGlobals.is_initializing = false
        SoarNavGlobals.grid_init_step = 0
        return
    end

    if SoarNavGlobals.grid_init_step == 1 then
        local height_m = 0
        local width_m = 0

        if SoarNavGlobals.use_polygon_area and SoarNavGlobals.polygon_bounds then
            SoarNavGlobals.grid_bounds = SoarNavGlobals.polygon_bounds
            local max_dist_sq = 0
            local poly_pts = SoarNavGlobals.polygon_points
            if poly_pts and #poly_pts > 1 then
                for i = 1, #poly_pts do
                    for j = i + 1, #poly_pts do
                        local dist = poly_pts[i]:get_distance(poly_pts[j])
                        local dist_sq = dist * dist
                        if dist_sq > max_dist_sq then
                            max_dist_sq = dist_sq
                        end
                    end
                end
                SoarNavGlobals.max_operational_distance = math.sqrt(max_dist_sq)
                log_gcs(MAV_SEVERITY.INFO, 2, string.format("Polygon max distance calculated: %.0fm.", SoarNavGlobals.max_operational_distance))
            end
        else
            local max_dist = p_max_dist:get()
            local corner_dist = max_dist * 1.4142
            local sw_corner_loc_calc = active_center:copy()
            sw_corner_loc_calc:offset_bearing(225, corner_dist)
            local ne_corner_loc_calc = active_center:copy()
            ne_corner_loc_calc:offset_bearing(45, corner_dist)
            SoarNavGlobals.grid_bounds = {
                min_lat = sw_corner_loc_calc:lat(), max_lat = ne_corner_loc_calc:lat(),
                min_lon = sw_corner_loc_calc:lng(), max_lon = ne_corner_loc_calc:lng()
            }
            SoarNavGlobals.max_operational_distance = (max_dist or 0) * 2
            log_gcs(MAV_SEVERITY.INFO, 2, string.format("Radius max diameter calculated: %.0fm.", SoarNavGlobals.max_operational_distance))
        end
        
        local sw_corner_loc = active_center:copy()
        sw_corner_loc:lat(SoarNavGlobals.grid_bounds.min_lat)
        sw_corner_loc:lng(SoarNavGlobals.grid_bounds.min_lon)
        local ne_corner_loc = active_center:copy()
        ne_corner_loc:lat(SoarNavGlobals.grid_bounds.max_lat)
        ne_corner_loc:lng(SoarNavGlobals.grid_bounds.max_lon)
        local xy = sw_corner_loc:get_distance_NE(ne_corner_loc)
        height_m = xy:x()
        width_m = xy:y()

        if height_m < 1 or width_m < 1 then
            log_gcs(MAV_SEVERITY.ERROR, 0, "Grid area too small. Init aborted.")
            SoarNavGlobals.is_initializing = false
            SoarNavGlobals.grid_init_step = 0
            return
        end

        local area_m2 = height_m * width_m
        local approx_cell_size_m = math.sqrt(area_m2 / SoarNavConstants.max_total_grid_cells)
        local final_cell_size_m = math.max(SoarNavConstants.min_cell_size_m, approx_cell_size_m)
        SoarNavGlobals.grid_rows = math.max(1, math.floor(height_m / final_cell_size_m))
        SoarNavGlobals.grid_cols = math.max(1, math.floor(width_m / final_cell_size_m))
        SoarNavGlobals.grid_init_step = 2

    elseif SoarNavGlobals.grid_init_step == 2 then
        local total_cells = SoarNavGlobals.grid_rows * SoarNavGlobals.grid_cols
        local cells_processed = 0
        local cells_per_call = SoarNavConstants.grid_init_cells_per_call
        while SoarNavGlobals.grid_populate_index <= total_cells and cells_processed < cells_per_call do
            table.insert(SoarNavGlobals.grid_cells, {visit_count = 0})
            SoarNavGlobals.grid_populate_index = SoarNavGlobals.grid_populate_index + 1
            cells_processed = cells_processed + 1
        end
        if SoarNavGlobals.grid_populate_index > total_cells then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Grid: %d rows, %d cols. Scan cells...", SoarNavGlobals.grid_rows, SoarNavGlobals.grid_cols))
            SoarNavGlobals.grid_init_row = 1
            SoarNavGlobals.grid_init_col = 1
            SoarNavGlobals.grid_init_step = 3
        end
    elseif SoarNavGlobals.grid_init_step == 3 then
        local cells_processed = 0
        local max_dist_for_check = p_max_dist:get() or 500
        local max_dist_sq = max_dist_for_check * max_dist_for_check
        local center_lat_float = active_center:lat()/1e7
        local center_lon_float = active_center:lng()/1e7
        local deg_to_rad = math.pi / 180
        local lon_dist_factor = math.cos(center_lat_float * deg_to_rad) * 111320.0
        local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat) / SoarNavGlobals.grid_rows
        local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon) / SoarNavGlobals.grid_cols
        while SoarNavGlobals.grid_init_row <= SoarNavGlobals.grid_rows and cells_processed < SoarNavConstants.grid_init_cells_per_call do
            local r = SoarNavGlobals.grid_init_row
            local c = SoarNavGlobals.grid_init_col
            local cell_center_lat = SoarNavGlobals.grid_bounds.min_lat + (r - 0.5) * cell_height_lat
            local cell_center_lon = SoarNavGlobals.grid_bounds.min_lon + (c - 0.5) * cell_width_lon
            local cell_is_valid = false
            if SoarNavGlobals.use_polygon_area then
                local center = SoarNavGlobals.polygon_origin:copy()
                center:lat(cell_center_lat)
                center:lng(cell_center_lon)
                if is_point_in_polygon(center) then
                    cell_is_valid = true
                end
            else
                local dy = (cell_center_lat/1e7 - center_lat_float) * 111132.9
                local dx = (cell_center_lon/1e7 - center_lon_float) * lon_dist_factor
                local dist_sq = dx*dx + dy*dy
                if dist_sq <= max_dist_sq then
                    cell_is_valid = true
                end
            end
            if cell_is_valid then
                local cell_index = (r - 1) * SoarNavGlobals.grid_cols + c
                table.insert(SoarNavGlobals.valid_cell_indices, cell_index)
            end
            cells_processed = cells_processed + 1
            SoarNavGlobals.grid_init_col = SoarNavGlobals.grid_init_col + 1
            if SoarNavGlobals.grid_init_col > SoarNavGlobals.grid_cols then
                SoarNavGlobals.grid_init_col = 1
                SoarNavGlobals.grid_init_row = SoarNavGlobals.grid_init_row + 1
            end
        end
        if SoarNavGlobals.grid_init_row > SoarNavGlobals.grid_rows then
            SoarNavGlobals.is_initializing = false
            SoarNavGlobals.grid_initialized = true
            SoarNavGlobals.grid_init_step = 0
            SoarNavGlobals.unvisited_cell_indices = {}
            for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                table.insert(SoarNavGlobals.unvisited_cell_indices, v)
            end
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Grid validated: %d cells.", #SoarNavGlobals.valid_cell_indices))
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Exploration grid ready.")
        end
    end
end

-- Tracks which grid cell the aircraft is currently in and updates its visit count.
local function update_visited_cell()
    local loc = ahrs:get_location()
    if not loc then return end
    local current_cell_index = get_cell_index_from_location(loc)
    if current_cell_index and current_cell_index > 0 and current_cell_index <= #SoarNavGlobals.grid_cells then
        if current_cell_index ~= SoarNavGlobals.last_cell_index then
            if SoarNavGlobals.grid_cells[current_cell_index] then
                if SoarNavGlobals.grid_cells[current_cell_index].visit_count == 0 then
                    for i = #SoarNavGlobals.unvisited_cell_indices, 1, -1 do
                        if SoarNavGlobals.unvisited_cell_indices[i] == current_cell_index then
                            table.remove(SoarNavGlobals.unvisited_cell_indices, i)
                            break
                        end
                    end
                end
                SoarNavGlobals.grid_cells[current_cell_index].visit_count = SoarNavGlobals.grid_cells[current_cell_index].visit_count + 1
                SoarNavGlobals.last_cell_index = current_cell_index
            end
        end
    end
end

-- Removes expired thermal hotspots from memory and returns a list of currently valid ones.
local function clean_and_get_hotspots()
    local now_ms = millis():toint()
    local lifetime_ms = p_tmem_life:get() * 1000
    for i = #SoarNavGlobals.thermal_hotspots, 1, -1 do
        local hotspot = SoarNavGlobals.thermal_hotspots[i]
        local age_ms = now_ms - hotspot.timestamp:toint()
        if age_ms >= lifetime_ms then
            table.remove(SoarNavGlobals.thermal_hotspots, i)
        end
    end
    return SoarNavGlobals.thermal_hotspots
end

-- Records a new thermal hotspot in memory after exiting THERMAL mode.
local function stop_and_record_thermal()
    SoarNavGlobals.is_monitoring_thermal = false
    if not SoarNavGlobals.current_thermal_stats or not SoarNavGlobals.current_thermal_stats.entry_location or SoarNavGlobals.current_thermal_stats.sample_count == 0 then
        log_gcs(MAV_SEVERITY.WARNING, 1, "Thermal exit: no data sampled.")
        return
    end
    local avg_strength = SoarNavGlobals.current_thermal_stats.total_strength / SoarNavGlobals.current_thermal_stats.sample_count
    local max_strength = SoarNavGlobals.current_thermal_stats.max_strength
    local hotspot_loc = SoarNavGlobals.current_thermal_stats.entry_location:copy()
    local variance = calculate_thermal_variance(SoarNavGlobals.current_thermal_stats.samples)
    local consistency = variance < SoarNavConstants.THERMAL_CONSISTENCY_VARIANCE_THRESHOLD and "consistent" or "variable"
    local wind_vec = get_wind_vector()
    if wind_vec then
        local wind_vel = wind_vec:length()
        if wind_vel and wind_vel == wind_vel and wind_vel < 1/0 then
            local wind_heading_rad = wind_vec:xy():angle()
            if wind_heading_rad and wind_heading_rad == wind_heading_rad then
                local upwind_bearing = math.deg(wind_heading_rad)
                local base_comp_time = p_wind_comp:get()
                local adaptive_comp_time = base_comp_time * math.max(0.5, math.min(2.0, avg_strength / 1.5))
                if wind_vel <= 1.0 then
                    adaptive_comp_time = math.max(adaptive_comp_time, 10)
                end
                local offset_dist = wind_vel * adaptive_comp_time
                hotspot_loc:offset_bearing(upwind_bearing, offset_dist)
            end
        end
    end

    local is_inside_area = false
    if SoarNavGlobals.use_polygon_area then
        if is_point_in_polygon(hotspot_loc) then
            is_inside_area = true
        end
    else
        local active_center = get_active_center_location()
        if active_center and active_center:get_distance(hotspot_loc) <= (p_max_dist:get() + p_wp_radius:get() * 2) then
            is_inside_area = true
        end
    end

    if not is_inside_area then
        log_gcs(MAV_SEVERITY.INFO, 1, "Thermal ignored: out of area.")
        SoarNavGlobals.current_thermal_stats = {}
        return
    end

    local timestamp_saved = millis()
    local new_hotspot = {
        loc = hotspot_loc,
        timestamp = timestamp_saved,
        avg_strength = avg_strength,
        max_strength = max_strength,
        consistency = consistency,
        duration = (timestamp_saved:toint() - SoarNavGlobals.current_thermal_stats.start_time:toint()) / 1000,
        wind_vec = wind_vec,
		failed_attempts = 0,
    }
    local new_cell = get_cell_index_from_location(hotspot_loc)
    if not new_cell then
       log_gcs(MAV_SEVERITY.INFO, 1, "Cell index fail, fallback.")
       new_cell = -1
    end
    new_hotspot.cell = new_cell
    for _, existing in ipairs(SoarNavGlobals.thermal_hotspots) do
        local existing_cell = get_cell_index_from_location(existing.loc)
        if existing_cell == new_cell then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal ignored: duplicate in cell %d.", new_cell))
            SoarNavGlobals.current_thermal_stats = {}
            return
        end
    end
    table.insert(SoarNavGlobals.thermal_hotspots, new_hotspot)
    SoarNavGlobals.last_used_hotspot_timestamp = timestamp_saved
    if #SoarNavGlobals.thermal_hotspots > SoarNavConstants.max_hotspots then
        table.sort(SoarNavGlobals.thermal_hotspots, function(a, b)
            return (a.avg_strength or 0) < (a.avg_strength or 0)
        end)
        table.remove(SoarNavGlobals.thermal_hotspots, 1)
        log_gcs(MAV_SEVERITY.INFO, 2, "Weakest thermal removed.")
    end
    log_thermal_event(new_hotspot)
    SoarNavGlobals.current_thermal_stats = {}
end

-- Periodically samples the climb rate while the aircraft is in THERMAL mode.
local function sample_thermal_strength()
    local ned_velocity = ahrs:get_velocity_NED()
    if ned_velocity then
        local vel_d = ned_velocity:z()
        local climb_rate = -vel_d
        SoarNavGlobals.current_thermal_stats.total_strength = SoarNavGlobals.current_thermal_stats.total_strength + climb_rate
        if climb_rate > SoarNavGlobals.current_thermal_stats.max_strength then
            SoarNavGlobals.current_thermal_stats.max_strength = climb_rate
        end
        table.insert(SoarNavGlobals.current_thermal_stats.samples, climb_rate)
        if #SoarNavGlobals.current_thermal_stats.samples > 10 then
            table.remove(SoarNavGlobals.current_thermal_stats.samples, 1)
        end
        SoarNavGlobals.current_thermal_stats.sample_count = SoarNavGlobals.current_thermal_stats.sample_count + 1
        SoarNavGlobals.last_thermal_sample_ms = millis()
    end
end

-- Initializes the data structure for monitoring a new thermal.
local function start_thermal_monitoring()
    local loc = ahrs:get_location()
    if not loc then
        log_gcs(MAV_SEVERITY.WARNING, 1, "Can't monitor thermal: no location.")
        return
    end
    SoarNavGlobals.is_monitoring_thermal = true
    SoarNavGlobals.last_thermal_sample_ms = millis()
    SoarNavGlobals.current_thermal_stats = {
        entry_location = loc,
        max_strength = -99,
        total_strength = 0,
        sample_count = 0,
        samples = {},
        start_time = millis()
    }
    table.insert(SoarNavGlobals.thermal_entry_timestamps, millis())
end

-- Pre-calculates polygon vertices in local XY coordinates for faster boundary checks.
local function prepare_polygon_xy_cache()
    local poly = SoarNavGlobals.polygon_points
    if not poly or #poly < 3 then return end

    local origin = poly[1]:copy()
    SoarNavGlobals.polygon_origin = origin
    SoarNavGlobals.polygon_xy = {}

    for i = 1, #poly do
        local xy = origin:get_distance_NE(poly[i])
        SoarNavGlobals.polygon_xy[i] = { x = xy:x(), y = xy:y() }
    end
    log_gcs(MAV_SEVERITY.INFO, 2, "Polygon XY cache initialized.")
end

-- Reads a polygon file from the SD card and populates the polygon points list.
local function read_polygon_file(filename)
    local f = io.open(filename, "r")
    if not f then
        log_gcs(MAV_SEVERITY.ERROR, 0, string.format("Can't open polygon file %s.", filename))
        return nil
    end

    SoarNavGlobals.polygon_points = {}
    SoarNavGlobals.polygon_bounds = {min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7}
    local home = ahrs:get_home()
    if not home then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Can't read polygon: home not set.")
        f:close()
        return nil
    end

    for line in f:lines() do
        if not line:match("^#") and line:match("%S") then
            local lat_str, lon_str = line:match("^%s*([%d%.%-]+)%s+([%d%.%-]+)%s*$")
            if lat_str and lon_str then
                local lat, lon = tonumber(lat_str), tonumber(lon_str)
                if lat and lon then
                    local point = home:copy()
                    point:lat(lat * 1e7)
                    point:lng(lon * 1e7)

                    table.insert(SoarNavGlobals.polygon_points, point)
                    SoarNavGlobals.polygon_bounds.min_lat = math.min(SoarNavGlobals.polygon_bounds.min_lat, lat * 1e7)
                    SoarNavGlobals.polygon_bounds.max_lat = math.max(SoarNavGlobals.polygon_bounds.max_lat, lat * 1e7)
                    SoarNavGlobals.polygon_bounds.min_lon = math.min(SoarNavGlobals.polygon_bounds.min_lon, lon * 1e7)
                    SoarNavGlobals.polygon_bounds.max_lon = math.max(SoarNavGlobals.polygon_bounds.max_lon, lon * 1e7)
                end
            end
        end
    end
    f:close()

    if #SoarNavGlobals.polygon_points < 3 then
        log_gcs(MAV_SEVERITY.ERROR, 0, string.format("Poly file %s < 3 pts. Using radius.", filename))
        SoarNavGlobals.polygon_bounds = nil
        return nil
    end

    if #SoarNavGlobals.polygon_points >= 3 then
        local first_pt = SoarNavGlobals.polygon_points[1]
        local last_pt = SoarNavGlobals.polygon_points[#SoarNavGlobals.polygon_points]

        if first_pt:get_distance(last_pt) > 0 then
            table.insert(SoarNavGlobals.polygon_points, first_pt:copy())
            log_gcs(MAV_SEVERITY.INFO, 2, "Auto-closed polygon.")
        end
    end
    log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Polygon loaded: %d points.", #SoarNavGlobals.polygon_points))
    prepare_polygon_xy_cache()
    return true
end

-- Generates a randomized target location around a central point.
local function generate_target_around_point(target_loc, radius_m)
    local rand_radius_m = math.sqrt(math.random()) * radius_m
    local max_dist = p_max_dist:get()
    if not SoarNavGlobals.use_polygon_area and max_dist > 0 and rand_radius_m > max_dist then
        return nil
    end
    
    local new_loc = target_loc:copy()
    new_loc:offset_bearing(math.random() * 360, math.sqrt(math.random()) * rand_radius_m)
    local active_center = get_active_center_location()
    if not active_center then
        return nil
    end
    if SoarNavGlobals.use_polygon_area then
        if not is_point_in_polygon(new_loc) then
            return nil
        end
    end

    return new_loc
end

-- Formats and logs the details for a newly generated waypoint.
local function log_new_waypoint(dist_to_wp)
    local d = tonumber(dist_to_wp) or 0
    log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("New WP: %s | Dist: %.0fm.", SoarNavGlobals.g_waypoint_source_info, d))
    if SoarNavGlobals.target_loc ~= nil then
        log_gcs(MAV_SEVERITY.NOTICE, 2, string.format(" > Target: %.5f, %.5f.", SoarNavGlobals.target_loc:lat()/1e7, SoarNavGlobals.target_loc:lng()/1e7))
    end
end

-- Selects the best thermal from memory, applies drift, and validates it is within the flight area.
local function select_best_thermal_waypoint()
    local valid_hotspots = clean_and_get_hotspots()
    if #valid_hotspots == 0 then return false end

    local good_hotspots = {}
    local good_hotspots_excluding_last = {}
    
    for _, hotspot in ipairs(valid_hotspots) do
        if hotspot.avg_strength and hotspot.avg_strength > 0 then
            table.insert(good_hotspots, hotspot)
            if SoarNavGlobals.last_used_hotspot_timestamp == 0 or hotspot.timestamp:toint() ~= SoarNavGlobals.last_used_hotspot_timestamp:toint() then
                table.insert(good_hotspots_excluding_last, hotspot)
            end
        end
    end

    local final_hotspot_list
    if #good_hotspots_excluding_last > 0 then
        final_hotspot_list = good_hotspots_excluding_last
    else
        final_hotspot_list = good_hotspots
    end

    if #final_hotspot_list == 0 then
        log_gcs(MAV_SEVERITY.INFO, 1, "No positive thermals. Using grid.")
        return false
    end

    table.sort(final_hotspot_list, function(a, b) return a.avg_strength > b.avg_strength end)
    local best_hotspot = final_hotspot_list[1]

    local drifted_loc = best_hotspot.loc:copy()
    local age_seconds = (millis():toint() - best_hotspot.timestamp:toint()) / 1000.0
    local wind_vec = get_wind_vector()
    local drift_dist = 0

    if wind_vec and wind_vec:length() > 0.5 then
        local wind_speed = wind_vec:length()
        if wind_speed and wind_speed == wind_speed and wind_speed < 1/0 then
            local calculated_drift = wind_speed * age_seconds
            if calculated_drift and calculated_drift > 1 and calculated_drift == calculated_drift and calculated_drift < 1/0 then
                local wind_heading_rad = wind_vec:xy():angle()
                if wind_heading_rad and wind_heading_rad == wind_heading_rad then
                    local drift_bearing = math.deg(wind_heading_rad)
                    drifted_loc:offset_bearing(drift_bearing, calculated_drift)
                    drift_dist = calculated_drift
                end
            end
        end
    end

    local is_valid_target = false
    if SoarNavGlobals.use_polygon_area then
        if is_point_in_polygon(drifted_loc) then
            is_valid_target = true
        end
    else
        local center = get_active_center_location()
        if center and center:get_distance(drifted_loc) <= p_max_dist:get() then
            is_valid_target = true
        end
    end

    if is_valid_target then
        if drift_dist > 0 then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("LOW ENERGY: Best thermal drifted %.0fm.", drift_dist))
        end
        SoarNavGlobals.target_loc = drifted_loc
        SoarNavGlobals.g_waypoint_source_info = string.format("BestThrm(Drift, +%.1f)", best_hotspot.avg_strength)
        SoarNavGlobals.last_used_hotspot_timestamp = best_hotspot.timestamp
        return true
    else
        log_gcs(MAV_SEVERITY.INFO, 1, "Best thermal out of area. Ignoring.")
        return false
    end
end

-- Core logic for searching and selecting a new waypoint based on energy state and thermal memory.
local function search_for_new_waypoint()
    local active_center = get_active_center_location()
    if not active_center then return end

    local energy_status = assess_energy_state()

    if energy_status == "LOW" or energy_status == "CRITICAL" then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Energy State: %s. Evaluating...", energy_status))
        local valid_hotspots = clean_and_get_hotspots()
        if #valid_hotspots > 0 and select_best_thermal_waypoint() then
            log_gcs(MAV_SEVERITY.INFO, 2, "Best thermal selected.")
            SoarNavGlobals.waypoint_start_time_ms = millis()
            SoarNavGlobals.waypoint_search_in_progress = false
            SoarNavGlobals.last_commanded_roll_deg = 0
            SoarNavGlobals.last_heading_error = 0
            local loc = ahrs:get_location()
            if loc then
                local dist_to_wp = loc:get_distance(SoarNavGlobals.target_loc)
                log_new_waypoint(dist_to_wp)
                SoarNavGlobals.initial_distance_to_wp = dist_to_wp
                SoarNavGlobals.reroute_check_armed = true
            end
            return
        else
            if #valid_hotspots > 0 then
                log_gcs(MAV_SEVERITY.INFO, 1, "Thermal select failed, emergency grid.")
            else
                log_gcs(MAV_SEVERITY.INFO, 2, "No thermals, emergency grid.")
            end
            SoarNavGlobals.g_waypoint_source_info = string.format("Grid (Alt: %s)", energy_status)
        end
    else
        log_gcs(MAV_SEVERITY.INFO, 2, "Alt OK. Standard search.")
    end

    local now_ms = millis()
    local recent_success_count = 0
    local i = #SoarNavGlobals.thermal_entry_timestamps
    while i > 0 do
        local entry_time = SoarNavGlobals.thermal_entry_timestamps[i]
        if entry_time and (now_ms:toint() - entry_time:toint()) > SoarNavConstants.thermal_history_window_ms then
            table.remove(SoarNavGlobals.thermal_entry_timestamps, i)
        else
            recent_success_count = recent_success_count + 1
        end
        i = i - 1
    end

    local MIN_TMEM_CHANCE = 20
    local MAX_TMEM_CHANCE = 60
    local MAX_SUCCESS_FOR_SCALING = 3
    local success_factor = math.min(recent_success_count / MAX_SUCCESS_FOR_SCALING, 1.0)
    local dynamic_tmem_chance = MIN_TMEM_CHANCE + (MAX_TMEM_CHANCE - MIN_TMEM_CHANCE) * success_factor
    local use_hotspot_logic = false
    local valid_hotspots_raw = clean_and_get_hotspots()
    local valid_hotspots = {}
    if (p_tmem_enable:get()) == 1 and #valid_hotspots_raw > 0 then
        if SoarNavGlobals.last_used_hotspot_timestamp ~= 0 then
            for _, hotspot in ipairs(valid_hotspots_raw) do
                if hotspot.timestamp and hotspot.timestamp:toint() ~= SoarNavGlobals.last_used_hotspot_timestamp:toint() then
                    table.insert(valid_hotspots, hotspot)
                end
            end
            if #valid_hotspots == 0 and #valid_hotspots_raw > 0 then
                valid_hotspots = valid_hotspots_raw
                log_gcs(MAV_SEVERITY.INFO, 2, "Only one hotspot, re-selection OK.")
            end
        else
            valid_hotspots = valid_hotspots_raw
        end

        if #valid_hotspots > 0 and math.random(1, 100) <= dynamic_tmem_chance and not SoarNavGlobals.force_grid_after_reset then
            use_hotspot_logic = true
        elseif #valid_hotspots == 0 and #valid_hotspots_raw > 0 and not SoarNavGlobals.logged_ignore_message then
            log_gcs(MAV_SEVERITY.INFO, 1, "Last thermal ignored, using grid.")
            SoarNavGlobals.logged_ignore_message = true
        end
    end

    local source_for_log = "Unknown"
    local new_wp_found = false

    if use_hotspot_logic and not SoarNavGlobals.force_grid_after_reset then
        local selected_hotspot
        if #valid_hotspots == 1 then
            selected_hotspot = valid_hotspots[1]
        else
            local idx1 = math.random(1, #valid_hotspots)
            local idx2 = math.random(1, #valid_hotspots)
            local h1 = valid_hotspots[idx1]
            local h2 = valid_hotspots[idx2]
            if h1.avg_strength and h2.avg_strength then
                selected_hotspot = (h1.avg_strength > h2.avg_strength) and h1 or h2
            else
                selected_hotspot = h1
            end
        end

        if selected_hotspot and selected_hotspot.timestamp then
            local age_seconds = (millis():toint() - selected_hotspot.timestamp:toint()) / 1000.0
            local center_loc = selected_hotspot.loc:copy()
            local drift_dist = 0

            local wind_vec = selected_hotspot.wind_vec or get_wind_vector()
            if wind_vec then
                local wind_speed = wind_vec:length()
                if wind_speed and wind_speed == wind_speed and wind_speed < 1/0 then
                    local calculated_drift = wind_speed * age_seconds
                    if calculated_drift and calculated_drift > 0 and calculated_drift == calculated_drift and calculated_drift < 1/0 then
                        local wind_heading_rad = wind_vec:xy():angle()
                        if wind_heading_rad and wind_heading_rad == wind_heading_rad then
                            local wind_dir_bearing = (math.deg(wind_heading_rad) + 180 + 360) % 360
                            center_loc:offset_bearing(wind_dir_bearing, calculated_drift)
                            drift_dist = calculated_drift
                        end
                    end
                end
            end

            local candidate_loc = generate_target_around_point(center_loc, 250)
            if candidate_loc then
                local is_valid_target = false
                if SoarNavGlobals.use_polygon_area then
                    if is_point_in_polygon(candidate_loc) then
                        is_valid_target = true
                    end
                else
                    local center = get_active_center_location()
                    if center and center:get_distance(candidate_loc) <= p_max_dist:get() then
                        is_valid_target = true
                    end
                end

                if is_valid_target then
                    log_gcs(MAV_SEVERITY.INFO, 2, string.format("Targeting hotspot: avg %.1fm/s.", selected_hotspot.avg_strength or 0))
                    log_gcs(MAV_SEVERITY.INFO, 2, string.format("Drift prediction. Age: %.1fs, Dist: %.1fm.", age_seconds, drift_dist))
                    SoarNavGlobals.current_selected_hotspot = selected_hotspot
                    SoarNavGlobals.target_loc = candidate_loc
                    new_wp_found = true
                    source_for_log = string.format("DriftThrm(+%.1f)", selected_hotspot.avg_strength or 0)
                end
            end
        elseif selected_hotspot then
            log_gcs(MAV_SEVERITY.WARNING, 1, "Selected hotspot has no timestamp.")
        end
    end

    if not new_wp_found then
        if not SoarNavGlobals.grid_initialized or not SoarNavGlobals.valid_cell_indices or #SoarNavGlobals.valid_cell_indices == 0 then
            log_gcs(MAV_SEVERITY.WARNING, 1, "Grid not ready, using random WP.")
            source_for_log = "Random Fallback"
            local center_point_loc = get_active_center_location()
            if center_point_loc then
                local max_dist = p_max_dist:get()
                local rand_dist = math.sqrt(math.random()) * max_dist
                local rand_bearing = math.random() * 360
                local fallback_loc = center_point_loc:copy()
                fallback_loc:offset_bearing(rand_bearing, rand_dist)
                SoarNavGlobals.target_loc = fallback_loc
                new_wp_found = true
            end
        else
            if #SoarNavGlobals.unvisited_cell_indices == 0 then
                log_gcs(MAV_SEVERITY.NOTICE, 1, "Exploration complete. Resetting grid.")
                for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
                    if cell_idx and cell_idx > 0 and cell_idx <= #SoarNavGlobals.grid_cells and SoarNavGlobals.grid_cells[cell_idx] then
                        SoarNavGlobals.grid_cells[cell_idx].visit_count = 0
                    end
                end
                SoarNavGlobals.unvisited_cell_indices = {}
                for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                    table.insert(SoarNavGlobals.unvisited_cell_indices, v)
                end
                SoarNavGlobals.last_cell_index = nil
                SoarNavGlobals.force_grid_after_reset = true
            end

            if #SoarNavGlobals.unvisited_cell_indices > 0 then
                SoarNavGlobals.force_grid_after_reset = false
                local random_list_idx = math.random(1, #SoarNavGlobals.unvisited_cell_indices)
                local chosen_cell_index = SoarNavGlobals.unvisited_cell_indices[random_list_idx]
                table.remove(SoarNavGlobals.unvisited_cell_indices, random_list_idx)

                local row = math.floor((chosen_cell_index - 1) / SoarNavGlobals.grid_cols) + 1
                local col = ((chosen_cell_index - 1) % SoarNavGlobals.grid_cols) + 1
                local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat) / SoarNavGlobals.grid_rows
                local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon) / SoarNavGlobals.grid_cols
                local candidate_lat = SoarNavGlobals.grid_bounds.min_lat + (row - 1) * cell_height_lat + math.random() * cell_height_lat
                local candidate_lon = SoarNavGlobals.grid_bounds.min_lon + (col - 1) * cell_width_lon + math.random() * cell_width_lon

                local new_target_loc = ahrs:get_location():copy()
                new_target_loc:lat(candidate_lat)
                new_target_loc:lng(candidate_lon)
                SoarNavGlobals.target_loc = new_target_loc
                new_wp_found = true
                source_for_log = string.format("Cell: %d", chosen_cell_index)
            else
                log_gcs(MAV_SEVERITY.INFO, 2, "No unexplored cells found.")
                return
            end
        end
    end

    if new_wp_found then
        SoarNavGlobals.waypoint_start_time_ms = millis()
        SoarNavGlobals.g_waypoint_source_info = source_for_log
        SoarNavGlobals.waypoint_search_in_progress = false
        SoarNavGlobals.last_commanded_roll_deg = 0
        SoarNavGlobals.last_heading_error = 0
        SoarNavGlobals.last_progress_check_ms = 0
        SoarNavGlobals.distance_at_last_check = -1
        SoarNavGlobals.stuck_counter = 0
        local loc = ahrs:get_location()
        if loc and SoarNavGlobals.target_loc then
            local dist_to_wp = loc:get_distance(SoarNavGlobals.target_loc)
            log_new_waypoint(dist_to_wp)
            SoarNavGlobals.initial_distance_to_wp = dist_to_wp
            SoarNavGlobals.reroute_check_armed = true
        end
    end
end

-- Calculates a dynamic waypoint timeout based on current wind speed.
local function get_wp_timeout()
    local base_timeout = SoarNavConstants.wp_timeout_ms
    local wind_vec = get_wind_vector()
    if wind_vec and wind_vec:x() and wind_vec:y() then
        local wind_speed = wind_vec:length()
        if wind_speed and wind_speed > 1.0 then
            local adaptive_timeout = base_timeout * (1 + (wind_speed / 15))
            return math.min(adaptive_timeout, base_timeout * 2.5)
        end
    end
    return base_timeout
end

-- Main function, called on every script update cycle.
update_body = function()
    local current_time_ms = millis()
    local current_snav_max_dist = p_max_dist:get()

    if current_snav_max_dist ~= SoarNavGlobals.last_snav_max_dist_value then
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("SNAV_MAX_DIST changed from %.0f to %.0f. Re-init grid.", SoarNavGlobals.last_snav_max_dist_value, current_snav_max_dist))
        SoarNavGlobals.target_loc = nil
        SoarNavGlobals.is_initializing = false
        SoarNavGlobals.grid_initialized = false
        SoarNavGlobals.last_snav_max_dist_value = current_snav_max_dist
        if current_snav_max_dist > 0 then
            SoarNavGlobals.use_polygon_area = false
            SoarNavGlobals.polygon_points = {}
            SoarNavGlobals.polygon_bounds = nil
            log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Radius Mode: %.0fm.", current_snav_max_dist))
            if SoarNavGlobals.last_snav_max_dist_value == 0 then
                SoarNavGlobals.dynamic_center_location = ahrs:get_location():copy()
                if SoarNavGlobals.dynamic_center_location then
                     log_gcs(MAV_SEVERITY.NOTICE, 1, "Radius center set to current location.")
                end
            end
        else
            if read_polygon_file(SoarNavConstants.polygon_filename) then
                SoarNavGlobals.use_polygon_area = true
                log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Polygon Mode: %d points loaded.", #SoarNavGlobals.polygon_points))
                SoarNavGlobals.dynamic_center_location = nil
            else
                log_gcs(MAV_SEVERITY.ERROR, 0, "Failed to load polygon, reverting to radial mode.")
                p_max_dist:set(500)
                SoarNavGlobals.use_polygon_area = false
                SoarNavGlobals.grid_initialized = false
                log_gcs(MAV_SEVERITY.WARNING, 0, "Forcing SNAV_MAX_DIST to 500 due to polygon load failure.")
            end
        end
    end

    if (p_enable:get()) == 0 then
        if SoarNavGlobals.script_state ~= SCRIPT_STATE.IDLE then set_script_state(SCRIPT_STATE.IDLE, "Script disabled by user.") end
        return update, 1000
    end

    if not validate_params() then
        set_script_state(SCRIPT_STATE.ERROR, "Invalid SNAV parameters detected. Disabling.")
        return update, 5000
    end

    if SoarNavGlobals.script_state == SCRIPT_STATE.ERROR then
        return update, 5000
    end

    if not SoarNavGlobals.rc_limits_read and arming:is_armed() then
        SoarNavGlobals.rc1_min = param:get('RC1_MIN') or 1000
        SoarNavGlobals.rc1_max = param:get('RC1_MAX') or 2000
        SoarNavGlobals.rc1_trim = param:get('RC1_TRIM') or 1500
        SoarNavGlobals.rc2_min = param:get('RC2_MIN') or 1000
        SoarNavGlobals.rc2_max = param:get('RC2_MAX') or 2000
        SoarNavGlobals.rc2_trim = param:get('RC2_TRIM') or 1500
        SoarNavGlobals.rc4_min = param:get('RC4_MIN') or 1000
        SoarNavGlobals.rc4_max = param:get('RC4_MAX') or 2000
        SoarNavGlobals.rc4_trim = param:get('RC4_TRIM') or 1500
        SoarNavGlobals.rc_limits_read = true
        log_gcs(MAV_SEVERITY.INFO, 1, "RC1/RC2/RC4 limits read.")
    end

    local loc = ahrs:get_location()
    if not loc then
        SoarNavGlobals.location_error_count = SoarNavGlobals.location_error_count + 1
        if SoarNavGlobals.location_error_count > SoarNavConstants.max_location_errors then
            set_script_state(SCRIPT_STATE.ERROR, "Persistent location error, disabling.")
        end
        return update, 200
    else
        SoarNavGlobals.location_error_count = safe_decrement(SoarNavGlobals.location_error_count)
    end

    if arming:is_armed() and not SoarNavGlobals.grid_initialized and not SoarNavGlobals.is_initializing then
        local current_snav_max_dist_check = p_max_dist:get()
        if current_snav_max_dist_check == 0 and not SoarNavGlobals.use_polygon_area then
            if read_polygon_file(SoarNavConstants.polygon_filename) then
                SoarNavGlobals.use_polygon_area = true
            end
        end
        if SoarNavGlobals.use_polygon_area or (current_snav_max_dist_check > 0) then
            initialize_grid()
        end
    end

    if SoarNavGlobals.is_initializing then
        manage_grid_initialization()
        return update, 200
    end

    local roll_ch  = p_rcmap_roll:get()
    local pitch_ch = p_rcmap_pitch:get()
    local yaw_ch   = p_rcmap_yaw:get()

    local roll_ch_obj  = rc:get_channel(roll_ch)
    local pitch_ch_obj = rc:get_channel(pitch_ch)
    local yaw_ch_obj   = rc:get_channel(yaw_ch)
    if not roll_ch_obj or not pitch_ch_obj or not yaw_ch_obj then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Invalid RC channel mapping.")
        return update, 100
    end

    local roll_in  = roll_ch_obj :norm_input_dz()
    local pitch_in = pitch_ch_obj:norm_input_dz()
    local yaw_in   = yaw_ch_obj  :norm_input_dz()

    local is_outside_roll_dz  = math.abs(roll_in)  > 0
    local is_outside_pitch_dz = math.abs(pitch_in) > 0
    local is_outside_yaw_dz   = math.abs(yaw_in)   > 0

    if SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
        local pilot_is_holding_input = is_outside_pitch_dz or is_outside_yaw_dz or is_outside_roll_dz
        if pilot_is_holding_input then
            SoarNavGlobals.last_pilot_input_ms = current_time_ms:toint()
        end
        check_pitch_gesture()
        check_roll_gesture()
        if not SoarNavGlobals.manual_override_active then
            if not pilot_is_holding_input and SoarNavGlobals.last_pilot_input_ms ~= 0 and (current_time_ms:toint() - SoarNavGlobals.last_pilot_input_ms) > SoarNavConstants.pilot_resume_delay_ms then
                set_script_state(SCRIPT_STATE.NAVIGATING, "Resuming navigation.")
            end
        end
        return update, 200
    else
        if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING and (is_outside_pitch_dz or is_outside_yaw_dz) then
            SoarNavGlobals.last_pilot_input_ms = current_time_ms:toint()
            set_script_state(SCRIPT_STATE.PILOT_OVERRIDE, "Pilot override detected.")
            SoarNavGlobals.pitch_gesture_state = "idle"
            SoarNavGlobals.pitch_gesture_count = 0
            SoarNavGlobals.pitch_gesture_triggered_this_override = false
            SoarNavGlobals.roll_gesture_state = "idle"
            SoarNavGlobals.roll_gesture_count = 0
            SoarNavGlobals.manual_override_active = false
            return update, 200
        end
    end

    local current_mode = vehicle:get_mode()
    local is_in_thermal_mode = (current_mode == SoarNavConstants.mode_thermal)

    if (p_tmem_enable:get()) == 1 then
        if is_in_thermal_mode and not SoarNavGlobals.was_in_thermal_mode then
            start_thermal_monitoring()
        elseif not is_in_thermal_mode and SoarNavGlobals.was_in_thermal_mode then
            stop_and_record_thermal()
        end
    end
    SoarNavGlobals.was_in_thermal_mode = is_in_thermal_mode

    if SoarNavGlobals.is_monitoring_thermal then
        local time_since_sample_ms = nil
        if SoarNavGlobals.last_thermal_sample_ms then
            time_since_sample_ms = current_time_ms:toint() - SoarNavGlobals.last_thermal_sample_ms:toint()
        end
        local strength = SoarNavGlobals.current_thermal_stats.max_strength or 1.0
        if strength <= 0 then strength = 1.0 end
        local sample_interval = math.max(1000, math.min(5000, 3000 / strength))
        if time_since_sample_ms and (time_since_sample_ms > sample_interval) then
            sample_thermal_strength()
        end
    end

    local script_switch_high = (rc:get_aux_cached(SoarNavConstants.rc_opt_soaring_active) == 2)
    local home = ahrs:get_home()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    local can_navigate = false

    if arming:is_armed() and script_switch_high and (current_mode == SoarNavConstants.mode_fbwb or current_mode == SoarNavConstants.mode_cruise) and home and dist_from_home_3d and ahrs:get_yaw_rad() and SoarNavGlobals.grid_initialized then
        local min_alt = p_soar_alt_min:get()
        local current_alt = -dist_from_home_3d:z()
        if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING or SoarNavGlobals.script_state == SCRIPT_STATE.THERMAL_PAUSE then
            if (current_alt >= (min_alt - 5)) then can_navigate = true end
        else
            if (current_alt >= min_alt) then can_navigate = true end
        end
    end

    if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING then
        if is_in_thermal_mode then
            set_script_state(SCRIPT_STATE.THERMAL_PAUSE, "Thermal detected. Pausing and monitoring.")
        elseif not can_navigate then
            set_script_state(SCRIPT_STATE.IDLE, "Navigation conditions not met.")
        end
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.THERMAL_PAUSE then
        if not is_in_thermal_mode then
            set_script_state(SCRIPT_STATE.IDLE, "Exited thermal, resuming.")
        end
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.IDLE then
        if can_navigate then
            set_script_state(SCRIPT_STATE.NAVIGATING, "Navigation conditions met, starting.")
        end
    end

    if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING then
        if not SoarNavGlobals.target_loc then
            if not SoarNavGlobals.waypoint_search_in_progress then
                log_gcs(MAV_SEVERITY.INFO, 1, "Searching for new waypoint...")
                SoarNavGlobals.waypoint_search_in_progress = true
            end
            search_for_new_waypoint()
        else
            SoarNavGlobals.distance_to_wp = loc:get_distance(SoarNavGlobals.target_loc)

            if SoarNavGlobals.reroute_check_armed and SoarNavGlobals.initial_distance_to_wp > 0 and SoarNavGlobals.distance_to_wp <= (SoarNavGlobals.initial_distance_to_wp / 2) then
                SoarNavGlobals.reroute_check_armed = false

                local function check_tactical_reroute_conditions()
                    local min_path_for_reroute = SoarNavGlobals.max_operational_distance / 2
                    if SoarNavGlobals.initial_distance_to_wp <= min_path_for_reroute then
                        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Reroute skipped: path < %.0fm.", min_path_for_reroute))
                        return false
                    end

                    local total_valid_cells = #SoarNavGlobals.valid_cell_indices
                    if total_valid_cells == 0 then return false end
                    local unvisited_count = #SoarNavGlobals.unvisited_cell_indices
                    
                    if unvisited_count <= 1 then
                        log_gcs(MAV_SEVERITY.INFO, 2, "Reroute skipped: no cells remain.")
                        return false
                    end

                    local explored_ratio = (total_valid_cells - unvisited_count) / total_valid_cells
                    if explored_ratio >= 0.9 then
                        log_gcs(MAV_SEVERITY.INFO, 2, "Reroute skipped: grid >90% explored.")
                        return false
                    end

                    if math.random(1, 100) > SoarNavConstants.mid_flight_reroute_chance then
                        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Reroute skipped: probability check (%d%%).", SoarNavConstants.mid_flight_reroute_chance))
                        return false
                    end
                    
                    return true
                end

                if check_tactical_reroute_conditions() then
                    log_gcs(MAV_SEVERITY.NOTICE, 1, "Tactical reroute engaged.")
            
                    local old_target_cell = get_cell_index_from_location(SoarNavGlobals.target_loc)
                    if old_target_cell and SoarNavGlobals.grid_cells[old_target_cell] then
                        local already_in_list = false
                        for _, cell in ipairs(SoarNavGlobals.unvisited_cell_indices) do
                            if cell == old_target_cell then
                                already_in_list = true
                                break
                            end
                        end
                        if not already_in_list then
                           table.insert(SoarNavGlobals.unvisited_cell_indices, old_target_cell)
                           log_gcs(MAV_SEVERITY.INFO, 2, string.format("Cell %d returned to unvisited list.", old_target_cell))
                        end
                    end
            
                    SoarNavGlobals.target_loc = nil
                    SoarNavGlobals.waypoint_search_in_progress = true
                    SoarNavGlobals.initial_distance_to_wp = -1
            
                    return update, 200
                end
            end

            local time_on_current_wp = current_time_ms:toint() - SoarNavGlobals.waypoint_start_time_ms:toint()
            if SoarNavGlobals.waypoint_start_time_ms:toint() > 0 and time_on_current_wp > get_wp_timeout() then
                log_gcs(MAV_SEVERITY.WARNING, 1, "WP Timeout. New WP.")
                SoarNavGlobals.target_loc = nil
                SoarNavGlobals.waypoint_search_in_progress = true
                SoarNavGlobals.is_repositioning = false
                return update, 200
            end

            if SoarNavGlobals.distance_to_wp < (p_wp_radius:get()) then
                if SoarNavGlobals.is_repositioning then
                    log_gcs(MAV_SEVERITY.INFO, 1, "Repositioned. Re-engaging target.")
                    SoarNavGlobals.target_loc = SoarNavGlobals.original_target_loc
                    SoarNavGlobals.is_repositioning = false
                    SoarNavGlobals.original_target_loc = nil
                    SoarNavGlobals.waypoint_start_time_ms = millis()
                    local current_loc = ahrs:get_location()
                    if current_loc and SoarNavGlobals.target_loc then
                       SoarNavGlobals.initial_distance_to_wp = current_loc:get_distance(SoarNavGlobals.target_loc)
                       SoarNavGlobals.reroute_check_armed = true
                    end
                    SoarNavGlobals.distance_to_wp = -1
                    SoarNavGlobals.stuck_counter = 0
                    SoarNavGlobals.last_progress_check_ms = 0
                    SoarNavGlobals.distance_at_last_check = -1
                else
                    log_gcs(MAV_SEVERITY.INFO, 1, "Waypoint reached.")
                    if SoarNavGlobals.current_selected_hotspot
                       and string.find(SoarNavGlobals.g_waypoint_source_info, "DriftThrm") then
                        local hotspot_to_check
                        for i, hotspot in ipairs(SoarNavGlobals.thermal_hotspots) do
                            if hotspot.timestamp == SoarNavGlobals.current_selected_hotspot.timestamp then
                                hotspot_to_check = hotspot
                                break
                            end
                        end
                        
                        if hotspot_to_check then
                            hotspot_to_check.failed_attempts = (hotspot_to_check.failed_attempts or 0) + 1
                            if hotspot_to_check.failed_attempts >= 2 then
                                table.remove(SoarNavGlobals.thermal_hotspots, i)
                                log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal removed (attempts: %d).", hotspot_to_check.failed_attempts))
                            else
                                log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal not found (attempt %d).", hotspot_to_check.failed_attempts))
                            end
                        end
                        SoarNavGlobals.current_selected_hotspot = nil
                    end

                    SoarNavGlobals.target_loc = nil
                    SoarNavGlobals.distance_to_wp = -1
                    SoarNavGlobals.waypoint_search_in_progress = true
                end
                return update, 200
            end

            if SoarNavGlobals.waypoint_start_time_ms:toint() > 0 and time_on_current_wp > SoarNavConstants.STUCK_GRACE_PERIOD_MS and not SoarNavGlobals.is_repositioning then
                if SoarNavGlobals.last_progress_check_ms == 0 or (current_time_ms:toint() - SoarNavGlobals.last_progress_check_ms) > SoarNavConstants.STUCK_PROGRESS_CHECK_INTERVAL_MS then
                    if SoarNavGlobals.distance_at_last_check ~= -1 then
                        local progress_made = SoarNavGlobals.distance_at_last_check - SoarNavGlobals.distance_to_wp
                        if progress_made < SoarNavConstants.STUCK_MIN_PROGRESS_M then
                            SoarNavGlobals.stuck_counter = SoarNavGlobals.stuck_counter + 1
                            log_gcs(MAV_SEVERITY.INFO, 2, string.format("Stuck counter: %d.", SoarNavGlobals.stuck_counter))
                        else
                            SoarNavGlobals.stuck_counter = 0
                        end
                    end
                    SoarNavGlobals.last_progress_check_ms = current_time_ms:toint()
                    SoarNavGlobals.distance_at_last_check = SoarNavGlobals.distance_to_wp
                end
            end

            local wind_vec = get_wind_vector()
            local wind_speed = wind_vec and wind_vec:length() or 0
            local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 4)))
            if SoarNavGlobals.stuck_counter >= adaptive_stuck_limit and not SoarNavGlobals.is_repositioning then
                SoarNavGlobals.original_target_loc = SoarNavGlobals.target_loc
                SoarNavGlobals.is_repositioning = true
                local current_loc = ahrs:get_location()
                if wind_vec and current_loc then
                    local repo_dist_m = math.max(150, math.min(700, wind_speed * 60))
                    log_gcs(MAV_SEVERITY.INFO, 1, string.format("Stuck. Repositioning upwind %.0fm.", repo_dist_m))
                    local wind_heading_rad = wind_vec:xy():angle()
                    local upwind_bearing = (math.deg(wind_heading_rad) + 180 + 360) % 360
                    local repo_loc = current_loc:copy()
                    repo_loc:offset_bearing(upwind_bearing, repo_dist_m)
                    SoarNavGlobals.target_loc = repo_loc
                    SoarNavGlobals.waypoint_start_time_ms = millis()
                    SoarNavGlobals.stuck_counter = 0
                    SoarNavGlobals.last_progress_check_ms = 0
                    SoarNavGlobals.distance_at_last_check = -1
                    SoarNavGlobals.initial_distance_to_wp = -1
                    SoarNavGlobals.reroute_check_armed = false
                    log_gcs(MAV_SEVERITY.INFO, 1, "Tactical upwind WP set.")
                else
                    SoarNavGlobals.is_repositioning = false
                    SoarNavGlobals.target_loc = nil
                end
                 return update, 200
            end

            local curr_lat, curr_lon = loc:lat()/1e7, loc:lng()/1e7
            local target_heading = calculate_bearing(curr_lat, curr_lon, SoarNavGlobals.target_loc:lat()/1e7, SoarNavGlobals.target_loc:lng()/1e7)
            local current_heading = math.deg(ahrs:get_yaw_rad())
            local heading_error = (target_heading - current_heading + 540) % 360 - 180
            if SoarNavGlobals.last_grid_update_ms == 0 or (current_time_ms:toint() - SoarNavGlobals.last_grid_update_ms) > SoarNavConstants.grid_update_interval_ms then
                update_visited_cell()
                SoarNavGlobals.last_grid_update_ms = current_time_ms:toint()
                if (p_log_lvl:get()) >= 2 then
                    local roll_limit_for_log = p_roll_limit:get()
                    local p_gain_for_log = p_nav_p:get()
                    local d_gain_for_log = p_nav_d:get()
                    local error_derivative_for_log = (heading_error - SoarNavGlobals.last_heading_error) * 5
                    local p_term_for_log = heading_error * p_gain_for_log
                    local d_term_for_log = error_derivative_for_log * d_gain_for_log
                    local clamped_p_term_for_log = math.max(-roll_limit_for_log, math.min(roll_limit_for_log, p_term_for_log))
                    local smoothed_p_cmd_for_log = (SoarNavConstants.roll_smoothing_factor * clamped_p_term_for_log) + ((1.0 - SoarNavConstants.roll_smoothing_factor) * SoarNavGlobals.last_commanded_roll_deg)
                    local desired_roll_for_log = smoothed_p_cmd_for_log + d_term_for_log
                    desired_roll_for_log = math.max(-roll_limit_for_log, math.min(roll_limit_for_log, desired_roll_for_log))
                    local target_line = string.format("Nav to: %s", SoarNavGlobals.g_waypoint_source_info or "Unknown")
                    local wp_state_line = string.format("WP: D:%.0fm, Hdg Err:%+.0f, Roll:%+.1f", SoarNavGlobals.distance_to_wp, heading_error, desired_roll_for_log)
                    local visited, total_cells = 0, #SoarNavGlobals.valid_cell_indices
                    if total_cells > 0 then
                        visited = total_cells - #SoarNavGlobals.unvisited_cell_indices
                    end
                    local grid_percent = total_cells > 0 and (100 * visited / total_cells) or 0
                    local grid_line = string.format("Grid: %d/%d expl. %.0f%% | Curr. Cell: %d", visited, total_cells, grid_percent, SoarNavGlobals.last_cell_index or 0)
                    local valid_hotspots_log = clean_and_get_hotspots()
                    local tmem_count = #valid_hotspots_log
                    local tmem_line
                    if tmem_count > 0 then
                        local best_hotspot = valid_hotspots_log[1]
                        for i = 2, tmem_count do
                            if valid_hotspots_log[i].avg_strength and best_hotspot.avg_strength and valid_hotspots_log[i].avg_strength > best_hotspot.avg_strength then
                                best_hotspot = valid_hotspots_log[i]
                            end
                        end
                        tmem_line = string.format("Thermal mem: %d active (Best: +%.1fm/s)", tmem_count, best_hotspot.avg_strength or 0)
                    else
                        tmem_line = "Thermal mem: 0 active"
                    end
                    log_gcs(MAV_SEVERITY.INFO, 2, target_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, wp_state_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, grid_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, tmem_line)
                end
            end
            local max_roll_angle = math.max(10, math.min(50, p_roll_limit:get()))
            local p_gain = math.max(0, p_nav_p:get())
            local d_gain = math.max(0, p_nav_d:get())
            local error_derivative = (heading_error - SoarNavGlobals.last_heading_error) * 5
            SoarNavGlobals.last_heading_error = heading_error
            local p_term = heading_error * p_gain
            local d_term = error_derivative * d_gain
            local raw_pd_command = p_term + d_term
            local smoothed_command
            if math.abs(raw_pd_command) > math.abs(SoarNavGlobals.last_commanded_roll_deg) then
                smoothed_command = (SoarNavConstants.roll_smoothing_factor * raw_pd_command) + ((1.0 - SoarNavConstants.roll_smoothing_factor) * SoarNavGlobals.last_commanded_roll_deg)
            else
                smoothed_command = raw_pd_command
            end
            local desired_roll_deg = math.max(-max_roll_angle, math.min(max_roll_angle, smoothed_command))
            SoarNavGlobals.last_commanded_roll_deg = desired_roll_deg
            local system_max_roll = p_sys_roll_limit:get()
            if system_max_roll <= 0 then system_max_roll = 45 end
            local roll_normalized = desired_roll_deg / system_max_roll
            local roll_pwm_value
            if roll_normalized > 0 then
                roll_pwm_value = SoarNavGlobals.rc1_trim + roll_normalized * (SoarNavGlobals.rc1_max - SoarNavGlobals.rc1_trim)
            else
                roll_pwm_value = SoarNavGlobals.rc1_trim + roll_normalized * (SoarNavGlobals.rc1_trim - SoarNavGlobals.rc1_min)
            end
            if SoarNavGlobals.rc_roll_channel then
                SoarNavGlobals.rc_roll_channel:set_override(math.floor(roll_pwm_value))
            else
                set_script_state(SCRIPT_STATE.ERROR, "Roll channel object is nil, cannot set override.")
            end
        end
    end
    return update, 200
end

-- Wrapper to catch and log any runtime errors in the main loop.
update = function()
    local results = {pcall(update_body)}
    local ok = table.remove(results, 1)

    if not ok then
        log_gcs(MAV_SEVERITY.CRITICAL, 0, "SoarNav CRITICAL ERROR: " .. tostring(results[1]))
        return update, 5000
    end

    return table.unpack(results)
end

log_gcs(MAV_SEVERITY.INFO, 1, "SoarNav Script Initialized.")
return update()