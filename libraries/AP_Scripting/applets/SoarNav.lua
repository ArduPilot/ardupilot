--[[
SoarNav: an advanced feature for hunting thermals by Marco Robustini.
-- Version 0.9.8.2 - 2025/08/13

SoarNav is an advanced Lua script for intelligent, autonomous soaring
that enhances ArduPilot's capabilities. It uses a suite of strategic
features to maximize flight time within a defined area.

The script employs a hybrid exploration strategy, dynamically balancing
the search for new grid cells with guided re-visiting of the least
explored areas based on recent thermal success. This is coupled with an
intelligent thermal memory that saves thermal core locations. A
sophisticated clustering algorithm then identifies promising hot zones
using a weighted score based on nearby thermals' strength, age, and
proximity.

Real-time thermal analysis is performed using adaptive sampling for a
responsive strength calculation. The script is architected for robust
performance with a modular state machine and parameter caching. Safety
is enhanced with dynamic anti-stuck logic, tactical rerouting, and
thermal street detection capabilities.

================================================================================
Key Features
================================================================================

- **Hybrid Exploration Strategy (Virtual Grid)**:
  Employs a dual-mode grid search, dynamically alternating between
  **Guided Exploration** (re-visiting the least-explored cells) and
  **Pure Exploration** (seeking new, unvisited cells) based on recent
  thermal success.

- **Efficient & Optimized**:
  The script is optimized to reduce CPU load by leveraging native ArduPilot
  APIs, a modular state machine, and parameter caching.

- **Advanced Thermal Analysis (Strength & Quality)**:
  Utilizes **Adaptive Sampling** (a weighted EMA based on climb rate) to
  rapidly and accurately calculate a thermal's average strength, peak
  strength, and **consistency** (steady vs. variable lift).

- **Intelligent Thermal Memory (Core, Clustering & Focus Mode)**:
  Saves not just the entry point but the **thermal core** (location of
  strongest lift) for greater accuracy. A sophisticated **clustering**
  algorithm identifies "hot zones" using a weighted score of nearby
  thermals' strength, age, and proximity. Can engage a **Focus Mode** to
  search high-density areas.

- **Energy-Aware & Dual-Strategy Decision Making**:
  Uses a **time-corrected low-pass filter** on normalized altitude to
  determine energy state, ensuring smooth and stable transitions between
  NORMAL, LOW, and CRITICAL states.

- **Thermal Streeting (Pattern Recognition)**:
  Instead of treating thermals as isolated events, the script attempts
  to identify **"thermal streets"**—lines of lift aligned with the wind to
  proactively search along the projected street axis.

- **Dynamic Thermal Drift Prediction**:
  Uses the current wind vector and a hotspot's age to predict its new,
  drifted position, increasing the chances of a successful intercept.

- **Safety & Tactical Navigation**:
  Includes pre-flight parameter validation, robust and **dynamic
  anti-stuck logic** that increases repositioning distance and varies the
  upwind angle on subsequent attempts, and a tactical mid-flight re-route.

- **Adaptive Waypoint Timeout**:
  Intelligently adjusts the time allowed to reach a waypoint based on
  current wind conditions, allowing more time in strong headwinds.

- **Intuitive Pilot Override & Stick Gestures**:
  - **Temporary Override**: Pitch or Yaw input instantly pauses
    autonomous navigation.
  - **Persistent Override (Roll Gesture)**: A rapid roll gesture toggles a
    persistent manual override.
  - **Dynamic Area Re-centering (Pitch Gesture)**: A rapid pitch gesture
    re-centers the circular search area to the aircraft's current location.

- **Dual Area System (Radius or Polygon)**:
  Supports both a circular area and a custom polygon loaded from the SD card.
  The SNAV_ENABLE parameter allows selection of multiple polygon files.

- **Advanced Logging System**:
  A multi-level logging system provides clear operational feedback, from
  key events to detailed real-time status for debugging.

--------------------------------------------------------------------------------
Script Parameters (SNAV_*)
--------------------------------------------------------------------------------
- SNAV_ENABLE: Master switch and Polygon Selector. 0:Disabled. >0:Enabled.
  When SNAV_MAX_DIST is 0, this value also selects the polygon file to use
  (e.g., 1 for snav1.poly, 2 for snav2.poly, up to 9). If the selected number
  is out of range or the file is invalid, the script will automatically
  load the lowest-numbered available polygon (e.g., snav1.poly) as a safe
  fallback.
- SNAV_LOG_LVL: Sets the verbosity level for GCS messages (0:Silent, 1:Events,
  2:Detailed).
- SNAV_MAX_DIST: Radius in meters of the circular flight area (0 for polygon).
- SNAV_ROLL_LIMIT: Maximum roll angle in degrees for navigation.
- SNAV_WP_RADIUS: Acceptance radius in meters for waypoints.
- SNAV_NAV_P / SNAV_NAV_D: Gains for the PD navigation controller.
- SNAV_TMEM_ENABLE: Enables (1) or disables (0) the Thermal Memory feature.
- SNAV_TMEM_LIFE: Lifetime in seconds of a stored thermal hotspot.
- SNAV_STUCK_EFF: Minimum path efficiency to not be considered "stuck".
  Higher is more sensitive.
- SNAV_STUCK_TIME: Grace time in seconds before anti-stuck logic becomes active.
- SNAV_REENG_DWELL: Time in seconds to loiter at a re-engagement point before
  aborting.
- SNAV_RETRY_THR: Score threshold (0-100) to trigger a retry on a weak thermal.
  Lower is more aggressive.
- SNAV_STREET_TOL: Angle tolerance in degrees for detecting a thermal street.
- SNAV_REROUTE_P: Probability (%) of a mid-flight tactical reroute. Higher is
  more opportunistic.
- SNAV_NEC_WEIGHT: Weight of the 'necessity' factor (low altitude) in the retry
  score.
- SNAV_TMEM_MIN_S: Minimum thermal strength (m/s) to be saved to memory.
- SNAV_FOCUS_THR: Density score threshold to trigger Focus Mode.
- SNAV_STRAT_HIST: Time window (s) for thermal success history to influence
  strategy.
- SNAV_WP_TIMEOUT: Base time in seconds to reach a waypoint before timeout.
- SNAV_REROUTE_MIN: Minimum angle in degrees for a tactical reroute maneuver.
- SNAV_REROUTE_MAX: Maximum angle in degrees for a tactical reroute maneuver.
- SNAV_DYN_SOALT: Controls the Glide Cone behavior. 0:Disabled, 1:Fully Linked
  (MIN/CUTOFF/MAX), 2:MIN Only (capped at 2/3 of CUTOFF).
- SNAV_GC_MARGIN: Safety altitude margin added on top of the calculated glide
  altitude for the Glide Cone feature.
- SNAV_GC_PAD: Extra altitude padding to ensure arrival over the home point,
  not at ground level, for the Glide Cone feature.

================================================================================
CHANGELOG / DEVELOPER NOTES
================================================================================
- v0.9.8.2 (2025/08/16): Major Tactical Re-route Overhaul & Hardening.
  The re-route maneuver is now calculated relative to the aircraft's current
  **heading (HDG)**, not the bearing to the old waypoint. This makes the
  maneuver a more intuitive and predictable tactical turn from the pilot's
  perspective. The logic was also hardened against transient nil values from
  the AHRS to prevent errors. The post-reroute search distance is now
  adaptive, scaling with both grid cell size and total flight area for
  more effective new waypoint selection.
- v0.9.8.1 (2025/08/15): Implemented Glide Cone and Robust Motor Failure Detection.
  Added a major safety feature ("Glide Cone") to prevent the script from flying
  too low for a safe, motor-off glide back home. This logic is fully
  configurable via the new SNAV_DYN_SOALT parameter to dynamically modulate
  soaring altitudes (MIN, CUTOFF, MAX) based on distance from home, and
  correctly restores boot values when autonomous navigation is paused.
  This version also introduces a critical Motor Failure Detection system with
  automatic RTL. Tightly integrated with the Glide Cone, this feature is
  active only when SNAV_DYN_SOALT is enabled, ensuring an RTL is only
  commanded when a safe glide return is possible. The system monitors for
  propulsion failure if the aircraft descends below SOAR_ALT_MIN and fails to
  establish a positive climb after a configurable delay. To maximize robustness
  and prevent false positives, the check is automatically suppressed in several
  key contexts: during Pilot Override, in THERMAL mode, within a dynamic
  safety bubble near home, and when the script is not in its primary
  NAVIGATING state. The logic is also resilient to dynamic changes in the
  SOAR_ALT_MIN threshold and requires both a low climb rate and a failure to
  gain altitude before triggering, making it robust against turbulence.
- v0.9.8.0 (2025/08/13): Major User Tunability Overhaul & GCS Alerting Refinements.
  Converted key algorithm constants into thirteen new tunable SNAV_* parameters,
  giving the pilot direct control over the script's core behavior.
  Added parameters for Anti-Stuck (SNAV_STUCK_EFF, SNAV_STUCK_TIME), Thermal
  Strategy (SNAV_RETRY_THR, SNAV_NEC_WEIGHT, SNAV_REENG_DWELL, SNAV_TMEM_MIN_S,
  SNAV_FOCUS_THR, SNAV_STRAT_HIST), and Navigation (SNAV_STREET_TOL,
  SNAV_REROUTE_P, SNAV_WP_TIMEOUT, SNAV_REROUTE_MIN, SNAV_REROUTE_MAX).
  Reorganized GCS message severities for more intuitive pilot feedback
  (e.g., anti-stuck is now a WARNING). General code cleanup and correction
  of minor logical bugs and unused variables.
- v0.9.7.1 (2025/08/13): Final Code Hardening & Critical Navigation Fix.
  Corrected a critical navigation bug where a unit mismatch (radians vs.
  degrees) in the bearing calculation caused the aircraft to fly straight. The
  controller now correctly converts all units, restoring navigation.
  Corrected the RC roll override release mechanism to use the proper API
  value (0) during state transitions, preventing runtime errors. Hardened
  system roll limit fetching with a safe fallback and added a time hysteresis
  to the CRITICAL energy state trigger to prevent false alarms in turbulent
  air. Unified all bearing calculations to use the native ArduPilot API.
- v0.9.7 (2025/08/12): Major Reroute Logic Overhaul & Logging System Refinements.
  This is a significant update consolidating numerous bug fixes, strategic
  enhancements, and quality-of-life improvements.
  Overhauled the tactical reroute system for greater intelligence and
  robustness. The maneuver is now skipped for thermal targets only if the
  aircraft is not 'stuck'. Fixed a critical bug causing desynchronization
  of the unvisited cell list, widened the post-reroute search arc for
  better effectiveness, and added a safety guard-rail for nil targets.
  Significantly refined the GCS logging system to improve clarity and reduce
  noise. Suppressed repetitive Level 1 logs during Focus Mode and Re-Engage
  maneuvers, and re-leveled detailed thermal target messages to Level 2.
  Corrected a log synchronization bug on script startup and improved the
  clarity of pilot override and resume messages.
  Refactored the core navigation handler to eliminate one-cycle delays,
  ensuring immediate and reliable new waypoint selection after completions,
  timeouts, or reroutes.
  Increased user control by making the adaptive waypoint timeout logic fully
  parametric via constants. Improved internal code documentation for key
  features.
- v0.9.63 (2025/08/11): Scoring, Re-engagement & Altitude Safety.
  Added a two-leg re-engage maneuver (Fly-Out → Re-Entry) with a wind/age-drifted
  re-entry target and an optional dwell at Re-Entry (REENGAGE_DWELL_MS).
  Tactical reroute is skipped while re-engaging.
  The decision to retry a weak/uncertain thermal is now driven by a weighted
  score (uncertainty from thermal strength, necessity from energy/altitude).
  Added safety check to prevent thermal-seeking above SOAR_ALT_MAX, forcing
  a neutral grid search to align with ArduPilot's safety philosophy.
- v0.9.62 (2025/08/10): Addressed key logical bugs in thermal selection and
  exploration strategies.
  Ensured the selected hotspot from memory is correctly referenced, fixing the
  failed attempt tracking logic.
  Corrected thermal street projection to navigate downwind, aligning with
  expected meteorological behavior.
  Fixed mathematical error in random waypoint generation to provide uniform
  area exploration and prevent central clustering.
  Refined GCS logging to make 'LOW ENERGY' messages conditional to the actual
  energy state.
- v0.9.61 (2025/08/09): Fixed critical logical bugs to improve reliability and
  efficiency.
  Hotspot saving now correctly stores the wind vector present at thermal entry.
  Hardened the wind cache logic to prevent potential type errors during
  initialization.
  Corrected swapped height and width in grid creation for more efficient
  exploration.
  Unified thermal waypoint labels to fix a critical reroute skip bug.
- v0.9.60 (2025/08/07): Refactored main navigation handler into smaller
  functions.
  PD controller derivative term is now time-corrected (uses dt).
  Improved hotspot selection to be deterministic instead of random tournament.
  Moved hardcoded "magic numbers" to SoarNavConstants.
  Cleaned up unused variables and comment formatting.
- v0.9.59 (2025/08/05): Major architectural refactoring.
  Implemented modular state machine for better readability and maintenance.
  Fixed memory leak by merging thermal entry timestamps into hotspot objects.
  Added parameter caching in main loop to reduce overhead.
- v0.9.58 (2025/08/05): Major algorithmic and logic enhancements.
  Implemented hybrid grid exploration (guided/random) based on success rate.
  Introduced dual-mode roll smoothing based on param sign.
  Replaced energy state logic with a time-corrected low-pass filter.
  Overhauled thermal density calculation to a weighted score model.
  Implemented adaptive thermal sampling using a weighted EMA.
  Enhanced anti-stuck logic with dynamic repositioning.
- v0.9.57 (2025/08/05): Added 90-degree reroute logic for tactical reroutes.
  100% compliant with Lua checker.
- v0.9.56 (2025/08/04): Implemented high-impact optimizations: Bounding box
  check for polygons, debounced (distance-based) grid cell updates, and
  pre-calculation of grid cell centers for faster waypoint selection.
- v0.9.55 (2025/08/04): Introduced Focus Mode; upon detecting a thermal
  cluster, the script now concentrates its search in a high-density sub-grid.
  The Cluster Radius is now dynamic, scaling with the flight area's size.
  Implemented Lost Thermal-Awareness to force grid exploration after
  consecutive failed attempts.
- v0.9.54 (2025/08/04): Implemented Thermal Core Estimation to save the
  location of the strongest lift, not just the entry point. Added Thermal
  Clustering logic to prioritize exploring areas with a high density of
  previously found thermals.
- v0.9.53 (2025/08/03): Added multi-polygon file selection via SNAV_ENABLE.
  Maximum cell values, cell size and default wp radius remodeled.
- v0.9.52 (2025/08/03): Added "Thermal Streeting" pattern recognition logic to
  proactively search for thermals along wind lines. Refactored waypoint
  selection logic for cleaner integration. Updated header description.
- v0.9.51 (2025/08/02): Added Mid-flight Tactical Re-route feature, shortened
  all GCS log messages, corrected thermal direction drift, minor cosmetic fix.
- v0.9.41 (2025/08/01): Fixed a critical logic bug in the anti-stuck feature
  where the script would not re-engage the original waypoint after completing
  an upwind repositioning maneuver.
- v0.9.4 (2025/07/31): Major refactoring to leverage native ArduPilot APIs
  (e.g., Location objects, get_distance, offset_bearing) instead of manual
  Lua calculations.
- v0.9.2 (2025/07/29): Initial version published with manual geospatial
  calculations.
]]

math.randomseed(millis():toint())

-- Mavlink severity levels for GCS messages
local MAV_SEVERITY = {
    CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6
}

-- Script operational states
local SCRIPT_STATE = {
    IDLE = 0,
    NAVIGATING = 1,
    PILOT_OVERRIDE = 2,
    THERMAL_PAUSE = 3,
    ERROR = 4
}

local SoarNavWaypointSources = {
    THERMAL_MEMORY_DRIFT = "Thrm(Drift, +%.1f)",
    THERMAL_MEMORY_NODRIFT = "Thrm(NoDrift, +%.1f)",
    THERMAL_STREET = "Thrml Street",
    REENGAGE_FLYOUT = "Re-Engage FlyOut",
    REENGAGE_ENTRY = "Re-Engage ReEntry",
    FOCUS_MODE = "Focus (WP %d)",
    GRID_GUIDED = "Grid (Guided)",
    GRID_PURE = "Grid (Pure)",
    GRID_ENERGY = "Grid (Alt: %s)",
    GRID_CELL = "Cell: %d",
    RANDOM_FALLBACK = "Random Fallback",
    UNKNOWN = "Unknown"
}

--[[
 // @Param: SNAV_ENABLE
 // @DisplayName: SoarNav Enable / Poly Selector
 // @Description: Enable script and select polygon. 0:Disabled, >0:Enabled. If MAX_DIST is 0, value 1-9 selects snavX.poly.
 // @Values: 0: Disable script. 1-9: Enable script and select the corresponding polygon file (snav1.poly, snav2.poly, etc.).
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
 // @Description: Maximum distance from home or 0 for polygon area
 // @Units: m
]]
--[[
 // @Param: SNAV_ROLL_LIMIT
 // @DisplayName: Roll Limit
 // @Description: Maximum roll angle for navigation
 // @Units: deg
]]
--[[
 // @Param: SNAV_WP_RADIUS
 // @DisplayName: Waypoint Radius
 // @Description: Acceptance radius for virtual waypoints
 // @Units: m
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
 // @Description: Lifetime of a thermal hotspot
 // @Units: s
]]
--[[
 // @Param: SNAV_STUCK_EFF
 // @DisplayName: Anti-Stuck Efficiency
 // @Description: Minimum path efficiency to not be considered "stuck", higher is more sensitive.
 // @Range: 0.1 0.9
]]
--[[
 // @Param: SNAV_STUCK_TIME
 // @DisplayName: Anti-Stuck Grace Time
 // @Description: Grace time before anti-stuck logic becomes active.
 // @Units: s
]]
--[[
 // @Param: SNAV_REENG_DWELL
 // @DisplayName: Re-engage Dwell Time
 // @Description: Time to loiter at a re-engagement point before aborting.
 // @Units: s
]]
--[[
 // @Param: SNAV_RETRY_THR
 // @DisplayName: Thermal Retry Threshold
 // @Description: Score threshold (0-100) to trigger a retry on a weak thermal. Lower is more aggressive.
 // @Range: 0 100
]]
--[[
 // @Param: SNAV_STREET_TOL
 // @DisplayName: Thermal Street Tolerance
 // @Description: Angle tolerance for detecting a thermal street.
 // @Units: deg
]]
--[[
 // @Param: SNAV_REROUTE_P
 // @DisplayName: Tactical Reroute Probability
 // @Description: Probability of a mid-flight tactical reroute. Higher is more opportunistic.
 // @Units: %
]]
--[[
 // @Param: SNAV_NEC_WEIGHT
 // @DisplayName: Retry Score Necessity Weight
 // @Description: Weight of the 'necessity' factor (low altitude) in the retry score. Higher makes it more conservative.
 // @Range: 0 100
]]
--[[
 // @Param: SNAV_TMEM_MIN_S
 // @DisplayName: Min Thermal Strength to Save
 // @Description: Minimum thermal strength (m/s) to be saved to memory.
 // @Units: m/s
]]
--[[
 // @Param: SNAV_FOCUS_THR
 // @DisplayName: Focus Mode Threshold
 // @Description: Density score threshold to trigger Focus Mode.
]]
--[[
 // @Param: SNAV_STRAT_HIST
 // @DisplayName: Strategy History Window
 // @Description: Time window for thermal success history to influence strategy.
 // @Units: s
]]
--[[
 // @Param: SNAV_WP_TIMEOUT
 // @DisplayName: Base WP Timeout
 // @Description: Base time to reach a waypoint before timeout.
 // @Units: s
]]
--[[
 // @Param: SNAV_REROUTE_MIN
 // @DisplayName: Reroute Min Angle
 // @Description: Minimum angle for a tactical reroute maneuver.
 // @Units: deg
]]
--[[
 // @Param: SNAV_REROUTE_MAX
 // @DisplayName: Reroute Max Angle
 // @Description: Maximum angle for a tactical reroute maneuver.
 // @Units: deg
]]
--[[
 // @Param: SNAV_DYN_SOALT
 // @DisplayName: SoarNav Dynamic Altitude Mode
 // @Description: Controls the dynamic altitude (Glide Cone) behavior. 0:Disabled, 1:Fully Linked (MIN/CUTOFF/MAX), 2:MIN Only (capped at 2/3 of CUTOFF).
 // @Values: 0:Disabled,1:Fully Linked,2:MIN Only (Capped)
]]
--[[
 // @Param: SNAV_GC_MARGIN
 // @DisplayName: Glide Cone Safety Margin
 // @Description: Safety altitude margin added on top of the calculated glide altitude.
 // @Units: m
]]
--[[
 // @Param: SNAV_GC_PAD
 // @DisplayName: Glide Cone Home Padding
 // @Description: Extra altitude padding to ensure arrival over the home point, not at ground level.
 // @Units: m
]]

local PARAM_TABLE_KEY = 111
local PARAM_PREFIX = "SNAV_"

local param_list = {
    {name = "ENABLE", default = 0},
    {name = "LOG_LVL", default = 1},
    {name = "MAX_DIST", default = 500},
    {name = "ROLL_LIMIT", default = 30},
    {name = "WP_RADIUS", default = 50},
    {name = "NAV_P", default = 0.6},
    {name = "NAV_D", default = 0.05},
    {name = "TMEM_ENABLE", default = 1},
    {name = "TMEM_LIFE", default = 1200},
    {name = "STUCK_EFF", default = 0.35},
    {name = "STUCK_TIME", default = 30},
    {name = "REENG_DWELL", default = 7},
    {name = "RETRY_THR", default = 30},
    {name = "STREET_TOL", default = 30},
    {name = "REROUTE_P", default = 50},
    {name = "NEC_WEIGHT", default = 50},
    {name = "TMEM_MIN_S", default = 0.2},
    {name = "FOCUS_THR", default = 1.0},
    {name = "STRAT_HIST", default = 900},
    {name = "WP_TIMEOUT", default = 300},
    {name = "REROUTE_MIN", default = 80},
    {name = "REROUTE_MAX", default = 100},
    {name = "DYN_SOALT", default = 0},
    {name = "GC_MARGIN", default = 25},
    {name = "GC_PAD", default = 20},
}

-- Creates the script's parameter table in ArduPilot
local function add_params()
    local ok, err = pcall(function()
        assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, #param_list),
               string.format("ERROR: SoarNav - Could not add param table '%s'.", PARAM_PREFIX))
        for i, p in ipairs(param_list) do
            assert(param:add_param(PARAM_TABLE_KEY, i, p.name, p.default),
                   string.format("ERROR: SoarNav - Could not add param %s%s.", PARAM_PREFIX, p.name))
        end
    end)
    if not ok then
        gcs:send_text(MAV_SEVERITY.WARNING,
                      "SoarNav: " .. "SoarNav parameters skipped: " .. tostring(err))
    end
end

add_params()

-- Bind script parameters to local variables for efficient access
local p_enable, p_log_lvl, p_max_dist, p_roll_limit, p_wp_radius, p_nav_p, p_nav_d,
      p_tmem_enable, p_tmem_life, p_soar_alt_min, p_soar_alt_max,
      p_rcmap_roll, p_rcmap_pitch, p_rcmap_yaw,
      p_stuck_eff, p_stuck_time, p_reeng_dwell, p_retry_thr, p_street_tol, p_reroute_p,
      p_nec_weight, p_tmem_min_s, p_focus_thr, p_strat_hist, p_wp_timeout,
      p_reroute_min, p_reroute_max, p_soar_alt_cutoff, p_soar_polar_cd0, p_soar_polar_b, p_airspeed_cruise, p_dyn_soalt, p_gc_margin, p_gc_pad

-- Helper function for safely binding parameters
local function bind_param(name)
    local p = Parameter(name)
    if not p then
        gcs:send_text(MAV_SEVERITY.WARNING, "SoarNav: Missing parameter: "..name)
    end
    return p
end

local function bind_params()
    p_enable = bind_param(PARAM_PREFIX .. "ENABLE")
    p_log_lvl = bind_param(PARAM_PREFIX .. "LOG_LVL")
    p_max_dist = bind_param(PARAM_PREFIX .. "MAX_DIST")
    p_nav_d = bind_param(PARAM_PREFIX .. "NAV_D")
    p_nav_p = bind_param(PARAM_PREFIX .. "NAV_P")
    p_rcmap_pitch = bind_param("RCMAP_PITCH")
    p_rcmap_roll = bind_param("RCMAP_ROLL")
    p_rcmap_yaw = bind_param("RCMAP_YAW")
    p_roll_limit = bind_param(PARAM_PREFIX .. "ROLL_LIMIT")
    p_soar_alt_max = bind_param("SOAR_ALT_MAX")
    p_soar_alt_min = bind_param("SOAR_ALT_MIN")
    p_tmem_enable = bind_param(PARAM_PREFIX .. "TMEM_ENABLE")
    p_tmem_life = bind_param(PARAM_PREFIX .. "TMEM_LIFE")
    p_wp_radius = bind_param(PARAM_PREFIX .. "WP_RADIUS")
    p_stuck_eff = bind_param(PARAM_PREFIX .. "STUCK_EFF")
    p_stuck_time = bind_param(PARAM_PREFIX .. "STUCK_TIME")
    p_reeng_dwell = bind_param(PARAM_PREFIX .. "REENG_DWELL")
    p_retry_thr = bind_param(PARAM_PREFIX .. "RETRY_THR")
    p_street_tol = bind_param(PARAM_PREFIX .. "STREET_TOL")
    p_reroute_p = bind_param(PARAM_PREFIX .. "REROUTE_P")
    p_nec_weight = bind_param(PARAM_PREFIX .. "NEC_WEIGHT")
    p_tmem_min_s = bind_param(PARAM_PREFIX .. "TMEM_MIN_S")
    p_focus_thr = bind_param(PARAM_PREFIX .. "FOCUS_THR")
    p_strat_hist = bind_param(PARAM_PREFIX .. "STRAT_HIST")
    p_wp_timeout = bind_param(PARAM_PREFIX .. "WP_TIMEOUT")
    p_reroute_min = bind_param(PARAM_PREFIX .. "REROUTE_MIN")
    p_reroute_max = bind_param(PARAM_PREFIX .. "REROUTE_MAX")
    p_soar_alt_cutoff = bind_param("SOAR_ALT_CUTOFF")
    p_soar_polar_cd0 = bind_param("SOAR_POLAR_CD0")
    p_soar_polar_b = bind_param("SOAR_POLAR_B")
    p_airspeed_cruise = bind_param("AIRSPEED_CRUISE")
    p_dyn_soalt = bind_param(PARAM_PREFIX .. "DYN_SOALT")
    p_gc_margin = bind_param(PARAM_PREFIX .. "GC_MARGIN")
    p_gc_pad = bind_param(PARAM_PREFIX .. "GC_PAD")
end

bind_params()

-- Static constants used throughout the script
local SoarNavConstants = {
    ---------------------------------------------------------------------------
    -- ALGORITHMIC THRESHOLDS & TIMINGS (UPPERCASE)
    ---------------------------------------------------------------------------
    -- Safety & Anti-Stuck
    STUCK_PROGRESS_CHECK_INTERVAL_MS = 20000,
    STUCK_MIN_PROGRESS_M = 10,
    STUCK_DISTANCE_INCREMENT_M = 50,
    MAX_LOCATION_ERRORS = 10,

    -- Thermal Logic
    THERMAL_CONSISTENCY_VARIANCE_THRESHOLD = 0.5,
    MIN_HOTSPOTS_FOR_STREET = 2,
    CRITICAL_TREND_HYSTERESIS_MS = 3000,

    -- Navigation & Control
    WP_TIMEOUT_WIND_MODE = 1,
    WP_TIMEOUT_WIND_REF_MPS = 15,
    WP_TIMEOUT_WIND_MIN_MULT = 0.5,
    WP_TIMEOUT_WIND_MAX_MULT = 2.5,
    ROLL_SMOOTHING_FACTOR = 0.1,
    STATUS_LOG_INTERVAL_MS = 2000,
    HOTSPOT_EXPLORATION_RADIUS_M = 250,
    TACTICAL_REROUTE_MIN_PATH_DIVISOR = 4,
    REROUTE_BEARING_OFFSETS = {0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90},

    -- Pilot Input & Stick Gestures
    PILOT_RESUME_DELAY_MS = 5000,
    PITCH_GESTURE_THRESHOLD = 0.5,
    PITCH_GESTURE_COUNT_TARGET = 4,
    PITCH_GESTURE_TIMEOUT_MS = 2000,
    ROLL_GESTURE_THRESHOLD = 0.5,
    ROLL_GESTURE_COUNT_TARGET = 4,
    ROLL_GESTURE_TIMEOUT_MS = 2000,

    ---------------------------------------------------------------------------
    -- GENERAL CONFIGURATION (lowercase)
    ---------------------------------------------------------------------------
    min_cell_size_m = 50,
    max_total_grid_cells = 200,
    grid_init_cells_per_call = 20,
    polygon_filename_prefix = "snav",
    polygon_filename_suffix = ".poly",
    max_polygon_index = 9,
    max_hotspots = 10,
    strong_thermal_threshold_mps = 3.0,
    rc_opt_soaring_active = 88,
    mode_fbwb = 6,
    mode_cruise = 7,
    mode_thermal = 24,
	
    ---------------------------------------------------------------------------
    -- Motor Failure Detection
    ---------------------------------------------------------------------------
    MOTOR_FAILURE_CHECK_DELAY_MS = 5000,
    MOTOR_FAILURE_CLIMB_RATE_THRESHOLD_MPS = 0.2,
    RTL_MODE_CODE = 11,
    THROTTLE_ACTIVE_THRESHOLD = 10,
    MOTOR_RPM_FAIL_THROTTLE_PERCENT = 15,
    MOTOR_RPM_FAIL_RPM_MIN = 100,
    MOTOR_RPM_FAIL_DEBOUNCE_MS = 1500,
}

-- Global table for managing the script's state, organized by function
local SoarNavGlobals = {
    ---------------------------------------------------------------------------
    -- 1. SCRIPT CORE STATE & NAVIGATION
    ---------------------------------------------------------------------------
    script_state = SCRIPT_STATE.IDLE,
    target_loc = nil,
    g_waypoint_source_info = "N/A",
    distance_to_wp = -1,
    initial_distance_to_wp = -1,
    waypoint_start_time_ms = nil,
    waypoint_search_in_progress = false,
    navigation_start_delay_counter = 0,
    ---------------------------------------------------------------------------
    -- 2. FLIGHT AREA & GRID SYSTEM
    ---------------------------------------------------------------------------
    use_polygon_area = false,
    max_operational_distance = 0,
    dynamic_center_location = nil,
    polygon_points = {},
    polygon_bounds = nil,
    polygon_load_attempted = false,
    grid_initialized = false,
    grid_bounds = nil,
    grid_rows = 0,
    grid_cols = 0,
    grid_cell_size_m = 0,
    grid_cells = {},
    grid_cell_centers = {},
    valid_cell_indices = {},
    unvisited_cell_indices = {},
    unvisited_set = {},
    is_initializing = false,
    grid_init_step = 0,
    grid_init_row = 1,
    grid_init_col = 1,
    grid_populate_index = 1,
    last_cell_index = nil,
    last_cell_check_loc = nil,
    force_grid_after_reset = false,

    ---------------------------------------------------------------------------
    -- 3. THERMAL INTELLIGENCE
    ---------------------------------------------------------------------------
    was_in_thermal_mode = false,
    is_monitoring_thermal = false,
    last_thermal_sample_ms = nil,
    current_thermal_stats = {},
    thermal_hotspots = {},
    current_selected_hotspot = nil,
    last_used_hotspot_timestamp = nil,
    lost_thermal_counter = 0,
    thermal_to_retry = nil,
    reengage_final_target = nil,
    reengage_hold_until_ms = nil,
    reengage_hold_active = false,
    reengage_hold_last_msg_ms = nil,
    is_in_focus_mode = false,
    focus_area_center = nil,
    effective_cluster_radius = 400,
    focus_wp_counter = 0,
    focus_wp_timeout = 3,

    ---------------------------------------------------------------------------
    -- 4. SAFETY & TACTICAL MANEUVERS
    ---------------------------------------------------------------------------
    is_repositioning = false,
    original_target_loc = nil,
    stuck_counter = 0,
    loc_at_last_check = nil,
    last_progress_check_ms = nil,
    distance_at_last_check = -1,
    reroute_check_armed = false,
    reroute_origin_loc = nil,
    reroute_desired_bearing = nil,
    initial_soar_alt_min = nil,
    initial_soar_alt_max = nil,
    initial_soar_alt_cutoff = nil,
    last_armed_state = nil,
    restored_on_disarm = false,
    last_glide_cone_update_ms = nil,
    gcone_param_warning_sent = nil,
    gcone_headwind_flag = nil,
    motor_failure_check_active = false,
    motor_on_start_time_ms = nil,
    altitude_at_check_start_m = nil,
    soar_alt_min_at_check = nil,
    rpm_failure_start_ms = nil,
    rpm_check_counter = 0,

    ---------------------------------------------------------------------------
    -- 5. PILOT INPUT & OVERRIDES
    ---------------------------------------------------------------------------
    last_pilot_input_ms = nil,
    manual_override_active = false,
    pitch_gesture_state = "idle",
    pitch_gesture_count = 0,
    pitch_gesture_start_ms = nil,
    pitch_gesture_triggered_this_override = false,
    roll_gesture_state = "idle",
    roll_gesture_count = 0,
    roll_gesture_start_ms = nil,

    ---------------------------------------------------------------------------
    -- 6. SYSTEM STATE & ENVIRONMENT
    ---------------------------------------------------------------------------
    energy_state = "NORMAL",
    filtered_alt_factor = 0,
    energy_state_transition_ms = 0,
    negative_trend_start_ms = nil,
    cached_wind = nil,
    wind_cache_time = nil,
    last_alt = 0,
    last_alt_timestamp = nil,
    last_update_time_ms = nil,
    location_error_count = 0,
    last_status_log_ms = nil,
    last_snav_max_dist_value = -1,
    last_snav_enable_value = -1,

    ---------------------------------------------------------------------------
    -- 7. RC CONTROL & HARDWARE INTERFACE
    ---------------------------------------------------------------------------
    rc_roll_channel = rc:get_channel(p_rcmap_roll:get() or 1),
    rc_limits_read = false,
    last_heading_error = 0,
    last_commanded_roll_deg = 0,
    rc_roll_min = 1000, rc_roll_max = 2000, rc_roll_trim = 1500
}

SoarNavGlobals.rpm_check_done = false

-- #############################################################################
-- ## UTILITY FUNCTIONS (DEFINED EARLY TO PREVENT 'UNDEFINED GLOBAL' ERRORS)
-- #############################################################################

-- Safely retrieves a parameter value with optional default and min/max clamping.
local function get_safe_param(p, default, min, max)
    local value = p:get()
    if value == nil then
        return default
    end
    if min and max then
        return math.min(max, math.max(min, value))
    else
        return value
    end
end

-- Safely calculates the difference between two millis() timestamps.
local function safe_time_diff(newer, older)
    if not newer or not older then
        return 0
    end
    local diff = newer:toint() - older:toint()
    return (diff >= 0) and diff or 0
end

-- Centralized function for sending formatted GCS messages.
local function log_gcs(sev, lvl, msg)
    local cur = (p_log_lvl and p_log_lvl:get()) or 1
    if cur >= lvl then
        gcs:send_text(sev, "SoarNav: " .. msg)
    end
end

-- Converts a wind vector into the compass bearing (0-359 deg) from which it originates.
local function wind_vector_to_bearing_deg(w)
    local ang = (math.deg(w:xy():angle()) + 360) % 360
    return (450 - ang) % 360
end

-- Checks if a Location object is inside the defined polygon using a ray-casting algorithm.
local function is_point_in_polygon(loc)
    if not loc or not SoarNavGlobals.polygon_bounds then return false end

    local bounds = SoarNavGlobals.polygon_bounds
    if not bounds then return false end
    local lat, lon = loc:lat(), loc:lng()

    if lat < bounds.min_lat or lat > bounds.max_lat or lon < bounds.min_lon or lon > bounds.max_lon then
        return false
    end

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

-- Returns the active center for navigation (either Home or a dynamic, pilot-set location).
local function get_active_center_location()
    if SoarNavGlobals.dynamic_center_location then
        return SoarNavGlobals.dynamic_center_location
    else
        return ahrs:get_home()
    end
end

-- Predicts the drifted location of a thermal based on wind and age.
local function predict_thermal_drift(loc, wind, age_sec)
    if wind and #SoarNavGlobals.thermal_hotspots > 0 then
        local wind_bearing_info = wind_vector_to_bearing_deg(wind)
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Drift | Wind: %.1fm/s @ %.0f", wind:length(), wind_bearing_info))
    end

    if not wind or wind:length() < 1.0 then
        return loc:copy()
    end
    local drift_dist = wind:length() * age_sec * 0.7
    local drifted_loc = loc:copy()
    local wind_bearing_deg = wind_vector_to_bearing_deg(wind)
    drifted_loc:offset_bearing((wind_bearing_deg + 180) % 360, drift_dist)
    return drifted_loc
end

-- Checks if a given location is within the defined flight area (polygon or radius).
local function in_flight_area(loc)
    if not loc then
        return false
    end
    if SoarNavGlobals.use_polygon_area then
        return is_point_in_polygon(loc)
    else
        local center = get_active_center_location()
        if not center then
            return false
        end
        return center:get_distance(loc) <= p_max_dist:get()
    end
end

-- Safely decrements a number, ensuring it does not go below zero.
local function safe_decrement(n)
    return math.max(0, n - 1)
end

-- Returns a cached wind vector, refreshing it periodically to save resources.
local function get_wind_vector()
    local now = millis()
    if not SoarNavGlobals.wind_cache_time then
        SoarNavGlobals.wind_cache_time = now
    end
    if not SoarNavGlobals.cached_wind or safe_time_diff(now, SoarNavGlobals.wind_cache_time) > 5000 then
        SoarNavGlobals.cached_wind = ahrs:wind_estimate()
        SoarNavGlobals.wind_cache_time = now
    end
    local wind = SoarNavGlobals.cached_wind
    if not wind or wind:length() < 0.1 then
        local zero_vector = Vector3f()
        zero_vector:x(0)
        zero_vector:y(0)
        zero_vector:z(0)
        return zero_vector
    end
    return wind
end

-- Validates key SNAV_* parameters to ensure they are within safe and logical ranges.
local function validate_params()
    local ok = true

    local function clamp_warn(name, val, lo, hi)
        if val == nil then return end
        if (lo and val < lo) or (hi and val > hi) then
            log_gcs(MAV_SEVERITY.WARNING, 1, string.format("%s out of range [%s..%s]: %.3f", name, tostring(lo), tostring(hi), val))
        end
    end

    local max_dist = p_max_dist:get()
    if max_dist < 0 then
        log_gcs(MAV_SEVERITY.ERROR, 0, "SNAV_MAX_DIST cannot be negative.")
        ok = false
    end
    if p_reroute_min and p_reroute_max then
        local rmin = p_reroute_min:get()
        local rmax = p_reroute_max:get()
        if rmin > rmax then
            log_gcs(MAV_SEVERITY.ERROR, 0, "SNAV_REROUTE_MIN must be <= SNAV_REROUTE_MAX.")
            ok = false
        end
    end

    clamp_warn("SNAV_ROLL_LIMIT", p_roll_limit:get(), 10, 60)
    clamp_warn("SNAV_WP_RADIUS",  p_wp_radius:get(), 10, 300)
    clamp_warn("SNAV_NAV_P",      p_nav_p:get(), 0.05, 2.0)
    clamp_warn("SNAV_NAV_D",      p_nav_d:get(), 0.0, 1.0)
    clamp_warn("SNAV_STUCK_EFF",  p_stuck_eff:get(), 0.1, 1.0)
    clamp_warn("SNAV_STUCK_TIME", p_stuck_time:get(), 5, 120)
    clamp_warn("SNAV_REENG_DWELL",p_reeng_dwell:get(), 0, 120)
    clamp_warn("SNAV_RETRY_THR",  p_retry_thr:get(), 0, 100)
    clamp_warn("SNAV_STREET_TOL", p_street_tol:get(), 5, 90)
    clamp_warn("SNAV_REROUTE_P",  p_reroute_p:get(), 0, 100)
    clamp_warn("SNAV_NEC_WEIGHT", p_nec_weight:get(), 0, 100)
    clamp_warn("SNAV_TMEM_MIN_S", p_tmem_min_s:get(), 0.0, 5.0)
    clamp_warn("SNAV_FOCUS_THR",  p_focus_thr:get(), 0.0, 5.0)
    clamp_warn("SNAV_STRAT_HIST", p_strat_hist:get(), 60, 3600)
    clamp_warn("SNAV_WP_TIMEOUT", p_wp_timeout:get(), 30, 900)

    return ok
end

-- Determines the grid cell index for a given geographical location.
---@param loc Location_ud
---@return integer|nil
local function get_cell_index_from_location(loc)
    local G = SoarNavGlobals
    if not G.grid_initialized then
        return nil
    end
    local b = G.grid_bounds
    if not b then
        return nil
    end
    local lat_span = b.lat_span
    local lon_span = b.lon_span
    if not lat_span or not lon_span or lat_span == 0 or lon_span == 0 then
        return nil
    end
    local lat, lon = loc:lat(), loc:lng()
    if lat < b.min_lat or lat > b.max_lat or lon < b.min_lon or lon > b.max_lon then
        return nil
    end
    local rows, cols = G.grid_rows, G.grid_cols
    local lat_frac = (lat - b.min_lat) / lat_span
    local lon_frac = (lon - b.min_lon) / lon_span
    local row = math.floor(lat_frac * rows) + 1
    local col = math.floor(lon_frac * cols) + 1
    row = math.max(1, math.min(rows, row))
    col = math.max(1, math.min(cols, col))
    return (row - 1) * cols + col
end

-- Dynamically updates SOAR_ALT_MIN/MAX/CUTOFF based on the calculated glide cone to home.
-- Hardened version with pilot-override freeze, reset-hold, max-alt guard, non-reducing reset,
-- bucketing, widened hysteresis, and safety checks.
local function update_dynamic_soar_alt_min()
    if SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE
       or SoarNavGlobals.manual_override_active then
        return
    end
    local override_active = (SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE)
                            or (SoarNavGlobals.manual_override_active == true)
    if override_active then
        SoarNavGlobals.gc_reset_hold_start_ms = nil
        if not SoarNavGlobals.override_reset_done then
            local init_min    = SoarNavGlobals.initial_soar_alt_min
            local init_cutoff = SoarNavGlobals.initial_soar_alt_cutoff
            local init_max    = SoarNavGlobals.initial_soar_alt_max
            if init_min and init_cutoff and init_max and p_soar_alt_min and p_soar_alt_cutoff and p_soar_alt_max then
                local cur_min = p_soar_alt_min:get()
                local cur_cut = p_soar_alt_cutoff:get()
                local cur_max = p_soar_alt_max:get()
                local changed = false
                if cur_min ~= init_min then p_soar_alt_min:set(init_min); changed = true end
                if cur_cut ~= init_cutoff then p_soar_alt_cutoff:set(init_cutoff); changed = true end
                if cur_max ~= init_max then p_soar_alt_max:set(init_max); changed = true end
                if changed then
                    log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("GC Rst(Override)->MIN:%.0f C:%.0f M:%.0f", init_min, init_cutoff, init_max))
                end
            end
            SoarNavGlobals.override_reset_done = true
        end
        return
    else
        SoarNavGlobals.override_reset_done = nil
    end

    if not p_dyn_soalt or not p_soar_polar_b or not p_soar_polar_cd0 or not p_airspeed_cruise or
       not p_soar_alt_min or not p_soar_alt_max or not p_soar_alt_cutoff or not p_gc_margin or not p_gc_pad then
        return
    end

    local initial_min     = SoarNavGlobals.initial_soar_alt_min
    local initial_cutoff  = SoarNavGlobals.initial_soar_alt_cutoff
    local initial_max     = SoarNavGlobals.initial_soar_alt_max
    local current_min     = p_soar_alt_min:get()
    if not initial_min or not initial_cutoff or not initial_max or not current_min then
        return
    end

    local dyn_soalt_mode = math.floor((p_dyn_soalt:get() or 0) + 0.5)
    if dyn_soalt_mode == 0 then
        return
    end

    local current_loc     = ahrs:get_location()
    local target_home_loc = ahrs:get_home()
    if not current_loc or not target_home_loc then
        return
    end

    local ned_pos = ahrs:get_relative_position_NED_home()
    if not ned_pos then
        return
    end
    local current_alt = -ned_pos:z()
    local now = millis()

    local GCONE_RESET_SURPLUS_M = 50
    local RESET_HOLD_MS         = 20000

    local polar_b             = p_soar_polar_b:get()
    local polar_cd0           = p_soar_polar_cd0:get()
    local best_glide_airspeed = p_airspeed_cruise:get()
    if  not polar_b or polar_b <= 0 or polar_b ~= polar_b
     or not polar_cd0 or polar_cd0 <= 0 or polar_cd0 ~= polar_cd0
     or not best_glide_airspeed or best_glide_airspeed <= 0 or best_glide_airspeed ~= best_glide_airspeed then
        if not SoarNavGlobals.gcone_param_warning_sent then
            log_gcs(MAV_SEVERITY.WARNING, 1, "GCone disabled: Set SOAR_POLAR_CD0/B and AIRSPEED params.")
            SoarNavGlobals.gcone_param_warning_sent = true
        end
        return
    end

    local efficiency_max = 1 / math.sqrt(4 * polar_cd0 * polar_b)
    if not efficiency_max or efficiency_max <= 0 or efficiency_max ~= efficiency_max or efficiency_max == 1/0 then
        return
    end

    local vec_to_home = current_loc:get_distance_NE(target_home_loc)
    if not vec_to_home then
        return
    end
    local wind_vec  = get_wind_vector()
    local wind_2d   = wind_vec and wind_vec:xy() or nil
    local wind_comp = 0
    if wind_2d and vec_to_home:length() > 0 then
        wind_comp = (vec_to_home:x() * wind_2d:x() + vec_to_home:y() * wind_2d:y()) / vec_to_home:length()
        if not wind_comp or wind_comp ~= wind_comp or wind_comp == 1/0 or wind_comp == -1/0 then
            wind_comp = 0
        end
    end

    local gs_to_home = best_glide_airspeed + wind_comp
    if not gs_to_home or gs_to_home <= 0 then
        gs_to_home = 0.1
    end

    local sink_best = best_glide_airspeed / efficiency_max
    if not sink_best or sink_best <= 0 or sink_best ~= sink_best then
        return
    end
    local eff_ld = gs_to_home / sink_best
    if not eff_ld or eff_ld <= 0 or eff_ld ~= eff_ld then
        return
    end

    local dist_home = current_loc:get_distance(target_home_loc) or 0
    if dist_home < 0 then dist_home = 0 end
    local required_alt_glide = dist_home / eff_ld
    if not required_alt_glide or required_alt_glide < 0 or required_alt_glide ~= required_alt_glide then
        required_alt_glide = 0
    end

    local pad                = p_gc_pad:get() or 0
    local margin             = p_gc_margin:get() or 0
    local calculated_min_alt = required_alt_glide + pad + margin

    local target_min
    local target_cutoff = initial_cutoff
    local target_max    = initial_max

    if current_alt > (calculated_min_alt + GCONE_RESET_SURPLUS_M) then
        if not SoarNavGlobals.gc_reset_hold_start_ms then
            SoarNavGlobals.gc_reset_hold_start_ms = now
            return
        elseif safe_time_diff(now, SoarNavGlobals.gc_reset_hold_start_ms) >= RESET_HOLD_MS then
            if current_alt > (initial_max + 10) then
                SoarNavGlobals.gc_reset_hold_start_ms = nil
                return
            end
            target_min    = initial_min
            target_cutoff = initial_cutoff
            target_max    = initial_max
            SoarNavGlobals.gc_reset_hold_start_ms = nil
        else
            return
        end
    else
        SoarNavGlobals.gc_reset_hold_start_ms = nil
        target_min = math.floor(math.max(initial_min, calculated_min_alt))
        target_min = math.floor((target_min + 5) / 10) * 10
        if dyn_soalt_mode == 1 then
            local link_threshold = math.floor(initial_cutoff * (2.0 / 3.0))
            if target_min > link_threshold then
                local delta = target_min - initial_min
                target_cutoff = initial_cutoff + delta
                target_max    = initial_max + delta
            end
        elseif dyn_soalt_mode == 2 then
            local cap_altitude = math.floor(initial_cutoff * (2.0 / 3.0))
            if target_min > cap_altitude then
                target_min = cap_altitude
            end
        end
    end

    local ned_velocity = ahrs:get_velocity_NED()
    if ned_velocity then
        local climb_rate = -ned_velocity:z()
        if target_min < current_min and climb_rate and climb_rate > 1.5 then
            return
        end
    end

    local hysteresis_up   = 20
    local hysteresis_down = 80
    local delta = target_min - current_min
	
    local GAP_CUT_MIN = math.max(math.floor(initial_cutoff / 3 + 0.5), 30)
    local GAP_MAX_MIN = math.max((initial_max - initial_cutoff), 50)

    if target_cutoff < (target_min + GAP_CUT_MIN) then
        target_cutoff = target_min + GAP_CUT_MIN
    end
    if target_max < (target_cutoff + GAP_MAX_MIN) then
        target_max = target_cutoff + GAP_MAX_MIN
    end

    local needs_update = (delta >= hysteresis_up) or (delta <= -hysteresis_down)
    if needs_update then
        p_soar_alt_min:set(target_min)
        p_soar_alt_cutoff:set(target_cutoff)
        p_soar_alt_max:set(target_max)
        log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Glide Cone: Safe return alt now %.0fm", target_min))
        if target_min == initial_min and current_min > initial_min then
            log_gcs(MAV_SEVERITY.NOTICE, 2, string.format("GC Rst(Srpls)->MIN:%.0f C:%.0f M:%.0f", target_min, target_cutoff, target_max))
        else
            local reason_str = string.format("GC Set(M%d)", dyn_soalt_mode)
            log_gcs(MAV_SEVERITY.NOTICE, 2, string.format("%s->MIN:%.0f C:%.0f M:%.0f", reason_str, target_min, target_cutoff, target_max))
        end
    end
end

-- Checks if a waypoint source is a thermal or thermal street.
local function is_thermal_target(source_info)
    local src = tostring(source_info)
    return string.find(src, "Thrm") or (src == SoarNavWaypointSources.THERMAL_STREET)
end

-- #############################################################################
-- ## CORE SCRIPT LOGIC
-- #############################################################################

-- Assesses the aircraft's energy state (NORMAL, LOW, CRITICAL) based on altitude.
local function assess_energy_state()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    if not dist_from_home_3d then return "UNKNOWN" end

    local current_alt = -dist_from_home_3d:z()
    local now = millis()

    local trend = 0
    if SoarNavGlobals.last_alt_timestamp then
        local dt = safe_time_diff(now, SoarNavGlobals.last_alt_timestamp) / 1000.0
        if dt > 0.5 then
            trend = (current_alt - SoarNavGlobals.last_alt) / dt
        end
    end
    SoarNavGlobals.last_alt = current_alt
    SoarNavGlobals.last_alt_timestamp = now

    local min_alt = p_soar_alt_min:get() or 100
    local max_alt = p_soar_alt_max:get() or 400
    local alt_range = max_alt - min_alt
    local raw_alt_factor = 0
    if alt_range > 0 then
        raw_alt_factor = math.min(1, math.max(0, (current_alt - min_alt) / alt_range))
    end

    local filter_alpha = 0.05
    if SoarNavGlobals.energy_state_transition_ms == 0 then
        SoarNavGlobals.filtered_alt_factor = raw_alt_factor
        SoarNavGlobals.energy_state_transition_ms = now
    else
        local dt_filter = safe_time_diff(now, SoarNavGlobals.energy_state_transition_ms) / 1000.0
        SoarNavGlobals.filtered_alt_factor = SoarNavGlobals.filtered_alt_factor + (raw_alt_factor - SoarNavGlobals.filtered_alt_factor) * (1 - math.exp(-dt_filter * filter_alpha))
        SoarNavGlobals.energy_state_transition_ms = now
    end

    local new_state = "NORMAL"
    if SoarNavGlobals.filtered_alt_factor < 0.25 then
        new_state = "LOW"
    end
    if SoarNavGlobals.filtered_alt_factor < 0.1 then
        new_state = "CRITICAL"
    end

    if trend < -0.3 then
        if not SoarNavGlobals.negative_trend_start_ms then
            SoarNavGlobals.negative_trend_start_ms = now
        end
        if safe_time_diff(now, SoarNavGlobals.negative_trend_start_ms) >= SoarNavConstants.CRITICAL_TREND_HYSTERESIS_MS and new_state ~= "CRITICAL" then
            new_state = "CRITICAL"
            log_gcs(MAV_SEVERITY.WARNING, 2, string.format("Trend override: sinking at %.1fm/s", trend))
        end
    else
        SoarNavGlobals.negative_trend_start_ms = nil
    end

    if SoarNavGlobals.energy_state ~= new_state then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Energy state changed: %s -> %s", SoarNavGlobals.energy_state, new_state))
        SoarNavGlobals.energy_state = new_state
    end

    return SoarNavGlobals.energy_state
end

-- Formats and logs the details of a recorded thermal event.
local function log_thermal_event(hotspot)
    log_gcs(MAV_SEVERITY.INFO, 2, string.format("Thermal Saved: %.1f m/s avg", hotspot.avg_strength))
    log_gcs(MAV_SEVERITY.INFO, 2, string.format(" > Max strength: %.1f m/s", hotspot.max_strength))
    log_gcs(MAV_SEVERITY.INFO, 2, string.format(" > Quality: %s", hotspot.consistency))

    local wind = get_wind_vector()
    if wind then
        local wind_dir = math.floor(wind_vector_to_bearing_deg(wind))
        local wind_str = string.format("%.1f m/s @ %d deg", wind:length(), wind_dir)
        log_gcs(MAV_SEVERITY.INFO, 2, string.format(" > Wind: %s", wind_str))
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

local function restore_boot_alts(reason)
    if SoarNavGlobals and SoarNavGlobals.initial_soar_alt_min and SoarNavGlobals.initial_soar_alt_max and SoarNavGlobals.initial_soar_alt_cutoff then
        if p_soar_alt_min then p_soar_alt_min:set(SoarNavGlobals.initial_soar_alt_min) end
        if p_soar_alt_max then p_soar_alt_max:set(SoarNavGlobals.initial_soar_alt_max) end
        if p_soar_alt_cutoff then p_soar_alt_cutoff:set(SoarNavGlobals.initial_soar_alt_cutoff) end
        SoarNavGlobals.last_glide_cone_update_ms = nil
        log_gcs(MAV_SEVERITY.NOTICE, 2, string.format("Reset (%s): M%.0f,C%.0f,X%.0f", reason or "Unk", SoarNavGlobals.initial_soar_alt_min, SoarNavGlobals.initial_soar_alt_cutoff, SoarNavGlobals.initial_soar_alt_max))
    end
end

-- Manages script state transitions and associated cleanup actions.
local function set_script_state(new_state, reason)
    if SoarNavGlobals.script_state ~= new_state then
        local prev_state = SoarNavGlobals.script_state
        if new_state == SCRIPT_STATE.PILOT_OVERRIDE then
            restore_boot_alts('Pilot Override')
        end

        if new_state == SCRIPT_STATE.IDLE and prev_state ~= SCRIPT_STATE.THERMAL_PAUSE then
            restore_boot_alts('Entering IDLE')
        end

        if new_state == SCRIPT_STATE.IDLE and SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
            SoarNavGlobals.pitch_gesture_state = "idle"
            SoarNavGlobals.pitch_gesture_count = 0
            SoarNavGlobals.pitch_gesture_start_ms = nil
            SoarNavGlobals.pitch_gesture_triggered_this_override = false
            SoarNavGlobals.roll_gesture_state = "idle"
            SoarNavGlobals.roll_gesture_count = 0
            SoarNavGlobals.roll_gesture_start_ms = nil
            SoarNavGlobals.manual_override_active = false
        end
        SoarNavGlobals.script_state = new_state
        local _r = tostring(reason or "")
        if _r == "Exited thermal, resuming." or _r == "Navigation conditions met, starting." then
            log_gcs(MAV_SEVERITY.NOTICE, 2, reason)
        else
            log_gcs(MAV_SEVERITY.NOTICE, 1, reason)
        end
        if new_state ~= SCRIPT_STATE.NAVIGATING then
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

    local gesture_timeout = false
    if SoarNavGlobals.pitch_gesture_start_ms then
        if safe_time_diff(millis(), SoarNavGlobals.pitch_gesture_start_ms) > SoarNavConstants.PITCH_GESTURE_TIMEOUT_MS then
            gesture_timeout = true
        end
    end

    if SoarNavGlobals.pitch_gesture_state ~= "idle" and gesture_timeout then
        SoarNavGlobals.pitch_gesture_state = "idle"
        SoarNavGlobals.pitch_gesture_count = 0
    end

    local pitch_chan_obj = rc:get_channel(p_rcmap_pitch:get() or 2)
    if not pitch_chan_obj then return end
    local normalized_pitch = pitch_chan_obj:norm_input()

    if SoarNavGlobals.pitch_gesture_state == "idle" then
        if math.abs(normalized_pitch) > SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_start_ms = millis()
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
    local gesture_timeout = false
    if SoarNavGlobals.roll_gesture_start_ms then
        if safe_time_diff(millis(), SoarNavGlobals.roll_gesture_start_ms) > SoarNavConstants.ROLL_GESTURE_TIMEOUT_MS then
            gesture_timeout = true
        end
    end

    if SoarNavGlobals.roll_gesture_state ~= "idle" and gesture_timeout then
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
    end

    local roll_chan = p_rcmap_roll:get()
    local chan1 = rc:get_channel(roll_chan)
    if not chan1 then return end
    local normalized_roll = chan1:norm_input()

    if SoarNavGlobals.roll_gesture_state == "idle" then
        if math.abs(normalized_roll) > SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_start_ms = millis()
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
        SoarNavGlobals.roll_gesture_start_ms = nil
    end
end

-- Kicks off the grid generation process.
local function initialize_grid()
    SoarNavGlobals.is_initializing = true
    SoarNavGlobals.grid_initialized = false
    SoarNavGlobals.grid_init_step = 1
    SoarNavGlobals.grid_cells = {}
    SoarNavGlobals.grid_cell_centers = {}
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
                log_gcs(MAV_SEVERITY.INFO, 1, string.format("Polygon max distance calculated: %.0fm.", SoarNavGlobals.max_operational_distance))
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
        end

        local sw_corner_loc = active_center:copy()
        sw_corner_loc:lat(SoarNavGlobals.grid_bounds.min_lat)
        sw_corner_loc:lng(SoarNavGlobals.grid_bounds.min_lon)
        local ne_corner_loc = active_center:copy()
        ne_corner_loc:lat(SoarNavGlobals.grid_bounds.max_lat)
        ne_corner_loc:lng(SoarNavGlobals.grid_bounds.max_lon)
        local xy = sw_corner_loc:get_distance_NE(ne_corner_loc)
        local width_m = xy:x()
        local height_m = xy:y()

        if height_m < 1 or width_m < 1 then
            log_gcs(MAV_SEVERITY.ERROR, 0, "Grid area too small. Init aborted.")
            SoarNavGlobals.is_initializing = false
            SoarNavGlobals.grid_init_step = 0
            return
        end

        local area_size = math.max(width_m, height_m)
        SoarNavGlobals.effective_cluster_radius = math.min(600, math.max(200, area_size * 0.15))
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("Eff. Cluster Radius: %.0fm", SoarNavGlobals.effective_cluster_radius))

        local area_m2 = height_m * width_m
        local approx_cell_size_m = math.sqrt(area_m2 / SoarNavConstants.max_total_grid_cells)
        local final_cell_size_m = math.max(SoarNavConstants.min_cell_size_m, approx_cell_size_m)
        SoarNavGlobals.grid_rows = math.max(1, math.floor(height_m / final_cell_size_m))
        SoarNavGlobals.grid_cols = math.max(1, math.floor(width_m / final_cell_size_m))
        SoarNavGlobals.grid_cell_size_m = final_cell_size_m
        SoarNavGlobals.grid_init_step = 2

        local b = SoarNavGlobals.grid_bounds
        if b and b.max_lat and b.min_lat and b.max_lon and b.min_lon then
            b.lat_span = b.max_lat - b.min_lat
            b.lon_span = b.max_lon - b.min_lon
        else
            b.lat_span = 0
            b.lon_span = 0
        end

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
        local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat) / SoarNavGlobals.grid_rows
        local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon) / SoarNavGlobals.grid_cols
        while SoarNavGlobals.grid_init_row <= SoarNavGlobals.grid_rows and cells_processed < SoarNavConstants.grid_init_cells_per_call do
            local r = SoarNavGlobals.grid_init_row
            local c = SoarNavGlobals.grid_init_col
            local cell_center_lat = SoarNavGlobals.grid_bounds.min_lat + (r - 0.5) * cell_height_lat
            local cell_center_lon = SoarNavGlobals.grid_bounds.min_lon + (c - 0.5) * cell_width_lon
            local center = ahrs:get_home():copy()
            center:lat(cell_center_lat)
            center:lng(cell_center_lon)
            if in_flight_area(center) then
                local cell_index = (r - 1) * SoarNavGlobals.grid_cols + c
                table.insert(SoarNavGlobals.valid_cell_indices, cell_index)
                SoarNavGlobals.grid_cell_centers[cell_index] = center
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
            SoarNavGlobals.unvisited_set = {}
            for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                table.insert(SoarNavGlobals.unvisited_cell_indices, v)
                SoarNavGlobals.unvisited_set[v] = true
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
    if not current_cell_index or current_cell_index <= 0 or current_cell_index > #SoarNavGlobals.grid_cells then
        return
    end
    if current_cell_index ~= SoarNavGlobals.last_cell_index then
        if SoarNavGlobals.unvisited_set[current_cell_index] then
            SoarNavGlobals.unvisited_set[current_cell_index] = nil
            for i = #SoarNavGlobals.unvisited_cell_indices, 1, -1 do
                if SoarNavGlobals.unvisited_cell_indices[i] == current_cell_index then
                    table.remove(SoarNavGlobals.unvisited_cell_indices, i)
                    break
                end
            end
        end
        if SoarNavGlobals.grid_cells[current_cell_index] then
            SoarNavGlobals.grid_cells[current_cell_index].visit_count = SoarNavGlobals.grid_cells[current_cell_index].visit_count + 1
        end
        SoarNavGlobals.last_cell_index = current_cell_index
    end
end

-- Removes expired thermal hotspots from memory and returns a list of currently valid ones.
local function clean_and_get_hotspots()
    local now_ms = millis()
    local lifetime_ms = p_tmem_life:get() * 1000
    for i = #SoarNavGlobals.thermal_hotspots, 1, -1 do
        local hotspot = SoarNavGlobals.thermal_hotspots[i]
        if hotspot and hotspot.timestamp then
            if safe_time_diff(now_ms, hotspot.timestamp) >= lifetime_ms then
                table.remove(SoarNavGlobals.thermal_hotspots, i)
            end
        end
    end
    return SoarNavGlobals.thermal_hotspots
end

-- Calculates a weighted density score for each hotspot based on nearby thermals.
local function calculate_hotspot_density(hotspots)
    if not hotspots or #hotspots < 2 then
        for _, h in ipairs(hotspots or {}) do h.cluster_density = 0 end
        return
    end

    local now = millis()
    local cluster_radius = SoarNavGlobals.effective_cluster_radius
    local tmem_life_ms = (p_tmem_life:get() or 1200) * 1000
    local scaled_strengths = {}

    for i, h in ipairs(hotspots) do
        local age_factor = 1.0 - math.min(1.0, safe_time_diff(now, h.timestamp) / tmem_life_ms)
        scaled_strengths[i] = (h.avg_strength or 0) * age_factor
    end

    for i, h1 in ipairs(hotspots) do
        local density_score = 0
        for j, h2 in ipairs(hotspots) do
            if i ~= j then
                local distance = h1.loc:get_distance(h2.loc)
                if distance < cluster_radius then
                    local distance_factor = 1.0 - (distance / cluster_radius)
                    density_score = density_score + (scaled_strengths[j] * distance_factor)
                end
            end
        end
        h1.cluster_density = density_score + scaled_strengths[i]
    end
end

-- Records a new thermal hotspot in memory after exiting THERMAL mode.
local function stop_and_record_thermal()
    SoarNavGlobals.is_monitoring_thermal = false
    if not SoarNavGlobals.current_thermal_stats or not SoarNavGlobals.current_thermal_stats.core_location or SoarNavGlobals.current_thermal_stats.avg_strength == 0 then
        log_gcs(MAV_SEVERITY.WARNING, 1, "Thermal exit: no data. (Lost Thermal)")
        SoarNavGlobals.lost_thermal_counter = SoarNavGlobals.lost_thermal_counter + 1
        SoarNavGlobals.current_thermal_stats = {}
        return
    end

    local max_strength = SoarNavGlobals.current_thermal_stats.max_strength
    if (max_strength or 0) < p_tmem_min_s:get() then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Thermal ignored: too weak (max %.1f m/s)", max_strength))
        SoarNavGlobals.current_thermal_stats = {}
        return
    end

    SoarNavGlobals.lost_thermal_counter = 0

    if SoarNavGlobals.is_in_focus_mode then
        log_gcs(MAV_SEVERITY.NOTICE, 1, "Thermal found, exiting focus.")
        SoarNavGlobals.is_in_focus_mode = false
        SoarNavGlobals.focus_area_center = nil
        SoarNavGlobals.focus_wp_counter = 0
    end

    local avg_strength = SoarNavGlobals.current_thermal_stats.avg_strength
    local hotspot_loc = SoarNavGlobals.current_thermal_stats.core_location:copy()
    local variance = calculate_thermal_variance(SoarNavGlobals.current_thermal_stats.samples)
    local consistency = variance < SoarNavConstants.THERMAL_CONSISTENCY_VARIANCE_THRESHOLD and "consistent" or "variable"

    if not in_flight_area(hotspot_loc) then
        log_gcs(MAV_SEVERITY.INFO, 1, "Thermal ignored: out of area.")
        SoarNavGlobals.current_thermal_stats = {}
        return
    end

    local timestamp_saved = millis()
    local duration = 0
    if SoarNavGlobals.current_thermal_stats.start_time then
        duration = safe_time_diff(timestamp_saved, SoarNavGlobals.current_thermal_stats.start_time) / 1000
    end

    local new_hotspot = {
        loc = hotspot_loc,
        timestamp = timestamp_saved,
        entry_time = SoarNavGlobals.current_thermal_stats.start_time,
        avg_strength = avg_strength,
        max_strength = max_strength,
        consistency = consistency,
        duration = duration,
        wind_vec = SoarNavGlobals.current_thermal_stats.wind_at_entry,
        failed_attempts = 0,
        cluster_density = 0,
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
            log_gcs(MAV_SEVERITY.INFO, 2, string.format("Thermal ignored: duplicate in cell %d.", new_cell))
            SoarNavGlobals.current_thermal_stats = {}
            return
        end
    end
    table.insert(SoarNavGlobals.thermal_hotspots, new_hotspot)
    SoarNavGlobals.last_used_hotspot_timestamp = timestamp_saved
    if #SoarNavGlobals.thermal_hotspots > SoarNavConstants.max_hotspots then
        table.sort(SoarNavGlobals.thermal_hotspots, function(a, b)
            return (a.avg_strength or 0) < (b.avg_strength or 0)
        end)
        table.remove(SoarNavGlobals.thermal_hotspots, 1)
        log_gcs(MAV_SEVERITY.INFO, 2, "Weakest thermal removed.")
    end
    log_thermal_event(new_hotspot)

    calculate_hotspot_density(SoarNavGlobals.thermal_hotspots)

    for _, hotspot in ipairs(SoarNavGlobals.thermal_hotspots) do
        if hotspot.timestamp:toint() == new_hotspot.timestamp:toint() then
            if hotspot.cluster_density and hotspot.cluster_density >= p_focus_thr:get() and not SoarNavGlobals.is_in_focus_mode then
                log_gcs(MAV_SEVERITY.NOTICE, 1, "Focus Mode ON (Density)")
                SoarNavGlobals.is_in_focus_mode = true
                SoarNavGlobals.focus_area_center = new_hotspot.loc:copy()
                SoarNavGlobals.focus_wp_counter = 0
                local total_strength = 0
                local count = 0
                for _, h_cluster in ipairs(SoarNavGlobals.thermal_hotspots) do
                    if h_cluster.loc:get_distance(new_hotspot.loc) < SoarNavGlobals.effective_cluster_radius then
                        total_strength = total_strength + (h_cluster.avg_strength or 0)
                        count = count + 1
                    end
                end
                local avg_cluster_strength = count > 0 and total_strength / count or 0
                local dynamic_timeout = math.floor(3 + (avg_cluster_strength * 2))
                SoarNavGlobals.focus_wp_timeout = math.min(10, math.max(3, dynamic_timeout))
                log_gcs(MAV_SEVERITY.INFO, 2, string.format("Focus Timeout set to %d WPs.", SoarNavGlobals.focus_wp_timeout))
            end
            break
        end
    end

    local uncertainty_score = 0
    if (new_hotspot.max_strength or 0) < SoarNavConstants.strong_thermal_threshold_mps then
        uncertainty_score = (SoarNavConstants.strong_thermal_threshold_mps - (new_hotspot.max_strength or 0)) * 25
        uncertainty_score = math.max(0, math.min(50, uncertainty_score))
    end

    local necessity_score = (1.0 - SoarNavGlobals.filtered_alt_factor) * p_nec_weight:get()

    local total_score = uncertainty_score + necessity_score

    log_gcs(MAV_SEVERITY.INFO, 2, string.format("Retry Score: %.0f (Unc:%.0f Nec:%.0f)", total_score, uncertainty_score, necessity_score))

    if total_score >= p_retry_thr:get() then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Retry weak/uncertain (Score: %.0f)", total_score))
        SoarNavGlobals.thermal_to_retry = new_hotspot
    end

    SoarNavGlobals.current_thermal_stats = {}
end

-- Periodically samples the climb rate while the aircraft is in THERMAL mode.
local function sample_thermal_strength()
    local ned_velocity = ahrs:get_velocity_NED()
    if not ned_velocity then return end
    local climb_rate = -ned_velocity:z()
    local sample_weight = math.min(1.0, math.abs(climb_rate) / 3.0)
    if SoarNavGlobals.current_thermal_stats.avg_strength then
        SoarNavGlobals.current_thermal_stats.avg_strength = (SoarNavGlobals.current_thermal_stats.avg_strength * (1 - sample_weight)) + (climb_rate * sample_weight)
    else
        SoarNavGlobals.current_thermal_stats.avg_strength = climb_rate
    end
    if climb_rate > SoarNavGlobals.current_thermal_stats.max_strength then
        SoarNavGlobals.current_thermal_stats.max_strength = climb_rate
        local current_loc = ahrs:get_location()
        if current_loc then
            SoarNavGlobals.current_thermal_stats.core_location = current_loc:copy()
        end
    end
    table.insert(SoarNavGlobals.current_thermal_stats.samples, climb_rate)
    if #SoarNavGlobals.current_thermal_stats.samples > 10 then
        table.remove(SoarNavGlobals.current_thermal_stats.samples, 1)
    end
    SoarNavGlobals.last_thermal_sample_ms = millis()
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
        core_location = loc:copy(),
        max_strength = -99,
        avg_strength = 0,
        samples = {},
        start_time = millis(),
        wind_at_entry = get_wind_vector()
    }
end

-- Pre-calculates polygon vertices in local XY coordinates for faster boundary checks.
local function prepare_polygon_xy_cache()
    if #SoarNavGlobals.polygon_points < 3 then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Polygon invalid: less than 3 points")
        return
    end
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
        return nil
    end

    SoarNavGlobals.polygon_points = {}
    SoarNavGlobals.polygon_bounds = {min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7}
    local home = ahrs:get_home()
    if not home then
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
        return nil
    end

    if #SoarNavGlobals.polygon_points >= 3 then
        local first_pt = SoarNavGlobals.polygon_points[1]
        local last_pt = SoarNavGlobals.polygon_points[#SoarNavGlobals.polygon_points]

        if first_pt:get_distance(last_pt) > 0 then
            table.insert(SoarNavGlobals.polygon_points, first_pt:copy())
        end
    end
    prepare_polygon_xy_cache()
    return true
end

-- Generates a randomized target location around a central point.
local function generate_target_around_point(target_loc, radius_m)
    local rand_radius_m = math.sqrt(math.random()) * radius_m
    if not SoarNavGlobals.use_polygon_area and rand_radius_m > p_max_dist:get() then
        return nil
    end

    local new_loc = target_loc:copy()
    new_loc:offset_bearing(math.random() * 360, rand_radius_m)

    if not in_flight_area(new_loc) then
        return nil
    end

    return new_loc
end

-- Formats and logs the details for a newly generated waypoint.
local function log_new_waypoint(dist_to_wp)
    if SoarNavGlobals.is_in_focus_mode and SoarNavGlobals.focus_wp_counter > 1 then
        return
    end

    local d = tonumber(dist_to_wp) or 0
    local src = SoarNavGlobals.g_waypoint_source_info
    local now = millis()

    if src == SoarNavWaypointSources.REENGAGE_ENTRY or src == SoarNavWaypointSources.REENGAGE_FLYOUT then
        if src ~= SoarNavGlobals.tgt_l1_last_src then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("TGT: %s 🎯 %.0fm", src, d))
            SoarNavGlobals.tgt_l1_last_ms = now
            SoarNavGlobals.tgt_l1_last_d = d
            SoarNavGlobals.tgt_l1_last_src = src
        end
        return
    end

    SoarNavGlobals.tgt_l1_last_ms = SoarNavGlobals.tgt_l1_last_ms or nil
    SoarNavGlobals.tgt_l1_last_d = SoarNavGlobals.tgt_l1_last_d or nil
    SoarNavGlobals.tgt_l1_last_src = SoarNavGlobals.tgt_l1_last_src or nil
    local ok_time = (SoarNavGlobals.tgt_l1_last_ms == nil) or (safe_time_diff(now, SoarNavGlobals.tgt_l1_last_ms) >= 3000)
    local ok_dist = (SoarNavGlobals.tgt_l1_last_d == nil) or (math.abs(d - (SoarNavGlobals.tgt_l1_last_d or 0)) >= 20)
    local ok_src = (SoarNavGlobals.tgt_l1_last_src == nil) or (src ~= SoarNavGlobals.tgt_l1_last_src)

    if ok_time or ok_dist or ok_src then
        local log_level = 1
        if is_thermal_target(src) then
            log_level = 2
        end
        log_gcs(MAV_SEVERITY.INFO, log_level, string.format("TGT: %s 🎯 %.0fm", src, d))
        SoarNavGlobals.tgt_l1_last_ms = now
        SoarNavGlobals.tgt_l1_last_d = d
        SoarNavGlobals.tgt_l1_last_src = src
    end

    if SoarNavGlobals.target_loc ~= nil then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format(" ↳ Coords: %.5f, %.5f", SoarNavGlobals.target_loc:lat()/1e7, SoarNavGlobals.target_loc:lng()/1e7))
    end
end

-- Finalizes the selection of a new waypoint by setting global state variables.
local function _finalize_waypoint_selection()
    SoarNavGlobals.waypoint_start_time_ms = millis()
    SoarNavGlobals.waypoint_search_in_progress = false
    SoarNavGlobals.last_commanded_roll_deg = 0
    SoarNavGlobals.last_heading_error = 0
    SoarNavGlobals.last_progress_check_ms = nil
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

-- Selects the best thermal from memory, applies drift, and validates it is within the flight area.
local function select_best_thermal_waypoint()
    local valid_hotspots = clean_and_get_hotspots()
    if #valid_hotspots == 0 then return false end

    local good_hotspots = {}
    local good_hotspots_excluding_last = {}
    local loc = ahrs:get_location()
    if not loc then
        return false
    end
    local min_dist_sq = (p_wp_radius:get() * 2) ^ 2

    for _, hotspot in ipairs(valid_hotspots) do
        if hotspot.avg_strength and hotspot.avg_strength > 0 then
            local dist_vec = loc:get_distance_NE(hotspot.loc)
            local dist_sq = dist_vec:x()^2 + dist_vec:y()^2
            if dist_sq and dist_sq > min_dist_sq then
                table.insert(good_hotspots, hotspot)
                local last_used_int = 0
                if SoarNavGlobals.last_used_hotspot_timestamp then
                    last_used_int = SoarNavGlobals.last_used_hotspot_timestamp:toint()
                end
                if last_used_int == 0 or hotspot.timestamp:toint() ~= last_used_int then
                    table.insert(good_hotspots_excluding_last, hotspot)
                end
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
        return false
    end

    table.sort(final_hotspot_list, function(a, b) return (a.avg_strength or 0) > (b.avg_strength or 0) end)
    local best_hotspot = final_hotspot_list[1]

    local age_seconds = 0
    if best_hotspot.timestamp then
        age_seconds = safe_time_diff(millis(), best_hotspot.timestamp) / 1000.0
    end
    local wind_vec = best_hotspot.wind_vec or get_wind_vector()
    local drifted_loc = predict_thermal_drift(best_hotspot.loc, wind_vec, age_seconds)
    local drift_dist = best_hotspot.loc:get_distance(drifted_loc)

    if in_flight_area(drifted_loc) then
        local log_msg = "Best thermal has drifted."
        if SoarNavGlobals.energy_state ~= "NORMAL" then
            log_msg = "LOW ENERGY: " .. log_msg
        end

        if drift_dist > 0 then
            log_gcs(MAV_SEVERITY.INFO, 2, log_msg)
            log_gcs(MAV_SEVERITY.INFO, 2, string.format(" > Drift distance: %.0fm", drift_dist))
            SoarNavGlobals.g_waypoint_source_info = string.format(SoarNavWaypointSources.THERMAL_MEMORY_DRIFT, best_hotspot.avg_strength)
        else
            SoarNavGlobals.g_waypoint_source_info = string.format(SoarNavWaypointSources.THERMAL_MEMORY_NODRIFT, best_hotspot.avg_strength)
        end

        SoarNavGlobals.current_selected_hotspot = best_hotspot
        SoarNavGlobals.target_loc = drifted_loc
        SoarNavGlobals.last_used_hotspot_timestamp = best_hotspot.timestamp
        return true
    else
        return false
    end
end

-- Selects a cell for guided exploration, prioritizing a random unvisited cell first.
local function find_least_visited_cell()
    local min_visits = math.huge
    local best_cell_index = nil
    if #SoarNavGlobals.unvisited_cell_indices > 0 then
        local random_idx = math.random(1, #SoarNavGlobals.unvisited_cell_indices)
        return SoarNavGlobals.unvisited_cell_indices[random_idx]
    end
    for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
        local cell_data = SoarNavGlobals.grid_cells[cell_idx]
        if cell_data and cell_data.visit_count < min_visits then
            min_visits = cell_data.visit_count
            best_cell_index = cell_idx
        end
    end
    return best_cell_index
end

-- Checks for thermal streets and sets a waypoint if one is found.
local function check_and_use_thermal_street()
    local hotspots = clean_and_get_hotspots()
    if #hotspots < SoarNavConstants.MIN_HOTSPOTS_FOR_STREET then return false end

    local wind_vec = get_wind_vector()
    if not wind_vec or wind_vec:length() < 1.0 then return false end

    table.sort(hotspots, function(a, b) return a.timestamp:toint() > b.timestamp:toint() end)

    local h1 = hotspots[1]
    local h2 = hotspots[2]

    if not h1 or not h2 or not h1.loc or not h2.loc then return false end

    local wind_bearing = wind_vector_to_bearing_deg(wind_vec)
    local street_bearing = h2.loc:get_bearing(h1.loc)

    local angle_diff = math.abs(((street_bearing - wind_bearing + 540) % 360) - 180)

    if angle_diff <= p_street_tol:get() then
        local projection_dist = h2.loc:get_distance(h1.loc)
        projection_dist = math.max(500, math.min(projection_dist, 2000))

        local new_target = h1.loc:copy()
        new_target:offset_bearing((wind_bearing + 180) % 360, projection_dist)

        local loc = ahrs:get_location()
        if in_flight_area(new_target) and loc then
            local dist_to_target = loc:get_distance(new_target)
            if dist_to_target > (p_wp_radius:get() * 2) then
                log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Thermal street TGT (Δ%.0f°) 🎯 %.0fm", angle_diff, dist_to_target))
                SoarNavGlobals.target_loc = new_target
                SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.THERMAL_STREET
                return true
            else
                log_gcs(MAV_SEVERITY.INFO, 2, "Street WP too close, ignoring.")
            end
        else
            log_gcs(MAV_SEVERITY.INFO, 2, "Street WP out of bounds, ignoring.")
        end
    end

    return false
end

-- Core logic for searching and selecting a new waypoint based on energy state and thermal memory.
local function search_for_new_waypoint()
    if SoarNavGlobals.thermal_to_retry then
        local hotspot_to_retry = SoarNavGlobals.thermal_to_retry
        SoarNavGlobals.thermal_to_retry = nil
        if not hotspot_to_retry or not hotspot_to_retry.loc or not hotspot_to_retry.timestamp or not hotspot_to_retry.wind_vec then
            log_gcs(MAV_SEVERITY.WARNING, 1, "Invalid thermal data for retry, skipping.")
        else
            local current_loc = ahrs:get_location()
            if current_loc then
                local bearing_to_thermal = current_loc:get_bearing(hotspot_to_retry.loc)
                local flyout_bearing = (bearing_to_thermal + 180) % 360

                local reengage_dist = (p_wp_radius:get() or 50) * 2.5

                local flyout_point = current_loc:copy()
                flyout_point:offset_bearing(flyout_bearing, reengage_dist)

                SoarNavGlobals.target_loc = flyout_point
                local age_ms = safe_time_diff(millis(), hotspot_to_retry.timestamp)
                local age_s = age_ms / 1000.0
                local wind = hotspot_to_retry.wind_vec or get_wind_vector()
                SoarNavGlobals.reengage_final_target = predict_thermal_drift(hotspot_to_retry.loc, wind, age_s)
                SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.REENGAGE_FLYOUT
                _finalize_waypoint_selection()
                return
            end
        end
    end

    local LOST_THERMAL_THRESHOLD = 2
    if SoarNavGlobals.lost_thermal_counter >= LOST_THERMAL_THRESHOLD then
        log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Lost %d thermals. Exploring grid.", SoarNavGlobals.lost_thermal_counter))
        SoarNavGlobals.force_grid_after_reset = true
        SoarNavGlobals.lost_thermal_counter = 0
    end

    if SoarNavGlobals.is_in_focus_mode then
        local focus_timeout = SoarNavGlobals.focus_wp_timeout or 3
        if focus_timeout == 0 then
            focus_timeout = SoarNavGlobals.focus_wp_timeout or 3
        end

        if SoarNavGlobals.focus_wp_counter >= focus_timeout then
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Focus Mode timeout. Back to grid.")
            SoarNavGlobals.is_in_focus_mode = false
            SoarNavGlobals.focus_area_center = nil
            SoarNavGlobals.focus_wp_counter = 0
        else
            local focus_radius = SoarNavGlobals.effective_cluster_radius / 2
            local new_target = generate_target_around_point(SoarNavGlobals.focus_area_center, focus_radius)
            if new_target then
                SoarNavGlobals.target_loc = new_target
                SoarNavGlobals.g_waypoint_source_info = string.format(SoarNavWaypointSources.FOCUS_MODE, SoarNavGlobals.focus_wp_counter + 1)
                SoarNavGlobals.focus_wp_counter = SoarNavGlobals.focus_wp_counter + 1
                _finalize_waypoint_selection()
                return
            else
                log_gcs(MAV_SEVERITY.WARNING, 2, "Focus Mode: Failed to generate valid WP. Exiting.")
                SoarNavGlobals.is_in_focus_mode = false
                SoarNavGlobals.focus_area_center = nil
                SoarNavGlobals.focus_wp_counter = 0
            end
        end
    end

    local active_center = get_active_center_location()
    if not active_center then return end

    local source_for_log = SoarNavWaypointSources.UNKNOWN
    local new_wp_found = false

    local now_ms = millis()
    local recent_success_count = 0
    for _, hotspot in ipairs(SoarNavGlobals.thermal_hotspots) do
        if hotspot.entry_time and safe_time_diff(now_ms, hotspot.entry_time) <= (p_strat_hist:get() * 1000) then
            recent_success_count = recent_success_count + 1
        end
    end

    local force_grid_search = false
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    if dist_from_home_3d then
        local current_alt = -dist_from_home_3d:z()
        local soar_alt_max = p_soar_alt_max:get()
        if soar_alt_max and soar_alt_max > 0 and current_alt > soar_alt_max then
            force_grid_search = true
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Max alt reached. Forcing grid search.")
        end
    end

    if not force_grid_search then
        local energy_status = assess_energy_state()

        if energy_status == "LOW" or energy_status == "CRITICAL" then
            if #clean_and_get_hotspots() > 0 and select_best_thermal_waypoint() then
                _finalize_waypoint_selection()
                return
            else
                SoarNavGlobals.g_waypoint_source_info = string.format(SoarNavWaypointSources.GRID_ENERGY, energy_status)
            end
        end

        if check_and_use_thermal_street() then
            _finalize_waypoint_selection()
            return
        end

        local MIN_TMEM_CHANCE = 20
        local MAX_TMEM_CHANCE = 60
        local MAX_SUCCESS_FOR_SCALING = 3
        local success_factor = math.min(recent_success_count / MAX_SUCCESS_FOR_SCALING, 1.0)
        local dynamic_tmem_chance = MIN_TMEM_CHANCE + (MAX_TMEM_CHANCE - MIN_TMEM_CHANCE) * success_factor
        local use_hotspot_logic = false
        local valid_hotspots_raw = clean_and_get_hotspots()
        calculate_hotspot_density(valid_hotspots_raw)
        table.sort(valid_hotspots_raw, function(a, b)
            local density_a = a.cluster_density or 0
            local density_b = b.cluster_density or 0
            if density_a ~= density_b then
                return density_a > density_b
            else
                return (a.avg_strength or 0) > (b.avg_strength or 0)
            end
        end)
        local valid_hotspots = {}
        if (p_tmem_enable:get()) == 1 and #valid_hotspots_raw > 0 then
            if SoarNavGlobals.last_used_hotspot_timestamp then
                local last_used_int = SoarNavGlobals.last_used_hotspot_timestamp:toint()
                for _, hotspot in ipairs(valid_hotspots_raw) do
                    if hotspot.timestamp and hotspot.timestamp:toint() ~= last_used_int then
                        table.insert(valid_hotspots, hotspot)
                    end
                end
                if #valid_hotspots == 0 and #valid_hotspots_raw > 0 then
                    valid_hotspots = valid_hotspots_raw
                end
            else
                valid_hotspots = valid_hotspots_raw
            end
            if #valid_hotspots > 0 and math.random(1, 100) <= dynamic_tmem_chance and not SoarNavGlobals.force_grid_after_reset then
                use_hotspot_logic = true
            end
        end

        if use_hotspot_logic and not SoarNavGlobals.force_grid_after_reset then
            local selected_hotspot = valid_hotspots[1]

            if selected_hotspot and selected_hotspot.timestamp then
                local age_seconds = safe_time_diff(millis(), selected_hotspot.timestamp) / 1000.0
                local center_loc = selected_hotspot.loc:copy()
                local wind_vec = selected_hotspot.wind_vec or get_wind_vector()
                if wind_vec then
                    local wind_speed = wind_vec:length()
                    if wind_speed and wind_speed == wind_speed and wind_speed < 1/0 then
                        local calculated_drift = wind_speed * age_seconds
                        if calculated_drift and calculated_drift > 0 and calculated_drift == calculated_drift and calculated_drift < 1/0 then
                            local wind_heading_rad = wind_vec:xy():angle()
                            if wind_heading_rad and wind_heading_rad == wind_heading_rad then
                                local wind_dir_bearing = (math.deg(wind_heading_rad) + 360) % 360
                                center_loc:offset_bearing(wind_dir_bearing, calculated_drift)
                            end
                        end
                    end
                end
                local candidate_loc = generate_target_around_point(center_loc, SoarNavConstants.HOTSPOT_EXPLORATION_RADIUS_M)
                if candidate_loc then
                    SoarNavGlobals.current_selected_hotspot = selected_hotspot
                    SoarNavGlobals.target_loc = candidate_loc
                    new_wp_found = true
                    source_for_log = string.format(SoarNavWaypointSources.THERMAL_MEMORY_DRIFT, selected_hotspot.avg_strength or 0)
                end
            end
        end
    end

    if not new_wp_found then
        if not SoarNavGlobals.grid_initialized or not SoarNavGlobals.valid_cell_indices or #SoarNavGlobals.valid_cell_indices == 0 then
            source_for_log = SoarNavWaypointSources.RANDOM_FALLBACK
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
                SoarNavGlobals.unvisited_set = {}
                for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                    table.insert(SoarNavGlobals.unvisited_cell_indices, v)
                    SoarNavGlobals.unvisited_set[v] = true
                end
                SoarNavGlobals.last_cell_index = nil
                SoarNavGlobals.force_grid_after_reset = true
            end

            if #SoarNavGlobals.unvisited_cell_indices > 0 then
                SoarNavGlobals.force_grid_after_reset = false
                local chosen_cell_index
                local current_loc = ahrs:get_location()

                if SoarNavGlobals.reroute_desired_bearing and current_loc then
                    local search_distance = math.max(SoarNavGlobals.grid_cell_size_m * 1.5,
                                                     (SoarNavGlobals.max_operational_distance or 2000) / 6)
                    local bearing_offsets = SoarNavConstants.REROUTE_BEARING_OFFSETS or {0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90}
                    for _, offset in ipairs(bearing_offsets) do
                        local target_bearing = (SoarNavGlobals.reroute_desired_bearing + offset + 360) % 360
                        local projected_point = current_loc:copy()
                        projected_point:offset_bearing(target_bearing, search_distance)
                        local candidate_cell_idx = get_cell_index_from_location(projected_point)
                        if candidate_cell_idx then
                            if SoarNavGlobals.unvisited_set[candidate_cell_idx] then
                                chosen_cell_index = candidate_cell_idx
                                local __base = SoarNavGlobals.reroute_desired_bearing or 0
                                local __abs = (__base + offset) % 360
                                if __abs < 0 then __abs = __abs + 360 end
                                local __dirs = { "N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW" }
                                local __ix = math.floor((__abs / 22.5) + 0.5) + 1
                                if __ix > #__dirs then __ix = 1 end
                                log_gcs(MAV_SEVERITY.INFO, 2, string.format("Reroute: using cell at %.0f° %s", __abs, __dirs[__ix]))
                                break
                            end
                        end
                    end
                end

                if not chosen_cell_index then
                    local success_rate_factor = math.min(1, recent_success_count / 3.0)
                    local guided_chance = 25 + (50 * success_rate_factor)
                    if math.random(1, 100) <= guided_chance then
                        source_for_log = SoarNavWaypointSources.GRID_GUIDED
                        chosen_cell_index = find_least_visited_cell()
                    else
                        source_for_log = SoarNavWaypointSources.GRID_PURE
                        if #SoarNavGlobals.unvisited_cell_indices > 0 then
                            local random_idx = math.random(1, #SoarNavGlobals.unvisited_cell_indices)
                            chosen_cell_index = SoarNavGlobals.unvisited_cell_indices[random_idx]
                        else
                            chosen_cell_index = find_least_visited_cell()
                        end
                    end

                    if SoarNavGlobals.reroute_desired_bearing then
                        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Reroute fail -> %s C:%d", source_for_log, chosen_cell_index or 0))
                    end
                end

                if not chosen_cell_index then
                    log_gcs(MAV_SEVERITY.WARNING, 1, "Grid search: no valid cell selected.")
                    return
                end

                SoarNavGlobals.reroute_desired_bearing = nil
                SoarNavGlobals.reroute_origin_loc = nil

                local precalculated_center = SoarNavGlobals.grid_cell_centers[chosen_cell_index]
                if precalculated_center then
                    local success = false
                    for _ = 1, 5 do
                        local new_target_loc = precalculated_center:copy()
                        local offset_radius = math.floor(SoarNavGlobals.grid_cell_size_m / 2.5)
                        local radial_dist = math.sqrt(math.random()) * offset_radius
                        new_target_loc:offset_bearing(math.random() * 360, radial_dist)

                        local min_dist_from_craft = (p_wp_radius:get() or 50) * 1.5
                        if in_flight_area(new_target_loc) and current_loc and current_loc:get_distance(new_target_loc) > min_dist_from_craft then
                            SoarNavGlobals.target_loc = new_target_loc
                            success = true
                            break
                        end
                    end

                    if success then
                        new_wp_found = true
                        if source_for_log ~= SoarNavWaypointSources.GRID_GUIDED and source_for_log ~= SoarNavWaypointSources.GRID_PURE then
                            source_for_log = string.format(SoarNavWaypointSources.GRID_CELL, chosen_cell_index)
                        end
                    else
                        log_gcs(MAV_SEVERITY.INFO, 2, "Generated WP too close, rejecting.")
                    end
                else
                    log_gcs(MAV_SEVERITY.WARNING, 1, string.format("Cell center lookup failed for index %d", chosen_cell_index))
                    if SoarNavGlobals.unvisited_set then
                        SoarNavGlobals.unvisited_set[chosen_cell_index] = nil
                    end
                    for j = #SoarNavGlobals.unvisited_cell_indices, 1, -1 do
                        if SoarNavGlobals.unvisited_cell_indices[j] == chosen_cell_index then
                            table.remove(SoarNavGlobals.unvisited_cell_indices, j)
                            break
                        end
                    end
                    return
                end
            else
                return
            end
        end
    end
    if new_wp_found then
        SoarNavGlobals.g_waypoint_source_info = source_for_log
        _finalize_waypoint_selection()
    end
end

-- Calculates a dynamic waypoint timeout based on current wind speed.
local function get_wp_timeout()
    local base_timeout = p_wp_timeout:get() * 1000
    local mode = SoarNavConstants.WP_TIMEOUT_WIND_MODE or 0
    if mode == 0 then
        return base_timeout
    end
    local ref = SoarNavConstants.WP_TIMEOUT_WIND_REF_MPS or 15
    local minmul = SoarNavConstants.WP_TIMEOUT_WIND_MIN_MULT or 0.5
    local maxmul = SoarNavConstants.WP_TIMEOUT_WIND_MAX_MULT or 2.5

    local wind_vec = get_wind_vector()
    if wind_vec then
        local wind_speed = wind_vec:length()
        if wind_speed and wind_speed > 0 and ref > 0 then
            local factor = 1 + (mode * (wind_speed / ref))
            local adaptive = base_timeout * factor
            if mode > 0 then
                return math.min(adaptive, base_timeout * maxmul)
            else
                return math.max(adaptive, base_timeout * minmul)
            end
        end
    end
    return base_timeout
end

-- Monitors for propulsion failure during expected climbs and triggers an emergency RTL.
local function check_motor_failure(current_mode, dist_from_home_3d)
    if SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE or not arming:is_armed() or not dist_from_home_3d then
        return
    end

    local dyn_soalt_mode = p_dyn_soalt:get()
    if dyn_soalt_mode == 0 or SoarNavGlobals.script_state ~= SCRIPT_STATE.NAVIGATING or current_mode == SoarNavConstants.mode_thermal then
        SoarNavGlobals.motor_failure_check_active = false
        SoarNavGlobals.motor_on_start_time_ms = nil
        SoarNavGlobals.rpm_failure_start_ms = nil
        return
    end

    local wp_radius = p_wp_radius:get() or 50
    local suppress_radius = math.max(100, wp_radius * 2)
    if dist_from_home_3d:xy():length() <= suppress_radius then
        return
    end

    local soar_alt_min = p_soar_alt_min:get()
    if not soar_alt_min then return end
    
    local current_alt = -dist_from_home_3d:z()
    local throttle_output = SRV_Channels:get_output_scaled(70) or 0
    local motor_should_be_on = (current_alt < soar_alt_min) and (throttle_output > SoarNavConstants.THROTTLE_ACTIVE_THRESHOLD)

    if not motor_should_be_on then
        SoarNavGlobals.motor_failure_check_active = false
        SoarNavGlobals.motor_on_start_time_ms = nil
        SoarNavGlobals.rpm_failure_start_ms = nil
        return
    end

    local now = millis()
    local rpm_value = RPM and RPM.get_rpm and RPM:get_rpm(0)

    local function trigger_rtl()
        log_gcs(MAV_SEVERITY.CRITICAL, 0, "MOTOR FAILURE DETECTED! RTL ACTIVATED.")
        vehicle:set_mode(SoarNavConstants.RTL_MODE_CODE)
        restore_boot_alts('MOTOR Failure RTL')
        set_script_state(SCRIPT_STATE.ERROR, "Motor Failure RTL.")
    end

    if rpm_value ~= nil and rpm_value >= 0 then
        SoarNavGlobals.motor_on_start_time_ms = nil
        local rpm_issue = (throttle_output > SoarNavConstants.MOTOR_RPM_FAIL_THROTTLE_PERCENT) and (rpm_value < SoarNavConstants.MOTOR_RPM_FAIL_RPM_MIN)
        
        if rpm_issue then
            if not SoarNavGlobals.rpm_failure_start_ms then
                SoarNavGlobals.rpm_failure_start_ms = now
            end
        else
            SoarNavGlobals.rpm_failure_start_ms = nil
        end

        if SoarNavGlobals.rpm_failure_start_ms and safe_time_diff(now, SoarNavGlobals.rpm_failure_start_ms) > SoarNavConstants.MOTOR_RPM_FAIL_DEBOUNCE_MS then
            log_gcs(MAV_SEVERITY.INFO, 2, "Motor Failure Detected via RPM")
            trigger_rtl()
        end
    else
        SoarNavGlobals.rpm_failure_start_ms = nil
        if not SoarNavGlobals.motor_failure_check_active then
            SoarNavGlobals.motor_failure_check_active = true
            SoarNavGlobals.motor_on_start_time_ms = now
            SoarNavGlobals.altitude_at_check_start_m = current_alt
            SoarNavGlobals.soar_alt_min_at_check = soar_alt_min
        end

        local time_since_motor_on = safe_time_diff(now, SoarNavGlobals.motor_on_start_time_ms)
        if time_since_motor_on > SoarNavConstants.MOTOR_FAILURE_CHECK_DELAY_MS then
            local ned_velocity = ahrs:get_velocity_NED()
            if not ned_velocity then return end

            local climb_rate = -ned_velocity:z()
            local altitude_gained_ok = -dist_from_home_3d:z() > (SoarNavGlobals.altitude_at_check_start_m + 1.0)

            if not (climb_rate >= SoarNavConstants.MOTOR_FAILURE_CLIMB_RATE_THRESHOLD_MPS) and not altitude_gained_ok then
                log_gcs(MAV_SEVERITY.INFO, 2, "Motor Failure Detected via Climb Rate")
                trigger_rtl()
            else
                SoarNavGlobals.motor_failure_check_active = false
                SoarNavGlobals.motor_on_start_time_ms = nil
            end
        end
    end
end

-- #############################################################################
-- ## NAVIGATION STATE HANDLER (REFACTORED LOGIC)
-- #############################################################################

-- Pre-declare the core state machine handler functions
local handle_idle, handle_navigating, handle_thermal_pause, handle_pilot_override

-- Determines if conditions are suitable for a tactical mid-flight reroute.
local function check_tactical_reroute_conditions()
    if SoarNavGlobals.is_in_focus_mode then
        log_gcs(MAV_SEVERITY.INFO, 2, "Reroute skipped: Focus Mode.")
        return false
    end

    local __is_thermal_target = is_thermal_target(SoarNavGlobals.g_waypoint_source_info)

    local wind_vec = get_wind_vector()
    local wind_speed = wind_vec and wind_vec:length() or 0
    local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 3)))
    local __stuck = (SoarNavGlobals.stuck_counter or 0) >= adaptive_stuck_limit

    if __is_thermal_target and not __stuck then
        log_gcs(MAV_SEVERITY.INFO, 2, "Reroute skipped: Thermal WP (not stuck).")
        return false
    end

    if SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_FLYOUT or SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_ENTRY then
        log_gcs(MAV_SEVERITY.INFO, 2, "Reroute skipped: Re-engage.")
        return false
    end

    local min_path_for_reroute = SoarNavGlobals.max_operational_distance / SoarNavConstants.TACTICAL_REROUTE_MIN_PATH_DIVISOR
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

    if math.random(1, 100) > p_reroute_p:get() then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Reroute skipped: probability check (%d%%).", p_reroute_p:get()))
        return false
    end

    return true
end

-- Checks the status of the current waypoint (reached, timeout, or reroute trigger).
local function check_waypoint_status(loc, current_time_ms, cached_params)
    SoarNavGlobals.distance_to_wp = loc:get_distance(SoarNavGlobals.target_loc)

    if SoarNavGlobals.reroute_check_armed and SoarNavGlobals.initial_distance_to_wp > 0 and SoarNavGlobals.distance_to_wp <= (SoarNavGlobals.initial_distance_to_wp / 2) then
        SoarNavGlobals.reroute_check_armed = false
        if check_tactical_reroute_conditions() then
            return "REROUTE"
        end
    end

    local time_on_current_wp = safe_time_diff(current_time_ms, SoarNavGlobals.waypoint_start_time_ms)
    if time_on_current_wp > get_wp_timeout() then
        return "TIMEOUT"
    end

    if SoarNavGlobals.distance_to_wp < cached_params.wp_radius then
        return "REACHED"
    end

    return "NAVIGATING"
end

-- Detects if the aircraft is stuck and initiates a repositioning maneuver.
local function manage_anti_stuck(loc, current_time_ms)
    if SoarNavGlobals.is_repositioning then return false end

    local time_on_current_wp = safe_time_diff(current_time_ms, SoarNavGlobals.waypoint_start_time_ms)
    if time_on_current_wp < (p_stuck_time:get() * 1000) then return false end

    local progress_check_timeout = safe_time_diff(current_time_ms, SoarNavGlobals.last_progress_check_ms) > SoarNavConstants.STUCK_PROGRESS_CHECK_INTERVAL_MS
    if not SoarNavGlobals.last_progress_check_ms or progress_check_timeout then
        if SoarNavGlobals.distance_at_last_check > 0 and SoarNavGlobals.loc_at_last_check then
            local progress_made = SoarNavGlobals.distance_at_last_check - SoarNavGlobals.distance_to_wp
            local distance_traveled = loc:get_distance(SoarNavGlobals.loc_at_last_check)
            local efficiency_ratio = 0
            if distance_traveled > 5 then
                efficiency_ratio = progress_made / distance_traveled
            end

            local min_eff = p_stuck_eff:get()
            local min_prog = SoarNavConstants.STUCK_MIN_PROGRESS_M
            if progress_made < min_prog or efficiency_ratio < min_eff then
                SoarNavGlobals.stuck_counter = SoarNavGlobals.stuck_counter + 1
                log_gcs(MAV_SEVERITY.INFO, 2,
                    string.format("AntiStuck: prog=%.1fm eff=%.2f (min=%.2f)",
                                  progress_made, efficiency_ratio, min_eff))
                local wind_vec_info = get_wind_vector()
                if wind_vec_info then
                    local wind_bearing_info = wind_vector_to_bearing_deg(wind_vec_info)
                    log_gcs(MAV_SEVERITY.WARNING, 1, string.format("NoProg E%.2f | W%.1fm/s@%.0f", efficiency_ratio, wind_vec_info:length(), wind_bearing_info))
                end
            else
                SoarNavGlobals.stuck_counter = 0
            end
        end
        SoarNavGlobals.last_progress_check_ms = current_time_ms
        SoarNavGlobals.distance_at_last_check = SoarNavGlobals.distance_to_wp
        SoarNavGlobals.loc_at_last_check = loc:copy()
    end

    local wind_vec = get_wind_vector()
    local wind_speed = wind_vec and wind_vec:length() or 0
    local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 3)))

    if SoarNavGlobals.stuck_counter >= adaptive_stuck_limit then
        SoarNavGlobals.original_target_loc = SoarNavGlobals.target_loc
        SoarNavGlobals.is_repositioning = true
        if wind_vec and loc then
            local repo_attempts = SoarNavGlobals.stuck_counter
            local base_dist = math.max(150, math.min(700, wind_speed * 60))
            local repo_dist_m = base_dist + (repo_attempts * SoarNavConstants.STUCK_DISTANCE_INCREMENT_M)
            local upwind_offset_deg = (repo_attempts * 45) % 360
            log_gcs(MAV_SEVERITY.WARNING, 1, string.format("Stuck! Repositioning %.0fm, offset %.0f", repo_dist_m, upwind_offset_deg))
            local upwind_bearing = (wind_vector_to_bearing_deg(wind_vec) + upwind_offset_deg) % 360
            local repo_loc = loc:copy()
            repo_loc:offset_bearing(upwind_bearing, repo_dist_m)
            SoarNavGlobals.target_loc = repo_loc
            _finalize_waypoint_selection()
            return true
        else
            SoarNavGlobals.is_repositioning = false
            SoarNavGlobals.target_loc = nil
            return true
        end
    end
    return false
end

-- Calculates and applies the roll command to steer towards the target waypoint.
local function update_navigation_controller(loc, cached_params, dt_s)
    local target_heading_deg = math.deg(loc:get_bearing(SoarNavGlobals.target_loc))
    local current_heading_deg = math.deg(ahrs:get_yaw_rad())
    local heading_error = (target_heading_deg - current_heading_deg + 540) % 360 - 180

    local error_derivative = 0
    if dt_s > 0.01 then
        local function wrap180(x) return (x + 540) % 360 - 180 end
        error_derivative = wrap180(heading_error - SoarNavGlobals.last_heading_error) / dt_s
    end
    SoarNavGlobals.last_heading_error = heading_error

    local p_term = heading_error * cached_params.nav_p
    local d_term = error_derivative * cached_params.nav_d
    local raw_pd_command = p_term + d_term

    local smooth_factor = SoarNavConstants.ROLL_SMOOTHING_FACTOR
    local abs_smooth_factor = math.abs(smooth_factor)
    local smoothed_command
    if smooth_factor > 0 then
        if math.abs(raw_pd_command) > math.abs(SoarNavGlobals.last_commanded_roll_deg) then
            smoothed_command = (abs_smooth_factor * raw_pd_command) + ((1.0 - abs_smooth_factor) * SoarNavGlobals.last_commanded_roll_deg)
        else
            smoothed_command = raw_pd_command
        end
    else
        smoothed_command = (abs_smooth_factor * raw_pd_command) + ((1.0 - abs_smooth_factor) * SoarNavGlobals.last_commanded_roll_deg)
    end

    local desired_roll_deg = math.max(-cached_params.roll_limit, math.min(cached_params.roll_limit, smoothed_command))
    SoarNavGlobals.last_commanded_roll_deg = desired_roll_deg

    local sys_roll_limit = cached_params.sys_roll_limit
    local roll_normalized = desired_roll_deg / sys_roll_limit

    local roll_pwm_value
    if roll_normalized > 0 then
        roll_pwm_value = SoarNavGlobals.rc_roll_trim + roll_normalized * (SoarNavGlobals.rc_roll_max - SoarNavGlobals.rc_roll_trim)
    else
        roll_pwm_value = SoarNavGlobals.rc_roll_trim + roll_normalized * (SoarNavGlobals.rc_roll_trim - SoarNavGlobals.rc_roll_min)
    end

    if SoarNavGlobals.rc_roll_channel then
        SoarNavGlobals.rc_roll_channel:set_override(math.floor(roll_pwm_value))
    else
        set_script_state(SCRIPT_STATE.ERROR, "ERR: Roll channel is nil. Cannot override.")
    end

    return heading_error
end

-- Logs detailed navigation and system status to the GCS at a regular interval.
local function log_navigation_status(current_time_ms, heading_error, cached_params)
    if not SoarNavGlobals.last_status_log_ms or safe_time_diff(current_time_ms, SoarNavGlobals.last_status_log_ms) > SoarNavConstants.STATUS_LOG_INTERVAL_MS then
        SoarNavGlobals.last_status_log_ms = current_time_ms

        if (cached_params.log_lvl >= 2) then
            local wp_state_line = string.format("WP: D:%.0fm, Hdg Err:%+.0f, Roll:%+.1f", SoarNavGlobals.distance_to_wp, heading_error, SoarNavGlobals.last_commanded_roll_deg)

            local visited, total_cells = 0, #SoarNavGlobals.valid_cell_indices
            if total_cells > 0 then visited = total_cells - #SoarNavGlobals.unvisited_cell_indices end
            local grid_percent = total_cells > 0 and (100 * visited / total_cells) or 0
            local grid_line = string.format("Grid:%d/%d %.0f%% | Cell:%d", visited, total_cells, grid_percent, SoarNavGlobals.last_cell_index or 0)

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

            log_gcs(MAV_SEVERITY.INFO, 2, wp_state_line)
            log_gcs(MAV_SEVERITY.INFO, 2, grid_line)
            log_gcs(MAV_SEVERITY.INFO, 2, tmem_line)
        end
    end
end

-- #############################################################################
-- ## CORE SCRIPT STATE MACHINE
-- #############################################################################

-- Handles the IDLE state, waiting for navigation conditions to be met.
handle_idle = function(can_navigate)
    if can_navigate then
        set_script_state(SCRIPT_STATE.NAVIGATING, "Navigation conditions met, starting.")
        SoarNavGlobals.navigation_start_delay_counter = 3
    end
end

-- Handles the NAVIGATING state, managing the flight to a waypoint.
handle_navigating = function(current_time_ms, loc, can_navigate, is_in_thermal_mode, cached_params, autotune_active, dt_s)
    if SoarNavGlobals.navigation_start_delay_counter > 0 then
        SoarNavGlobals.navigation_start_delay_counter = SoarNavGlobals.navigation_start_delay_counter - 1
        return
    end

    if is_in_thermal_mode then
        SoarNavGlobals.script_state = SCRIPT_STATE.THERMAL_PAUSE
        log_gcs(MAV_SEVERITY.NOTICE, 2, "Thermal detected. Script pausing.")
        return
    end
    if not can_navigate or autotune_active then
        set_script_state(SCRIPT_STATE.IDLE, "Navigation conditions no longer met.")
        return
    end

    ::restart_search::

    if not SoarNavGlobals.target_loc then
        if not SoarNavGlobals.waypoint_search_in_progress then
            log_gcs(MAV_SEVERITY.INFO, 2, "Searching new WP...")
            SoarNavGlobals.waypoint_search_in_progress = true
        end
        search_for_new_waypoint()
        if not SoarNavGlobals.target_loc then
            return
        end
    end

    local wp_status = check_waypoint_status(loc, current_time_ms, cached_params)

    if wp_status == "REACHED" then
        if SoarNavGlobals.is_repositioning then
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Repositioned. Re-engaging.")
            SoarNavGlobals.target_loc = SoarNavGlobals.original_target_loc
            SoarNavGlobals.is_repositioning = false
            SoarNavGlobals.original_target_loc = nil
            _finalize_waypoint_selection()
        else
            if SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_ENTRY then
                local now_ms = millis():toint()
                if SoarNavGlobals.reengage_hold_active and now_ms < SoarNavGlobals.reengage_hold_until_ms and not is_in_thermal_mode then
                    if (not SoarNavGlobals.reengage_hold_last_msg_ms) or ((now_ms - SoarNavGlobals.reengage_hold_last_msg_ms) >= 2000) then
                        log_gcs(MAV_SEVERITY.INFO, 2, "Re-Engage: holding at WP.")
                        SoarNavGlobals.reengage_hold_last_msg_ms = now_ms
                    end
                    SoarNavGlobals.target_loc = SoarNavGlobals.target_loc
                    _finalize_waypoint_selection()
                    return
                else
                    if SoarNavGlobals.reengage_hold_active then
                        log_gcs(MAV_SEVERITY.INFO, 2, "Re-Engage: hold complete.")
                    end
                    SoarNavGlobals.reengage_hold_active = false
                    SoarNavGlobals.reengage_hold_last_msg_ms = nil
                end
            end
            if SoarNavGlobals.reengage_final_target then
                log_gcs(MAV_SEVERITY.INFO, 2, "Re-Engage: FlyOut done.")
                SoarNavGlobals.target_loc = SoarNavGlobals.reengage_final_target
                SoarNavGlobals.reengage_final_target = nil
                SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.REENGAGE_ENTRY
                local reengage_dwell_ms = p_reeng_dwell:get() * 1000
                SoarNavGlobals.reengage_hold_until_ms = millis():toint() + reengage_dwell_ms
                SoarNavGlobals.reengage_hold_active = true
                SoarNavGlobals.reengage_hold_last_msg_ms = millis():toint()
                log_gcs(MAV_SEVERITY.INFO, 2, string.format("Re-Engage: holding at WP for %.1fs", reengage_dwell_ms/1000))
                _finalize_waypoint_selection()
                return
            end
            if not SoarNavGlobals.is_in_focus_mode then
                log_gcs(MAV_SEVERITY.INFO, 1, "Waypoint reached.")
            end
            local hotspot_to_check = SoarNavGlobals.current_selected_hotspot
            if is_thermal_target(SoarNavGlobals.g_waypoint_source_info) and hotspot_to_check then
                hotspot_to_check.failed_attempts = (hotspot_to_check.failed_attempts or 0) + 1
                if hotspot_to_check.failed_attempts >= 2 then
                    for i = #SoarNavGlobals.thermal_hotspots, 1, -1 do
                        local h = SoarNavGlobals.thermal_hotspots[i]
                        if h.timestamp and hotspot_to_check.timestamp and h.timestamp:toint() == hotspot_to_check.timestamp:toint() then
                            table.remove(SoarNavGlobals.thermal_hotspots, i)
                            break
                        end
                    end
                end
                SoarNavGlobals.current_selected_hotspot = nil
            end
            SoarNavGlobals.target_loc = nil
            goto restart_search
        end
    elseif wp_status == "REROUTE" then
        if not SoarNavGlobals.target_loc then
            log_gcs(MAV_SEVERITY.INFO, 1, "Reroute skipped: no target.")
            goto restart_search
        end
        log_gcs(MAV_SEVERITY.INFO, 2, "Tactical Reroute.")

        SoarNavGlobals.reroute_origin_loc = ahrs:get_location():copy()
        local yaw_rad = ahrs:get_yaw_rad()
        if not yaw_rad then
            log_gcs(MAV_SEVERITY.INFO, 1, "Re-route skipped: heading unavailable")
            SoarNavGlobals.target_loc = nil
            goto restart_search
        end
        local hdg = (math.deg(yaw_rad) + 360) % 360
        local offset = math.random(p_reroute_min:get(), p_reroute_max:get())
        if math.random() > 0.5 then offset = -offset end
        SoarNavGlobals.reroute_desired_bearing = (hdg + offset + 360) % 360

        local __abs = SoarNavGlobals.reroute_desired_bearing
        local __dirs = { "N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW" }
        local __ix = math.floor((__abs / 22.5) + 0.5) + 1
        if __ix > #__dirs then __ix = 1 end
        log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Re-route engaged: %.0f° %s", __abs, __dirs[__ix]))
        SoarNavGlobals.target_loc = nil
        goto restart_search
    end

    if manage_anti_stuck(loc, current_time_ms) then
        return
    end

    local should_update_cell = not SoarNavGlobals.last_cell_check_loc or loc:get_distance(SoarNavGlobals.last_cell_check_loc) > (SoarNavGlobals.grid_cell_size_m / 2)
    if should_update_cell then
        update_visited_cell()
        SoarNavGlobals.last_cell_check_loc = loc:copy()
    end

    local heading_error = update_navigation_controller(loc, cached_params, dt_s)
    log_navigation_status(current_time_ms, heading_error, cached_params)
end

-- Handles the THERMAL_PAUSE state while the aircraft is in ArduPilot's THERMAL mode.
handle_thermal_pause = function(is_in_thermal_mode)
    if not is_in_thermal_mode then
        set_script_state(SCRIPT_STATE.IDLE, "Exited thermal, resuming.")
    end
end

-- Handles the PILOT_OVERRIDE state, managing stick input and gesture detection.
handle_pilot_override = function(current_time_ms, is_outside_pitch_dz, is_outside_yaw_dz, is_outside_roll_dz, can_navigate)
    local pilot_is_holding_input = is_outside_pitch_dz or is_outside_yaw_dz or is_outside_roll_dz
    if pilot_is_holding_input then
        SoarNavGlobals.last_pilot_input_ms = current_time_ms
    end
    check_pitch_gesture()
    check_roll_gesture()
    if not SoarNavGlobals.manual_override_active then
        local resume_delay_passed = safe_time_diff(current_time_ms, SoarNavGlobals.last_pilot_input_ms) > SoarNavConstants.PILOT_RESUME_DELAY_MS
        if not pilot_is_holding_input and SoarNavGlobals.last_pilot_input_ms and resume_delay_passed then
            if can_navigate then
                set_script_state(SCRIPT_STATE.NAVIGATING, "Resuming navigation.")
            else
                set_script_state(SCRIPT_STATE.IDLE, "Set Cruise/FBWB to resume SoarNav.")
            end
        end
    end
end

-- The main logic loop of the script, containing the primary state machine.
update_body = function()
    if SoarNavGlobals.rpm_check_counter ~= nil and SoarNavGlobals.rpm_check_counter >= 0 then
    SoarNavGlobals.rpm_check_counter = SoarNavGlobals.rpm_check_counter + 1
    if SoarNavGlobals.rpm_check_counter == 5 then
        local rpm1_type = param:get("RPM1_TYPE") or 0
        local rpm2_type = param:get("RPM2_TYPE") or 0
        if rpm1_type > 0 or rpm2_type > 0 then
            log_gcs(MAV_SEVERITY.NOTICE, 1, "RPM sensor set, motor check uses RPM.")
        else
            log_gcs(MAV_SEVERITY.NOTICE, 1, "No RPM sensor, using climb-rate fallback.")
        end
        SoarNavGlobals.rpm_check_counter = -1
    end
end
    local is_armed_now = arming:is_armed()
    if SoarNavGlobals.last_armed_state == nil then SoarNavGlobals.last_armed_state = is_armed_now end
    if (not is_armed_now) and SoarNavGlobals.last_armed_state == true and not SoarNavGlobals.restored_on_disarm then
        restore_boot_alts('Disarmed')
        SoarNavGlobals.restored_on_disarm = true
    end
    if is_armed_now then SoarNavGlobals.restored_on_disarm = false end
    SoarNavGlobals.last_armed_state = is_armed_now
    if SoarNavGlobals.initial_soar_alt_min == nil and p_soar_alt_min and p_soar_alt_min:get() ~= nil then
        local min_val = p_soar_alt_min:get() or -1
        local max_val = p_soar_alt_max:get() or -1
        local cutoff_val = p_soar_alt_cutoff:get() or -1
        SoarNavGlobals.initial_soar_alt_min = min_val
        SoarNavGlobals.initial_soar_alt_max = max_val
        SoarNavGlobals.initial_soar_alt_cutoff = cutoff_val
        log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Base Alts: MIN %.0f, CUTOFF %.0f, MAX %.0f", min_val, cutoff_val, max_val))
    end
    local current_time_ms = millis()
    local dt_s = 0
    if SoarNavGlobals.last_update_time_ms then
        dt_s = safe_time_diff(current_time_ms, SoarNavGlobals.last_update_time_ms) / 1000.0
    end
    SoarNavGlobals.last_update_time_ms = current_time_ms
    local current_snav_enable = p_enable:get()
    if (current_snav_enable == 0) then
        if SoarNavGlobals.script_state ~= SCRIPT_STATE.IDLE then
            set_script_state(SCRIPT_STATE.IDLE, "Script disabled by user.")
        end
        return update, 1000
    end
    local current_snav_max_dist = p_max_dist:get()
    if (current_snav_max_dist ~= SoarNavGlobals.last_snav_max_dist_value) or (current_snav_enable ~= SoarNavGlobals.last_snav_enable_value) then
        log_gcs(MAV_SEVERITY.NOTICE, 1, "Parameter changed. Re-initializing area.")
        if current_snav_max_dist > 0 then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Radius Mode activated: %.0fm.", current_snav_max_dist))
        end
        SoarNavGlobals.target_loc = nil
        SoarNavGlobals.is_initializing = false
        SoarNavGlobals.grid_initialized = false
        SoarNavGlobals.polygon_load_attempted = false
        SoarNavGlobals.last_snav_max_dist_value = current_snav_max_dist
        SoarNavGlobals.last_snav_enable_value = current_snav_enable
        SoarNavGlobals.use_polygon_area = false
    end
    local sys_roll_limit = param:get("ROLL_LIMIT_DEG") or ((param:get("LIM_ROLL_CD") or 4500) / 100)
    if sys_roll_limit <= 0 then sys_roll_limit = 45 end
    local cached_params = {
        log_lvl = get_safe_param(p_log_lvl, 1, 0, 2),
        max_dist = get_safe_param(p_max_dist, 500, 0, nil),
        wp_radius = get_safe_param(p_wp_radius, 50, 10, 200),
        tmem_enabled = (get_safe_param(p_tmem_enable, 1, 0, 1) == 1),
        soar_alt_min = get_safe_param(p_soar_alt_min, 100, 0, nil),
        rcmap_roll = p_rcmap_roll:get(),
        rcmap_pitch = p_rcmap_pitch:get(),
        rcmap_yaw = p_rcmap_yaw:get(),
        roll_limit = get_safe_param(p_roll_limit, 30, 10, 50),
        nav_p = get_safe_param(p_nav_p, 0.6),
        nav_d = get_safe_param(p_nav_d, 0.05),
        sys_roll_limit = sys_roll_limit
    }
    local autotune_chan = rc:find_channel_for_option(107)
    local autotune_active = (autotune_chan and autotune_chan:get_aux_switch_pos() > 0)
    if not validate_params() then
        set_script_state(SCRIPT_STATE.ERROR, "Invalid SNAV params. Disabling script.")
        return update, 5000
    end
    if SoarNavGlobals.script_state == SCRIPT_STATE.ERROR then
        return update, 5000
    end
    if not SoarNavGlobals.rc_limits_read and arming:is_armed() then
        local roll_ch_num = p_rcmap_roll:get() or 1
        local pitch_ch_num = p_rcmap_pitch:get() or 2
        local yaw_ch_num = p_rcmap_yaw:get() or 4
        SoarNavGlobals.rc_roll_channel = rc:get_channel(roll_ch_num)
        SoarNavGlobals.rc_roll_min = param:get(string.format("RC%u_MIN", roll_ch_num)) or 1000
        SoarNavGlobals.rc_roll_max = param:get(string.format("RC%u_MAX", roll_ch_num)) or 2000
        SoarNavGlobals.rc_roll_trim = param:get(string.format("RC%u_TRIM", roll_ch_num)) or 1500
        local rc_pitch_min = param:get(string.format("RC%u_MIN", pitch_ch_num)) or 1000
        local rc_pitch_max = param:get(string.format("RC%u_MAX", pitch_ch_num)) or 2000
        local rc_pitch_trim = param:get(string.format("RC%u_TRIM", pitch_ch_num)) or 1500
        local rc_yaw_min = param:get(string.format("RC%u_MIN", yaw_ch_num)) or 1000
        local rc_yaw_max = param:get(string.format("RC%u_MAX", yaw_ch_num)) or 2000
        local rc_yaw_trim = param:get(string.format("RC%u_TRIM", yaw_ch_num)) or 1500
        if not (SoarNavGlobals.rc_roll_min < SoarNavGlobals.rc_roll_trim and SoarNavGlobals.rc_roll_trim < SoarNavGlobals.rc_roll_max) then
            log_gcs(MAV_SEVERITY.WARNING, 1, string.format("RC roll bounds unusual: min=%d trim=%d max=%d",
                SoarNavGlobals.rc_roll_min, SoarNavGlobals.rc_roll_trim, SoarNavGlobals.rc_roll_max))
        end
        if not (rc_pitch_min < rc_pitch_trim and rc_pitch_trim < rc_pitch_max) then
            log_gcs(MAV_SEVERITY.WARNING, 1, string.format("RC pitch bounds unusual: min=%d trim=%d max=%d",
                rc_pitch_min, rc_pitch_trim, rc_pitch_max))
        end
        if not (rc_yaw_min < rc_yaw_trim and rc_yaw_trim < rc_yaw_max) then
            log_gcs(MAV_SEVERITY.WARNING, 1, string.format("RC yaw bounds unusual: min=%d trim=%d max=%d",
                rc_yaw_min, rc_yaw_trim, rc_yaw_max))
        end
        SoarNavGlobals.rc_limits_read = true
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("RC limits read for R:%d, P:%d, Y:%d", roll_ch_num, pitch_ch_num, yaw_ch_num))
    end
    local loc = ahrs:get_location()
    if not loc then
        SoarNavGlobals.location_error_count = SoarNavGlobals.location_error_count + 1
        if SoarNavGlobals.location_error_count > SoarNavConstants.MAX_LOCATION_ERRORS then
            set_script_state(SCRIPT_STATE.ERROR, "Persistent location error, disabling.")
        end
        return update, 200
    else
        SoarNavGlobals.location_error_count = safe_decrement(SoarNavGlobals.location_error_count)
    end
    if arming:is_armed() and not SoarNavGlobals.grid_initialized and not SoarNavGlobals.is_initializing then
        local should_init = false
        if cached_params.max_dist > 0 then
            SoarNavGlobals.use_polygon_area = false
            should_init = true
            if SoarNavGlobals.last_snav_max_dist_value <= 0 then
                SoarNavGlobals.dynamic_center_location = ahrs:get_location():copy()
            end
        else
            if not SoarNavGlobals.polygon_load_attempted then
                local poly_index_to_try = math.min(current_snav_enable, SoarNavConstants.max_polygon_index)
                local poly_loaded = false
                local function try_load_polygon(index)
                    local base_filename = string.format("%s%d%s", SoarNavConstants.polygon_filename_prefix, index, SoarNavConstants.polygon_filename_suffix)
                    local paths_to_try = {
                        "/APM/SCRIPTS/" .. base_filename,
                        "/APM/Scripts/" .. base_filename,
                        "/APM/scripts/" .. base_filename,
                        "/SCRIPTS/" .. base_filename,
                        "/Scripts/" .. base_filename,
                        "/scripts/" .. base_filename,
                        base_filename
                    }
                    for _, path in ipairs(paths_to_try) do
                        if read_polygon_file(path) then
                            log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Loaded polygon '%s'", path))
                            return true
                        end
                    end
                    return false
                end
                if try_load_polygon(poly_index_to_try) then
                    poly_loaded = true
                else
                    log_gcs(MAV_SEVERITY.WARNING, 1, string.format("snav%d.poly failed. Using fallback.", poly_index_to_try))
                    for i = 1, SoarNavConstants.max_polygon_index do
                        if try_load_polygon(i) then
                            poly_loaded = true
                            break
                        end
                    end
                end
                if poly_loaded then
                    SoarNavGlobals.use_polygon_area = true
                    SoarNavGlobals.dynamic_center_location = nil
                    should_init = true
                else
                    log_gcs(MAV_SEVERITY.CRITICAL, 0, "No valid polygon file found. Mode disabled.")
                    SoarNavGlobals.use_polygon_area = false
                end
                SoarNavGlobals.polygon_load_attempted = true
            end
        end
        if should_init and not SoarNavGlobals.grid_initialized then
            initialize_grid()
        end
    end
    if SoarNavGlobals.is_initializing then
        manage_grid_initialization()
        if SoarNavGlobals.is_initializing then
            return update, 200
        end
    end
    local roll_ch_obj = rc:get_channel(cached_params.rcmap_roll)
    local pitch_ch_obj = rc:get_channel(cached_params.rcmap_pitch)
    local yaw_ch_obj = rc:get_channel(cached_params.rcmap_yaw)
    if not roll_ch_obj or not pitch_ch_obj or not yaw_ch_obj then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Invalid RC channel mapping.")
        return update, 100
    end
    local roll_in = roll_ch_obj:norm_input_dz()
    local pitch_in = pitch_ch_obj:norm_input_dz()
    local yaw_in = yaw_ch_obj:norm_input_dz()
    local is_outside_roll_dz = math.abs(roll_in) > 0
    local is_outside_pitch_dz = math.abs(pitch_in) > 0
    local is_outside_yaw_dz = math.abs(yaw_in) > 0
    if SoarNavGlobals.script_state ~= SCRIPT_STATE.PILOT_OVERRIDE and SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING and (is_outside_pitch_dz or is_outside_yaw_dz) then
        set_script_state(SCRIPT_STATE.PILOT_OVERRIDE, "Pilot override detected.")
        SoarNavGlobals.last_pilot_input_ms = current_time_ms
        SoarNavGlobals.pitch_gesture_state = "idle"
        SoarNavGlobals.pitch_gesture_count = 0
        SoarNavGlobals.pitch_gesture_triggered_this_override = false
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
        SoarNavGlobals.manual_override_active = false
    end
    local current_mode = vehicle:get_mode()
    local is_in_thermal_mode = (current_mode == SoarNavConstants.mode_thermal)
    if cached_params.tmem_enabled then
        if is_in_thermal_mode and not SoarNavGlobals.was_in_thermal_mode then
            start_thermal_monitoring()
        elseif not is_in_thermal_mode and SoarNavGlobals.was_in_thermal_mode then
            stop_and_record_thermal()
        end
    end
    SoarNavGlobals.was_in_thermal_mode = is_in_thermal_mode
    if SoarNavGlobals.is_monitoring_thermal then
        local time_since_sample_ms = -1
        if SoarNavGlobals.last_thermal_sample_ms then
            time_since_sample_ms = safe_time_diff(current_time_ms, SoarNavGlobals.last_thermal_sample_ms)
        end
        local strength = SoarNavGlobals.current_thermal_stats.max_strength or 1.0
        if strength <= 0 then strength = 1.0 end
        local sample_interval = math.max(500, math.min(3000, 2000 / (strength + 0.5)))
        if time_since_sample_ms > 0 and (time_since_sample_ms > sample_interval) then
            sample_thermal_strength()
        end
    end
    local script_switch_high = (rc:get_aux_cached(SoarNavConstants.rc_opt_soaring_active) == 2)
    local home = ahrs:get_home()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    local can_navigate = false
    if arming:is_armed() and script_switch_high and not autotune_active and (current_mode == SoarNavConstants.mode_fbwb or current_mode == SoarNavConstants.mode_cruise) and home and dist_from_home_3d and ahrs:get_yaw_rad() and SoarNavGlobals.grid_initialized then
        local current_alt = -dist_from_home_3d:z()
        if SoarNavGlobals.initial_soar_alt_min and current_alt >= (SoarNavGlobals.initial_soar_alt_min - 5) then
            can_navigate = true
        end
    end
    if can_navigate and SoarNavGlobals.initial_soar_alt_min
       and SoarNavGlobals.script_state ~= SCRIPT_STATE.PILOT_OVERRIDE
       and not SoarNavGlobals.manual_override_active then
        local throttle_output = SRV_Channels:get_output_scaled(70) or 0
        local motor_is_off = throttle_output < 10
        local now = millis()
        if motor_is_off and ((SoarNavGlobals.last_glide_cone_update_ms == nil)
            or (safe_time_diff(now, SoarNavGlobals.last_glide_cone_update_ms) > 10000)) then
            update_dynamic_soar_alt_min()
            SoarNavGlobals.last_glide_cone_update_ms = now
        end
    end
    if SoarNavGlobals.script_state == SCRIPT_STATE.IDLE then
        handle_idle(can_navigate)
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING then
        handle_navigating(current_time_ms, loc, can_navigate, is_in_thermal_mode, cached_params, autotune_active, dt_s)
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.THERMAL_PAUSE then
        handle_thermal_pause(is_in_thermal_mode)
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
        handle_pilot_override(current_time_ms, is_outside_pitch_dz, is_outside_yaw_dz, is_outside_roll_dz, can_navigate)
    end

    check_motor_failure(current_mode, dist_from_home_3d)

    return update, 200
end

-- A safe wrapper that executes the main loop and catches any runtime errors.
update = function()
    local results = {pcall(update_body)}
    local ok = table.remove(results, 1)

    if not ok then
        local full_error_msg = "SoarNav ERR: " .. tostring(results[1])
        local chunk_size = 48
        for i = 1, #full_error_msg, chunk_size do
            local chunk = full_error_msg:sub(i, i + chunk_size - 1)
            gcs:send_text(MAV_SEVERITY.CRITICAL, chunk)
        end
        return update, 5000
    end
    return table.unpack(results)
end

log_gcs(MAV_SEVERITY.NOTICE, 1, "SoarNav Script Initialized.")
return update()