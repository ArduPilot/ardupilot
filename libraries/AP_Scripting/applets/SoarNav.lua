--[[
SoarNav: an advanced feature for hunting thermals by Marco Robustini.
-- Version 1.1.2 - 2025/09/19

SoarNav is an advanced Lua script for intelligent, autonomous soaring that
enhances ArduPilot's capabilities. It uses a suite of strategic features to
maximize flight time within a defined area.

The script employs a hybrid exploration strategy, dynamically balancing the
search for new grid cells with guided re-visiting of the least explored areas
based on recent thermal success. This is coupled with an intelligent thermal
memory that saves thermal core locations. A sophisticated clustering algorithm
then identifies promising hot zones using a weighted score based on nearby
thermals' strength, age, and proximity.

Real-time thermal analysis is performed using adaptive sampling for a responsive
strength calculation. The script is architected for robust performance with a
modular state machine and parameter caching. Safety is enhanced with dynamic
anti-stuck logic, tactical rerouting, and thermal street detection.

================================================================================
                                 Key Features
================================================================================

- **Hybrid Exploration Strategy (Virtual Grid)**:
  Employs a dual-mode grid search, dynamically alternating between **Guided
  Exploration** (re-visiting least-explored cells) and **Pure Exploration**
  (seeking new, unvisited cells) based on recent thermal success.

- **Efficient & Optimized**:
  The script is optimized to reduce CPU load by leveraging native ArduPilot
  APIs, a modular state machine, and parameter caching.

- **Advanced Thermal Analysis (Strength & Quality)**:
  Utilizes **Adaptive Sampling** (a weighted EMA based on climb rate) to
  rapidly and accurately calculate a thermal's average strength, peak
  strength, and **consistency** (steady vs. variable lift).

- **Intelligent Thermal Memory (Core, Clustering & Focus Mode)**:
  Saves the **thermal core** (location of strongest lift) for greater
  accuracy. A sophisticated **clustering** algorithm identifies "hot zones"
  and can engage a **Focus Mode** to intensely search these areas.

- **Energy-Aware & Dual-Strategy Decision Making**:
  Uses a **time-corrected low-pass filter** on normalized altitude to
  determine energy state (NORMAL, LOW, CRITICAL) for stable transitions.

- **Thermal Streeting (Pattern Recognition)**:
  Identifies **"thermal streets"**—lines of lift aligned with the wind—to
  proactively search along the projected street axis.

- **Dynamic Thermal Drift Prediction**:
  Uses the current wind vector and a hotspot's age to predict its new,
  drifted position, increasing the chances of a successful intercept.

- **Safety & Tactical Navigation**:
  Includes pre-flight parameter validation, robust and **dynamic anti-stuck
  logic**, and a tactical mid-flight re-route capability.

- **Adaptive Waypoint Timeout**:
  Intelligently adjusts the time to reach a waypoint based on current wind
  conditions, allowing more time in strong headwinds.

- **Intuitive Pilot Override & Stick Gestures**:
  - **Temporary Override**: Pitch or Yaw input instantly pauses navigation.
  - **Persistent Override**: A rapid Roll gesture toggles manual override.
  - **Area Re-centering**: A rapid Pitch gesture re-centers the search area.

- **Flexible Area System (Radius, Rally Point, Polygon)**:
  Defines the operational area via radius, the flight controller's Rally Points,
  or a polygon file from the SD card, with a clear priority hierarchy
  (Radius > Rally Points > File).

- **Advanced Logging System**:
  A multi-level logging system provides clear operational feedback via GCS.

================================================================================
                        Script Parameters (SNAV_*)
================================================================================
- SNAV_ENABLE: Master switch. 0:Disabled. >0:Enabled. If RADIUS_M is 0,
  this value (1-9) selects the polygon file (snavX.poly). NOTE: If 3 or
  more Rally Points are present, they will take priority over any file.
- SNAV_LOG_LVL: GCS log verbosity (0:Silent, 1:Events).
- SNAV_RADIUS_M: Radius in meters of the circular flight area (0 for polygon).
- SNAV_ROLL_LIMIT: Maximum roll angle in degrees for navigation.
- SNAV_WP_RADIUS: Acceptance radius in meters for waypoints.
- SNAV_NAV_P / SNAV_NAV_D: Gains for the PD navigation controller.
- SNAV_TMEM_ENABLE: Enables (1) or disables (0) the Thermal Memory feature.
- SNAV_TMEM_LIFE: Lifetime in seconds of a stored thermal hotspot.
- SNAV_STUCK_EFF: Minimum path efficiency to not be "stuck" (higher is more
  sensitive).
- SNAV_STUCK_TIME: Grace time (sec) before anti-stuck logic becomes active.
- SNAV_REENG_DWELL: Time (sec) to loiter at a re-engagement point.
- SNAV_RETRY_THR: Score threshold (0-100) to trigger a retry on a weak
  thermal. Lower is more aggressive.
- SNAV_STREET_TOL: Angle tolerance (deg) for detecting a thermal street.
- SNAV_REROUTE_P: Probability (%) of a mid-flight tactical reroute.
- SNAV_NEC_WEIGHT: Weight of the 'necessity' factor (low altitude) in the
  retry score.
- SNAV_TMEM_MIN_S: Minimum thermal strength (m/s) to be saved to memory.
- SNAV_FOCUS_THR: Density score threshold to trigger Focus Mode.
- SNAV_STRAT_HIST: Time window (s) for history to influence strategy.
- SNAV_WP_TIMEOUT: Base time (sec) to reach a waypoint before timeout.
- SNAV_REROUTE_MIN/MAX: Min/Max angle (deg) for a tactical reroute.
- SNAV_DYN_SOALT: Controls Glide Cone behavior. 0:Disabled, 1:Fully Linked,
  2:MIN Only (capped).
- SNAV_GC_MARGIN: Safety altitude margin (m) added to the calculated glide
  altitude for the Glide Cone feature.
- SNAV_GC_PAD: Extra altitude padding (m) to ensure arrival over home point.

================================================================================
                         GCS Message Cheatsheet
================================================================================

--------------------------------------------------------------------------------
                         How to Decode GCS Messages
--------------------------------------------------------------------------------
All script messages are prefixed with "SoarNav: ". Target navigation messages
follow a specific format.

Example of a full message:
SoarNav: 🎯 [Pure] 182°S 670m 89/142

- SoarNav: : The standard prefix for all script messages.
- 🎯 [Pure]: The current navigation objective (see sections below).
- 182°S:    Required heading to the target, corrected for wind.
- 670m:     Distance in meters to the target.
- 89/142:   Number of grid cells visited / total valid cells in the area.
- Gxx%:     May replace the cell count, showing grid completion percentage.

--------------------------------------------------------------------------------
                            Initialization & State
--------------------------------------------------------------------------------
- SoarNav: Script V1.0.0 initialized.
  The script has loaded successfully and is ready.

- SoarNav: Rally A=1.80km2;RP=4;MD=1980m
  Displayed when using a polygon from the flight controller's Rally Points.

- SoarNav: Poly A=2.45km2;RP=5;MD=2150m
  Displayed when using a polygon loaded from a file. Shows total Area (A),
  Reference Points (RP, vertices), and Maximum Distance (MD).

- SoarNav: Radius A=0.79km2;RD=500m
  Displayed when using a circular area. Shows total Area (A) and the
  defined Radius (RD).

- SoarNav: Awaiting pilot activation (ROLL gesture).
  The script is armed and waiting for the pilot's start command (a rapid
  roll stick gesture).

- SoarNav: Pilot activation received.
  The activation gesture was recognized. Autonomous navigation is starting.

- SoarNav: Parameter changed. Re-initializing area.
  A key parameter (e.g., SNAV_RADIUS_M) has changed, and the operational
  area/grid will be re-initialized.

- SoarNav: Navigation conditions no longer met.
  Navigation is pausing because conditions are no longer valid (e.g., flight
  mode was changed).

--------------------------------------------------------------------------------
                         Pilot Override & Stick Commands
--------------------------------------------------------------------------------
- SoarNav: Pilot override detected.
  **Temporary Override**: Pilot has moved Pitch or Yaw sticks. Navigation
  is paused.

- SoarNav: Resuming navigation.
  Pilot has released the sticks for a few seconds. Autonomous navigation
  is resuming.

- SoarNav: Stick CMD: Manual override ON.
  **Persistent Override ON**: Pilot has performed a rapid Roll gesture.
  Navigation is paused until the gesture is repeated.

- SoarNav: Stick CMD: Manual override OFF.
  **Persistent Override OFF**: Pilot has repeated the Roll gesture. The
  script will resume navigation.

- SoarNav: Stick CMD: Radius Area re-centered.
  **Area Re-center**: Pilot has performed a rapid Pitch gesture during an
  override. The center of the circular search area is now the current
  aircraft location.

- SoarNav: Stick CMD: Polygon re-centered.
  **Area Re-center**: Same as above, but for a polygon area. The polygon's
  shape is maintained but its center is moved to the current location.

--------------------------------------------------------------------------------
                             Navigation & Targets
--------------------------------------------------------------------------------
- SoarNav: 🎯 [Thrm(Drift, +2.3)] 015°NNE 1590m
  Targets a thermal's *predicted* position, calculated from its last known
  location, age, and the current wind vector.

- SoarNav: 🎯 [Thrm(NoDrift, +1.8)] 332°NW 488m
  Targets a thermal's *last known* core location. This is used when wind
  data is unreliable or unavailable.

- SoarNav: 🎯 [Thermal Street] 215°SW 2105m
  Following a detected line of thermals that is aligned with the wind
  direction to maximize soaring distance.

- SoarNav: 🎯 [Re-Engage FlyOut] 188°S 250m 88/142
  **Phase 1** of retrying a weak thermal: flying *away* from it briefly to set
  up for a better re-entry angle.

- SoarNav: 🎯 [Re-Engage ReEntry] 010°N 430m 88/142
  **Phase 2**: flying back toward the thermal's predicted location from the
  new entry point.

- SoarNav: 🎯 [Focus (WP 3)] 095°E 120m
  Executing a micro-search in a high-potential zone ("hotspot"). The `WP 3`
  indicates navigation to the 3rd waypoint in this focused search pattern.

- SoarNav: 🎯 [Guided] 330°NNW 450m 88/142
  Targets the least-visited cell. **Note:** This strategy can be chosen
  probabilistically based on thermal success. If unvisited cells
  still exist, it will target one of them. True "least-visited"
  logic is used only after the grid is fully explored.

- SoarNav: 🎯 [Pure] 182°S 670m 89/142
  Targets a random, *unvisited* cell to expand the boundaries of the known
  area.

- SoarNav: 🎯 [Alt: LOW] 245°WSW 850m 90/142
  Energy recovery objective. Triggered by low altitude, it repositions the
  aircraft to regain height, typically upwind or towards known lift.

- SoarNav: 🎯 [Cell: 118] 045°NE 592m 91/142
  A fallback objective that explicitly targets a grid cell by its index.

- SoarNav: 🎯 [Random Fallback] 135°SE 1350m
  Selects a random point to break stagnation if no other logic can select a
  target.

- SoarNav: 🎯 [Re-route] 215°SW 1120m 90/142
  A tactical mid-flight course change to opportunistically explore a
  promising new direction.

 - SoarNav: 🎯 [Ridge] 270°W 820m s=0.60
  Engages upwind ridge lift: sets a safe, parallel-to-crest track offset
  upwind to exploit orographic lift when wind faces the slope.
  Shows bearing (deg+cardinal), distance, and ridge score (s).
  Requires Terrain data enabled.

--------------------------------------------------------------------------------
                           Thermals & Focus Mode
--------------------------------------------------------------------------------
- SoarNav: Thermal exit: no data. (Lost Thermal)
  Exited a thermal, but the lift was too weak or inconsistent to be saved
  to memory.

- SoarNav: Thermal found, exiting focus.
  A thermal was confirmed while in Focus Mode, so the micro-search is
  aborted to begin centering.

- SoarNav: Thermal ignored: out of area.
  A valid thermal was detected, but its core is outside the defined flight
  area and was therefore ignored.

- SoarNav: Focus Mode ON (Density)
  Engaging Focus Mode because a high density of recent, strong thermals has
  been detected in a concentrated zone.

- SoarNav: Focus Mode timeout. Back to grid.
  No thermal was confirmed within the Focus Mode time limit. Aborting the
  search and returning to normal grid exploration.

- SoarNav: Lost 2 thermals. Exploring grid.
  Multiple consecutive attempts to engage thermals have failed (default is 2).
  Switching back to grid exploration.

--------------------------------------------------------------------------------
                             Performance & Auto-Tuning
--------------------------------------------------------------------------------
- SoarNav: Polar Learn: CD0=0.02154 B=0.02361
  The in-flight aerodynamic auto-tuning feature (Polar Learn) has
  successfully collected enough clean glide data. It has calculated and
  applied new values for the aircraft's performance parameters
  (SOAR_POLAR_CD0 and SOAR_POLAR_B), leading to more accurate
  efficiency and glide range calculations.

--------------------------------------------------------------------------------
                             Safety & Anti-Stuck
--------------------------------------------------------------------------------
- SoarNav: Stuck! Repositioning 350m, offset 45°
  Anti-stuck logic has triggered. **Note: Distance and offset angle are
  dynamic and will increase on subsequent triggers.**

- SoarNav: NoProg E0.21 | W4.5m/s@210°
  Progress-check warning. Path Efficiency (E) is below the threshold. Also
  shows current Wind speed and direction.

- SoarNav: Repositioned. Re-engaging.
  The anti-stuck maneuver is complete. Resuming navigation toward the
  original target.

- SoarNav: Glide Cone: Safe return alt now 185m
  The dynamic minimum safe altitude required to glide home has been updated
  by the Glide Cone feature.

- SoarNav: MOTOR FAILURE DETECTED! RTL ACTIVATED.
  A critical failure of the propulsion system is suspected. An immediate
  Return To Launch (RTL) has been commanded.

- SoarNav: RTL Stall: Force direct Home route @100
  When RTL engages and the flight area was built from Rally points, the script
  waits ~3 seconds. If the aircraft is not making sufficient progress towards
  Home, SoarNav takes control by switching to GUIDED mode and commands the
  aircraft to fly directly to the Home point at RTL_ALTITUDE. Once the aircraft
  is within RTL_RADIUS of Home, the script returns control to the standard RTL
  mode to complete the loiter or landing sequence. The override is automatically
  suspended if the pilot manually changes flight modes. This feature is enabled
  by default.

- SoarNav: RTL Override: In home area. Loitering.
  The script's GUIDED mode override has successfully brought the aircraft back
  to the home area. Upon entering the RTL_RADIUS, the script's intervention
  concludes. The aircraft will remain in GUIDED mode, performing a precise
  loiter directly over the Home point until the pilot takes manual control
  by changing the flight mode.

--------------------------------------------------------------------------------
                             Errors & Parameters
--------------------------------------------------------------------------------
- SoarNav: SNAV_REROUTE_MIN <= REROUTE_MAX.
  Parameter validation error: The minimum reroute angle cannot exceed the
  maximum.

- SoarNav: SNAV_ROLL_LIMIT out of range [10..60]: 8
  A parameter's value is outside its recommended safe range.

- SoarNav: Invalid RC channel mapping.
  Critical setup error. The script cannot find the RC channels for
  roll/pitch/yaw as defined in RCMAP parameters.

- SoarNav: snav2.poly failed. Using fallback.
  The specified polygon file could not be loaded. The script is attempting
  to load a backup (e.g., snav1.poly).

- SoarNav: No polygon file; mode off.
  The script is set for polygon mode, but no valid snavX.poly file was found.

  - SoarNav: All strategies failed. Using fallback.
  All waypoint-selection strategies failed. A random safe fallback target
  was chosen.

================================================================================

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

================================================================================
]]

local in_flight_area
math.randomseed(millis():toint())

-- Mavlink severity levels for GCS messages
local MAV_SEVERITY = {
	CRITICAL = 2,
	ERROR = 3,
	WARNING = 4,
	NOTICE = 5,
	INFO = 6,
}

-- [[[ GCS MESSAGES — CENTRALIZED, ID-BASED ]]]
local MSG_ID = {
	AREA_INFO_POLY = 1,
	PARAM_OUT_OF_RANGE = 2,
	GESTURE_ACTIVATED = 3,
	ALL_STRAT_FAILED = 4,
	AUTO_START = 5,
	AWAITING_ACTIVATION = 6,
	BASE_ALTS_INFO = 7,
	NO_THERMAL_LOC = 8,
	CELL_INDEX_FAIL = 9,
	DISARMED = 10,
	ERR_ROLL_NIL = 11,
	EFF_CLUSTER_RADIUS = 12,
	GRID_READY = 13,
	FATAL_NO_WP = 14,
	FOCUS_ON_DENSITY = 15,
	FORCING_NEW_TARGET = 16,
	RPS_NEED_3 = 17,
	GCONE_RST_OVERRIDE = 18,
	GCONE_OFF_NO_PARAMS = 19,
	GCONE_ALT_UPDATE = 20,
	GRID_TOO_SMALL = 21,
	GRID_INIT_FAIL_NO_CENTER = 22,
	GRID_INIT_STARTED = 23,
	GRID_VALIDATED = 24,
	GRID_SCAN_INFO = 25,
	INVALID_RC_MAPPING = 26,
	INVALID_SNAV_PARAMS = 27,
	LOADED_POLY = 28,
	MOTOR_FAIL_RTL = 29,
	MOTOR_FAIL_REASON = 30,
	NAV_COND_MET = 31,
	NAV_COND_NOT_MET = 32,
	NO_RPM_FALLBACK = 33,
	NO_POLY_DISABLED = 34,
	NO_PROG_WARN = 35,
	PARAM_CHANGED_REINIT = 36,
	PERSISTENT_LOC_ERROR = 37,
	PILOT_ACTIVATION = 38,
	PILOT_OVERRIDE = 39,
	POLY_INVALID_POINTS = 40,
	POLY_MAX_DIST = 41,
	RC_P_LIMITS = 42,
	RC_R_LIMITS = 43,
	RC_Y_LIMITS = 44,
	RC_LIMITS_READ = 45,
	RPM_SENSOR_SET = 46,
	RADIUS_MODE_ACTIVATED = 47,
	RALLY_CHANGED_REINIT = 48,
	REROUTE_SKIP_HEADING = 49,
	REPOSITIONED = 50,
	REROUTE_SKIP_NO_TARGET = 51,
	RESUMING_NAV = 52,
	SNAV_REROUTE_MINMAX = 53,
	SNAV_RADIUS_NEGATIVE = 54,
	AREA_INFO_RADIUS = 55,
	SCRIPT_INITIALIZED = 56,
	SCRIPT_DISABLED = 57,
	SET_CRUISE_FBWB = 58,
	GCS_PREFIX = 59,
	MISSING_PARAM = 60,
	PARAM_SKIP = 61,
	STICK_CMD_MANUAL_OFF = 62,
	STICK_CMD_MANUAL_ON = 63,
	STICK_CMD_POLY_RECENTER = 64,
	STICK_CMD_RADIUS_RECENTER = 65,
	STICK_CMD_STUCK = 66,
	THERMAL_EXIT_NO_DATA = 67,
	THERMAL_FOUND_EXIT_FOCUS = 68,
	THERMAL_IGNORED_OOB = 69,
	POLY_FAILED_FALLBACK = 70,
	NAV_TARGET_SIMPLE = 71,
	NAV_TARGET_CELLS = 72,
	NAV_TARGET_REROUTE = 73,
	RTL_STALL_OVERRIDE = 74,
	RTL_OVERRIDE_RESUME = 75,
	NAV_TARGET_RIDGE = 76,
	POLAR_LEARN_UPDATE = 77,
}

-- === Output Symbol Set ===
local SNAV_USE_UNICODE = true
local SYM = SNAV_USE_UNICODE and "🎯" or "*"
local DEG = SNAV_USE_UNICODE and "°" or "deg"
local LEQ = SNAV_USE_UNICODE and "≤" or "<="

local MSG = {
	[MSG_ID.AREA_INFO_POLY] = "%s A=%.2fkm2;RP=%u;MD=%.0fm",
	[MSG_ID.PARAM_OUT_OF_RANGE] = "%s out of range [%s..%s]: %.3f",
	[MSG_ID.GESTURE_ACTIVATED] = "Activation gesture received.",
	[MSG_ID.ALL_STRAT_FAILED] = "All strategies failed. Using fallback.",
	[MSG_ID.AUTO_START] = "Auto-start: gesture not required.",
	[MSG_ID.AWAITING_ACTIVATION] = "Awaiting pilot activation (ROLL gesture).",
	[MSG_ID.BASE_ALTS_INFO] = "Base Alts: MIN %.0f, CUTOFF %.0f, MAX %.0f",
	[MSG_ID.NO_THERMAL_LOC] = "Can't monitor thermal: no location.",
	[MSG_ID.CELL_INDEX_FAIL] = "Cell index fail, fallback.",
	[MSG_ID.DISARMED] = "Disarmed.",
	[MSG_ID.ERR_ROLL_NIL] = "ERR: Roll channel is nil. Cannot override.",
	[MSG_ID.EFF_CLUSTER_RADIUS] = "Eff. Cluster Radius: %.0fm",
	[MSG_ID.GRID_READY] = "Exploration grid ready.",
	[MSG_ID.FATAL_NO_WP] = "FATAL: Could not set any valid waypoint.",
	[MSG_ID.FOCUS_ON_DENSITY] = "Focus Mode ON (Density)",
	[MSG_ID.FORCING_NEW_TARGET] = "Forcing new target search...",
	[MSG_ID.RPS_NEED_3] = "Found %d RPs. Need >= 3. Using file.",
	[MSG_ID.GCONE_RST_OVERRIDE] = "GC Rst(Override)->MIN:%.0f C:%.0f M:%.0f",
	[MSG_ID.GCONE_OFF_NO_PARAMS] = "GCone off: set POLAR_CD0/B & AIRSPEED.",
	[MSG_ID.GCONE_ALT_UPDATE] = "Glide Cone: Safe return alt now %.0fm",
	[MSG_ID.GRID_TOO_SMALL] = "Grid area too small. Init aborted.",
	[MSG_ID.GRID_INIT_FAIL_NO_CENTER] = "Grid init failed: no active center.",
	[MSG_ID.GRID_INIT_STARTED] = "Grid init started...",
	[MSG_ID.GRID_VALIDATED] = "Grid validated: %d cells.",
	[MSG_ID.GRID_SCAN_INFO] = "Grid: %d rows, %d cols. Scan cells...",
	[MSG_ID.INVALID_RC_MAPPING] = "Invalid RC channel mapping.",
	[MSG_ID.INVALID_SNAV_PARAMS] = "Invalid SNAV params. Disabling script.",
	[MSG_ID.LOADED_POLY] = "Loaded polygon '%s'",
	[MSG_ID.MOTOR_FAIL_RTL] = "MOTOR FAILURE DETECTED! RTL ACTIVATED.",
	[MSG_ID.MOTOR_FAIL_REASON] = "Motor Failure RTL.",
	[MSG_ID.NAV_COND_MET] = "Navigation conditions met, starting.",
	[MSG_ID.NAV_COND_NOT_MET] = "Navigation conditions no longer met.",
	[MSG_ID.NO_RPM_FALLBACK] = "No RPM sensor, using climb-rate fallback.",
	[MSG_ID.NO_POLY_DISABLED] = "No poly (Rally/File). Mode disabled.",
	[MSG_ID.NO_PROG_WARN] = "NoProg E%.2f | W%.1fm/s@%.0f",
	[MSG_ID.PARAM_CHANGED_REINIT] = "Parameter changed. Re-initializing area.",
	[MSG_ID.PERSISTENT_LOC_ERROR] = "Persistent location error, disabling.",
	[MSG_ID.PILOT_ACTIVATION] = "Pilot activation received.",
	[MSG_ID.PILOT_OVERRIDE] = "Pilot override detected.",
	[MSG_ID.POLY_INVALID_POINTS] = "Polygon invalid: less than 3 points",
	[MSG_ID.POLY_MAX_DIST] = "Polygon max distance calculated: %.0fm.",
	[MSG_ID.RC_P_LIMITS] = "RC P lim: %d/%d/%d",
	[MSG_ID.RC_R_LIMITS] = "RC R lim: %d/%d/%d",
	[MSG_ID.RC_Y_LIMITS] = "RC Y lim: %d/%d/%d",
	[MSG_ID.RC_LIMITS_READ] = "RC limits read for R:%d, P:%d, Y:%d",
	[MSG_ID.RPM_SENSOR_SET] = "RPM sensor set, motor check uses RPM.",
	[MSG_ID.RADIUS_MODE_ACTIVATED] = "Radius Mode activated: %.0fm.",
	[MSG_ID.RALLY_CHANGED_REINIT] = "Rally points changed; reinit.",
	[MSG_ID.REROUTE_SKIP_HEADING] = "Re-route skipped: heading unavailable",
	[MSG_ID.REPOSITIONED] = "Repositioned. Re-engaging.",
	[MSG_ID.REROUTE_SKIP_NO_TARGET] = "Reroute skipped: no target.",
	[MSG_ID.RESUMING_NAV] = "Resuming navigation.",
	[MSG_ID.SNAV_REROUTE_MINMAX] = "SNAV REROUTE_MIN {LEQ} REROUTE_MAX.",
	[MSG_ID.SNAV_RADIUS_NEGATIVE] = "SNAV_RADIUS_M cannot be negative.",
	[MSG_ID.AREA_INFO_RADIUS] = "Radius A=%.2fkm2;RD=%.0fm",
	[MSG_ID.SCRIPT_INITIALIZED] = "Script initialized.",
	[MSG_ID.SCRIPT_DISABLED] = "Script disabled by user.",
	[MSG_ID.SET_CRUISE_FBWB] = "Set Cruise/FBWB to resume SoarNav.",
	[MSG_ID.GCS_PREFIX] = "SoarNav: %s",
	[MSG_ID.MISSING_PARAM] = "SoarNav: Missing parameter: %s",
	[MSG_ID.PARAM_SKIP] = "SoarNav: SoarNav parameters skipped: %s",
	[MSG_ID.STICK_CMD_MANUAL_OFF] = "Stick CMD: Manual override OFF.",
	[MSG_ID.STICK_CMD_MANUAL_ON] = "Stick CMD: Manual override ON.",
	[MSG_ID.STICK_CMD_POLY_RECENTER] = "Stick CMD: Polygon re-centered.",
	[MSG_ID.STICK_CMD_RADIUS_RECENTER] = "Stick CMD: Radius Area re-centered.",
	[MSG_ID.STICK_CMD_STUCK] = "Stuck! Repositioning %.0fm, offset %.0f",
	[MSG_ID.THERMAL_EXIT_NO_DATA] = "Thermal exit: no data. (Lost Thermal)",
	[MSG_ID.THERMAL_FOUND_EXIT_FOCUS] = "Thermal found, exiting focus.",
	[MSG_ID.THERMAL_IGNORED_OOB] = "Thermal ignored: out of area.",
	[MSG_ID.POLY_FAILED_FALLBACK] = "snav%d.poly failed. Using fallback.",
	[MSG_ID.NAV_TARGET_SIMPLE] = "{SYM} [%s] %.0f{DEG}%s %.0fm",
	[MSG_ID.NAV_TARGET_CELLS] = "{SYM} [%s] %.0f{DEG}%s %.0fm %d/%d",
	[MSG_ID.NAV_TARGET_REROUTE] = "{SYM} [Re-route] %.0f{DEG}%s %.0fm %d/%d",
	[MSG_ID.RTL_STALL_OVERRIDE] = "RTL Stall: Force direct Home route @%dm",
	[MSG_ID.RTL_OVERRIDE_RESUME] = "RTL Override: In home area. Loitering.",
	[MSG_ID.NAV_TARGET_RIDGE] = "{SYM} [Ridge] %.0f{DEG}%s %.0fm s=%.2f",
	[MSG_ID.POLAR_LEARN_UPDATE] = "Polar Learn: CD0=%.4f B=%.4f",
}

local function msgf_id(id, ...)
	local t = MSG[id]
	if t == nil then
		t = string.format("<MISSING MSG %s>", tostring(id))
	end
	t = t:gsub("{SYM}", SYM):gsub("{DEG}", DEG):gsub("{LEQ}", LEQ)
	local ok, res = pcall(string.format, t, ...)
	if ok then
		return res
	else
		return t
	end
end

local function send_gcs_id(sev, id, ...)
	return gcs:send_text(sev, msgf_id(id, ...))
end
-- [[[ END GCS MESSAGES ]]]

-- Script operational states
local SCRIPT_STATE = {
	IDLE = 0,
	NAVIGATING = 1,
	PILOT_OVERRIDE = 2,
	THERMAL_PAUSE = 3,
	ERROR = 4,
	WAITING_FOR_ACTIVATION = 5,
}

local SoarNavWaypointSources = {
	THERMAL_MEMORY_DRIFT = "Thrm(Drift, +%.1f)",
	THERMAL_MEMORY_NODRIFT = "Thrm(NoDrift, +%.1f)",
	THERMAL_STREET = "Thermal Street",
	REENGAGE_FLYOUT = "Re-Engage FlyOut",
	REENGAGE_ENTRY = "Re-Engage ReEntry",
	FOCUS_MODE = "Focus (WP %d)",
	GRID_GUIDED = "Guided",
	GRID_PURE = "Pure",
	RIDGE = "Ridge",
	RANDOM_FALLBACK = "Random Fallback",
	REROUTE = "Re-route",
	UNKNOWN = "Unknown",
}

--[[
 // @Param: SNAV_ENABLE
 // @DisplayName: SoarNav Enable / Poly Selector
 // @Description: Enable script and select polygon. 0:Disabled, >0:Enabled. If RADIUS_M is 0, value 1-9 selects snavX.poly.
 // @Values: 0: Disable script. 1-9: Enable script and select the corresponding polygon file (snav1.poly, snav2.poly, etc.).
]]
--[[
 // @Param: SNAV_LOG_LVL
 // @DisplayName: SoarNav Log Verbosity
 // @Description: GCS log verbosity
 // @Values: 0:Silent,1:Events
]]
--[[
 // @Param: SNAV_RADIUS_M
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
 // @Description: Navigation P-gain for roll controller (higher is more responsive]]
--[[
 // @Param: SNAV_NAV_D
 // @DisplayName: Navigation D Gain
 // @Description: Navigation D-gain for roll controller (dampens response]]
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
 // @Values: 0:Disabled,1:Fully Linked,2:MIN Only (Capped]]
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

do
	local PARAM_TABLE_KEY = 111
	local PARAM_PREFIX = "SNAV_"

	local param_list = {
		{ name = "ENABLE", default = 0 },
		{ name = "LOG_LVL", default = 1 },
		{ name = "RADIUS_M", default = 500 },
		{ name = "ROLL_LIMIT", default = 30 },
		{ name = "WP_RADIUS", default = 50 },
		{ name = "NAV_P", default = 0.6 },
		{ name = "NAV_D", default = 0.05 },
		{ name = "TMEM_ENABLE", default = 1 },
		{ name = "TMEM_LIFE", default = 1200 },
		{ name = "STUCK_EFF", default = 0.35 },
		{ name = "STUCK_TIME", default = 30 },
		{ name = "REENG_DWELL", default = 7 },
		{ name = "RETRY_THR", default = 30 },
		{ name = "STREET_TOL", default = 30 },
		{ name = "REROUTE_P", default = 50 },
		{ name = "NEC_WEIGHT", default = 50 },
		{ name = "TMEM_MIN_S", default = 0.2 },
		{ name = "FOCUS_THR", default = 1.0 },
		{ name = "STRAT_HIST", default = 900 },
		{ name = "WP_TIMEOUT", default = 300 },
		{ name = "REROUTE_MIN", default = 80 },
		{ name = "REROUTE_MAX", default = 100 },
		{ name = "DYN_SOALT", default = 0 },
		{ name = "GC_MARGIN", default = 25 },
		{ name = "GC_PAD", default = 20 },
	}

	local function add_params()
		local ok, err = pcall(function()
			assert(
				param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, #param_list),
				string.format("ERROR: SoarNav - Could not add param table '%s'.", PARAM_PREFIX)
			)
			for i, p in ipairs(param_list) do
				assert(
					param:add_param(PARAM_TABLE_KEY, i, p.name, p.default),
					string.format("ERROR: SoarNav - Could not add param %s%s.", PARAM_PREFIX, p.name)
				)
			end
		end)
		if not ok then
			send_gcs_id(MAV_SEVERITY.WARNING, MSG_ID.PARAM_SKIP, tostring(err))
		end
	end

	add_params()
end

-- Single table for all parameter handles
local P = {}

local function pget(ph, default)
	if ph and ph.get then
		local ok, v = pcall(function()
			return ph:get()
		end)
		if ok and v ~= nil then
			return v
		end
	end
	return default
end

-- Helper function for safely binding parameters
local function bind_param(name)
	local ok, p_or_err = pcall(Parameter, name)
	if not ok then
		send_gcs_id(MAV_SEVERITY.WARNING, MSG_ID.MISSING_PARAM, name)
		return nil
	end
	return p_or_err
end

-- Binds all needed parameters (handles) and caches them for fast safe get/set.
local function bind_params()
	local PARAM_PREFIX = "SNAV_"
	P.enable = bind_param(PARAM_PREFIX .. "ENABLE")
	P.log_lvl = bind_param(PARAM_PREFIX .. "LOG_LVL")
	P.radius_m = bind_param(PARAM_PREFIX .. "RADIUS_M")
	P.nav_d = bind_param(PARAM_PREFIX .. "NAV_D")
	P.nav_p = bind_param(PARAM_PREFIX .. "NAV_P")
	P.rcmap_pitch = bind_param("RCMAP_PITCH")
	P.rcmap_roll = bind_param("RCMAP_ROLL")
	P.rcmap_yaw = bind_param("RCMAP_YAW")
	P.roll_limit = bind_param(PARAM_PREFIX .. "ROLL_LIMIT")
	P.soar_alt_max = bind_param("SOAR_ALT_MAX")
	P.soar_alt_min = bind_param("SOAR_ALT_MIN")
	P.tmem_enable = bind_param(PARAM_PREFIX .. "TMEM_ENABLE")
	P.tmem_life = bind_param(PARAM_PREFIX .. "TMEM_LIFE")
	P.wp_radius = bind_param(PARAM_PREFIX .. "WP_RADIUS")
	P.stuck_eff = bind_param(PARAM_PREFIX .. "STUCK_EFF")
	P.stuck_time = bind_param(PARAM_PREFIX .. "STUCK_TIME")
	P.reeng_dwell = bind_param(PARAM_PREFIX .. "REENG_DWELL")
	P.retry_thr = bind_param(PARAM_PREFIX .. "RETRY_THR")
	P.street_tol = bind_param(PARAM_PREFIX .. "STREET_TOL")
	P.reroute_p = bind_param(PARAM_PREFIX .. "REROUTE_P")
	P.nec_weight = bind_param(PARAM_PREFIX .. "NEC_WEIGHT")
	P.tmem_min_s = bind_param(PARAM_PREFIX .. "TMEM_MIN_S")
	P.focus_thr = bind_param(PARAM_PREFIX .. "FOCUS_THR")
	P.strat_hist = bind_param(PARAM_PREFIX .. "STRAT_HIST")
	P.wp_timeout = bind_param(PARAM_PREFIX .. "WP_TIMEOUT")
	P.reroute_min = bind_param(PARAM_PREFIX .. "REROUTE_MIN")
	P.reroute_max = bind_param(PARAM_PREFIX .. "REROUTE_MAX")
	P.soar_alt_cutoff = bind_param("SOAR_ALT_CUTOFF")
	P.soar_polar_cd0 = bind_param("SOAR_POLAR_CD0")
	P.soar_polar_b = bind_param("SOAR_POLAR_B")
	P.airspeed_cruise = bind_param("AIRSPEED_CRUISE")
	P.dyn_soalt = bind_param(PARAM_PREFIX .. "DYN_SOALT")
	P.gc_margin = bind_param(PARAM_PREFIX .. "GC_MARGIN")
	P.gc_pad = bind_param(PARAM_PREFIX .. "GC_PAD")
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
	RTLH_NAVT_HOME_TOL_M = 30,
	RTL_MODE_CODE = 11,
	GUIDED_MODE_CODE = 15,
	THROTTLE_SRV_CHANNEL = 70,
	ACTIVATION_OVERRIDE_GRACE_MS = 3000,
	RTLH_CHECK_DELAY_MS = 3000,

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
	HOTSPOT_EXPLORATION_RADIUS_M = 250,
	TACTICAL_REROUTE_MIN_PATH_DIVISOR = 4,

	-- Pilot Input & Stick Gestures
	PILOT_RESUME_DELAY_MS = 5000,
	ACTIVATION_GRACE_MS = 2000,
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
	grid_init_cells_per_call = 10,
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
	has_been_activated = false,
	activation_requested = false,
	activation_wait_notified = false,
	polar_est = {
		s11 = 0,
		s22 = 0,
		s12 = 0,
		y1 = 0,
		y2 = 0,
		n = 0,
		last_fit_ms = 0,
		last_sample_ms = 0,
		last_saved_cd0 = nil,
		last_saved_b = nil,
		last_commit_ms = 0,
		stable_count = 0,
		sink_bias = 0,
		err_ema = 0,
	},

	---------------------------------------------------------------------------
	-- 2. FLIGHT AREA & GRID SYSTEM
	---------------------------------------------------------------------------
	use_polygon_area = false,
	area_announced = false,
	max_operational_distance = 0,
	dynamic_center_location = nil,
	polygon_points = {},
	polygon_bounds = nil,
	polygon_centroid = nil,
	polygon_vertex_offsets = {},
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
	initial_soar_alt_min = nil,
	initial_soar_alt_max = nil,
	initial_soar_alt_cutoff = nil,
	last_armed_state = nil,
	restored_on_disarm = false,
	last_glide_cone_update_ms = nil,
	gcone_param_warning_sent = nil,
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
	require_initial_activation_gesture = true,

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
	last_snav_radius_m_value = -1,
	last_snav_enable_value = -1,
	last_rally_point_count = -1,
	rally_signature = nil,
	last_rally_poll_ms = nil,
	last_polar_update_ms = nil,
	using_rally_points = false,
	rtlh_en = true,
	RTLH = {
		active = false,
		engaged_guided = false,
		abort_until_next_rtl = false,
		t0_ms = 0,
		d0_m = 0,
		last_mode = nil,
	},

	---------------------------------------------------------------------------
	-- 7. RC CONTROL & HARDWARE INTERFACE
	---------------------------------------------------------------------------
	rc_roll_channel = rc:get_channel(pget(P.rcmap_roll, 1)),
	rc_limits_read = false,
	last_heading_error = 0,
	last_commanded_roll_deg = 0,
	rc_roll_min = 1000,
	rc_roll_max = 2000,
	rc_roll_trim = 1500,
}

-- #############################################################################
-- ## UTILITY FUNCTIONS (DEFINED EARLY TO PREVENT 'UNDEFINED GLOBAL' ERRORS)
-- #############################################################################

local set_target_safely

-- Safely retrieves a parameter value with optional default and min/max clamping.
local function get_safe_param(p, default, min, max)
	if not p then
		return default
	end
	local value = p:get()
	if value == nil then
		return default
	end
	if min and max then
		return math.min(max, math.max(min, value))
	end
	return value
end

-- Safely calculates the difference between two millis() timestamps.
local function safe_time_diff(newer, older)
	if not newer or not older then
		return 0
	end
	local function to_ms(x)
		if type(x) == "number" then
			return x
		end
		local ok, v = pcall(function()
			return x:toint()
		end)
		if ok and v then
			return v
		end
		return 0
	end
	local diff = to_ms(newer) - to_ms(older)
	return (diff >= 0) and diff or 0
end

-- Centralized function for sending formatted GCS messages.
local function log_gcs(sev, lvl, id, ...)
	local cur = (P.log_lvl and P.log_lvl:get()) or 1
	if cur >= lvl then
		send_gcs_id(sev, MSG_ID.GCS_PREFIX, msgf_id(id, ...))
	end
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

-- Helper function for Focus Mode to generate a random waypoint within a radius
local function generate_target_around_point(center_loc, radius_m)
	if not center_loc or not radius_m or radius_m <= 0 then
		return nil
	end
	local new_target = center_loc:copy()
	local bearing_deg = math.random() * 360
	local distance_m = math.sqrt(math.random()) * radius_m
	new_target:offset_bearing(bearing_deg, distance_m)
	return new_target
end

-- Calculates the required heading in degrees to reach a target, compensating for wind drift.
local function get_wind_corrected_heading_deg(current_loc, target_loc)
	local function wrap360(x)
		return (x % 360 + 360) % 360
	end
	local function wrap180(x)
		return (x + 540) % 360 - 180
	end

	if not current_loc or not target_loc then
		return 0
	end

	local bearing_deg = wrap360(math.deg(current_loc:get_bearing(target_loc)))
	local wind = get_wind_vector()
	local wind_mps = (wind and wind:length()) or 0
	local Va = (P.airspeed_cruise and P.airspeed_cruise:get()) or 15

	if wind_mps < 2.0 or Va <= 1.0 then
		return bearing_deg
	end

	local brg_rad = math.rad(bearing_deg)
	local t_n, t_e = math.cos(brg_rad), math.sin(brg_rad)
	local w_n, w_e = wind:x(), wind:y()
	local cw = (-w_e * t_n + w_n * t_e)
	local disc = Va * Va - cw * cw

	if disc >= 0 then
		local aw = (w_n * t_n + w_e * t_e)
		local s = aw + math.sqrt(disc)
		local a_n = s * t_n - w_n
		local a_e = s * t_e - w_e
		local hdg_wch = wrap360(math.deg(math.atan(a_e, a_n)))
		local wca = wrap180(hdg_wch - bearing_deg)
		local WCA_MAX = 30.0
		wca = math.max(-WCA_MAX, math.min(WCA_MAX, wca))
		return wrap360(bearing_deg + wca)
	end

	return bearing_deg
end

-- Converts a wind vector into the compass bearing (0-359 deg) from which it originates.
local function wind_vector_to_bearing_deg(w)
	local ang = (math.deg(w:xy():angle()) + 360) % 360
	return (450 - ang) % 360
end

-- Estimates ridge-lift suitability score at a location using local slope and wind alignment.
function SoarNavGlobals.ridge_score_at_loc(loc)
	if not loc or not terrain or not terrain:enabled() then
		return 0
	end
	local wind = get_wind_vector()
	if not wind then
		return 0
	end
	local wlen = wind:length() or 0
	if wlen < 2.0 then
		return 0
	end

	local dx = SoarNavGlobals.grid_cell_size_m
	if (not dx) or dx <= 0 then
		dx = (SoarNavConstants.min_cell_size_m or 50)
	end
	dx = math.max(30, math.min(dx, 120))

	local h0 = terrain:height_amsl(loc, true)
	if not h0 then
		return 0
	end
	local pE = loc:copy()
	pE:offset_bearing(90, dx)
	local pN = loc:copy()
	pN:offset_bearing(0, dx)
	local hE = terrain:height_amsl(pE, true)
	if not hE then
		return 0
	end
	local hN = terrain:height_amsl(pN, true)
	if not hN then
		return 0
	end

	local gx = (hE - h0) / dx
	local gy = (hN - h0) / dx
	local gnorm = math.sqrt(gx * gx + gy * gy)
	if gnorm < 0.05 then
		return 0
	end

	local ux, uy = gx / gnorm, gy / gnorm
	local wx, wy = -wind:x(), -wind:y()
	local wnorm = math.sqrt(wx * wx + wy * wy)
	if wnorm <= 1e-3 then
		return 0
	end
	wx, wy = wx / wnorm, wy / wnorm

	local facing = math.max(0, ux * wx + uy * wy)
	local slope = math.min(1.0, gnorm / 0.25)
	local windgain = math.min(1.0, (wlen or 0) / 8.0)

	local score = facing * (0.5 + 0.5 * slope) * windgain
	return score, ux, uy, wx, wy
end

-- Attempts to select a ridge-based waypoint with time budget; offsets upwind and aligns parallel to the ridge.
function SoarNavGlobals.try_ridge_target()
	if not SoarNavGlobals.grid_initialized then
		return false
	end
	if not SoarNavGlobals.grid_cell_centers then
		return false
	end
	if not terrain or not terrain:enabled() then
		return false
	end

	local now_ms = millis()
	local last_ms = SoarNavGlobals.ridge_last_ms or 0
	if (now_ms - last_ms) < 1500 then
		return false
	end

	local wind = get_wind_vector()
	if not wind or wind:length() < 2.0 then
		return false
	end

	local pool = SoarNavGlobals.valid_cell_indices or {}
	if not pool or #pool == 0 then
		return false
	end

	local max_eval = math.min(6, #pool)
	local evaluated = 0
	local best_s, best_loc = 0, nil
	local best_ux, best_uy = 0, 0
	local used = {}
	local t0 = millis()
	local tries, max_tries = 0, math.min(#pool, 24)

	while evaluated < max_eval and tries < max_tries do
		if (millis() - t0) > 3 then
			break
		end
		tries = tries + 1
		local idx = math.random(1, #pool)
		if not used[idx] then
			used[idx] = true
			local center = SoarNavGlobals.grid_cell_centers and SoarNavGlobals.grid_cell_centers[pool[idx]]
			local area_ok = false
			if center then
				local _ifa = in_flight_area
				local ok, res = false, false
				if _ifa then
					ok, res = pcall(_ifa, center)
				end
				area_ok = ok and res or false
			end
			if area_ok then
				local s1, ux, uy = SoarNavGlobals.ridge_score_at_loc(center)
				if s1 and s1 > best_s then
					best_s, best_loc = s1, center
					best_ux, best_uy = ux or 0, uy or 0
				end
				evaluated = evaluated + 1
			end
		end
	end

	if not best_loc or best_s < 0.15 then
		SoarNavGlobals.ridge_last_ms = now_ms
		return false
	end

	local target = best_loc:copy()
	local wp_r = (P.wp_radius and P.wp_radius:get()) or 50
	local upwind_offset = wp_r
	local wind_to_brg = (wind_vector_to_bearing_deg(wind) + 0) % 360
	target:offset_bearing(wind_to_brg, upwind_offset)

	local tx, ty = -best_uy, best_ux
	local tang_brg = math.deg(math.atan(tx, ty))
	if tang_brg < 0 then
		tang_brg = tang_brg + 360
	end

	local along = math.max(60, math.min(300, ((P.wp_radius and P.wp_radius:get()) or 50) * 1.5))
	target:offset_bearing(tang_brg, along)

	set_target_safely(target)
	SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.RIDGE

	local _dist, _hdg, _dir = 0, 0, ""
	local _cur = ahrs:get_location()
	if _cur then
		_dist = _cur:get_distance(target)
		_hdg = _cur:get_bearing(target)
		local __dirs =
			{ "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" }
		local __ix = math.floor((_hdg / 22.5) + 0.5) + 1
		if __ix > #__dirs then
			__ix = 1
		end
		_dir = __dirs[__ix]
	end

	log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.NAV_TARGET_RIDGE, _hdg, _dir, _dist, best_s)
	SoarNavGlobals.ridge_last_ms = now_ms
	return true
end

-- Checks if a Location object is inside the defined polygon using a ray-casting algorithm.
local function is_point_in_polygon(loc)
	if not loc or not SoarNavGlobals.polygon_bounds then
		return false
	end

	local bounds = SoarNavGlobals.polygon_bounds
	if not bounds then
		return false
	end
	local lat, lon = loc:lat(), loc:lng()

	if lat < bounds.min_lat or lat > bounds.max_lat or lon < bounds.min_lon or lon > bounds.max_lon then
		return false
	end

	local poly = SoarNavGlobals.polygon_xy
	local origin = SoarNavGlobals.polygon_origin
	if not poly or not origin or #poly < 3 then
		return false
	end

	local xy = origin:get_distance_NE(loc)
	local px = xy:x()
	local py = xy:y()

	local inside = false
	local j = #poly
	for i = 1, #poly do
		local xi, yi = poly[i].x, poly[i].y
		local xj, yj = poly[j].x, poly[j].y
		if ((yi > py) ~= (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi) then
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

-- Returns current AGL in meters relative to Home.
local function get_agl_m()
	local ned = ahrs:get_relative_position_NED_home()
	if not ned then
		return 100
	end
	return math.max(1, -ned:z())
end

-- Returns k_drift and r_mult coefficients based on thermal age.
local function lifecycle_coeff(age_s, tmem_life_s)
	local life = math.max(60, tmem_life_s or 1200)
	local g = math.min(180, life * 0.15)
	local m = math.min(600, life * 0.50)
	local d = math.max(60, life - g - m)
	if age_s < g then
		local u = age_s / g
		return 0.55 + 0.45 * u, 0.70 + 0.30 * u
	elseif age_s < g + m then
		return 1.00, 1.00
	else
		local u = math.min(1.0, (age_s - g - m) / d)
		return 1.00 + 0.50 * u, 1.00 + 0.60 * u
	end
end

-- Predicts the drifted location of a thermal based on wind and age.
local function predict_thermal_drift(loc, wind, age_sec)
	if not loc or not wind or wind:length() < 0.5 then
		return loc and loc:copy() or loc
	end
	local age = math.max(0, age_sec or 0)
	local agl = get_agl_m()
	local soar_min = pget(P.soar_alt_min, 0) or 0
	local soar_max = pget(P.soar_alt_max, 600) or 600
	local est_bl_top = math.max(300, math.min(1200, (soar_max - soar_min > 0) and (soar_max - soar_min) or 600))
	local alpha = 0.12
	local veer_deg = 8
	local Wc = wind:length()
	local Veff_factor
	if agl <= est_bl_top then
		Veff_factor = 1.0 / (1.0 + alpha)
	else
		local r = est_bl_top / agl
		Veff_factor = (1.0 - r) + (r ^ (alpha + 1)) / (1.0 + alpha)
	end
	local k_age, _ = lifecycle_coeff(age, pget(P.tmem_life, 1200))
	local drift_speed = Wc * Veff_factor * k_age
	local drift_dist = drift_speed * age
	local wind_bearing_deg = wind_vector_to_bearing_deg(wind)
	local veer_scale = (math.min(1.0, agl / est_bl_top)) ^ 0.7
	local drift_bearing = (wind_bearing_deg + 180 + veer_deg * veer_scale) % 360
	local out = loc:copy()
	out:offset_bearing(drift_bearing, drift_dist)
	return out
end

-- Checks if a given location is within the defined flight area (polygon or radius).
in_flight_area = function(loc)
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
		return center:get_distance(loc) <= pget(P.radius_m, 0)
	end
end

-- Converts a Location to local XY (meters) relative to the polygon origin.
local function loc_to_xy(loc)
	local origin = SoarNavGlobals.polygon_origin
	if not origin or not loc then
		return nil, nil
	end
	local v = origin:get_distance_NE(loc)
	return v and v:x() or nil, v and v:y() or nil
end

-- Builds a Location from local XY (meters) offsets relative to the polygon origin.
local function xy_to_loc(x, y)
	local origin = SoarNavGlobals.polygon_origin
	if not origin then
		return nil
	end
	local tmp = origin:copy()
	tmp:offset_bearing(90, x or 0)
	tmp:offset_bearing(0, y or 0)
	return tmp
end

-- Computes the closest point to P on segment AB in XY and returns qx, qy, t.
local function nearest_on_seg(px, py, ax, ay, bx, by)
	local abx, aby = (bx - ax), (by - ay)
	local apx, apy = (px - ax), (py - ay)
	local ab2 = abx * abx + aby * aby
	local t = ab2 > 0 and math.max(0, math.min(1, (apx * abx + apy * aby) / ab2)) or 0
	return ax + t * abx, ay + t * aby, t
end

-- If outside, projects the point just inside the polygon along the nearest edge, using optional padding (m).
local function clamp_inside_polygon(loc, pad_m)
	if not SoarNavGlobals.use_polygon_area then
		return loc
	end
	if not loc then
		return loc
	end
	if in_flight_area(loc) then
		return loc
	end
	local px, py = loc_to_xy(loc)
	if not px then
		return loc
	end
	local poly = SoarNavGlobals.polygon_xy
	if not poly or #poly < 3 then
		return loc
	end
	local best_dx, best_dy, best_d = 0, 0, math.huge
	local n = #poly
	for i = 1, n do
		local j = (i % n) + 1
		local ax, ay = poly[i].x, poly[i].y
		local bx, by = poly[j].x, poly[j].y
		local qx, qy = nearest_on_seg(px, py, ax, ay, bx, by)
		local dx, dy = px - qx, py - qy
		local d = dx * dx + dy * dy
		if d < best_d then
			best_d = d
			best_dx, best_dy = dx, dy
		end
	end
	local len = math.sqrt(best_d)
	local nx, ny = 0, 0
	if len > 1e-6 then
		nx, ny = best_dx / len, best_dy / len
	end
	local pad = math.max(0, pad_m or 0)
	local rx, ry = px + nx * (pad + 1.0), py + ny * (pad + 1.0)
	local res = xy_to_loc(rx, ry)
	if res and in_flight_area(res) then
		return res
	end
	return loc
end

-- Returns true if the XY polygon is convex (no sign change in successive edge cross-products).
local function is_convex(poly)
	local n = poly and #poly or 0
	if n < 4 then
		return true
	end
	local sign = 0
	for i = 1, n do
		local i1 = i
		local i2 = (i % n) + 1
		local i3 = (i2 % n) + 1
		local dx1 = poly[i2].x - poly[i1].x
		local dy1 = poly[i2].y - poly[i1].y
		local dx2 = poly[i3].x - poly[i2].x
		local dy2 = poly[i3].y - poly[i2].y
		local z = dx1 * dy2 - dy1 * dx2
		if z ~= 0 then
			local s = (z > 0) and 1 or -1
			if sign == 0 then
				sign = s
			elseif s ~= sign then
				return false
			end
		end
	end
	return true
end

-- Precomputes polygon_edges (ax,ay,bx,by + per-edge AABBs) and sets polygon_is_convex for fast segment checks.
local function build_polygon_edges()
	local poly = SoarNavGlobals.polygon_xy
	local n = poly and #poly or 0
	local edges = {}
	for i = 1, n do
		local j = (i % n) + 1
		local ax, ay = poly[i].x, poly[i].y
		local bx, by = poly[j].x, poly[j].y
		local minx = (ax < bx) and ax or bx
		local maxx = (ax > bx) and ax or bx
		local miny = (ay < by) and ay or by
		local maxy = (ay > by) and ay or by
		edges[i] = { ax = ax, ay = ay, bx = bx, by = by, minx = minx, maxx = maxx, miny = miny, maxy = maxy }
	end
	SoarNavGlobals.polygon_edges = edges
	SoarNavGlobals.polygon_is_convex = is_convex(poly)
end

-- Checks if the current→target segment stays inside the polygon using XY edge intersections with bbox culling and a short-lived cache.
local function segment_stays_inside(current, target)
	if not SoarNavGlobals.use_polygon_area then
		return true
	end
	if not (current and target) then
		return true
	end
	local dist = current:get_distance(target)
	local skip_dist = math.max(120, ((P and P.wp_radius and P.wp_radius:get()) or 50) * 4)
	if dist < skip_dist then
		return true
	end
	local cache = SoarNavGlobals._seg_cache or {}
	local now = millis()
	local clat = current:lat()
	local clon = current:lng()
	local tlat = target:lat()
	local tlon = target:lng()
	if
		cache.clat == clat
		and cache.clon == clon
		and cache.tlat == tlat
		and cache.tlon == tlon
		and cache.ms
		and (now - cache.ms) < 400
	then
		return cache.res
	end
	if not in_flight_area(current) or not in_flight_area(target) then
		SoarNavGlobals._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = false }
		return false
	end
	if SoarNavGlobals.polygon_is_convex then
		SoarNavGlobals._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
		return true
	end
	local cx, cy = loc_to_xy(current)
	local tx, ty = loc_to_xy(target)
	if not cx or not tx then
		SoarNavGlobals._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
		return true
	end
	local edges = SoarNavGlobals.polygon_edges or {}
	local den, u, v
	for i = 1, #edges do
		local e = edges[i]
		local ax, ay, bx, by = e.ax, e.ay, e.bx, e.by
		if
			not (
				(math.max(cx or 0, tx or 0) < (e.minx or 0))
				or (math.min(cx or 0, tx or 0) > (e.maxx or 0))
				or (math.max(cy or 0, ty or 0) < (e.miny or 0))
				or (math.min(cy or 0, ty or 0) > (e.maxy or 0))
			)
		then
			den = (bx - ax) * (ty - cy) - (by - ay) * (tx - cx)
			if math.abs(den) > 1e-9 then
				u = ((cx - ax) * (ty - cy) - (cy - ay) * (tx - cx)) / den
				v = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / den
				if u >= 0 and u <= 1 and v >= 0 and v <= 1 then
					SoarNavGlobals._seg_cache =
						{ clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = false }
					return false
				end
			end
		end
	end
	SoarNavGlobals._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
	return true
end

-- Pulls the target inward (or clamps) so the current->target segment remains inside the polygon.
local function adjust_target_segment(current, target)
	if segment_stays_inside(current, target) then
		return target
	end
	local cx, cy = loc_to_xy(current)
	local tx, ty = loc_to_xy(target)
	if not cx then
		return target
	end
	local ks = { 0.8, 0.6, 0.4, 0.2 }
	for _, k in ipairs(ks) do
		local nx = cx + k * (tx - cx)
		local ny = cy + k * (ty - cy)
		local cand = xy_to_loc(nx, ny)
		if cand and in_flight_area(cand) and segment_stays_inside(current, cand) then
			return cand
		end
	end
	local pad = (P and P.wp_radius and P.wp_radius:get()) or 50
	local clamped = clamp_inside_polygon(target, pad)
	if segment_stays_inside(current, clamped) then
		return clamped
	end
	return target
end

-- Central setter: clamps candidate inside, enforces an in-polygon path from current, then sets SoarNavGlobals.target_loc.
set_target_safely = function(candidate_loc)
	if not candidate_loc then
		return false
	end
	if not SoarNavGlobals.use_polygon_area then
		SoarNavGlobals.target_loc = candidate_loc
		return true
	end
	local pad = (P and P.wp_radius and P.wp_radius:get()) or 50
	local target = candidate_loc
	if not in_flight_area(target) then
		target = clamp_inside_polygon(target, pad * 0.5)
	end
	local cur = ahrs:get_location()
	if cur and in_flight_area(cur) then
		if not segment_stays_inside(cur, target) then
			target = adjust_target_segment(cur, target)
		end
	end
	SoarNavGlobals.target_loc = target
	return true
end

-- Safely decrements a number, ensuring it does not go below zero.
local function safe_decrement(n)
	return math.max(0, n - 1)
end

-- Validates key SNAV_* parameters to ensure they are within safe and logical ranges.
local function validate_params()
	local ok = true

	local function clamp_warn(name, val, lo, hi, msg_id)
		if val == nil then
			return
		end
		if (lo and val < lo) or (hi and val > hi) then
			log_gcs(MAV_SEVERITY.WARNING, 1, msg_id, name, tostring(lo), tostring(hi), val)
		end
	end

	local radius_m = (P.radius_m and pget(P.radius_m, 0) or 0)
	if radius_m < 0 then
		log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.SNAV_RADIUS_NEGATIVE)
		ok = false
	end
	if P.reroute_min and P.reroute_max then
		local rmin = P.reroute_min:get()
		local rmax = P.reroute_max:get()
		if rmin > rmax then
			log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.SNAV_REROUTE_MINMAX)
			ok = false
		end
	end

	clamp_warn("SNAV_ROLL_LIMIT", P.roll_limit:get(), 10, 60, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_WP_RADIUS", P.wp_radius:get(), 10, 300, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NAV_P", P.nav_p:get(), 0.05, 2.0, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NAV_D", P.nav_d:get(), 0.0, 1.0, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STUCK_EFF", P.stuck_eff:get(), 0.1, 1.0, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STUCK_TIME", P.stuck_time:get(), 5, 120, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_REENG_DWELL", P.reeng_dwell:get(), 0, 120, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_RETRY_THR", P.retry_thr:get(), 0, 100, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STREET_TOL", P.street_tol:get(), 5, 90, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_REROUTE_P", P.reroute_p:get(), 0, 100, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NEC_WEIGHT", P.nec_weight:get(), 0, 100, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_TMEM_MIN_S", P.tmem_min_s:get(), 0.0, 5.0, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_FOCUS_THR", P.focus_thr:get(), 0.0, 5.0, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STRAT_HIST", P.strat_hist:get(), 60, 3600, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_WP_TIMEOUT", P.wp_timeout:get(), 30, 900, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_DYN_SOALT", P.dyn_soalt:get(), 0, 2, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_GC_MARGIN", P.gc_margin:get(), 0, 150, MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_GC_PAD", P.gc_pad:get(), 0, 100, MSG_ID.PARAM_OUT_OF_RANGE)

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
local function update_dynamic_soar_alt_min()
	if SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE or SoarNavGlobals.manual_override_active then
		return
	end
	local override_active = (SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE)
		or (SoarNavGlobals.manual_override_active == true)
	if override_active then
		SoarNavGlobals.gc_reset_hold_start_ms = nil
		if not SoarNavGlobals.override_reset_done then
			local init_min = SoarNavGlobals.initial_soar_alt_min
			local init_cutoff = SoarNavGlobals.initial_soar_alt_cutoff
			local init_max = SoarNavGlobals.initial_soar_alt_max
			if init_min and init_cutoff and init_max and P.soar_alt_min and P.soar_alt_cutoff and P.soar_alt_max then
				local cur_min = P.soar_alt_min:get()
				local cur_cut = P.soar_alt_cutoff:get()
				local cur_max = P.soar_alt_max:get()
				local changed = false
				if cur_min ~= init_min then
					P.soar_alt_min:set(init_min)
					changed = true
				end
				if cur_cut ~= init_cutoff then
					P.soar_alt_cutoff:set(init_cutoff)
					changed = true
				end
				if cur_max ~= init_max then
					P.soar_alt_max:set(init_max)
					changed = true
				end
				if changed then
					log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.GCONE_RST_OVERRIDE, init_min, init_cutoff, init_max)
				end
			end
			SoarNavGlobals.override_reset_done = true
		end
		return
	else
		SoarNavGlobals.override_reset_done = nil
	end

	if
		not P.dyn_soalt
		or not P.soar_polar_b
		or not P.soar_polar_cd0
		or not P.airspeed_cruise
		or not P.soar_alt_min
		or not P.soar_alt_max
		or not P.soar_alt_cutoff
		or not P.gc_margin
		or not P.gc_pad
	then
		return
	end

	local initial_min = SoarNavGlobals.initial_soar_alt_min
	local initial_cutoff = SoarNavGlobals.initial_soar_alt_cutoff
	local initial_max = SoarNavGlobals.initial_soar_alt_max
	local current_min = P.soar_alt_min:get()
	if not initial_min or not initial_cutoff or not initial_max or not current_min then
		return
	end

	local dyn_soalt_mode = math.floor((P.dyn_soalt:get() or 0) + 0.5)
	if dyn_soalt_mode == 0 then
		return
	end

	local current_loc = ahrs:get_location()
	local home_location_raw = ahrs:get_home()
	if not current_loc or not home_location_raw then
		return
	end

	local target_home_loc = Location()
	target_home_loc:lat(home_location_raw:lat())
	target_home_loc:lng(home_location_raw:lng())
	target_home_loc:alt(home_location_raw:alt())

	local ned_pos = ahrs:get_relative_position_NED_home()
	if not ned_pos then
		return
	end
	local current_alt = -ned_pos:z()
	local now = millis()

	local GCONE_RESET_SURPLUS_M = 50
	local RESET_HOLD_MS = 20000

	local polar_b = P.soar_polar_b:get()
	local polar_cd0 = P.soar_polar_cd0:get()
	local best_glide_airspeed = P.airspeed_cruise:get()
	if
		not polar_b
		or polar_b <= 0
		or polar_b ~= polar_b
		or not polar_cd0
		or polar_cd0 <= 0
		or polar_cd0 ~= polar_cd0
		or not best_glide_airspeed
		or best_glide_airspeed <= 0
		or best_glide_airspeed ~= best_glide_airspeed
	then
		if not SoarNavGlobals.gcone_param_warning_sent then
			log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.GCONE_OFF_NO_PARAMS)
			SoarNavGlobals.gcone_param_warning_sent = true
		end
		return
	end

	local efficiency_max = 1 / math.sqrt(4 * polar_cd0 * polar_b)
	if not efficiency_max or efficiency_max <= 0 or efficiency_max ~= efficiency_max or efficiency_max == 1 / 0 then
		return
	end

	local vec_to_home = current_loc:get_distance_NE(target_home_loc)
	if not vec_to_home then
		return
	end
	local wind_vec = get_wind_vector()
	local wind_2d = wind_vec and wind_vec:xy() or nil
	local wind_comp = 0
	if wind_2d and vec_to_home:length() > 0 then
		wind_comp = (vec_to_home:x() * wind_2d:x() + vec_to_home:y() * wind_2d:y()) / vec_to_home:length()
		if not wind_comp or wind_comp ~= wind_comp or wind_comp == 1 / 0 or wind_comp == -1 / 0 then
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
	if dist_home < 0 then
		dist_home = 0
	end
	local required_alt_glide = dist_home / eff_ld
	if not required_alt_glide or required_alt_glide < 0 or required_alt_glide ~= required_alt_glide then
		required_alt_glide = 0
	end

	local pad = P.gc_pad:get() or 0
	local margin = P.gc_margin:get() or 0
	local calculated_min_alt_glide = required_alt_glide + pad + margin

	local required_terrain_min_alt = 0
	if terrain and terrain:enabled() then
		local cur_ter, _ = terrain:height_amsl(current_loc, true)
		local home_ter, _ = terrain:height_amsl(target_home_loc, true)

		if cur_ter and home_ter then
			local terrain_delta = cur_ter - home_ter
			required_terrain_min_alt = initial_min + terrain_delta
		end
	end

	local calculated_min_alt = math.max(initial_min, calculated_min_alt_glide, required_terrain_min_alt)

	local target_min
	local target_cutoff = initial_cutoff
	local target_max = initial_max

	if current_alt > (calculated_min_alt + GCONE_RESET_SURPLUS_M) then
		if not SoarNavGlobals.gc_reset_hold_start_ms then
			SoarNavGlobals.gc_reset_hold_start_ms = now
			return
		elseif safe_time_diff(now, SoarNavGlobals.gc_reset_hold_start_ms) >= RESET_HOLD_MS then
			if current_alt > (initial_max + 10) then
				SoarNavGlobals.gc_reset_hold_start_ms = nil
				return
			end
			target_min = initial_min
			target_cutoff = initial_cutoff
			target_max = initial_max
			SoarNavGlobals.gc_reset_hold_start_ms = nil
		else
			return
		end
	else
		SoarNavGlobals.gc_reset_hold_start_ms = nil
		target_min = math.floor(calculated_min_alt)
		target_min = math.floor((target_min + 5) / 10) * 10
		if dyn_soalt_mode == 1 then
			local link_threshold = math.floor(initial_cutoff * (2.0 / 3.0))
			if target_min > link_threshold then
				local delta = target_min - initial_min
				target_cutoff = initial_cutoff + delta
				target_max = initial_max + delta
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

	local hysteresis_up = 20
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
		local changed = false
		if P.soar_alt_min:get() ~= target_min then
			P.soar_alt_min:set(target_min)
			changed = true
		end
		if P.soar_alt_cutoff:get() ~= target_cutoff then
			P.soar_alt_cutoff:set(target_cutoff)
			changed = true
		end
		if P.soar_alt_max:get() ~= target_max then
			P.soar_alt_max:set(target_max)
			changed = true
		end

		if changed then
			local alt_range = SoarNavGlobals.initial_soar_alt_max - SoarNavGlobals.initial_soar_alt_min
			if alt_range > 0 then
				local warning_margin_m = alt_range / 10.0
				if current_alt < (target_min + warning_margin_m) then
					log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.GCONE_ALT_UPDATE, target_min)
				end
			else
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.GCONE_ALT_UPDATE, target_min)
			end
		end
	end
end

-- Dynamically learns the aircraft's glide performance to improve efficiency and range calculations.
function update_polar_learning()
	local pe = SoarNavGlobals.polar_est
	if not pe then
		pe = {
			s11 = 0,
			s22 = 0,
			s12 = 0,
			y1 = 0,
			y2 = 0,
			n = 0,
			last_fit_ms = 0,
			last_sample_ms = 0,
			last_commit_ms = 0,
			last_saved_cd0 = nil,
			last_saved_b = nil,
			sink_bias = 0,
			err_ema = 0,
			stable_count = 0,
			first_sample_ms = 0,
			v_ema = 0,
			v2_ema = 0,
			s_mag_log_ema = 0,
		}
		SoarNavGlobals.polar_est = pe
	end

	local throttle = SRV_Channels:get_output_scaled(SoarNavConstants.THROTTLE_SRV_CHANNEL) or 0
	if throttle > SoarNavConstants.THROTTLE_ACTIVE_THRESHOLD then
		return
	end

	local roll_deg = math.abs(ahrs:get_roll_rad() * (180 / math.pi))
	if roll_deg > 12 then
		return
	end

	local vned = ahrs:get_velocity_NED()
	if not vned then
		return
	end

	local wind = get_wind_vector()
	local va_n = (vned:x() or 0) - ((wind and wind:x()) or 0)
	local va_e = (vned:y() or 0) - ((wind and wind:y()) or 0)
	local v = math.sqrt(va_n * va_n + va_e * va_e)
	if not v or v <= 8 then
		return
	end

	local sink_raw = vned:z()
	if sink_raw < 0.10 then
		return
	end

	local now = millis()
	if not pe.first_sample_ms or pe.first_sample_ms == 0 then
		pe.first_sample_ms = now
	end
	local dt_ms = safe_time_diff(now, (pe.last_sample_ms or 0))
	pe.last_sample_ms = now

	if dt_ms > 0 then
		local k = 1 - math.exp(-dt_ms / 60000)
		pe.sink_bias = (pe.sink_bias or 0) + k * (sink_raw - (pe.sink_bias or 0))
		pe.v_ema = (pe.v_ema or v) + k * (v - (pe.v_ema or v))
		pe.v2_ema = (pe.v2_ema or v * v) + k * (v * v - (pe.v2_ema or v * v))
	end
	local sink = sink_raw - (pe.sink_bias or 0)

	local tau = 120000
	if dt_ms > 0 then
		local decay = math.exp(-dt_ms / tau)
		pe.s11 = (pe.s11 or 0) * decay
		pe.s22 = (pe.s22 or 0) * decay
		pe.s12 = (pe.s12 or 0) * decay
		pe.y1 = (pe.y1 or 0) * decay
		pe.y2 = (pe.y2 or 0) * decay
		pe.n = (pe.n or 0) * decay
	end

	local f1 = v * v * v
	local f2 = 1 / v
	pe.s11 = (pe.s11 or 0) + f1 * f1
	pe.s22 = (pe.s22 or 0) + f2 * f2
	pe.s12 = (pe.s12 or 0) + f1 * f2
	pe.y1 = (pe.y1 or 0) + f1 * sink
	pe.y2 = (pe.y2 or 0) + f2 * sink
	pe.n = (pe.n or 0) + 1

	if safe_time_diff(now, pe.first_sample_ms or 0) < 60000 then
		return
	end
	if (pe.n or 0) < 150 then
		return
	end
	if pe.last_fit_ms and safe_time_diff(now, pe.last_fit_ms) < 8000 then
		return
	end
	pe.last_fit_ms = now

	local cd0 = P.soar_polar_cd0:get()
	local bb = P.soar_polar_b:get()
	if not pe.last_saved_cd0 then
		pe.last_saved_cd0 = cd0
	end
	if not pe.last_saved_b then
		pe.last_saved_b = bb
	end

	local v_var = math.max(0, (pe.v2_ema or 0) - (pe.v_ema or 0) ^ 2)
	local v_std = math.sqrt(v_var)
	local vc = (P.airspeed_cruise and P.airspeed_cruise:get()) or 15
	if not vc or vc <= 0 then
		vc = 15
	end

	if v_std < 0.8 then
		local sink_model = (cd0 * v * v + bb) / v
		if not sink_model or sink_model <= 0 then
			return
		end
		local s_mag_inst = sink / sink_model
		if s_mag_inst < 0.90 then
			s_mag_inst = 0.90
		end
		if s_mag_inst > 1.10 then
			s_mag_inst = 1.10
		end
		pe.s_mag_log_ema = (pe.s_mag_log_ema or 0) + 0.2 * (math.log(s_mag_inst) - (pe.s_mag_log_ema or 0))
		local s_mag = math.exp(pe.s_mag_log_ema or 0)
		local new_cd0 = cd0 * s_mag
		local new_b = bb * s_mag
		if new_b < 0.005 then
			new_b = 0.005
		end
		if new_b > 0.060 then
			new_b = 0.060
		end
		if new_cd0 < 0.005 then
			new_cd0 = 0.005
		end
		if new_cd0 > 0.500 then
			new_cd0 = 0.500
		end
		local new_cd0_q = math.floor(new_cd0 * 10000 + 0.5) / 10000
		local new_b_q = math.floor(new_b * 10000 + 0.5) / 10000
		local is_first_commit = (pe.last_commit_ms == 0)
		local rl_ok = is_first_commit or (safe_time_diff(now, pe.last_commit_ms) >= 90000)
		local rel_cd0 = math.abs(new_cd0_q - pe.last_saved_cd0) / pe.last_saved_cd0
		local rel_b = math.abs(new_b_q - pe.last_saved_b) / pe.last_saved_b
		local sig = is_first_commit or (rel_cd0 >= 0.03) or (rel_b >= 0.06)
		if sig and rl_ok then
			pe.stable_count = (pe.stable_count or 0) + 1
		else
			pe.stable_count = 0
		end
		if pe.stable_count >= 3 then
			P.soar_polar_cd0:set(new_cd0_q)
			P.soar_polar_b:set(new_b_q)
			pe.last_commit_ms = now
			pe.last_saved_cd0 = new_cd0_q
			pe.last_saved_b = new_b_q
			pe.stable_count = 0
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.POLAR_LEARN_UPDATE, new_cd0_q, new_b_q)
		end
		return
	end

	local det = (pe.s11 or 0) * (pe.s22 or 0) - (pe.s12 or 0) * (pe.s12 or 0)
	if not det or det <= 1e-9 then
		return
	end

	local a_est = ((pe.y1 or 0) * (pe.s22 or 0) - (pe.y2 or 0) * (pe.s12 or 0)) / det
	local b_est = ((pe.y2 or 0) * (pe.s11 or 0) - (pe.y1 or 0) * (pe.s12 or 0)) / det
	if not a_est or not b_est or a_est <= 0 or b_est <= 0 or a_est ~= a_est or b_est ~= b_est then
		return
	end

	local pred = a_est * (v ^ 3) + b_est * (1 / v)
	local e = math.abs(sink - pred)
	pe.err_ema = (pe.err_ema == 0) and e or (0.9 * pe.err_ema + 0.1 * e)
	if pe.err_ema > 0.6 then
		return
	end

	local r_est = b_est / a_est
	local r_cur = bb / cd0
	if not r_est or not r_cur or r_est <= 0 or r_cur <= 0 then
		return
	end

	local h = math.sqrt(r_est / r_cur)
	if h < 0.90 then
		h = 0.90
	end
	if h > 1.10 then
		h = 1.10
	end
	local cd0_shape = cd0 / h
	local b_shape = bb * h

	local denom = a_est * vc * vc + b_est / (vc * vc)
	if not denom or denom <= 0 or denom ~= denom then
		return
	end
	local eff = 1 / denom
	if not eff or eff <= 0 or eff ~= eff then
		return
	end

	local target_prod = 1 / (4 * eff * eff)
	local prod_shape = cd0_shape * b_shape
	if not prod_shape or prod_shape <= 0 or not target_prod or target_prod <= 0 then
		return
	end

	local s_prod = math.sqrt(target_prod / prod_shape)
	if s_prod < 0.90 then
		s_prod = 0.90
	end
	if s_prod > 1.15 then
		s_prod = 1.15
	end

	local new_cd0 = cd0_shape * s_prod
	local new_b = b_shape * s_prod
	if new_b < 0.005 then
		new_b = 0.005
	end
	if new_b > 0.060 then
		new_b = 0.060
	end
	if new_cd0 < 0.005 then
		new_cd0 = 0.005
	end
	if new_cd0 > 0.500 then
		new_cd0 = 0.500
	end

	local new_cd0_q = math.floor(new_cd0 * 1000 + 0.5) / 1000
	local new_b_q = math.floor(new_b * 1000 + 0.5) / 1000

	local is_first_commit = (pe.last_commit_ms == 0)
	local rl_ok = is_first_commit or (safe_time_diff(now, pe.last_commit_ms) >= 90000)
	local rel_cd0 = math.abs(new_cd0_q - pe.last_saved_cd0) / pe.last_saved_cd0
	local rel_b = math.abs(new_b_q - pe.last_saved_b) / pe.last_saved_b
	local sig = is_first_commit or (rel_cd0 >= 0.03) or (rel_b >= 0.06)

	if sig and rl_ok then
		pe.stable_count = (pe.stable_count or 0) + 1
	else
		pe.stable_count = 0
	end

	if pe.stable_count >= 3 then
		P.soar_polar_cd0:set(new_cd0_q)
		P.soar_polar_b:set(new_b_q)
		pe.last_commit_ms = now
		pe.last_saved_cd0 = new_cd0_q
		pe.last_saved_b = new_b_q
		pe.stable_count = 0
		send_gcs_id(MAV_SEVERITY.INFO, MSG_ID.POLAR_LEARN_UPDATE, new_cd0_q, new_b_q)
	end
end

-- Checks if a waypoint source is a thermal or thermal street.
local function is_thermal_target(source_info)
	local src = tostring(source_info)
	return string.find(src, "Thrm") or (src == SoarNavWaypointSources.THERMAL_STREET)
end

-- Announces area details for polygon mode (km², vertex count, max distance).
function announce_polygon_area()
	local poly = SoarNavGlobals.polygon_xy
	if not poly or #poly < 3 then
		return
	end
	local n = #poly
	local same_first_last = (poly[1].x == poly[n].x) and (poly[1].y == poly[n].y)
	local m = same_first_last and (n - 1) or n
	local acc = 0
	for i = 1, m do
		local j = (i % m) + 1
		acc = acc + poly[i].x * poly[j].y - poly[j].x * poly[i].y
	end
	local area_m2 = math.abs(acc * 0.5)
	local md_m = 0
	for i = 1, m do
		for j = i + 1, m do
			local dx = poly[i].x - poly[j].x
			local dy = poly[i].y - poly[j].y
			local d = math.sqrt(dx * dx + dy * dy)
			if d > md_m then
				md_m = d
			end
		end
	end
	local area_km2 = area_m2 * 1.0e-6

	local prefix = SoarNavGlobals.using_rally_points and "Rally" or "Poly"
	log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.AREA_INFO_POLY, prefix, area_km2, m, md_m)
	SoarNavGlobals.area_announced = true
end

-- Announces area details for radius mode (km² and radius).
function announce_radius_area()
	local rad_m = P.radius_m and pget(P.radius_m, 0) or 0
	if rad_m > 0 then
		local area_km2 = (math.pi * rad_m * rad_m) * 1.0e-6
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.AREA_INFO_RADIUS, area_km2, rad_m)
		SoarNavGlobals.area_announced = true
	end
end

-- Precomputes EN-metric polygon cache and origin from polygon points.
local function prepare_polygon_xy_cache()
	if #SoarNavGlobals.polygon_points < 3 then
		log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.POLY_INVALID_POINTS)
		return
	end
	local poly = SoarNavGlobals.polygon_points
	if not poly or #poly < 3 then
		return
	end

	local origin = poly[1]:copy()
	SoarNavGlobals.polygon_origin = origin
	SoarNavGlobals.polygon_xy = {}

	for i = 1, #poly do
		local xy = origin:get_distance_NE(poly[i])
		SoarNavGlobals.polygon_xy[i] = { x = xy:x(), y = xy:y() }
	end

	build_polygon_edges()
end

-- #############################################################################
-- ## CORE SCRIPT LOGIC
-- #############################################################################

-- Assesses the aircraft's energy state (NORMAL, LOW, CRITICAL) based on altitude.
local function assess_energy_state()
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	if not dist_from_home_3d then
		return "UNKNOWN"
	end

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

	local min_alt = P.soar_alt_min:get() or 100
	local max_alt = P.soar_alt_max:get() or 400
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
		SoarNavGlobals.filtered_alt_factor = SoarNavGlobals.filtered_alt_factor
			+ (raw_alt_factor - SoarNavGlobals.filtered_alt_factor) * (1 - math.exp(-dt_filter * filter_alpha))
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
		if
			safe_time_diff(now, SoarNavGlobals.negative_trend_start_ms)
				>= SoarNavConstants.CRITICAL_TREND_HYSTERESIS_MS
			and new_state ~= "CRITICAL"
		then
			new_state = "CRITICAL"
		end
	else
		SoarNavGlobals.negative_trend_start_ms = nil
	end

	if SoarNavGlobals.energy_state ~= new_state then
		SoarNavGlobals.energy_state = new_state
	end

	return SoarNavGlobals.energy_state
end

-- Calculates the statistical variance of climb rate samples to determine thermal consistency.
local function calculate_thermal_variance(samples)
	if not samples or #samples < 2 then
		return 0
	end
	local sum = 0
	for _, val in ipairs(samples) do
		sum = sum + val
	end
	local mean = sum / #samples
	local variance_sum = 0
	for _, val in ipairs(samples) do
		variance_sum = variance_sum + (val - mean) ^ 2
	end
	return variance_sum / #samples
end

local function restore_boot_alts(_)
	if
		SoarNavGlobals
		and SoarNavGlobals.initial_soar_alt_min
		and SoarNavGlobals.initial_soar_alt_max
		and SoarNavGlobals.initial_soar_alt_cutoff
	then
		if P.soar_alt_min then
			P.soar_alt_min:set(SoarNavGlobals.initial_soar_alt_min)
		end
		if P.soar_alt_max then
			P.soar_alt_max:set(SoarNavGlobals.initial_soar_alt_max)
		end
		if P.soar_alt_cutoff then
			P.soar_alt_cutoff:set(SoarNavGlobals.initial_soar_alt_cutoff)
		end
		SoarNavGlobals.last_glide_cone_update_ms = nil
	end
end

-- Manages script state transitions and associated cleanup actions.
function set_script_state(new_state, reason_id, ...)
	if SoarNavGlobals.script_state ~= new_state then
		local prev_state = SoarNavGlobals.script_state
		if new_state == SCRIPT_STATE.PILOT_OVERRIDE then
			restore_boot_alts("Pilot Override")
		end

		if new_state == SCRIPT_STATE.IDLE and prev_state ~= SCRIPT_STATE.THERMAL_PAUSE then
			restore_boot_alts("Entering IDLE")
		end

		if new_state == SCRIPT_STATE.IDLE and SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
			SoarNavGlobals.pitch_gesture_state = "idle"
			SoarNavGlobals.pitch_gesture_count = 0
			SoarNavGlobals.manual_override_active = false
			SoarNavGlobals.last_pilot_input_ms = nil
			SoarNavGlobals.pitch_gesture_start_ms = nil
			SoarNavGlobals.pitch_gesture_triggered_this_override = false
			SoarNavGlobals.roll_gesture_state = "idle"
			SoarNavGlobals.roll_gesture_count = 0
			SoarNavGlobals.roll_gesture_start_ms = nil
			SoarNavGlobals.manual_override_active = false
		end
		SoarNavGlobals.script_state = new_state

		if reason_id then
			log_gcs(MAV_SEVERITY.NOTICE, 1, reason_id, ...)
		end

		if new_state == SCRIPT_STATE.NAVIGATING then
			if prev_state ~= SCRIPT_STATE.NAVIGATING then
				SoarNavGlobals.target_loc = nil
				SoarNavGlobals.activation_requested = false
				SoarNavGlobals.activation_wait_notified = false
				SoarNavGlobals.navigation_start_delay_counter = 0
			end
		end
	end
end

-- Detects a rapid pitch stick gesture to re-center the search area.
local function check_pitch_gesture()
	local mode = vehicle:get_mode()
	local soaring_switch_high = (rc:get_aux_cached(SoarNavConstants.rc_opt_soaring_active) == 2)
	local in_fw_nav_mode = (mode == SoarNavConstants.mode_fbwb or mode == SoarNavConstants.mode_cruise)

	if SoarNavGlobals.script_state ~= SCRIPT_STATE.PILOT_OVERRIDE or not soaring_switch_high or not in_fw_nav_mode then
		SoarNavGlobals.pitch_gesture_state = "idle"
		SoarNavGlobals.pitch_gesture_count = 0
		SoarNavGlobals.pitch_gesture_start_ms = nil
		SoarNavGlobals.pitch_gesture_triggered_this_override = false
		return
	end

	if SoarNavGlobals.pitch_gesture_triggered_this_override then
		return
	end
	if SoarNavGlobals.script_state ~= SCRIPT_STATE.PILOT_OVERRIDE then
		SoarNavGlobals.pitch_gesture_triggered_this_override = false
	end
	if SoarNavGlobals.pitch_gesture_triggered_this_override then
		return
	end
	local gesture_timeout = false
	if SoarNavGlobals.pitch_gesture_start_ms then
		if
			safe_time_diff(millis(), SoarNavGlobals.pitch_gesture_start_ms) > SoarNavConstants.PITCH_GESTURE_TIMEOUT_MS
		then
			gesture_timeout = true
		end
	end

	if SoarNavGlobals.pitch_gesture_state ~= "idle" and gesture_timeout then
		SoarNavGlobals.pitch_gesture_state = "idle"
		SoarNavGlobals.pitch_gesture_count = 0
	end

	local pitch_chan_obj = rc:get_channel(P.rcmap_pitch:get() or 2)
	if not pitch_chan_obj then
		return
	end
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
		local current_loc = ahrs:get_location()
		if not current_loc then
			return
		end

		if SoarNavGlobals.use_polygon_area then
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.STICK_CMD_POLY_RECENTER)
			SoarNavGlobals.polygon_centroid = current_loc:copy()
			local offsets = SoarNavGlobals.polygon_vertex_offsets
			if not offsets or #offsets < 3 then
				log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.POLY_INVALID_POINTS)
				return
			end
			local new_points = {}
			for _, offset in ipairs(offsets) do
				local new_point = current_loc:copy()
				new_point:lat(current_loc:lat() + offset.lat)
				new_point:lng(current_loc:lng() + offset.lon)
				table.insert(new_points, new_point)
			end
			SoarNavGlobals.polygon_bounds =
				{ min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7 }
			for _, pt in ipairs(new_points) do
				local lat_i7 = pt:lat()
				local lon_i7 = pt:lng()
				SoarNavGlobals.polygon_bounds.min_lat = math.min(SoarNavGlobals.polygon_bounds.min_lat, lat_i7)
				SoarNavGlobals.polygon_bounds.max_lat = math.max(SoarNavGlobals.polygon_bounds.max_lat, lat_i7)
				SoarNavGlobals.polygon_bounds.min_lon = math.min(SoarNavGlobals.polygon_bounds.min_lon, lon_i7)
				SoarNavGlobals.polygon_bounds.max_lon = math.max(SoarNavGlobals.polygon_bounds.max_lon, lon_i7)
			end
			SoarNavGlobals.polygon_points = new_points
			SoarNavGlobals.dynamic_center_location = nil
			prepare_polygon_xy_cache()
		else
			if (P.radius_m and pget(P.radius_m, 0) or 0) <= 0 then
				return
			end
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.STICK_CMD_RADIUS_RECENTER)
			SoarNavGlobals.dynamic_center_location = current_loc:copy()
		end

		SoarNavGlobals.target_loc = nil
		SoarNavGlobals.grid_initialized = false
		SoarNavGlobals.force_grid_reinit = true
		SoarNavGlobals.area_announced = false
		SoarNavGlobals.pitch_gesture_triggered_this_override = true
		SoarNavGlobals.pitch_gesture_state = "idle"
		SoarNavGlobals.pitch_gesture_count = 0

		if SoarNavGlobals.manual_override_active then
			SoarNavGlobals.manual_override_active = false
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.STICK_CMD_MANUAL_OFF)
		end
		SoarNavGlobals.last_pilot_input_ms = nil
		set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.RESUMING_NAV)
	end
end

-- Detects a rapid roll stick gesture to activate the script or toggle manual override.
local function check_roll_gesture()
	local gesture_timeout = false
	if SoarNavGlobals.roll_gesture_start_ms then
		if
			safe_time_diff(millis(), SoarNavGlobals.roll_gesture_start_ms) > SoarNavConstants.ROLL_GESTURE_TIMEOUT_MS
		then
			gesture_timeout = true
		end
	end

	if SoarNavGlobals.roll_gesture_state ~= "idle" and gesture_timeout then
		SoarNavGlobals.roll_gesture_state = "idle"
		SoarNavGlobals.roll_gesture_count = 0
	end

	local roll_chan = pget(P.rcmap_roll, 1)
	local chan1 = rc:get_channel(roll_chan)
	if not chan1 then
		return false
	end
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
		local was_activation_gesture = false
		if SoarNavGlobals.script_state == SCRIPT_STATE.WAITING_FOR_ACTIVATION then
			SoarNavGlobals.activation_requested = true
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.GESTURE_ACTIVATED)

			SoarNavGlobals.activation_grace_start_ms = millis()
			was_activation_gesture = true
		else
			SoarNavGlobals.manual_override_active = not SoarNavGlobals.manual_override_active
			if SoarNavGlobals.manual_override_active then
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.STICK_CMD_MANUAL_ON)
			else
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.STICK_CMD_MANUAL_OFF)
			end
		end
		SoarNavGlobals.roll_gesture_state = "idle"
		SoarNavGlobals.roll_gesture_count = 0
		SoarNavGlobals.roll_gesture_start_ms = nil
		return was_activation_gesture
	end

	return false
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
	log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.GRID_INIT_STARTED)
end

-- Manages the multi-step, non-blocking grid generation and validation.
local function manage_grid_initialization()
	if not SoarNavGlobals.is_initializing then
		return
	end
	local active_center = get_active_center_location()
	if not active_center then
		log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.GRID_INIT_FAIL_NO_CENTER)
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
				if not SoarNavGlobals.area_announced then
					announce_polygon_area()
				end
				log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.POLY_MAX_DIST, SoarNavGlobals.max_operational_distance)
			end
		else
			local radius_m = (P.radius_m and pget(P.radius_m, 0) or 0)
			local corner_dist = radius_m * 1.4142
			local sw_corner_loc_calc = active_center:copy()
			sw_corner_loc_calc:offset_bearing(225, corner_dist)
			local ne_corner_loc_calc = active_center:copy()
			ne_corner_loc_calc:offset_bearing(45, corner_dist)
			SoarNavGlobals.grid_bounds = {
				min_lat = sw_corner_loc_calc:lat(),
				max_lat = ne_corner_loc_calc:lat(),
				min_lon = sw_corner_loc_calc:lng(),
				max_lon = ne_corner_loc_calc:lng(),
			}
			SoarNavGlobals.max_operational_distance = (radius_m or 0) * 2
			if not SoarNavGlobals.area_announced then
				announce_radius_area()
			end
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
			log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.GRID_TOO_SMALL)
			SoarNavGlobals.is_initializing = false
			SoarNavGlobals.grid_init_step = 0
			return
		end

		local area_size = math.max(width_m, height_m)
		SoarNavGlobals.effective_cluster_radius = math.min(600, math.max(200, area_size * 0.15))
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.EFF_CLUSTER_RADIUS, SoarNavGlobals.effective_cluster_radius)

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
			table.insert(SoarNavGlobals.grid_cells, { visit_count = 0 })
			SoarNavGlobals.grid_populate_index = SoarNavGlobals.grid_populate_index + 1
			cells_processed = cells_processed + 1
		end
		if SoarNavGlobals.grid_populate_index > total_cells then
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.GRID_SCAN_INFO, SoarNavGlobals.grid_rows, SoarNavGlobals.grid_cols)
			SoarNavGlobals.grid_init_row = 1
			SoarNavGlobals.grid_init_col = 1
			SoarNavGlobals.grid_init_step = 3
		end
	elseif SoarNavGlobals.grid_init_step == 3 then
		local cells_processed = 0
		local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat)
			/ SoarNavGlobals.grid_rows
		local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon)
			/ SoarNavGlobals.grid_cols
		while
			SoarNavGlobals.grid_init_row <= SoarNavGlobals.grid_rows
			and cells_processed < SoarNavConstants.grid_init_cells_per_call
		do
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
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.GRID_VALIDATED, #SoarNavGlobals.valid_cell_indices)
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.GRID_READY)
		end
	end
end

-- Tracks which grid cell the aircraft is currently in and updates its visit count.
local function update_visited_cell()
	local loc = ahrs:get_location()
	if not loc then
		return
	end
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
			SoarNavGlobals.grid_cells[current_cell_index].visit_count = SoarNavGlobals.grid_cells[current_cell_index].visit_count
				+ 1
		end
		SoarNavGlobals.last_cell_index = current_cell_index
	end
end

-- Removes expired thermal hotspots from memory and returns a list of currently valid ones.
local function clean_and_get_hotspots()
	local now_ms = millis()
	local lifetime_ms = P.tmem_life:get() * 1000
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
		for _, h in ipairs(hotspots or {}) do
			h.cluster_density = 0
		end
		return
	end

	local now = millis()
	local cluster_radius = SoarNavGlobals.effective_cluster_radius
	local tmem_life_ms = (P.tmem_life:get() or 1200) * 1000

	for i, h1 in ipairs(hotspots) do
		local density_score = 0
		for j, h2 in ipairs(hotspots) do
			if i ~= j then
				local distance = h1.loc:get_distance(h2.loc)
				if distance < cluster_radius then
					local distance_factor = 1.0 - (distance / cluster_radius)
					local age_ms = safe_time_diff(now, h2.timestamp)
					local age_factor = 1.0 - math.min(1.0, age_ms / tmem_life_ms)

					density_score = density_score + (h2.avg_strength or 0) * distance_factor * age_factor
				end
			end
		end
		h1.cluster_density = density_score
	end
end

-- Records a new thermal hotspot in memory after exiting THERMAL mode.
local function stop_and_record_thermal()
	SoarNavGlobals.is_monitoring_thermal = false
	if
		not SoarNavGlobals.current_thermal_stats
		or not SoarNavGlobals.current_thermal_stats.core_location
		or SoarNavGlobals.current_thermal_stats.avg_strength == 0
	then
		log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.THERMAL_EXIT_NO_DATA)
		SoarNavGlobals.lost_thermal_counter = SoarNavGlobals.lost_thermal_counter + 1
		SoarNavGlobals.current_thermal_stats = {}
		return
	end

	local max_strength = SoarNavGlobals.current_thermal_stats.max_strength
	if (max_strength or 0) < pget(P.tmem_min_s, 0.2) then
		SoarNavGlobals.current_thermal_stats = {}
		return
	end

	SoarNavGlobals.lost_thermal_counter = 0

	if SoarNavGlobals.is_in_focus_mode then
		log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.THERMAL_FOUND_EXIT_FOCUS)
		SoarNavGlobals.is_in_focus_mode = false
		SoarNavGlobals.focus_area_center = nil
		SoarNavGlobals.focus_wp_counter = 0
	end

	local avg_strength = SoarNavGlobals.current_thermal_stats.avg_strength
	local hotspot_loc = SoarNavGlobals.current_thermal_stats.core_location:copy()
	local variance = calculate_thermal_variance(SoarNavGlobals.current_thermal_stats.samples)
	local consistency_score = variance < SoarNavConstants.THERMAL_CONSISTENCY_VARIANCE_THRESHOLD and 1.0 or 0.5

	if not in_flight_area(hotspot_loc) then
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.THERMAL_IGNORED_OOB)
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
		consistency_score = consistency_score,
		duration = duration,
		wind_vec = SoarNavGlobals.current_thermal_stats.wind_at_entry,
		failed_attempts = 0,
		cluster_density = 0,
	}
	local new_cell = get_cell_index_from_location(hotspot_loc)
	if not new_cell then
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.CELL_INDEX_FAIL)
		new_cell = -1
	end
	new_hotspot.cell = new_cell
	for _, existing in ipairs(SoarNavGlobals.thermal_hotspots) do
		local existing_cell = get_cell_index_from_location(existing.loc)
		if existing_cell and new_cell and existing_cell == new_cell then
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
	end

	calculate_hotspot_density(SoarNavGlobals.thermal_hotspots)

	for _, hotspot in ipairs(SoarNavGlobals.thermal_hotspots) do
		if hotspot.timestamp:toint() == new_hotspot.timestamp:toint() then
			if
				hotspot.cluster_density
				and hotspot.cluster_density >= pget(P.focus_thr, 1.0)
				and not SoarNavGlobals.is_in_focus_mode
			then
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.FOCUS_ON_DENSITY)
				SoarNavGlobals.is_in_focus_mode = true
				SoarNavGlobals.focus_area_center = new_hotspot.loc:copy()
				SoarNavGlobals.focus_start_time = millis()
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
			end
			break
		end
	end

	local uncertainty_score = 0
	if (new_hotspot.max_strength or 0) < SoarNavConstants.strong_thermal_threshold_mps then
		uncertainty_score = (SoarNavConstants.strong_thermal_threshold_mps - (new_hotspot.max_strength or 0)) * 25
		uncertainty_score = math.max(0, math.min(50, uncertainty_score))
	end

	local necessity_score = (1.0 - SoarNavGlobals.filtered_alt_factor) * pget(P.nec_weight, 50)

	local total_score = uncertainty_score + necessity_score

	if total_score >= pget(P.retry_thr, 30) then
		SoarNavGlobals.thermal_to_retry = new_hotspot
	end

	SoarNavGlobals.current_thermal_stats = {}
end

-- Periodically samples the climb rate while the aircraft is in THERMAL mode.
local function sample_thermal_strength()
	local ned_velocity = ahrs:get_velocity_NED()
	if not ned_velocity then
		return
	end
	local climb_rate = -ned_velocity:z()
	local sample_weight = math.min(1.0, math.abs(climb_rate) / 3.0)
	if SoarNavGlobals.current_thermal_stats.avg_strength then
		SoarNavGlobals.current_thermal_stats.avg_strength = (
			SoarNavGlobals.current_thermal_stats.avg_strength * (1 - sample_weight)
		) + (climb_rate * sample_weight)
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
		log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.NO_THERMAL_LOC)
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
		wind_at_entry = get_wind_vector(),
	}
end

-- Reads a polygon file from the SD card and populates the polygon points list.
local function read_polygon_file(filename)
	local f = io.open(filename, "r")
	if not f then
		return nil
	end
	SoarNavGlobals.polygon_points = {}
	SoarNavGlobals.polygon_bounds =
		{ min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7 }
	local home = ahrs:get_home()
	if not home then
		f:close()
		return nil
	end
	local sum_lat, sum_lon, count = 0, 0, 0
	for line in f:lines() do
		if not line:match("^#") and line:match("%S") then
			local lat_str, lon_str = line:match("^%s*([%d%.%-]+)%s+([%d%.%-]+)%s*$")
			if lat_str and lon_str then
				local lat = tonumber(lat_str)
				local lon = tonumber(lon_str)
				if lat and lon then
					local point = home:copy()
					local lat_i7 = lat * 1e7
					local lon_i7 = lon * 1e7
					point:lat(lat_i7)
					point:lng(lon_i7)
					table.insert(SoarNavGlobals.polygon_points, point)
					sum_lat = sum_lat + lat_i7
					sum_lon = sum_lon + lon_i7
					count = count + 1
					SoarNavGlobals.polygon_bounds.min_lat = math.min(SoarNavGlobals.polygon_bounds.min_lat, lat_i7)
					SoarNavGlobals.polygon_bounds.max_lat = math.max(SoarNavGlobals.polygon_bounds.max_lat, lat_i7)
					SoarNavGlobals.polygon_bounds.min_lon = math.min(SoarNavGlobals.polygon_bounds.min_lon, lon_i7)
					SoarNavGlobals.polygon_bounds.max_lon = math.max(SoarNavGlobals.polygon_bounds.max_lon, lon_i7)
				end
			end
		end
	end
	f:close()
	if count < 3 then
		return nil
	end
	local center_lat = sum_lat / count
	local center_lon = sum_lon / count
	local center_loc = home:copy()
	center_loc:lat(center_lat)
	center_loc:lng(center_lon)
	SoarNavGlobals.polygon_centroid = center_loc
	SoarNavGlobals.polygon_vertex_offsets = {}
	for _, p in ipairs(SoarNavGlobals.polygon_points) do
		table.insert(SoarNavGlobals.polygon_vertex_offsets, { lat = p:lat() - center_lat, lon = p:lng() - center_lon })
	end
	local first_pt = SoarNavGlobals.polygon_points[1]
	local last_pt = SoarNavGlobals.polygon_points[#SoarNavGlobals.polygon_points]
	if first_pt:get_distance(last_pt) > 1 then
		table.insert(SoarNavGlobals.polygon_points, first_pt:copy())
		table.insert(
			SoarNavGlobals.polygon_vertex_offsets,
			{ lat = first_pt:lat() - center_lat, lon = first_pt:lng() - center_lon }
		)
	end
	prepare_polygon_xy_cache()
	SoarNavGlobals.using_rally_points = false
	return true
end

-- Reads rally points as a polygon (fast & checker-proof).
function read_rally_points_as_polygon()
	if not rally or not rally.get_rally_location then
		return nil
	end
	local home = ahrs:get_home()
	if not home then
		return nil
	end

	local initial_points = {}
	local rally_index = 0
	while true do
		local loc = rally:get_rally_location(rally_index)
		if not loc then
			break
		end
		table.insert(initial_points, loc)
		rally_index = rally_index + 1
		if rally_index > 100 then
			break
		end
	end
	if #initial_points < 3 then
		return nil
	end

	local pts_xy = {}
	for _, p in ipairs(initial_points) do
		local xy = home:get_distance_NE(p)
		table.insert(pts_xy, { x = xy:x(), y = xy:y(), loc = p, vec = xy })
	end

	local function cross(o, a, b)
		return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
	end

	table.sort(pts_xy, function(A, B)
		if A.x == B.x then
			return A.y < B.y
		else
			return A.x < B.x
		end
	end)

	local lower = {}
	for _, p in ipairs(pts_xy) do
		while #lower >= 2 and cross(lower[#lower - 1], lower[#lower], p) <= 0 do
			table.remove(lower)
		end
		table.insert(lower, p)
	end

	local upper = {}
	for idx = #pts_xy, 1, -1 do
		local p = pts_xy[idx]
		while #upper >= 2 and cross(upper[#upper - 1], upper[#upper], p) <= 0 do
			table.remove(upper)
		end
		table.insert(upper, p)
	end

	local hull = {}
	for i = 1, #lower - 1 do
		table.insert(hull, lower[i])
	end
	for i = 1, #upper - 1 do
		table.insert(hull, upper[i])
	end

	if #hull < 3 then
		table.sort(pts_xy, function(A, B)
			return A.vec:angle() < B.vec:angle()
		end)
		hull = {}
		local last = nil
		for _, p in ipairs(pts_xy) do
			if not last or p.x ~= last.x or p.y ~= last.y then
				table.insert(hull, p)
				last = p
			end
		end
		if #hull < 3 then
			return nil
		end
	end

	local ordered = {}
	for _, h in ipairs(hull) do
		table.insert(ordered, h.loc)
	end
	SoarNavGlobals.polygon_points = ordered

	SoarNavGlobals.polygon_bounds =
		{ min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7 }
	local sum_lat, sum_lon = 0, 0
	for _, p in ipairs(SoarNavGlobals.polygon_points) do
		local lat_i7, lon_i7 = p:lat(), p:lng()
		sum_lat = sum_lat + lat_i7
		sum_lon = sum_lon + lon_i7
		SoarNavGlobals.polygon_bounds.min_lat = math.min(SoarNavGlobals.polygon_bounds.min_lat, lat_i7)
		SoarNavGlobals.polygon_bounds.max_lat = math.max(SoarNavGlobals.polygon_bounds.max_lat, lat_i7)
		SoarNavGlobals.polygon_bounds.min_lon = math.min(SoarNavGlobals.polygon_bounds.min_lon, lon_i7)
		SoarNavGlobals.polygon_bounds.max_lon = math.max(SoarNavGlobals.polygon_bounds.max_lon, lon_i7)
	end

	local center_lat = sum_lat / #SoarNavGlobals.polygon_points
	local center_lon = sum_lon / #SoarNavGlobals.polygon_points
	local center_loc = home:copy()
	center_loc:lat(center_lat)
	center_loc:lng(center_lon)
	SoarNavGlobals.polygon_centroid = center_loc

	SoarNavGlobals.polygon_vertex_offsets = {}
	for _, p in ipairs(SoarNavGlobals.polygon_points) do
		table.insert(SoarNavGlobals.polygon_vertex_offsets, { lat = p:lat() - center_lat, lon = p:lng() - center_lon })
	end

	local first_pt = SoarNavGlobals.polygon_points[1]
	local last_pt = SoarNavGlobals.polygon_points[#SoarNavGlobals.polygon_points]
	if first_pt:get_distance(last_pt) > 1 then
		table.insert(SoarNavGlobals.polygon_points, first_pt:copy())
	end

	SoarNavGlobals.using_rally_points = true
	prepare_polygon_xy_cache()
	return true
end

-- Announces the new navigation target to the GCS with its wind-corrected heading and distance.
function log_new_waypoint(dist_to_wp)
	if SoarNavGlobals.is_in_focus_mode and SoarNavGlobals.focus_wp_counter > 1 then
		return
	end

	local d = tonumber(dist_to_wp) or 0
	local src = SoarNavGlobals.g_waypoint_source_info

	local current_loc = ahrs:get_location()
	if not current_loc or not SoarNavGlobals.target_loc then
		return
	end

	local hdg_deg = get_wind_corrected_heading_deg(current_loc, SoarNavGlobals.target_loc)

	local __dirs =
		{ "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" }
	local __ix = math.floor((hdg_deg / 22.5) + 0.5) + 1
	if __ix > #__dirs then
		__ix = 1
	end
	local hdg_dir = __dirs[__ix]

	local total_valid_cells = #SoarNavGlobals.valid_cell_indices
	local visited_cells = total_valid_cells - #SoarNavGlobals.unvisited_cell_indices

	local __show_cells = true

	if
		string.find(src, "Thrm", 1, true)
		or src == SoarNavWaypointSources.THERMAL_STREET
		or src == SoarNavWaypointSources.RANDOM_FALLBACK
	then
		__show_cells = false
	end

	if src == SoarNavWaypointSources.FOCUS_MODE or string.find(src, "Focus", 1, true) then
		__show_cells = false
	end
	if src == SoarNavWaypointSources.REENGAGE_ENTRY or src == SoarNavWaypointSources.REENGAGE_FLYOUT then
		__show_cells = false
	end

	if __show_cells then
		log_gcs(
			MAV_SEVERITY.INFO,
			1,
			MSG_ID.NAV_TARGET_CELLS,
			src,
			hdg_deg,
			hdg_dir,
			d,
			visited_cells,
			total_valid_cells
		)
	else
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.NAV_TARGET_SIMPLE, src, hdg_deg, hdg_dir, d)
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
	if #valid_hotspots == 0 then
		return false
	end

	local good_hotspots = {}
	local good_hotspots_excluding_last = {}
	local loc = ahrs:get_location()
	if not loc then
		return false
	end
	local min_dist_sq = (pget(P.wp_radius, 50) * 2) ^ 2

	for _, hotspot in ipairs(valid_hotspots) do
		if hotspot.avg_strength and hotspot.avg_strength > 0 then
			local dist_vec = loc:get_distance_NE(hotspot.loc)
			local dist_sq = dist_vec:x() ^ 2 + dist_vec:y() ^ 2
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

	local quality_weight = 0.3
	if SoarNavGlobals.energy_state == "LOW" then
		quality_weight = 0.5
	elseif SoarNavGlobals.energy_state == "CRITICAL" then
		quality_weight = 0.7
	end

	table.sort(final_hotspot_list, function(a, b)
		local strength_weight = 1.0 - quality_weight
		local score_a = (a.avg_strength or 0) * strength_weight
			+ ((a.avg_strength or 0) * (a.consistency_score or 0.5)) * quality_weight
		local score_b = (b.avg_strength or 0) * strength_weight
			+ ((b.avg_strength or 0) * (b.consistency_score or 0.5)) * quality_weight
		return score_a > score_b
	end)

	for _, best_hotspot in ipairs(final_hotspot_list) do
		local age_seconds = 0
		if best_hotspot.timestamp then
			age_seconds = safe_time_diff(millis(), best_hotspot.timestamp) / 1000.0
		end
		local wind_vec = best_hotspot.wind_vec or get_wind_vector()
		local drifted_loc = predict_thermal_drift(best_hotspot.loc, wind_vec, age_seconds)

		if in_flight_area(drifted_loc) then
			local drift_dist = best_hotspot.loc:get_distance(drifted_loc)
			if drift_dist > 0 then
				SoarNavGlobals.g_waypoint_source_info =
					string.format(SoarNavWaypointSources.THERMAL_MEMORY_DRIFT, best_hotspot.avg_strength)
			else
				SoarNavGlobals.g_waypoint_source_info =
					string.format(SoarNavWaypointSources.THERMAL_MEMORY_NODRIFT, best_hotspot.avg_strength)
			end

			SoarNavGlobals.current_selected_hotspot = best_hotspot
			set_target_safely(drifted_loc)
			SoarNavGlobals.last_used_hotspot_timestamp = best_hotspot.timestamp
			return true
		end
	end

	return false
end

-- Selects the most upwind cell from a list of candidates.
local function score_and_select_best_cell(candidate_indices)
	if not candidate_indices or #candidate_indices == 0 then
		return nil
	end
	if #candidate_indices == 1 then
		return candidate_indices[1]
	end

	local loc = ahrs:get_location()
	local wind = get_wind_vector()
	if not loc or not wind or wind:length() < 1.0 then
		return candidate_indices[math.random(#candidate_indices)]
	end

	local upwind_bearing_rad = math.rad((wind_vector_to_bearing_deg(wind) + 180) % 360)

	local best_score = -math.huge
	local best_cell_index = nil

	for _, cell_idx in ipairs(candidate_indices) do
		local cell_center = SoarNavGlobals.grid_cell_centers[cell_idx]
		if cell_center then
			local bearing_to_cell_rad = math.rad(loc:get_bearing(cell_center))
			local angle_diff = math.abs(bearing_to_cell_rad - upwind_bearing_rad)
			if angle_diff > math.pi then
				angle_diff = 2 * math.pi - angle_diff
			end

			local score = math.cos(angle_diff)

			if score > best_score then
				best_score = score
				best_cell_index = cell_idx
			end
		end
	end

	return best_cell_index or candidate_indices[math.random(#candidate_indices)]
end

-- Checks for thermal streets and sets a waypoint if one is found.
local function check_and_use_thermal_street()
	local hotspots = clean_and_get_hotspots()
	if #hotspots < SoarNavConstants.MIN_HOTSPOTS_FOR_STREET then
		return false
	end

	local wind_vec = get_wind_vector()
	if not wind_vec or wind_vec:length() < 1.0 then
		return false
	end

	table.sort(hotspots, function(a, b)
		return a.timestamp:toint() > b.timestamp:toint()
	end)

	local h1 = hotspots[1]
	local h2 = hotspots[2]

	if not h1 or not h2 or not h1.loc or not h2.loc then
		return false
	end

	if h1.loc:get_distance(h2.loc) < 150 then
		return false
	end

	local wind_bearing = wind_vector_to_bearing_deg(wind_vec)
	local street_bearing = h2.loc:get_bearing(h1.loc)
	local angle_diff = math.abs(((street_bearing - wind_bearing + 540) % 360) - 180)

	if angle_diff <= P.street_tol:get() then
		local projection_dist = h2.loc:get_distance(h1.loc)
		projection_dist = math.max(500, math.min(projection_dist, 2000))

		local new_target = h1.loc:copy()
		new_target:offset_bearing((wind_bearing + 180) % 360, projection_dist)

		local loc = ahrs:get_location()
		if in_flight_area(new_target) and loc then
			local dist_to_target = loc:get_distance(new_target)
			if dist_to_target > (P.wp_radius:get() * 2) then
				set_target_safely(new_target)
				SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.THERMAL_STREET
				return true
			end
		end
	end

	return false
end

-- Core logic for searching and selecting a new waypoint based on energy state and thermal memory.
local function search_for_new_waypoint()
	local new_wp_found = false
	local source_for_log = SoarNavWaypointSources.UNKNOWN

	local throttle_output = SRV_Channels:get_output_scaled(SoarNavConstants.THROTTLE_SRV_CHANNEL) or 0
	local throttle_on = throttle_output > 1

	if throttle_on then
		SoarNavGlobals.thermal_to_retry = nil
		SoarNavGlobals.reengage_final_target = nil
		SoarNavGlobals.reengage_hold_active = false
		SoarNavGlobals.reengage_hold_until_ms = nil
	end

	if SoarNavGlobals.thermal_to_retry and not throttle_on then
		local hotspot_to_retry = SoarNavGlobals.thermal_to_retry
		SoarNavGlobals.thermal_to_retry = nil
		local current_loc = ahrs:get_location()
		if hotspot_to_retry and hotspot_to_retry.loc and current_loc then
			local bearing_to_thermal = current_loc:get_bearing(hotspot_to_retry.loc)
			local reengage_dist = (pget(P.wp_radius, 50)) * 3.5
			local candidates = {
				(bearing_to_thermal + 180) % 360,
				(bearing_to_thermal + 215) % 360,
				(bearing_to_thermal + 145) % 360,
				(bearing_to_thermal + 250) % 360,
				(bearing_to_thermal + 110) % 360,
			}
			local best_p = nil
			local best_s = -1e9
			for i = 1, #candidates do
				local b = candidates[i]
				local p = current_loc:copy()
				p:offset_bearing(b, reengage_dist)
				local ang = math.abs(((b - ((bearing_to_thermal + 180) % 360) + 540) % 360) - 180)
				local score = (math.cos(math.rad(ang)) * 2.0) + (current_loc:get_distance(p) * 0.001)
				if score > best_s then
					best_s = score
					best_p = p
				end
			end
			local flyout_point = best_p
				or (function()
					local fp = current_loc:copy()
					fp:offset_bearing(((bearing_to_thermal + 180) % 360), reengage_dist)
					return fp
				end)()
			set_target_safely(flyout_point)
			local age_s = safe_time_diff(millis(), hotspot_to_retry.timestamp) / 1000.0
			local wind = hotspot_to_retry.wind_vec or get_wind_vector()
			local predicted = predict_thermal_drift(hotspot_to_retry.loc, wind, age_s)
			local bearing_back = current_loc:get_bearing(predicted)
			local dist_back = current_loc:get_distance(predicted)
			local reentry_cap = reengage_dist * 2.0
			if dist_back > reentry_cap then
				local capped = current_loc:copy()
				capped:offset_bearing(bearing_back, reentry_cap)
				if SoarNavGlobals.use_polygon_area then
					if not in_flight_area(capped) then
						capped = clamp_inside_polygon(capped, (P and P.wp_radius and P.wp_radius:get()) or 50)
					end
					if not segment_stays_inside(current_loc, capped) then
						capped = adjust_target_segment(current_loc, capped)
					end
				end
				SoarNavGlobals.reengage_final_target = capped
			else
				SoarNavGlobals.reengage_final_target = predicted
			end
			source_for_log = SoarNavWaypointSources.REENGAGE_FLYOUT
			new_wp_found = true
		end
	end
	if new_wp_found then
		SoarNavGlobals.g_waypoint_source_info = source_for_log
		_finalize_waypoint_selection()
		return
	end

	local force_grid_search = false
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	if dist_from_home_3d then
		local current_alt = -dist_from_home_3d:z()
		local soar_alt_max = pget(P.soar_alt_max)
		if soar_alt_max and soar_alt_max > 0 and current_alt > soar_alt_max then
			force_grid_search = true
		end
	end
	if SoarNavGlobals.lost_thermal_counter >= 2 then
		SoarNavGlobals.force_grid_after_reset = true
		SoarNavGlobals.lost_thermal_counter = 0
	end

	local try_smart_strategies = true
	if SoarNavGlobals.force_grid_after_reset then
		SoarNavGlobals.force_grid_after_reset = false
		try_smart_strategies = false
	end

	if try_smart_strategies then
		if not force_grid_search then
			if SoarNavGlobals.is_in_focus_mode then
				if SoarNavGlobals.focus_wp_counter >= (SoarNavGlobals.focus_wp_timeout or 3) then
					SoarNavGlobals.is_in_focus_mode = false
				else
					local age_seconds = safe_time_diff(millis(), SoarNavGlobals.focus_start_time) / 1000.0
					local wind = get_wind_vector()
					local drifted_focus_center =
						predict_thermal_drift(SoarNavGlobals.focus_area_center, wind, age_seconds)
					local focus_radius = SoarNavGlobals.effective_cluster_radius / 2
					local new_target = generate_target_around_point(drifted_focus_center, focus_radius)
					if new_target then
						set_target_safely(new_target)
						source_for_log =
							string.format(SoarNavWaypointSources.FOCUS_MODE, SoarNavGlobals.focus_wp_counter + 1)
						SoarNavGlobals.focus_wp_counter = SoarNavGlobals.focus_wp_counter + 1
						new_wp_found = true
					else
						SoarNavGlobals.is_in_focus_mode = false
					end
				end
			end
			if not new_wp_found then
				local energy_status = assess_energy_state()
				if energy_status == "LOW" or energy_status == "CRITICAL" then
					if #clean_and_get_hotspots() > 0 and select_best_thermal_waypoint() then
						new_wp_found = true
						source_for_log = SoarNavGlobals.g_waypoint_source_info
					end
				end
			end
			if not new_wp_found and pget(P.tmem_enable, 1) == 1 and select_best_thermal_waypoint() then
				new_wp_found = true
				source_for_log = SoarNavGlobals.g_waypoint_source_info
			end
		end

		if not new_wp_found and SoarNavGlobals.try_ridge_target() then
			new_wp_found = true
			source_for_log = SoarNavWaypointSources.RIDGE
		end

		if not new_wp_found and check_and_use_thermal_street() then
			new_wp_found = true
			source_for_log = SoarNavGlobals.g_waypoint_source_info
		end
	end

	if not new_wp_found then
		if SoarNavGlobals.grid_initialized and #SoarNavGlobals.valid_cell_indices > 0 then
			if #SoarNavGlobals.unvisited_cell_indices == 0 then
				for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
					if SoarNavGlobals.grid_cells[cell_idx] then
						SoarNavGlobals.grid_cells[cell_idx].visit_count = 0
					end
				end
				SoarNavGlobals.unvisited_cell_indices = {}
				for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
					table.insert(SoarNavGlobals.unvisited_cell_indices, v)
				end
				SoarNavGlobals.force_grid_after_reset = true
			end

			local recent_success_count = 0
			local now_ms = millis()
			for _, hotspot in ipairs(SoarNavGlobals.thermal_hotspots) do
				if
					hotspot.entry_time
					and safe_time_diff(now_ms, hotspot.entry_time) <= (pget(P.strat_hist, 900) * 1000)
				then
					recent_success_count = recent_success_count + 1
				end
			end

			local chosen_cell_index
			local success_rate_factor = math.min(1, recent_success_count / 3.0)
			local guided_chance = 25 + (50 * success_rate_factor)

			if math.random(1, 100) <= guided_chance then
				source_for_log = SoarNavWaypointSources.GRID_GUIDED
				if #SoarNavGlobals.unvisited_cell_indices > 0 then
					chosen_cell_index = score_and_select_best_cell(SoarNavGlobals.unvisited_cell_indices)
				else
					local min_visits = math.huge
					for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
						min_visits = math.min(min_visits, SoarNavGlobals.grid_cells[cell_idx].visit_count)
					end
					local least_visited_indices = {}
					for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
						if SoarNavGlobals.grid_cells[cell_idx].visit_count == min_visits then
							table.insert(least_visited_indices, cell_idx)
						end
					end
					chosen_cell_index = score_and_select_best_cell(least_visited_indices)
				end
			else
				source_for_log = SoarNavWaypointSources.GRID_PURE
				if #SoarNavGlobals.unvisited_cell_indices > 0 then
					local random_table_index = math.random(1, #SoarNavGlobals.unvisited_cell_indices)
					chosen_cell_index = SoarNavGlobals.unvisited_cell_indices[random_table_index]
				else
					local random_table_index = math.random(1, #SoarNavGlobals.valid_cell_indices)
					chosen_cell_index = SoarNavGlobals.valid_cell_indices[random_table_index]
				end
			end

			if chosen_cell_index and SoarNavGlobals.grid_cell_centers[chosen_cell_index] then
				local precalculated_center = SoarNavGlobals.grid_cell_centers[chosen_cell_index]
				local current_loc = ahrs:get_location()
				local success = false
				for _ = 1, 5 do
					local new_target_loc = precalculated_center:copy()
					local offset_radius = math.floor(SoarNavGlobals.grid_cell_size_m / 2.5)
					new_target_loc:offset_bearing(math.random() * 360, math.sqrt(math.random()) * offset_radius)
					if
						in_flight_area(new_target_loc)
						and current_loc
						and current_loc:get_distance(new_target_loc) > (pget(P.wp_radius, 50))
					then
						set_target_safely(new_target_loc)
						success = true
						break
					end
				end
				if success then
					new_wp_found = true
				else
					if
						in_flight_area(precalculated_center)
						and current_loc
						and current_loc:get_distance(precalculated_center) > (pget(P.wp_radius, 50))
					then
						set_target_safely(precalculated_center)
						new_wp_found = true
					end
				end
			end
		end
	end

	if not new_wp_found then
		log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.ALL_STRAT_FAILED)
		local center_point = get_active_center_location()
		if center_point then
			local radius_m = SoarNavGlobals.max_operational_distance / 2
			if radius_m <= 0 then
				radius_m = (P.radius_m and pget(P.radius_m, 0) or 500)
			end

			for _ = 1, 10 do
				local fallback_loc = center_point:copy()
				fallback_loc:offset_bearing(math.random() * 360, math.sqrt(math.random()) * radius_m)
				if in_flight_area(fallback_loc) then
					set_target_safely(fallback_loc)
					source_for_log = SoarNavWaypointSources.RANDOM_FALLBACK
					new_wp_found = true
					break
				end
			end
		end
	end

	if new_wp_found then
		SoarNavGlobals.g_waypoint_source_info = source_for_log
		_finalize_waypoint_selection()
	else
		log_gcs(MAV_SEVERITY.CRITICAL, 0, MSG_ID.FATAL_NO_WP)
	end
end

-- Calculates a dynamic waypoint timeout based on current wind speed.
local function get_wp_timeout()
	local base_timeout = (pget(P.wp_timeout, 300)) * 1000
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

	local dyn_soalt_mode = P.dyn_soalt:get()
	if
		dyn_soalt_mode == 0
		or SoarNavGlobals.script_state ~= SCRIPT_STATE.NAVIGATING
		or current_mode == SoarNavConstants.mode_thermal
	then
		SoarNavGlobals.motor_failure_check_active = false
		SoarNavGlobals.motor_on_start_time_ms = nil
		SoarNavGlobals.rpm_failure_start_ms = nil
		return
	end

	local wp_radius = P.wp_radius:get() or 50
	local suppress_radius = math.max(100, wp_radius * 2)
	if dist_from_home_3d:xy():length() <= suppress_radius then
		return
	end

	local soar_alt_min = P.soar_alt_min:get()
	if not soar_alt_min then
		return
	end

	local current_alt = -dist_from_home_3d:z()
	local throttle_output = SRV_Channels:get_output_scaled(SoarNavConstants.THROTTLE_SRV_CHANNEL) or 0
	local motor_should_be_on = (current_alt < soar_alt_min)
		and (throttle_output > SoarNavConstants.THROTTLE_ACTIVE_THRESHOLD)

	if not motor_should_be_on then
		SoarNavGlobals.motor_failure_check_active = false
		SoarNavGlobals.motor_on_start_time_ms = nil
		SoarNavGlobals.rpm_failure_start_ms = nil
		return
	end

	local now = millis()
	local rpm_value = RPM and RPM.get_rpm and RPM:get_rpm(0)

	local function trigger_rtl()
		log_gcs(MAV_SEVERITY.CRITICAL, 0, MSG_ID.MOTOR_FAIL_RTL)
		vehicle:set_mode(SoarNavConstants.RTL_MODE_CODE)
		restore_boot_alts("MOTOR Failure RTL")
		set_script_state(SCRIPT_STATE.ERROR, MSG_ID.MOTOR_FAIL_REASON)
	end

	if rpm_value ~= nil and rpm_value >= 0 then
		SoarNavGlobals.motor_on_start_time_ms = nil
		local rpm_issue = (throttle_output > SoarNavConstants.MOTOR_RPM_FAIL_THROTTLE_PERCENT)
			and (rpm_value < SoarNavConstants.MOTOR_RPM_FAIL_RPM_MIN)

		if rpm_issue then
			if not SoarNavGlobals.rpm_failure_start_ms then
				SoarNavGlobals.rpm_failure_start_ms = now
			end
		else
			SoarNavGlobals.rpm_failure_start_ms = nil
		end

		if
			SoarNavGlobals.rpm_failure_start_ms
			and safe_time_diff(now, SoarNavGlobals.rpm_failure_start_ms)
				> SoarNavConstants.MOTOR_RPM_FAIL_DEBOUNCE_MS
		then
			trigger_rtl()
		end
	else
		SoarNavGlobals.rpm_failure_start_ms = nil
		if not SoarNavGlobals.motor_failure_check_active then
			SoarNavGlobals.motor_failure_check_active = true
			SoarNavGlobals.motor_on_start_time_ms = now
			SoarNavGlobals.altitude_at_check_start_m = current_alt
		end

		local time_since_motor_on = safe_time_diff(now, SoarNavGlobals.motor_on_start_time_ms)
		if time_since_motor_on > SoarNavConstants.MOTOR_FAILURE_CHECK_DELAY_MS then
			local ned_velocity = ahrs:get_velocity_NED()
			if not ned_velocity then
				return
			end

			local climb_rate = -ned_velocity:z()
			local altitude_gained_ok = -dist_from_home_3d:z() > (SoarNavGlobals.altitude_at_check_start_m + 1.0)

			if
				not (climb_rate >= SoarNavConstants.MOTOR_FAILURE_CLIMB_RATE_THRESHOLD_MPS) and not altitude_gained_ok
			then
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
local handle_idle, handle_navigating, handle_thermal_pause, handle_pilot_override, handle_waiting

-- Determines if conditions are suitable for a tactical mid-flight reroute.
local function check_tactical_reroute_conditions()
	if SoarNavGlobals.is_in_focus_mode then
		return false
	end

	local __is_thermal_target = is_thermal_target(SoarNavGlobals.g_waypoint_source_info)

	local wind_vec = get_wind_vector()
	local wind_speed = wind_vec and wind_vec:length() or 0
	local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 3)))
	local __stuck = (SoarNavGlobals.stuck_counter or 0) >= adaptive_stuck_limit

	if __is_thermal_target and not __stuck then
		return false
	end

	if
		SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_FLYOUT
		or SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_ENTRY
	then
		return false
	end

	local min_path_for_reroute = SoarNavGlobals.max_operational_distance
		/ SoarNavConstants.TACTICAL_REROUTE_MIN_PATH_DIVISOR
	if SoarNavGlobals.initial_distance_to_wp <= min_path_for_reroute then
		return false
	end

	local total_valid_cells = #SoarNavGlobals.valid_cell_indices
	if total_valid_cells == 0 then
		return false
	end

	local unvisited_count = #SoarNavGlobals.unvisited_cell_indices
	if unvisited_count <= 1 then
		return false
	end

	local explored_ratio = (total_valid_cells - unvisited_count) / total_valid_cells
	if explored_ratio >= 0.9 then
		return false
	end

	local base_prob = P.reroute_p:get()
	local dynamic_prob = base_prob

	if SoarNavGlobals.energy_state == "LOW" then
		dynamic_prob = dynamic_prob * 0.5
	elseif SoarNavGlobals.energy_state == "CRITICAL" then
		dynamic_prob = dynamic_prob * 0.25
	end

	if SoarNavGlobals.lost_thermal_counter > 0 then
		dynamic_prob = dynamic_prob * (1 + (SoarNavGlobals.lost_thermal_counter * 0.2))
	end

	dynamic_prob = math.max(0, math.min(100, dynamic_prob))

	if math.random(1, 100) > dynamic_prob then
		return false
	end

	return true
end

-- Checks the status of the current waypoint (reached, timeout, or reroute trigger).
local function check_waypoint_status(loc, current_time_ms, cached_params)
	SoarNavGlobals.distance_to_wp = loc:get_distance(SoarNavGlobals.target_loc)

	if
		SoarNavGlobals.reroute_check_armed
		and SoarNavGlobals.initial_distance_to_wp > 0
		and SoarNavGlobals.distance_to_wp <= (SoarNavGlobals.initial_distance_to_wp / 2)
	then
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
	if SoarNavGlobals.is_repositioning then
		return false
	end

	local time_on_current_wp = safe_time_diff(current_time_ms, SoarNavGlobals.waypoint_start_time_ms)
	if time_on_current_wp < (pget(P.stuck_time, 30) * 1000) then
		return false
	end

	local progress_check_timeout = safe_time_diff(current_time_ms, SoarNavGlobals.last_progress_check_ms)
		> SoarNavConstants.STUCK_PROGRESS_CHECK_INTERVAL_MS
	if not SoarNavGlobals.last_progress_check_ms or progress_check_timeout then
		if SoarNavGlobals.distance_at_last_check > 0 and SoarNavGlobals.loc_at_last_check then
			local progress_made = SoarNavGlobals.distance_at_last_check - SoarNavGlobals.distance_to_wp
			local distance_traveled = loc:get_distance(SoarNavGlobals.loc_at_last_check)
			local efficiency_ratio = 0
			if distance_traveled > 5 then
				efficiency_ratio = progress_made / distance_traveled
			end

			local min_eff = pget(P.stuck_eff, 0.35)
			local min_prog = SoarNavConstants.STUCK_MIN_PROGRESS_M
			if progress_made < min_prog or efficiency_ratio < min_eff then
				SoarNavGlobals.stuck_counter = SoarNavGlobals.stuck_counter + 1
				local wind_vec_info = get_wind_vector()
				if wind_vec_info then
					local wind_bearing_info = wind_vector_to_bearing_deg(wind_vec_info)
					log_gcs(
						MAV_SEVERITY.WARNING,
						1,
						MSG_ID.NO_PROG_WARN,
						efficiency_ratio,
						wind_vec_info:length(),
						wind_bearing_info
					)
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

			local repo_loc = loc:copy()
			repo_loc:offset_bearing((wind_vector_to_bearing_deg(wind_vec) + upwind_offset_deg) % 360, repo_dist_m)

			if in_flight_area(repo_loc) then
				log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.STICK_CMD_STUCK, repo_dist_m, upwind_offset_deg)
				set_target_safely(repo_loc)
				_finalize_waypoint_selection()
			else
				SoarNavGlobals.target_loc = nil
			end
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
	local function wrap360(x)
		return (x % 360 + 360) % 360
	end
	local function wrap180(x)
		return (x + 540) % 360 - 180
	end

	local target_heading_deg = get_wind_corrected_heading_deg(loc, SoarNavGlobals.target_loc)
	local current_heading_deg = wrap360(math.deg(ahrs:get_yaw_rad()))
	local heading_error = wrap180(target_heading_deg - current_heading_deg)

	local error_derivative = 0
	if dt_s > 0.01 then
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
			smoothed_command = (abs_smooth_factor * raw_pd_command)
				+ ((1.0 - abs_smooth_factor) * SoarNavGlobals.last_commanded_roll_deg)
		else
			smoothed_command = raw_pd_command
		end
	else
		smoothed_command = (abs_smooth_factor * raw_pd_command)
			+ ((1.0 - abs_smooth_factor) * SoarNavGlobals.last_commanded_roll_deg)
	end

	local desired_roll_deg = math.max(-cached_params.roll_limit, math.min(cached_params.roll_limit, smoothed_command))
	SoarNavGlobals.last_commanded_roll_deg = desired_roll_deg

	local sys_roll_limit = cached_params.sys_roll_limit
	local roll_normalized = desired_roll_deg / sys_roll_limit

	local roll_pwm_value
	if roll_normalized > 0 then
		roll_pwm_value = SoarNavGlobals.rc_roll_trim
			+ roll_normalized * (SoarNavGlobals.rc_roll_max - SoarNavGlobals.rc_roll_trim)
	else
		roll_pwm_value = SoarNavGlobals.rc_roll_trim
			+ roll_normalized * (SoarNavGlobals.rc_roll_trim - SoarNavGlobals.rc_roll_min)
	end

	if SoarNavGlobals.rc_roll_channel then
		SoarNavGlobals.rc_roll_channel:set_override(math.floor(roll_pwm_value))
	else
		set_script_state(SCRIPT_STATE.ERROR, MSG_ID.ERR_ROLL_NIL)
	end

	return heading_error
end

-- #############################################################################
-- ## CORE SCRIPT STATE MACHINE
-- #############################################################################

-- Handles the IDLE state, waiting for navigation conditions to be met.
handle_idle = function(can_navigate)
	if can_navigate then
		if not SoarNavGlobals.require_initial_activation_gesture then
			if not SoarNavGlobals.has_been_activated then
				SoarNavGlobals.has_been_activated = true
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.AUTO_START)
			end
			set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.NAV_COND_MET)
			return
		end
		if not SoarNavGlobals.has_been_activated then
			if not SoarNavGlobals.activation_wait_notified then
				if SoarNavGlobals.script_state ~= SCRIPT_STATE.WAITING_FOR_ACTIVATION then
					set_script_state(SCRIPT_STATE.WAITING_FOR_ACTIVATION, MSG_ID.AWAITING_ACTIVATION)
				end
				SoarNavGlobals.activation_wait_notified = true
			end
		else
			set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.NAV_COND_MET)
		end
	end
end

-- Handles the NAVIGATING state, managing the flight to a waypoint.
handle_navigating = function(
	current_time_ms,
	loc,
	can_navigate,
	is_in_thermal_mode,
	cached_params,
	autotune_active,
	dt_s
)
	if SoarNavGlobals.force_new_search then
		SoarNavGlobals.target_loc = nil
		SoarNavGlobals.force_new_search = false
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.FORCING_NEW_TARGET)
	end

	if SoarNavGlobals.navigation_start_delay_counter > 0 then
		SoarNavGlobals.navigation_start_delay_counter = SoarNavGlobals.navigation_start_delay_counter - 1
		return
	end

	if is_in_thermal_mode then
		set_script_state(SCRIPT_STATE.THERMAL_PAUSE)
		return
	end
	if (not can_navigate or autotune_active) and not is_in_thermal_mode then
		set_script_state(SCRIPT_STATE.IDLE, MSG_ID.NAV_COND_NOT_MET)
		return
	end

	while not SoarNavGlobals.target_loc do
		if not SoarNavGlobals.waypoint_search_in_progress then
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
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.REPOSITIONED)
			set_target_safely(SoarNavGlobals.original_target_loc)
			SoarNavGlobals.is_repositioning = false
			SoarNavGlobals.original_target_loc = nil
			_finalize_waypoint_selection()
		else
			if SoarNavGlobals.g_waypoint_source_info == SoarNavWaypointSources.REENGAGE_ENTRY then
				local now_ms = millis():toint()
				if
					SoarNavGlobals.reengage_hold_active
					and now_ms < SoarNavGlobals.reengage_hold_until_ms
					and not is_in_thermal_mode
				then
					if
						not SoarNavGlobals.reengage_hold_last_msg_ms
						or ((now_ms - SoarNavGlobals.reengage_hold_last_msg_ms) >= 2000)
					then
						SoarNavGlobals.reengage_hold_last_msg_ms = now_ms
					end
					return
				else
					SoarNavGlobals.reengage_hold_active = false
					SoarNavGlobals.reengage_hold_last_msg_ms = nil
				end
			end
			if SoarNavGlobals.reengage_final_target then
				set_target_safely(SoarNavGlobals.reengage_final_target)
				SoarNavGlobals.reengage_final_target = nil
				SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.REENGAGE_ENTRY
				local reengage_dwell_ms = P.reeng_dwell:get() * 1000
				SoarNavGlobals.reengage_hold_until_ms = millis():toint() + reengage_dwell_ms
				SoarNavGlobals.reengage_hold_active = true
				SoarNavGlobals.reengage_hold_last_msg_ms = millis():toint()
				_finalize_waypoint_selection()
				return
			end
			local hotspot_to_check = SoarNavGlobals.current_selected_hotspot
			if is_thermal_target(SoarNavGlobals.g_waypoint_source_info) and hotspot_to_check then
				hotspot_to_check.failed_attempts = (hotspot_to_check.failed_attempts or 0) + 1
				if hotspot_to_check.failed_attempts >= 2 then
					for i = #SoarNavGlobals.thermal_hotspots, 1, -1 do
						local h = SoarNavGlobals.thermal_hotspots[i]
						if
							h.timestamp
							and hotspot_to_check.timestamp
							and h.timestamp:toint() == hotspot_to_check.timestamp:toint()
						then
							table.remove(SoarNavGlobals.thermal_hotspots, i)
							break
						end
					end
				end
				SoarNavGlobals.current_selected_hotspot = nil
			end
			SoarNavGlobals.target_loc = nil
		end
	elseif wp_status == "REROUTE" then
		local original_target = SoarNavGlobals.target_loc
		if not original_target then
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.REROUTE_SKIP_NO_TARGET)
			SoarNavGlobals.target_loc = nil
			return
		end

		local reroute_origin = ahrs:get_location()
		local yaw_rad = ahrs:get_yaw_rad()
		if not reroute_origin or not yaw_rad then
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.REROUTE_SKIP_HEADING)
			SoarNavGlobals.target_loc = nil
			return
		end

		local hdg = (math.deg(yaw_rad) + 360) % 360
		local offset = math.random(pget(P.reroute_min, 80), pget(P.reroute_max, 100))
		if math.random() > 0.5 then
			offset = -offset
		end
		local desired_bearing = (hdg + offset + 360) % 360

		local distance = reroute_origin:get_distance(original_target)
		local new_target = reroute_origin:copy()
		new_target:offset_bearing(desired_bearing, distance)

		if not in_flight_area(new_target) then
			SoarNavGlobals.target_loc = nil
			return
		end

		set_target_safely(new_target)
		SoarNavGlobals.g_waypoint_source_info = SoarNavWaypointSources.REROUTE
		_finalize_waypoint_selection()
		return
	end

	if manage_anti_stuck(loc, current_time_ms) then
		return
	end

	if SoarNavGlobals.target_loc then
		local should_update_cell = not SoarNavGlobals.last_cell_check_loc
			or loc:get_distance(SoarNavGlobals.last_cell_check_loc) > (SoarNavGlobals.grid_cell_size_m / 2)
		if should_update_cell then
			update_visited_cell()
			SoarNavGlobals.last_cell_check_loc = loc:copy()
		end
		update_navigation_controller(loc, cached_params, dt_s)
	end
end

-- Handles the THERMAL_PAUSE state while the aircraft is in ArduPilot's THERMAL mode.
handle_thermal_pause = function(is_in_thermal_mode)
	if not is_in_thermal_mode then
		set_script_state(SCRIPT_STATE.NAVIGATING)
	end
end

-- Handles the WAITING_FOR_ACTIVATION state.
handle_waiting = function(can_navigate)
	if SoarNavGlobals.has_been_activated and can_navigate then
		set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.NAV_COND_MET)
		return
	end
	if not arming:is_armed() then
		set_script_state(SCRIPT_STATE.IDLE, MSG_ID.DISARMED)
		return
	end
	if can_navigate then
		check_roll_gesture()
		if SoarNavGlobals.activation_requested then
			SoarNavGlobals.has_been_activated = true
			SoarNavGlobals.activation_requested = false
			set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.PILOT_ACTIVATION)
		end
	end
end

-- Handles the PILOT_OVERRIDE state, managing stick input and gesture detection.
handle_pilot_override = function(
	current_time_ms,
	is_outside_pitch_dz,
	is_outside_yaw_dz,
	is_outside_roll_dz,
	can_navigate
)
	local pilot_is_holding_input = is_outside_pitch_dz or is_outside_yaw_dz or is_outside_roll_dz
	if pilot_is_holding_input then
		SoarNavGlobals.last_pilot_input_ms = current_time_ms
	end
	local __now = millis()
	if SoarNavGlobals.last_stick_tick_ms and safe_time_diff(__now, SoarNavGlobals.last_stick_tick_ms) < 200 then
		return
	end
	SoarNavGlobals.last_stick_tick_ms = __now

	check_roll_gesture()
	check_pitch_gesture()
	if not SoarNavGlobals.manual_override_active then
		local resume_delay_passed = safe_time_diff(current_time_ms, SoarNavGlobals.last_pilot_input_ms)
			> SoarNavConstants.PILOT_RESUME_DELAY_MS
		if not pilot_is_holding_input and SoarNavGlobals.last_pilot_input_ms and resume_delay_passed then
			if can_navigate then
				set_script_state(SCRIPT_STATE.NAVIGATING, MSG_ID.RESUMING_NAV)
			else
				set_script_state(SCRIPT_STATE.IDLE, MSG_ID.SET_CRUISE_FBWB)
			end
		end
	end
end

-- RTL→Home override when using Rally area; enable with SoarNavGlobals.rtlh_en=true
function SoarNav_RTLH_update(now_ms)
	if not SoarNavGlobals.rtlh_en then
		return
	end
	if not SoarNavGlobals.using_rally_points then
		return
	end

	SoarNavGlobals.RTLH._mode = vehicle:get_mode()

	if
		SoarNavGlobals.RTLH.last_mode ~= nil
		and SoarNavGlobals.RTLH.engaged_guided
		and SoarNavGlobals.RTLH.last_mode == SoarNavConstants.GUIDED_MODE_CODE
		and SoarNavGlobals.RTLH._mode ~= SoarNavConstants.GUIDED_MODE_CODE
	then
		SoarNavGlobals.RTLH.abort_until_next_rtl = true
		SoarNavGlobals.RTLH.engaged_guided = false
		if log_gcs then
			log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.RTL_OVERRIDE_RESUME)
		end
	end

	if
		SoarNavGlobals.RTLH.last_mode ~= SoarNavConstants.RTL_MODE_CODE
		and SoarNavGlobals.RTLH._mode == SoarNavConstants.RTL_MODE_CODE
	then
		SoarNavGlobals.RTLH.abort_until_next_rtl = false
		SoarNavGlobals.RTLH.active = false
	end

	SoarNavGlobals.RTLH.last_mode = SoarNavGlobals.RTLH._mode
	if SoarNavGlobals.RTLH.abort_until_next_rtl then
		return
	end

	if SoarNavGlobals.RTLH.engaged_guided and vehicle:get_mode() == SoarNavConstants.GUIDED_MODE_CODE then
		local here = ahrs:get_location()
		local home = ahrs:get_home()
		if here and home then
			local d = here:get_distance(home)
			local rtl_radius = param:get("RTL_RADIUS") or 0
			if rtl_radius == 0 then
				rtl_radius = 50
			end
			if d <= rtl_radius then
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.RTL_OVERRIDE_RESUME)
				SoarNavGlobals.RTLH.abort_until_next_rtl = true
				SoarNavGlobals.RTLH.engaged_guided = false
			end
		end
	end

	if SoarNavGlobals.RTLH._mode ~= SoarNavConstants.RTL_MODE_CODE then
		if SoarNavGlobals.RTLH.active then
			SoarNavGlobals.RTLH.active = false
		end
		return
	end

	if not SoarNavGlobals.RTLH.active then
		local home = ahrs:get_home()
		local here = ahrs:get_location()

		if not home or not here then
			return
		end

		SoarNavGlobals.RTLH.active = true
		SoarNavGlobals.RTLH.t0_ms = now_ms
		SoarNavGlobals.RTLH.d0_m = here:get_distance(home)
		return
	end

	if
		not SoarNavGlobals.RTLH.engaged_guided
		and (now_ms - SoarNavGlobals.RTLH.t0_ms) >= SoarNavConstants.RTLH_CHECK_DELAY_MS
	then
		local home = ahrs:get_home()
		local here = ahrs:get_location()

		if not home or not here then
			return
		end

		local d_now = here:get_distance(home)

		local rtl_radius = param:get("RTL_RADIUS") or 0
		if d_now <= (rtl_radius * 1.5) then
			return
		end

		local vned = ahrs:get_velocity_NED()
		local gs = (vned and math.sqrt((vned:x() or 0) ^ 2 + (vned:y() or 0) ^ 2)) or 0
		local needed_gain = math.max(15, 0.3 * gs * 3.0)

		if (SoarNavGlobals.RTLH.d0_m - d_now) < needed_gain then
			local alt_m = param:get("RTL_ALTITUDE") or 0
			local dest = Location()
			dest:lat(home:lat())
			dest:lng(home:lng())
			dest:alt(alt_m * 100)
			dest:change_alt_frame(1)
			vehicle:set_mode(SoarNavConstants.GUIDED_MODE_CODE)
			vehicle:set_target_location(dest)
			SoarNavGlobals.RTLH.engaged_guided = true
			if log_gcs then
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.RTL_STALL_OVERRIDE, math.floor(alt_m + 0.5))
			end
		end
	end
end

-- The main logic loop of the script, containing the primary state machine.
update_body = function()
	local period = 100
	if
		SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE
		or SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING
		or SoarNavGlobals.script_state == SCRIPT_STATE.WAITING_FOR_ACTIVATION
	then
		period = 50
	end
	if SoarNavGlobals.rpm_check_counter ~= nil and SoarNavGlobals.rpm_check_counter >= 0 then
		SoarNavGlobals.rpm_check_counter = SoarNavGlobals.rpm_check_counter + 1
		if SoarNavGlobals.rpm_check_counter == 5 then
			local rpm1_type = param:get("RPM1_TYPE") or 0
			local rpm2_type = param:get("RPM2_TYPE") or 0
			if rpm1_type > 0 or rpm2_type > 0 then
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.RPM_SENSOR_SET)
			else
				log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.NO_RPM_FALLBACK)
			end
			SoarNavGlobals.rpm_check_counter = -1
		end
	end
	local is_armed_now = arming:is_armed()
	if SoarNavGlobals.last_armed_state == nil then
		SoarNavGlobals.last_armed_state = is_armed_now
	end
	if (not is_armed_now) and SoarNavGlobals.last_armed_state == true and not SoarNavGlobals.restored_on_disarm then
		restore_boot_alts("Disarmed")
		SoarNavGlobals.restored_on_disarm = true
		SoarNavGlobals.activation_requested = false
		SoarNavGlobals.activation_wait_notified = false
	end
	if is_armed_now then
		SoarNavGlobals.restored_on_disarm = false
	end
	SoarNavGlobals.last_armed_state = is_armed_now
	if SoarNavGlobals.initial_soar_alt_min == nil and P.soar_alt_min and P.soar_alt_min:get() ~= nil then
		local min_val = P.soar_alt_min:get() or -1
		local max_val = P.soar_alt_max:get() or -1
		local cutoff_val = P.soar_alt_cutoff:get() or -1
		SoarNavGlobals.initial_soar_alt_min = min_val
		SoarNavGlobals.initial_soar_alt_max = max_val
		SoarNavGlobals.initial_soar_alt_cutoff = cutoff_val
		log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.BASE_ALTS_INFO, min_val, cutoff_val, max_val)
	end
	local current_time_ms = millis()
	local dt_s = 0
	if SoarNavGlobals.last_update_time_ms then
		dt_s = safe_time_diff(current_time_ms, SoarNavGlobals.last_update_time_ms) / 1000.0
	end
	SoarNavGlobals.last_update_time_ms = current_time_ms
	local current_snav_enable = pget(P.enable, 0)
	if current_snav_enable == 0 then
		if SoarNavGlobals.script_state ~= SCRIPT_STATE.IDLE then
			set_script_state(SCRIPT_STATE.IDLE, MSG_ID.SCRIPT_DISABLED)
		end
		return update, 1000
	end

	do
		if (pget(P.radius_m, 0) or 0) <= 0 and rally and rally.get_rally_location then
			local now = millis()
			if
				not SoarNavGlobals.last_rally_poll_ms
				or (safe_time_diff(now, SoarNavGlobals.last_rally_poll_ms) >= 1000)
			then
				SoarNavGlobals.last_rally_poll_ms = now
				local rally_points_table = {}
				local i = 0
				while true do
					local loc = rally:get_rally_location(i)
					if not loc then
						break
					end
					table.insert(rally_points_table, loc)
					i = i + 1
					if i > 100 then
						break
					end
				end
				local count = #rally_points_table
				local signature
				if count > 0 then
					table.sort(rally_points_table, function(a, b)
						if a:lat() == b:lat() then
							return a:lng() < b:lng()
						end
						return a:lat() < b:lat()
					end)
					local sum_lat, sum_lon = 0, 0
					for _, loc in ipairs(rally_points_table) do
						sum_lat = sum_lat + (loc:lat() or 0)
						sum_lon = sum_lon + (loc:lng() or 0)
					end
					signature = string.format("%d|%d|%d", count, sum_lat, sum_lon)
				else
					signature = "0|0|0"
				end
				local count_changed = (count ~= SoarNavGlobals.last_rally_point_count)
				local content_changed = (signature ~= SoarNavGlobals.rally_signature)
				local became_invalid = SoarNavGlobals.using_rally_points and (count < 3)
				local became_valid = (not SoarNavGlobals.using_rally_points) and (count >= 3)
				if count_changed or content_changed or became_invalid or became_valid then
					SoarNavGlobals.last_rally_point_count = count
					SoarNavGlobals.rally_signature = signature
					if (pget(P.radius_m, 0) or 0) <= 0 then
						set_script_state(SCRIPT_STATE.IDLE, MSG_ID.RALLY_CHANGED_REINIT)
						SoarNavGlobals.grid_initialized = false
						SoarNavGlobals.polygon_load_attempted = false
						SoarNavGlobals.area_announced = false
						SoarNavGlobals.using_rally_points = false
						SoarNavGlobals.target_loc = nil
						SoarNavGlobals.activation_requested = false
						SoarNavGlobals.activation_wait_notified = false
						SoarNavGlobals.navigation_start_delay_counter = 0
					end
				end
			end
		end
	end
	local current_snav_radius_m = pget(P.radius_m, 0)
	if
		(current_snav_radius_m ~= SoarNavGlobals.last_snav_radius_m_value)
		or (current_snav_enable ~= SoarNavGlobals.last_snav_enable_value)
	then
		log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.PARAM_CHANGED_REINIT)
		if current_snav_radius_m > 0 then
			log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.RADIUS_MODE_ACTIVATED, current_snav_radius_m)
			SoarNavGlobals.area_announced = false
			announce_radius_area()
		end
		SoarNavGlobals.target_loc = nil
		SoarNavGlobals.is_initializing = false
		SoarNavGlobals.grid_initialized = false
		SoarNavGlobals.polygon_load_attempted = false
		SoarNavGlobals.last_snav_radius_m_value = current_snav_radius_m
		SoarNavGlobals.last_snav_enable_value = current_snav_enable
		SoarNavGlobals.use_polygon_area = false
	end
	local sys_roll_limit = param:get("ROLL_LIMIT_DEG") or ((param:get("LIM_ROLL_CD") or 4500) / 100)
	if sys_roll_limit <= 0 then
		sys_roll_limit = 45
	end
	local cached_params = {
		log_lvl = get_safe_param(P.log_lvl, 1, 0, 1),
		radius_m = get_safe_param(P.radius_m, 500, 0, nil),
		wp_radius = get_safe_param(P.wp_radius, 50, 10, 200),
		tmem_enabled = (get_safe_param(P.tmem_enable, 1, 0, 1) == 1),
		soar_alt_min = get_safe_param(P.soar_alt_min, 100, 0, nil),
		rcmap_roll = P.rcmap_roll:get(),
		rcmap_pitch = P.rcmap_pitch:get(),
		rcmap_yaw = P.rcmap_yaw:get(),
		roll_limit = get_safe_param(P.roll_limit, 30, 10, 50),
		nav_p = get_safe_param(P.nav_p, 0.6),
		nav_d = get_safe_param(P.nav_d, 0.05),
		sys_roll_limit = sys_roll_limit,
	}
	local autotune_chan = rc:find_channel_for_option(107)
	local autotune_active = (autotune_chan and autotune_chan:get_aux_switch_pos() > 0)
	if not validate_params() then
		set_script_state(SCRIPT_STATE.ERROR, MSG_ID.INVALID_SNAV_PARAMS)
		return update, 5000
	end
	if SoarNavGlobals.script_state == SCRIPT_STATE.ERROR then
		return update, 5000
	end
	if not SoarNavGlobals.rc_limits_read and arming:is_armed() then
		local roll_ch_num = P.rcmap_roll:get() or 1
		local pitch_ch_num = P.rcmap_pitch:get() or 2
		local yaw_ch_num = P.rcmap_yaw:get() or 4
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
		if
			not (
				SoarNavGlobals.rc_roll_min < SoarNavGlobals.rc_roll_trim
				and SoarNavGlobals.rc_roll_trim < SoarNavGlobals.rc_roll_max
			)
		then
			log_gcs(
				MAV_SEVERITY.WARNING,
				1,
				MSG_ID.RC_R_LIMITS,
				SoarNavGlobals.rc_roll_min,
				SoarNavGlobals.rc_roll_trim,
				SoarNavGlobals.rc_roll_max
			)
		end
		if not (rc_pitch_min < rc_pitch_trim and rc_pitch_trim < rc_pitch_max) then
			log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.RC_P_LIMITS, rc_pitch_min, rc_pitch_trim, rc_pitch_max)
		end
		if not (rc_yaw_min < rc_yaw_trim and rc_yaw_trim < rc_yaw_max) then
			log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.RC_Y_LIMITS, rc_yaw_min, rc_yaw_trim, rc_yaw_max)
		end
		SoarNavGlobals.rc_limits_read = true
		log_gcs(MAV_SEVERITY.INFO, 1, MSG_ID.RC_LIMITS_READ, roll_ch_num, pitch_ch_num, yaw_ch_num)
	end
	local loc = ahrs:get_location()
	if not loc then
		SoarNavGlobals.location_error_count = SoarNavGlobals.location_error_count + 1
		if SoarNavGlobals.location_error_count > SoarNavConstants.MAX_LOCATION_ERRORS and arming:is_armed() then
			set_script_state(SCRIPT_STATE.ERROR, MSG_ID.PERSISTENT_LOC_ERROR)
		end
		return update, period
	else
		SoarNavGlobals.location_error_count = safe_decrement(SoarNavGlobals.location_error_count)
	end
	if arming:is_armed() and not SoarNavGlobals.grid_initialized and not SoarNavGlobals.is_initializing then
		local should_init = false
		if SoarNavGlobals.force_grid_reinit then
			should_init = true
			SoarNavGlobals.force_grid_reinit = false
		end

		if cached_params.radius_m > 0 then
			SoarNavGlobals.use_polygon_area = false
			SoarNavGlobals.using_rally_points = false
			should_init = true
			if SoarNavGlobals.last_snav_radius_m_value <= 0 then
				SoarNavGlobals.dynamic_center_location = ahrs:get_location():copy()
			end
		else
			if not SoarNavGlobals.polygon_load_attempted then
				if read_rally_points_as_polygon() then
					SoarNavGlobals.use_polygon_area = true
					SoarNavGlobals.using_rally_points = true
					should_init = true
				else
					SoarNavGlobals.using_rally_points = false
					if rally then
						local rally_point_count = 0
						while rally:get_rally_location(rally_point_count) do
							rally_point_count = rally_point_count + 1
						end
						if rally_point_count > 0 and rally_point_count < 3 then
							log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.RPS_NEED_3, rally_point_count)
						end
					end

					local poly_index_to_try = math.min(current_snav_enable, SoarNavConstants.max_polygon_index)
					local poly_loaded = false
					local function try_load_polygon(index)
						local base_filename = string.format(
							"%s%d%s",
							SoarNavConstants.polygon_filename_prefix,
							index,
							SoarNavConstants.polygon_filename_suffix
						)
						local paths_to_try =
							{ "/APM/SCRIPTS/" .. base_filename, "/SCRIPTS/" .. base_filename, base_filename }
						for _, path in ipairs(paths_to_try) do
							if read_polygon_file(path) then
								log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.LOADED_POLY, path)
								return true
							end
						end
						return false
					end

					if try_load_polygon(poly_index_to_try) then
						poly_loaded = true
					else
						log_gcs(MAV_SEVERITY.WARNING, 1, MSG_ID.POLY_FAILED_FALLBACK, poly_index_to_try)
						for i = 1, SoarNavConstants.max_polygon_index do
							if try_load_polygon(i) then
								poly_loaded = true
								break
							end
						end
					end

					if poly_loaded then
						SoarNavGlobals.use_polygon_area = true
						should_init = true
					else
						if rally then
							local rally_point_count = 0
							while rally:get_rally_location(rally_point_count) do
								rally_point_count = rally_point_count + 1
							end
							if rally_point_count == 0 then
								log_gcs(MAV_SEVERITY.CRITICAL, 0, MSG_ID.NO_POLY_DISABLED)
							end
						else
							log_gcs(MAV_SEVERITY.CRITICAL, 0, MSG_ID.NO_POLY_DISABLED)
						end
						SoarNavGlobals.use_polygon_area = false
					end
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
			return update, 50
		end
	end
	local roll_ch_obj = rc:get_channel(cached_params.rcmap_roll)
	local pitch_ch_obj = rc:get_channel(cached_params.rcmap_pitch)
	local yaw_ch_obj = rc:get_channel(cached_params.rcmap_yaw)
	if not roll_ch_obj or not pitch_ch_obj or not yaw_ch_obj then
		log_gcs(MAV_SEVERITY.ERROR, 0, MSG_ID.INVALID_RC_MAPPING)
		return update, 100
	end
	local roll_in = roll_ch_obj:norm_input_dz()
	local pitch_in = pitch_ch_obj:norm_input_dz()
	local yaw_in = yaw_ch_obj:norm_input_dz()
	local is_outside_roll_dz = math.abs(roll_in) > 0
	local is_outside_pitch_dz = math.abs(pitch_in) > 0
	local is_outside_yaw_dz = math.abs(yaw_in) > 0
	local gesture_just_completed = false
	if
		SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING
		or SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE
	then
		gesture_just_completed = check_roll_gesture()
	elseif SoarNavGlobals.script_state == SCRIPT_STATE.WAITING_FOR_ACTIVATION then
		local __mode = vehicle:get_mode()
		if __mode == SoarNavConstants.mode_fbwb or __mode == SoarNavConstants.mode_cruise then
			gesture_just_completed = check_roll_gesture()
		end
	end

	check_pitch_gesture()

	if
		not gesture_just_completed
		and not SoarNavGlobals.pitch_gesture_triggered_this_override
		and SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING
		and (math.abs(pitch_in) > 0.05 or math.abs(yaw_in) > 0.05)
		and not (
			SoarNavGlobals.activation_grace_start_ms
			and safe_time_diff(current_time_ms, SoarNavGlobals.activation_grace_start_ms)
				< SoarNavConstants.ACTIVATION_OVERRIDE_GRACE_MS
		)
	then
		set_script_state(SCRIPT_STATE.PILOT_OVERRIDE, MSG_ID.PILOT_OVERRIDE)
		SoarNavGlobals.last_pilot_input_ms = current_time_ms
		SoarNavGlobals.pitch_gesture_state = "idle"
		SoarNavGlobals.pitch_gesture_count = 0
		SoarNavGlobals.pitch_gesture_triggered_this_override = false
		SoarNavGlobals.manual_override_active = false
	end

	local current_mode = vehicle:get_mode()
	SoarNav_RTLH_update(current_time_ms)
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
		if strength <= 0 then
			strength = 1.0
		end
		local sample_interval = math.max(500, math.min(3000, 2000 / (strength + 0.5)))
		if time_since_sample_ms > 0 and (time_since_sample_ms > sample_interval) then
			sample_thermal_strength()
		end
	end
	local script_switch_high = (rc:get_aux_cached(SoarNavConstants.rc_opt_soaring_active) == 2)
	local home = ahrs:get_home()
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	local can_navigate = false
	if
		arming:is_armed()
		and script_switch_high
		and not autotune_active
		and (current_mode == SoarNavConstants.mode_fbwb or current_mode == SoarNavConstants.mode_cruise)
		and home
		and dist_from_home_3d
		and ahrs:get_yaw_rad()
		and SoarNavGlobals.grid_initialized
	then
		local current_alt = -dist_from_home_3d:z()
		if SoarNavGlobals.initial_soar_alt_min and current_alt >= (SoarNavGlobals.initial_soar_alt_min - 5) then
			can_navigate = true
		end
	end
	if
		can_navigate
		and SoarNavGlobals.initial_soar_alt_min
		and SoarNavGlobals.script_state ~= SCRIPT_STATE.PILOT_OVERRIDE
		and not SoarNavGlobals.manual_override_active
	then
		local throttle_output = SRV_Channels:get_output_scaled(SoarNavConstants.THROTTLE_SRV_CHANNEL) or 0
		local motor_is_off = throttle_output < 10
		local now = millis()
		if
			motor_is_off
			and (
				(SoarNavGlobals.last_glide_cone_update_ms == nil)
				or (safe_time_diff(now, SoarNavGlobals.last_glide_cone_update_ms) > 15000)
			)
		then
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
	elseif SoarNavGlobals.script_state == SCRIPT_STATE.WAITING_FOR_ACTIVATION then
		handle_waiting(can_navigate)
	end
	if
		arming:is_armed()
		and SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING
		and (SRV_Channels:get_output_scaled(SoarNavConstants.THROTTLE_SRV_CHANNEL) or 0) < SoarNavConstants.THROTTLE_ACTIVE_THRESHOLD
		and current_mode ~= SoarNavConstants.mode_thermal
	then
		update_polar_learning()
	end
	check_motor_failure(current_mode, dist_from_home_3d)
	return update, period
end

-- A safe wrapper that executes the main loop and catches any runtime errors.
update = function()
	local results = { pcall(update_body) }
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

log_gcs(MAV_SEVERITY.NOTICE, 1, MSG_ID.SCRIPT_INITIALIZED)

return update()
