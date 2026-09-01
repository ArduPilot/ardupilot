--[[
SoarNav: an advanced feature for hunting thermals by Marco Robustini.
-- Version 1.2.0 - 2025/12/12

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
  APIs, a modular state machine, parameter caching, and a new configurable
  task scheduler for fine-tuning performance.

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
- SNAV_LOG_LVL: GCS log verbosity (0:Silent, 1:Events, 2:Debug).
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
  2:MIN Only (capped), 3:Pure Glider (Evasive).
- SNAV_GC_MARGIN: Safety altitude margin (m) added to the calculated glide
  altitude for the Glide Cone feature.
- SNAV_GC_PAD: Extra altitude padding (m) to ensure arrival over home point.
- SNAV_TE_LOOK_S: Lookahead time (s) for Terrain Evasion. Higher is more
  sensitive/conservative.
- SNAV_TE_BUF_MIN: Absolute minimum AGL buffer (m) for Terrain Evasion.

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

- SoarNav: 🎯 [Terrain Evasion] 248°WSW 500m
  Executing a terrain avoidance maneuver. Targeting a temporary safe
  location.

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
                         Terrain Evasion Messages
--------------------------------------------------------------------------------
- SoarNav: 🎯 [Terrain Evasion] 045°NE → 420m
  **Terrain Evasion Active**: The system detected unsafe terrain ahead and is
  commanding an evasive turn. The arrow (→ right, ← left) indicates turn
  direction. Aircraft is heading 045° for 320m to clear the obstacle.

- SoarNav: 🎯 [TE Egress] 180°S ← 550m
  **Terrain Evasion Egress**: The evasion maneuver is complete. The aircraft
  is now flying to an egress waypoint to safely resume normal navigation.
  The arrow indicates the direction of the last evasive turn.

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

-- luacheck: ignore 212 (Unused argument)
---@diagnostic disable: undefined-field
---@diagnostic disable: deprecated
---@diagnostic disable: cast-local-type
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

-- ============================================================================
-- SCRIPT MASTER TABLE
-- ============================================================================
-- Encapsulates all script variables and functions into a single table
-- to solve the 100-global-variable limit.
-- ============================================================================
local SoarNav = {}

-- ============================================================================
-- MESSAGING & LOGGING
-- ============================================================================
SoarNav.Msg = {
	-- Mavlink severity levels for GCS messages
	MAV_SEVERITY = {
		CRITICAL = 2,
		ERROR = 3,
		WARNING = 4,
		NOTICE = 5,
		INFO = 6,
	},
	-- [[[ GCS MESSAGES — CENTRALIZED, ID-BASED ]]]
	MSG_ID = {
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
		TERRAIN_AVOID_MANEUVER = 78,
		TERRAIN_AVOID_DEBUG = 79,
		TERRAIN_DATA_MISSING_WARN = 80,
		TERRAIN_AVOID_DEBUG_EXT = 81,
		TERRAIN_DATA_UNRELIABLE = 82,
	},
	-- === Output Symbol Set ===
	SNAV_USE_UNICODE = true,
	SYM = "🎯",
	DEG = "°",
	LEQ = "≤",
	-- Message string table
	MSG = {},
}
-- Initialize symbol placeholders
SoarNav.Msg.SYM = SoarNav.Msg.SNAV_USE_UNICODE and "🎯" or "*"
SoarNav.Msg.DEG = SoarNav.Msg.SNAV_USE_UNICODE and "°" or "deg"
SoarNav.Msg.LEQ = SoarNav.Msg.SNAV_USE_UNICODE and "≤" or "<="
-- Populate the MSG table
SoarNav.Msg.MSG = {
	[SoarNav.Msg.MSG_ID.AREA_INFO_POLY] = "%s A=%.2fkm2;RP=%u;MD=%.0fm",
	[SoarNav.Msg.MSG_ID.PARAM_OUT_OF_RANGE] = "%s out of range [%s..%s]: %.3f",
	[SoarNav.Msg.MSG_ID.GESTURE_ACTIVATED] = "Activation gesture received.",
	[SoarNav.Msg.MSG_ID.ALL_STRAT_FAILED] = "All strategies failed. Using fallback.",
	[SoarNav.Msg.MSG_ID.AUTO_START] = "Auto-start: gesture not required.",
	[SoarNav.Msg.MSG_ID.AWAITING_ACTIVATION] = "Awaiting pilot activation (ROLL gesture).",
	[SoarNav.Msg.MSG_ID.BASE_ALTS_INFO] = "Base Alts: MIN %.0f, CUTOFF %.0f, MAX %.0f",
	[SoarNav.Msg.MSG_ID.NO_THERMAL_LOC] = "Can't monitor thermal: no location.",
	[SoarNav.Msg.MSG_ID.CELL_INDEX_FAIL] = "Cell index fail, fallback.",
	[SoarNav.Msg.MSG_ID.DISARMED] = "Disarmed.",
	[SoarNav.Msg.MSG_ID.ERR_ROLL_NIL] = "ERR: Roll channel is nil. Cannot override.",
	[SoarNav.Msg.MSG_ID.EFF_CLUSTER_RADIUS] = "Eff. Cluster Radius: %.0fm",
	[SoarNav.Msg.MSG_ID.GRID_READY] = "Exploration grid ready.",
	[SoarNav.Msg.MSG_ID.FATAL_NO_WP] = "FATAL: Could not set any valid waypoint.",
	[SoarNav.Msg.MSG_ID.FOCUS_ON_DENSITY] = "Focus Mode ON (Density)",
	[SoarNav.Msg.MSG_ID.FORCING_NEW_TARGET] = "Forcing new target search...",
	[SoarNav.Msg.MSG_ID.RPS_NEED_3] = "Found %d RPs. Need >= 3. Using file.",
	[SoarNav.Msg.MSG_ID.GCONE_RST_OVERRIDE] = "GC Rst(Override)->MIN:%.0f C:%.0f M:%.0f",
	[SoarNav.Msg.MSG_ID.GCONE_OFF_NO_PARAMS] = "GCone off: set POLAR_CD0/B & AIRSPEED.",
	[SoarNav.Msg.MSG_ID.GCONE_ALT_UPDATE] = "Glide Cone: Safe return alt now %.0fm",
	[SoarNav.Msg.MSG_ID.GRID_TOO_SMALL] = "Grid area too small. Init aborted.",
	[SoarNav.Msg.MSG_ID.GRID_INIT_FAIL_NO_CENTER] = "Grid init failed: no active center.",
	[SoarNav.Msg.MSG_ID.GRID_INIT_STARTED] = "Grid init started...",
	[SoarNav.Msg.MSG_ID.GRID_VALIDATED] = "Grid validated: %d cells.",
	[SoarNav.Msg.MSG_ID.GRID_SCAN_INFO] = "Grid: %d rows, %d cols. Scan cells...",
	[SoarNav.Msg.MSG_ID.INVALID_RC_MAPPING] = "Invalid RC channel mapping.",
	[SoarNav.Msg.MSG_ID.INVALID_SNAV_PARAMS] = "Invalid SNAV params. Disabling script.",
	[SoarNav.Msg.MSG_ID.LOADED_POLY] = "Loaded polygon '%s'",
	[SoarNav.Msg.MSG_ID.MOTOR_FAIL_RTL] = "MOTOR FAILURE DETECTED! RTL ACTIVATED.",
	[SoarNav.Msg.MSG_ID.MOTOR_FAIL_REASON] = "Motor Failure RTL.",
	[SoarNav.Msg.MSG_ID.NAV_COND_MET] = "Navigation conditions met, starting.",
	[SoarNav.Msg.MSG_ID.NAV_COND_NOT_MET] = "Navigation conditions no longer met.",
	[SoarNav.Msg.MSG_ID.NO_RPM_FALLBACK] = "No RPM sensor, using climb-rate fallback.",
	[SoarNav.Msg.MSG_ID.NO_POLY_DISABLED] = "No poly (Rally/File). Mode disabled.",
	[SoarNav.Msg.MSG_ID.NO_PROG_WARN] = "NoProg E%.2f | W%.1fm/s@%.0f",
	[SoarNav.Msg.MSG_ID.PARAM_CHANGED_REINIT] = "Parameter changed. Re-initializing area.",
	[SoarNav.Msg.MSG_ID.PERSISTENT_LOC_ERROR] = "Persistent location error, disabling.",
	[SoarNav.Msg.MSG_ID.PILOT_ACTIVATION] = "Pilot activation received.",
	[SoarNav.Msg.MSG_ID.PILOT_OVERRIDE] = "Pilot override detected.",
	[SoarNav.Msg.MSG_ID.POLY_INVALID_POINTS] = "Polygon invalid: less than 3 points",
	[SoarNav.Msg.MSG_ID.POLY_MAX_DIST] = "Polygon max distance calculated: %.0fm.",
	[SoarNav.Msg.MSG_ID.RC_P_LIMITS] = "RC P lim: %d/%d/%d",
	[SoarNav.Msg.MSG_ID.RC_R_LIMITS] = "RC R lim: %d/%d/%d",
	[SoarNav.Msg.MSG_ID.RC_Y_LIMITS] = "RC Y lim: %d/%d/%d",
	[SoarNav.Msg.MSG_ID.RC_LIMITS_READ] = "RC limits read for R:%d, P:%d, Y:%d",
	[SoarNav.Msg.MSG_ID.RPM_SENSOR_SET] = "RPM sensor set, motor check uses RPM.",
	[SoarNav.Msg.MSG_ID.RADIUS_MODE_ACTIVATED] = "Radius Mode activated: %.0fm.",
	[SoarNav.Msg.MSG_ID.RALLY_CHANGED_REINIT] = "Rally points changed; reinit.",
	[SoarNav.Msg.MSG_ID.REROUTE_SKIP_HEADING] = "Re-route skipped: heading unavailable",
	[SoarNav.Msg.MSG_ID.REPOSITIONED] = "Repositioned. Re-engaging.",
	[SoarNav.Msg.MSG_ID.REROUTE_SKIP_NO_TARGET] = "Reroute skipped: no target.",
	[SoarNav.Msg.MSG_ID.RESUMING_NAV] = "Resuming navigation.",
	[SoarNav.Msg.MSG_ID.SNAV_REROUTE_MINMAX] = "SNAV REROUTE_MIN {LEQ} REROUTE_MAX.",
	[SoarNav.Msg.MSG_ID.SNAV_RADIUS_NEGATIVE] = "SNAV_RADIUS_M cannot be negative.",
	[SoarNav.Msg.MSG_ID.AREA_INFO_RADIUS] = "Radius A=%.2fkm2;RD=%.0fm",
	[SoarNav.Msg.MSG_ID.SCRIPT_INITIALIZED] = "Script initialized.",
	[SoarNav.Msg.MSG_ID.SCRIPT_DISABLED] = "Script disabled by user.",
	[SoarNav.Msg.MSG_ID.SET_CRUISE_FBWB] = "Set Cruise/FBWB to resume SoarNav.",
	[SoarNav.Msg.MSG_ID.GCS_PREFIX] = "SoarNav: %s",
	[SoarNav.Msg.MSG_ID.MISSING_PARAM] = "SoarNav: Missing parameter: %s",
	[SoarNav.Msg.MSG_ID.PARAM_SKIP] = "SoarNav: SoarNav parameters skipped: %s",
	[SoarNav.Msg.MSG_ID.STICK_CMD_MANUAL_OFF] = "Stick CMD: Manual override OFF.",
	[SoarNav.Msg.MSG_ID.STICK_CMD_MANUAL_ON] = "Stick CMD: Manual override ON.",
	[SoarNav.Msg.MSG_ID.STICK_CMD_POLY_RECENTER] = "Stick CMD: Polygon re-centered.",
	[SoarNav.Msg.MSG_ID.STICK_CMD_RADIUS_RECENTER] = "Stick CMD: Radius Area re-centered.",
	[SoarNav.Msg.MSG_ID.STICK_CMD_STUCK] = "Stuck! Repositioning %.0fm, offset %.0f",
	[SoarNav.Msg.MSG_ID.THERMAL_EXIT_NO_DATA] = "Thermal exit: no data. (Lost Thermal)",
	[SoarNav.Msg.MSG_ID.THERMAL_FOUND_EXIT_FOCUS] = "Thermal found, exiting focus.",
	[SoarNav.Msg.MSG_ID.THERMAL_IGNORED_OOB] = "Thermal ignored: out of area.",
	[SoarNav.Msg.MSG_ID.POLY_FAILED_FALLBACK] = "snav%d.poly failed. Using fallback.",
	[SoarNav.Msg.MSG_ID.NAV_TARGET_SIMPLE] = "{SYM} [%s] %.0f{DEG}%s %.0fm",
	[SoarNav.Msg.MSG_ID.NAV_TARGET_CELLS] = "{SYM} [%s] %.0f{DEG}%s %.0fm %d/%d",
	[SoarNav.Msg.MSG_ID.NAV_TARGET_REROUTE] = "{SYM} [Re-route] %.0f{DEG}%s %.0fm %d/%d",
	[SoarNav.Msg.MSG_ID.RTL_STALL_OVERRIDE] = "RTL Stall: Force direct Home route @%dm",
	[SoarNav.Msg.MSG_ID.RTL_OVERRIDE_RESUME] = "RTL Override: In home area. Loitering.",
	[SoarNav.Msg.MSG_ID.NAV_TARGET_RIDGE] = "{SYM} [Ridge] %.0f{DEG}%s %.0fm s=%.2f",
	[SoarNav.Msg.MSG_ID.POLAR_LEARN_UPDATE] = "Polar Learn: CD0=%.4f B=%.4f",
	[SoarNav.Msg.MSG_ID.TERRAIN_AVOID_MANEUVER] = "TEvas: Path AGL %.1f < Buf %.1f",
	[SoarNav.Msg.MSG_ID.TERRAIN_AVOID_DEBUG] = "TE: Fa%.0f Ba%.0f B%.0f D%d L%.0f",
	[SoarNav.Msg.MSG_ID.TERRAIN_DATA_MISSING_WARN] = "TEvas: Terrain data missing!",
	[SoarNav.Msg.MSG_ID.TERRAIN_AVOID_DEBUG_EXT] = "TE: M%.0f Df%.0f R%.1f T%.0f S%d Z%d",
	[SoarNav.Msg.MSG_ID.TERRAIN_DATA_UNRELIABLE] = "TE: Data unreliable!",
}

-- ============================================================================
-- SCRIPT STATE & CONSTANTS
-- ============================================================================

-- Script operational states
SoarNav.State = {
	SCRIPT_STATE = {
		IDLE = 0,
		NAVIGATING = 1,
		PILOT_OVERRIDE = 2,
		THERMAL_PAUSE = 3,
		ERROR = 4,
		WAITING_FOR_ACTIVATION = 5,
	},
	TE_STATE = {
		IDLE = 0,
		MONITORING = 1,
		EVADING = 2,
		HOLD = 3,
	},
	-- Defines the source strings for GCS navigation messages.
	SoarNavWaypointSources = {
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
		TERRAIN_EVASION = "Terrain Evasion",
        TE_EGRESS = "TE Egress",
		UNKNOWN = "Unknown",
	},
	-- Global table for managing the script's state (formerly SoarNavGlobals)
	---------------------------------------------------------------------------
	-- 1. SCRIPT CORE STATE & NAVIGATION
	---------------------------------------------------------------------------
	script_state = 0, -- SCRIPT_STATE.IDLE
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
	polar_learn = true,
	last_nav_mode = nil,
	nav_mode_before_thermal = nil,
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
	polygon_xy = {},
	polygon_origin = nil,
	polygon_edges = {},
	polygon_is_convex = false,
	_seg_cache = {},
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
	force_grid_reinit = false,
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
	focus_start_time = nil,
	sink_best_ema = 0.9,
	ridge_last_ms = 0,
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
	TE = {
		hold_until_ms = 0,
		next_check_ms = 0,
		tevas_terrain_offset_m = 0,
		tevas_last_offset_update_ms = 0,
		evasion_active = false,
		last_evasion_dir = 0,
		last_evasion_ms = 0,
		last_set_target_ms = 0,
		last_te_msg_ms = 0,
		p_gain_scaler = 1.0,
		turn_arrow = "",
        last_safe_heading = nil,
		target_heading = nil,
		target_lever_m = nil,
		last_logged_heading = nil,
		scan_state = 0,
		scan_cand_idx = 0,
		scan_candidates = nil,
		scan_best = nil,
		scan_ctx = nil,
		state = 0,
		margin = 0,
		deficit = 0,
		deficit_ratio = 0,
		T_eff = 0,
		T_egress = 0,
		dir_new = 0,
		dir_old = 0,
		improvement = 0,
		speed_ref = 15,
		data_fail_count = 0,
		zone = 0,
		threat_type = "none",
	},
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
	polar_est = nil,
	gc_reset_hold_start_ms = nil,
	override_reset_done = false,
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
	require_initial_activation_gesture = false,
	activation_grace_start_ms = nil,
	last_stick_tick_ms = 0,
	force_new_search = false,
	force_egress = false,
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
		_mode = nil,
	},
	last_gcone_log_ms = nil,
	last_gcone_logged_alt = nil,
	_alt_gate_state = false,
	---------------------------------------------------------------------------
	-- 7. RC CONTROL & HARDWARE INTERFACE
	---------------------------------------------------------------------------
	rc_roll_channel = nil,
	rc_limits_read = false,
	last_heading_error = 0,
	last_commanded_roll_deg = 0,
	rc_roll_min = 1000,
	rc_roll_max = 2000,
	rc_roll_trim = 1500,
}
-- Initialize SCRIPT_STATE.IDLE
SoarNav.State.script_state = SoarNav.State.SCRIPT_STATE.IDLE

-- Static constants used throughout the script
SoarNav.Const = {
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
    HOME_CYLINDER_MULT = 4.0,
	-- Thermal Logic
	THERMAL_CONSISTENCY_VARIANCE_THRESHOLD = 0.5,
	MIN_HOTSPOTS_FOR_STREET = 2,
	CRITICAL_TREND_HYSTERESIS_MS = 3000,
	-- Navigation & Control
	WP_TIMEOUT_WIND_MODE = 1,
	WP_TIMEOUT_WIND_REF_MPS = 15,
	WP_TIMEOUT_WIND_MIN_MULT = 0.5,
	WP_TIMEOUT_WIND_MAX_MULT = 2.5,
	ROLL_SMOOTHING_FACTOR = -0.2,
	HUMANIZE_PILOT = 1,
	HUMAN_DWELL_MIN_MS = 200,
	HUMAN_DWELL_MAX_MS = 500,
	HUMAN_GAIN_JITTER = 0.05,
	HUMAN_BIAS_RW_DEG_PER_S = 0.10,
	HUMAN_BIAS_MAX_DEG = 1.0,
	HUMAN_MIN_BANK_DEG = 2.0,
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
	grid_init_cells_per_call = 25,
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

-- ============================================================================
-- SCHEDULER CONFIGURATION
-- ============================================================================
-- Configurable update intervals (in milliseconds) for "micro-taratura".
-- These control the execution frequency of non-critical tasks.
-- ============================================================================
SoarNav.Config = {
	Scheduler = {
		-- Main loop period when NAVIGATING or in OVERRIDE
		MAIN_LOOP_FAST_MS = 50,
		-- Main loop period when IDLE or in THERMAL
		MAIN_LOOP_SLOW_MS = 100,
		-- How often to check for parameter changes (SNAV_ENABLE, RADIUS_M)
		PARAM_CHECK_MS = 2000,
		-- How often to update the cached parameter values (SNavCache)
		CACHE_UPDATE_MS = 2000,
		-- How often to check for Rally Point changes
		RALLY_POLL_MS = 2000,
		-- How often to run the RTLH (RTL Helper) logic
		RTLH_UPDATE_MS = 500,
		-- How often to check for motor failure (when applicable)
		MOTOR_FAILURE_MS = 1000,
		-- How often to update the Glide Cone (Dynamic SOAR_ALT_MIN)
		GLIDE_CONE_MS = 20000,
		-- How often to run the Polar Learning (when active)
		POLAR_LEARN_MS = 2000,
		-- How often to run the grid cell visit counter (active navigation only)
		VISITED_CELL_MS = 1500,
		-- How often to run the Terrain Evasion (TE) logic (when active)
		-- Note: The TE logic has its own internal timer, this is a fallback.
		TEVAS_UPDATE_MS = 300,
		-- How often to check for stick gestures
		STICK_GESTURE_MS = 250,
		-- CPU budget per cycle in milliseconds (soft limit)
		CPU_BUDGET_MS = 8,
	},
}

-- ============================================================================
-- TERRAIN EVASION CONFIGURATION (Velocity-Adaptive Constants)
-- ============================================================================
SoarNav.TE_Config = {
	T_LOOKAHEAD_MIN     = 5.0,
	T_SPIKE_CHECK       = 1.0,
	T_SAMPLE_OFFSET     = 0.7,
	T_EARLY_EXIT        = 0.8,
	T_PATH_SKIP         = 4.5,
	T_LEVER_MAX         = 10.0,
	T_BUFFER_BASE       = 2.5,
	T_BUFFER_MAX        = 15.0,
	T_EGRESS_BASE       = 5.0,
	T_HOLD_BASE         = 0.6,
	T_HOLD_MAX_MULT     = 2.0,
	D_GATE              = 35,
	D_SIDE_COOLDOWN     = 70,
	D_OFFSET_INTERVAL   = 25,
	D_GATE_ROLL_BONUS   = 15,
	FLOOR_BUFFER        = 40,
	FLOOR_LOOKAHEAD     = 80,
	FLOOR_LEVER         = 60,
	K_FAN_HALF_MAX      = 1650,
	K_FAN_HALF_MIN      = 420,
	K_ANGLE_STEP        = 520,
	K_TURN_MAX_BASE     = 900,
	K_TURN_MAX_CAP      = 1200,
	K_DUPLICATE_THR     = 270,
	T_MIN_GAIN          = 0.4,
	T_MIN_GAIN_SIDE     = 0.7,
	PCT_MIN_GAIN        = 0.10,
	PCT_MIN_GAIN_SIDE   = 0.20,
	PCT_SAFE_MARGIN     = 0.25,
	PCT_BUFFER_HI       = 0.30,
	ALPHA_TERRAIN_BASE  = 0.02,
	ALPHA_TERRAIN_K     = 1000,
	ALPHA_GS_BASE       = 0.06,
	ALPHA_GS_K          = 500,
	ALPHA_VARIO_BASE    = 0.10,
	ALPHA_VARIO_K       = 400,
	P_GAIN_INCR_BASE    = 0.3,
	P_GAIN_INCR_K       = 100,
	P_DEFICIT_MULT_BASE = 0.8,
	P_DEFICIT_MULT_K    = 75,
	SINK_LEE_FACTOR     = 0.5,
	SAFETY_PATH_FACTOR  = 1.15,
	NUM_SEGMENT_SAMPLES = 3,
	NUM_HEADING_SAMPLES = 2,
	SPEED_MIN           = 8,
	SPEED_MAX           = 80,
	H_GATE_T_MULT       = 1.4,
	H_GATE_BUF_MULT     = 4.0,
	LAT_H_GATE_T_MULT   = 1.0,
	LAT_H_GATE_BUF_MULT = 3.0,
	FRONT_DIST_FRACS    = {0.4, 0.7, 1.0},
	SIDE_R_FRACS        = {0.8, 1.4},
	SIDE_T_FRACS        = {0.2, 0.35},
	SEGMENT_SPACING     = {0.1, 0.2, 0.35, 0.5, 0.65, 0.8, 0.9, 1.0},
	HEADING_SPACING     = {0.2, 0.4, 0.6, 0.8, 1.0},
	THREAT_FRONT_OFFSETS = {35, -35},
	THREAT_SIDE_OFFSETS  = {20, -20},
	PENALTY_ANGLE_THR   = 80,
	PENALTY_MULT        = 3.0,
	EMERGENCY_DEFICIT   = 1.5,
	EMERGENCY_AGL_GAIN  = 50,
	ROLL_THR_GATE       = 10,
	ROLL_THR_HOLD       = 12,
	LEVER_T_FACTORS     = {3.0, 1.5, 3.0},
	RANGEFINDER_MAX     = 500,
	DATA_FAIL_LIMIT     = 3,
}

-- ============================================================================
-- SCHEDULER STATE
-- ============================================================================
-- Holds the last-run timestamps for the task scheduler.
-- ============================================================================
SoarNav.Scheduler = {
	param_check_ms = 0,
	cache_update_ms = 0,
	rally_poll_ms = 0,
	rtlh_update_ms = 0,
	motor_failure_ms = 0,
	glide_cone_ms = 0,
	polar_learn_ms = 0,
	visited_cell_ms = 0,
	stick_gesture_ms = 0,
	tevas_update_ms = 0,
}

-- ============================================================================
-- PARAMETER MANAGEMENT
-- ============================================================================

-- Single table for all parameter handles
SoarNav.Params = {}
-- Single cache table for all parameters, updated periodically
SoarNav.Cache = {}

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
 // @Values: 0:Silent,1:Events,2:Debug
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
 // @Description: Lifetime
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
 // @Description: Controls the dynamic altitude (Glide Cone) behavior. 0:Disabled, 1:Fully Linked (MIN/CUTOFF/MAX), 2:MIN Only (capped at 2/3 of CUTOFF), 3:Pure Glider (Evasive terrain avoidance).
 // @Values: 0:Disabled,1:Fully Linked,2:MIN Only (Capped),3:Pure Glider (Evasive)
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
--[[
 // @Param: SNAV_TE_LOOK_S
 // @DisplayName: Terrain Evasion Lookahead Time
 // @Description: Base lookahead time in seconds for terrain scanning. Higher values make the system more conservative, detecting obstacles earlier.
 // @Units: s
]]
--[[
 // @Param: SNAV_TE_BUF_MIN
 // @DisplayName: Terrain Evasion Minimum Buffer
 // @Description: Absolute minimum AGL buffer in meters that the terrain evasion system will always maintain regardless of other factors.
 // @Units: m
]]

-- ============================================================================
-- HELPER FUNCTIONS (UTILITIES)
-- ============================================================================

-- Formats a message string by ID and arguments.
function SoarNav:msgf_id(id, ...)
	local t = self.Msg.MSG[id]
	if t == nil then
		t = string.format("<MISSING MSG %s>", tostring(id))
	end
	t = t:gsub("{SYM}", self.Msg.SYM):gsub("{DEG}", self.Msg.DEG):gsub("{LEQ}", self.Msg.LEQ)
	local ok, res = pcall(string.format, t, ...)
	if ok then
		return res
	else
		return t
	end
end

-- Sends a formatted message to the GCS.
function SoarNav:send_gcs_id(sev, id, ...)
	return gcs:send_text(sev, self:msgf_id(id, ...))
end

-- Centralized function for sending formatted GCS messages.
function SoarNav:log_gcs(sev, lvl, id, ...)
	local cur = self.Cache.log_lvl or 1
	if cur < lvl then
		return
	end
	self:send_gcs_id(sev, self.Msg.MSG_ID.GCS_PREFIX, self:msgf_id(id, ...))
end

-- Creates and registers all SNAV_* parameters with ArduPilot.
function SoarNav:add_params()
	local PARAM_TABLE_KEY = 111
	local PARAM_PREFIX = "SNAV_"
	-- List of all parameters managed by this script.
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
        { name = "TE_LOOK_S", default = 10 },
		{ name = "TE_BUF_MIN", default = 80 },
	}
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
		self:send_gcs_id(self.Msg.MAV_SEVERITY.WARNING, self.Msg.MSG_ID.PARAM_SKIP, tostring(err))
	end
end

-- Safely retrieves a parameter value from its handle.
function SoarNav:pget(ph, default)
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

-- Helper function for safely binding parameters.
function SoarNav:bind_param(name)
	local ok, p_or_err = pcall(Parameter, name)
	if not ok then
		self:send_gcs_id(self.Msg.MAV_SEVERITY.WARNING, self.Msg.MSG_ID.MISSING_PARAM, name)
		return nil
	end
	return p_or_err
end

-- Binds all needed parameters (handles) and caches them for fast safe get/set.
function SoarNav:bind_params()
	local PARAM_PREFIX = "SNAV_"
	self.Params.enable = self:bind_param(PARAM_PREFIX .. "ENABLE")
	self.Params.log_lvl = self:bind_param(PARAM_PREFIX .. "LOG_LVL")
	self.Params.radius_m = self:bind_param(PARAM_PREFIX .. "RADIUS_M")
	self.Params.nav_d = self:bind_param(PARAM_PREFIX .. "NAV_D")
	self.Params.nav_p = self:bind_param(PARAM_PREFIX .. "NAV_P")
	self.Params.rcmap_pitch = self:bind_param("RCMAP_PITCH")
	self.Params.rcmap_roll = self:bind_param("RCMAP_ROLL")
	self.Params.rcmap_yaw = self:bind_param("RCMAP_YAW")
	self.Params.roll_limit = self:bind_param(PARAM_PREFIX .. "ROLL_LIMIT")
	self.Params.soar_alt_max = self:bind_param("SOAR_ALT_MAX")
	self.Params.soar_alt_min = self:bind_param("SOAR_ALT_MIN")
	self.Params.tmem_enable = self:bind_param(PARAM_PREFIX .. "TMEM_ENABLE")
	self.Params.tmem_life = self:bind_param(PARAM_PREFIX .. "TMEM_LIFE")
	self.Params.wp_radius = self:bind_param(PARAM_PREFIX .. "WP_RADIUS")
	self.Params.stuck_eff = self:bind_param(PARAM_PREFIX .. "STUCK_EFF")
	self.Params.stuck_time = self:bind_param(PARAM_PREFIX .. "STUCK_TIME")
	self.Params.reeng_dwell = self:bind_param(PARAM_PREFIX .. "REENG_DWELL")
	self.Params.retry_thr = self:bind_param(PARAM_PREFIX .. "RETRY_THR")
	self.Params.street_tol = self:bind_param(PARAM_PREFIX .. "STREET_TOL")
	self.Params.reroute_p = self:bind_param(PARAM_PREFIX .. "REROUTE_P")
	self.Params.nec_weight = self:bind_param(PARAM_PREFIX .. "NEC_WEIGHT")
	self.Params.tmem_min_s = self:bind_param(PARAM_PREFIX .. "TMEM_MIN_S")
	self.Params.focus_thr = self:bind_param(PARAM_PREFIX .. "FOCUS_THR")
	self.Params.strat_hist = self:bind_param(PARAM_PREFIX .. "STRAT_HIST")
	self.Params.wp_timeout = self:bind_param(PARAM_PREFIX .. "WP_TIMEOUT")
	self.Params.reroute_min = self:bind_param(PARAM_PREFIX .. "REROUTE_MIN")
	self.Params.reroute_max = self:bind_param(PARAM_PREFIX .. "REROUTE_MAX")
	self.Params.soar_alt_cutoff = self:bind_param("SOAR_ALT_CUTOFF")
	self.Params.soar_polar_cd0 = self:bind_param("SOAR_POLAR_CD0")
	self.Params.soar_polar_b = self:bind_param("SOAR_POLAR_B")
	self.Params.airspeed_cruise = self:bind_param("AIRSPEED_CRUISE")
	self.Params.dyn_soalt = self:bind_param(PARAM_PREFIX .. "DYN_SOALT")
	self.Params.gc_margin = self:bind_param(PARAM_PREFIX .. "GC_MARGIN")
	self.Params.gc_pad = self:bind_param(PARAM_PREFIX .. "GC_PAD")
    self.Params.te_look_s = self:bind_param(PARAM_PREFIX .. "TE_LOOK_S")
	self.Params.te_buf_min = self:bind_param(PARAM_PREFIX .. "TE_BUF_MIN")
end

-- Safely retrieves a parameter value with optional default and min/max clamping.
function SoarNav:get_safe_param(p, default, min, max)
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
function SoarNav:safe_time_diff(newer, older)
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

-- Returns a cached wind vector, refreshing it periodically to save resources.
function SoarNav:get_wind_vector()
	local G = self.State
	local now = millis()
	if not G.wind_cache_time then
		G.wind_cache_time = now
	end
	if not G.cached_wind or self:safe_time_diff(now, G.wind_cache_time) > 5000 then
		G.cached_wind = ahrs:wind_estimate()
		G.wind_cache_time = now
	end
	local wind = G.cached_wind
	if not wind or wind:length() < 0.1 then
		local zero_vector = Vector3f()
		zero_vector:x(0)
		zero_vector:y(0)
		zero_vector:z(0)
		return zero_vector
	end
	return wind
end

-- Generates a random waypoint within a radius of a center point.
function SoarNav:generate_target_around_point(center_loc, radius_m)
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
function SoarNav:get_wind_corrected_heading_deg(current_loc, target_loc)
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
	local wind = self:get_wind_vector()
	local wind_mps = (wind and wind:length()) or 0
	local Va = self.Cache.airspeed_cruise or 15

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
function SoarNav:wind_vector_to_bearing_deg(w)
	if not w or type(w.xy) ~= "function" then
		return 0
	end
	local ok, ang = pcall(function()
		return (math.deg(w:xy():angle()) + 360) % 360
	end)
	if not ok or not ang then
		return 0
	end
	return (450 - ang) % 360
end

-- Estimates ridge-lift suitability score at a location using local slope and wind alignment.
function SoarNav:ridge_score_at_loc(loc)
	local G = self.State
	local C = self.Const
	if not loc or not terrain or not terrain:enabled() then
		return 0
	end
	local wind = self:get_wind_vector()
	if not wind then
		return 0
	end
	local wlen = wind:length() or 0
	if wlen < 2.0 then
		return 0
	end

	local dx = G.grid_cell_size_m
	if (not dx) or dx <= 0 then
		dx = (C.min_cell_size_m or 50)
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

-- Attempts to select a ridge-based waypoint with time budget.
function SoarNav:try_ridge_target()
	local G = self.State
	local Cache = self.Cache
	if not G.grid_initialized then
		return false
	end
	if not G.grid_cell_size_m or G.grid_cell_size_m <= 0 then
		return false
	end
	if not terrain or not terrain:enabled() then
		return false
	end

	local now_ms = millis()
	local last_ms = G.ridge_last_ms or 0
	if (now_ms - last_ms) < 1500 then
		return false
	end
	G.ridge_last_ms = now_ms

	local wind = self:get_wind_vector()
	if not wind or wind:length() < 2.0 then
		return false
	end

	local loc = ahrs:get_location()
	if not loc then
		return false
	end

	local dx = G.grid_cell_size_m
	local s_curr, ux_curr, uy_curr = self:ridge_score_at_loc(loc)

	local best_s = s_curr or 0
	local best_loc = loc:copy()
	local best_ux = ux_curr or 0
	local best_uy = uy_curr or 0

	local bearings = {0, 45, 90, 135, 180, 225, 270, 315}

	for i = 1, #bearings do
		local cand_loc = loc:copy()
		cand_loc:offset_bearing(bearings[i], dx)

		local ok, res = pcall(self.in_flight_area, self, cand_loc)
		local area_ok = ok and res or false

		if area_ok then
			local s_cand, ux_cand, uy_cand = self:ridge_score_at_loc(cand_loc)
			if s_cand and s_cand > best_s then
				best_s = s_cand
				best_loc = cand_loc:copy()
				best_ux = ux_cand or 0
				best_uy = uy_cand or 0
			end
		end
	end

	if not best_loc or best_s < 0.15 then
		return false
	end

	local target = best_loc:copy()
	local wp_r = Cache.wp_radius or 50
	local upwind_offset = wp_r
	local wind_to_brg = (self:wind_vector_to_bearing_deg(wind) + 0) % 360
	target:offset_bearing(wind_to_brg, upwind_offset)

	local tx, ty = -best_uy, best_ux
	local tang_brg = math.deg(math.atan(tx, ty))
	if tang_brg < 0 then
		tang_brg = tang_brg + 360
	end

	local along = math.max(60, math.min(300, (Cache.wp_radius or 50) * 1.5))
	target:offset_bearing(tang_brg, along)

	self:set_target_safely(target)
	G.g_waypoint_source_info = G.SoarNavWaypointSources.RIDGE

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

	self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.NAV_TARGET_RIDGE, _hdg, _dir, _dist, best_s)
	return true
end

-- Checks if a Location object is inside the defined polygon using a ray-casting algorithm.
function SoarNav:is_point_in_polygon(loc)
	local G = self.State
	if not loc or not G.polygon_bounds then
		return false
	end

	local bounds = G.polygon_bounds
	if not bounds then
		return false
	end
	local lat, lon = loc:lat(), loc:lng()

	if lat < bounds.min_lat or lat > bounds.max_lat or lon < bounds.min_lon or lon > bounds.max_lon then
		return false
	end

	local poly = G.polygon_xy
	local origin = G.polygon_origin
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
function SoarNav:get_active_center_location()
	local G = self.State
	if G.dynamic_center_location then
		return G.dynamic_center_location
	else
		return ahrs:get_home()
	end
end

-- Returns current AGL in meters relative to Home.
function SoarNav:get_agl_m()
	local ned = ahrs:get_relative_position_NED_home()
	if not ned then
		return 100
	end
	return math.max(1, -ned:z())
end

-- Returns k_drift and r_mult coefficients based on thermal age.
function SoarNav:lifecycle_coeff(age_s, tmem_life_s)
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
function SoarNav:predict_thermal_drift(loc, wind, age_sec)
	local Cache = self.Cache
	if not loc then
		local fallback = ahrs:get_location()
		if not fallback then
			fallback = ahrs:get_home()
		end
		return fallback
	end
	if type(loc.copy) ~= "function" then
		return loc
	end
	if not wind or type(wind.length) ~= "function" then
		return loc:copy()
	end
	local ok_len, Wc = pcall(function()
		return wind:length()
	end)
	if not ok_len or not Wc or Wc < 0.5 then
		return loc:copy()
	end
	local ok, drift_loc = pcall(function()
		local age = math.max(0, age_sec or 0)
		local agl = self:get_agl_m()
		local soar_min = Cache.soar_alt_min or 0
		local soar_max = Cache.soar_alt_max or 600
		local est_bl_top = math.max(300, math.min(1200, (soar_max - soar_min > 0) and (soar_max - soar_min) or 600))
		local alpha = 0.12
		local veer_deg = 8
		local Veff_factor
		if agl <= est_bl_top then
			Veff_factor = 1.0 / (1.0 + alpha)
		else
			local r = est_bl_top / math.max(agl, 0.001)
			Veff_factor = (1.0 - r) + (r ^ (alpha + 1)) / (1.0 + alpha)
		end
		local k_age = select(1, self:lifecycle_coeff(age, Cache.tmem_life))
		local drift_speed = Wc * Veff_factor * k_age
		local drift_dist = drift_speed * age
		local wind_bearing_deg = self:wind_vector_to_bearing_deg(wind)
		local veer_scale = (math.min(1.0, agl / est_bl_top)) ^ 0.7
		local drift_bearing = (wind_bearing_deg + 180 + veer_deg * veer_scale) % 360
		local out = loc:copy()
		out:offset_bearing(drift_bearing, drift_dist)
		return out
	end)
	if ok and drift_loc then
		return drift_loc
	end
	return loc:copy()
end

-- Checks if a given location is within the defined flight area (polygon or radius).
function SoarNav:in_flight_area(loc)
	local G = self.State
	local Cache = self.Cache
	if not loc then
		return false
	end
	if G.use_polygon_area then
		return self:is_point_in_polygon(loc)
	else
		local center = self:get_active_center_location()
		if not center then
			return false
		end
		return center:get_distance(loc) <= (Cache.radius_m or 0)
	end
end

-- Converts a Location to local XY (meters) relative to the polygon origin.
function SoarNav:loc_to_xy(loc)
	local origin = self.State.polygon_origin
	if not origin or not loc then
		return nil, nil
	end
	local v = origin:get_distance_NE(loc)
	return v and v:x() or nil, v and v:y() or nil
end

-- Builds a Location from local XY (meters) offsets relative to the polygon origin.
function SoarNav:xy_to_loc(x, y)
	local origin = self.State.polygon_origin
	if not origin then
		return nil
	end
	local tmp = origin:copy()
	tmp:offset_bearing(90, x or 0)
	tmp:offset_bearing(0, y or 0)
	return tmp
end

-- Computes the closest point to P on segment AB in XY and returns qx, qy, t.
function SoarNav:nearest_on_seg(px, py, ax, ay, bx, by)
	local abx, aby = (bx - ax), (by - ay)
	local apx, apy = (px - ax), (py - ay)
	local ab2 = abx * abx + aby * aby
	local t = ab2 > 0 and math.max(0, math.min(1, (apx * abx + apy * aby) / ab2)) or 0
	return ax + t * abx, ay + t * aby, t
end

-- If outside, projects the point just inside the polygon along the nearest edge.
function SoarNav:clamp_inside_polygon(loc, pad_m)
	local G = self.State
	if not G.use_polygon_area then
		return loc
	end
	if not loc then
		return loc
	end
	if self:in_flight_area(loc) then
		return loc
	end
	local px, py = self:loc_to_xy(loc)
	if not px then
		return loc
	end
	local poly = G.polygon_xy
	if not poly or #poly < 3 then
		return loc
	end
	local best_dx, best_dy, best_d = 0, 0, math.huge
	local n = #poly
	for i = 1, n do
		local j = (i % n) + 1
		local ax, ay = poly[i].x, poly[i].y
		local bx, by = poly[j].x, poly[j].y
		local qx, qy = self:nearest_on_seg(px, py, ax, ay, bx, by)
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
	local rx, ry = px - nx * (len + pad + 1.0), py - ny * (len + pad + 1.0)
	local res = self:xy_to_loc(rx, ry)
	if res and self:in_flight_area(res) then
		return res
	end
	return loc
end

-- Returns true if the XY polygon is convex.
function SoarNav:is_convex(poly)
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

-- Precomputes polygon edges for fast segment checks.
function SoarNav:build_polygon_edges()
	local G = self.State
	local poly = G.polygon_xy
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
	G.polygon_edges = edges
	G.polygon_is_convex = self:is_convex(poly)
end

-- Checks if the current->target segment stays inside the polygon.
function SoarNav:segment_stays_inside(current, target)
	local G = self.State
	local Cache = self.Cache
	if not G.use_polygon_area then
		return true
	end
	if not (current and target) then
		return true
	end
	local dist = current:get_distance(target)
	local skip_dist = math.max(120, (Cache.wp_radius or 50) * 4)
	if dist < skip_dist then
		return true
	end
	local cache = G._seg_cache or {}
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
	if not self:in_flight_area(current) or not self:in_flight_area(target) then
		G._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = false }
		return false
	end
	if G.polygon_is_convex then
		G._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
		return true
	end
	local cx, cy = self:loc_to_xy(current)
	local tx, ty = self:loc_to_xy(target)
	if not cx or not tx then
		G._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
		return true
	end
	local edges = G.polygon_edges or {}
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
					G._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = false }
					return false
				end
			end
		end
	end
	G._seg_cache = { clat = clat, clon = clon, tlat = tlat, tlon = tlon, ms = now, res = true }
	return true
end

-- Pulls the target inward so the current->target segment remains inside the polygon.
function SoarNav:adjust_target_segment(current, target)
	if self:segment_stays_inside(current, target) then
		return target
	end
	local cx, cy = self:loc_to_xy(current)
	local tx, ty = self:loc_to_xy(target)
	if not cx then
		return target
	end
	local ks = { 0.8, 0.6, 0.4, 0.2 }
	for _, k in ipairs(ks) do
		local nx = cx + k * (tx - cx)
		local ny = cy + k * (ty - cy)
		local cand = self:xy_to_loc(nx, ny)
		if cand and self:in_flight_area(cand) and self:segment_stays_inside(current, cand) then
			return cand
		end
	end
	local pad = self.Cache.wp_radius or 50
	local clamped = self:clamp_inside_polygon(target, pad)
	if self:segment_stays_inside(current, clamped) then
		return clamped
	end
	return target
end

-- Central setter: clamps candidate inside and enforces an in-polygon path from current.
function SoarNav:set_target_safely(candidate_loc)
	local G = self.State
	if not candidate_loc then
		return false
	end
	if not G.use_polygon_area then
		G.target_loc = candidate_loc
		return true
	end
	local pad = self.Cache.wp_radius or 50
	local target = candidate_loc
	if not self:in_flight_area(target) then
		target = self:clamp_inside_polygon(target, pad * 0.5)
	end
	local cur = ahrs:get_location()
	if cur and self:in_flight_area(cur) then
		if not self:segment_stays_inside(cur, target) then
			target = self:adjust_target_segment(cur, target)
		end
	end
	G.target_loc = target
	return true
end

-- Safely decrements a number, ensuring it does not go below zero.
function SoarNav:safe_decrement(n)
	return math.max(0, n - 1)
end

-- Validates key SNAV_* parameters to ensure they are within safe and logical ranges.
function SoarNav:validate_params()
	local ok = true
	local Cache = self.Cache

	local function clamp_warn(name, val, lo, hi, msg_id)
		if val == nil then
			return
		end
		if (lo and val < lo) or (hi and val > hi) then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, msg_id, name, tostring(lo), tostring(hi), val)
		end
	end

	if (Cache.radius_m or 0) < 0 then
		self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.SNAV_RADIUS_NEGATIVE)
		ok = false
	end

	if Cache.reroute_min and Cache.reroute_max and Cache.reroute_min > Cache.reroute_max then
		self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.SNAV_REROUTE_MINMAX)
		ok = false
	end

	clamp_warn("SNAV_ROLL_LIMIT", Cache.roll_limit, 10, 60, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_WP_RADIUS", Cache.wp_radius, 10, 300, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NAV_P", Cache.nav_p, 0.05, 2.0, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NAV_D", Cache.nav_d, 0.0, 1.0, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STUCK_EFF", Cache.stuck_eff, 0.1, 1.0, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STUCK_TIME", Cache.stuck_time, 5, 120, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_REENG_DWELL", Cache.reeng_dwell, 0, 120, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_RETRY_THR", Cache.retry_thr, 0, 100, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STREET_TOL", Cache.street_tol, 5, 90, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_REROUTE_P", Cache.reroute_p, 0, 100, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_NEC_WEIGHT", Cache.nec_weight, 0, 100, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_TMEM_MIN_S", Cache.tmem_min_s, 0.0, 5.0, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_FOCUS_THR", Cache.focus_thr, 0.0, 5.0, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_STRAT_HIST", Cache.strat_hist, 60, 3600, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_WP_TIMEOUT", Cache.wp_timeout, 30, 900, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_DYN_SOALT", Cache.dyn_soalt, 0, 3, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_GC_MARGIN", Cache.gc_margin, 0, 150, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)
	clamp_warn("SNAV_GC_PAD", Cache.gc_pad, 0, 100, self.Msg.MSG_ID.PARAM_OUT_OF_RANGE)

	return ok
end

-- Determines the grid cell index for a given geographical location.
function SoarNav:get_cell_index_from_location(loc)
	local G = self.State
	if not G or not G.grid_initialized or not loc or type(loc.lat) ~= "function" or type(loc.lng) ~= "function" then
		return nil
	end
	local b = G.grid_bounds
	if not b or not b.min_lat or not b.max_lat or not b.min_lon or not b.max_lon then
		return nil
	end
	local lat_span = b.lat_span
	local lon_span = b.lon_span
	if not lat_span or not lon_span or lat_span <= 0 or lon_span <= 0 then
		return nil
	end
	local lat = loc:lat()
	local lon = loc:lng()
	if not lat or not lon then
		return nil
	end
	local rows, cols = G.grid_rows, G.grid_cols
	if not rows or not cols or rows <= 0 or cols <= 0 then
		return nil
	end
	local eps = 1e-6
	if lat < b.min_lat - eps or lat > b.max_lat + eps or lon < b.min_lon - eps or lon > b.max_lon + eps then
		return nil
	end
	if lat < b.min_lat then
		lat = b.min_lat
	end
	if lat > b.max_lat then
		lat = b.max_lat
	end
	if lon < b.min_lon then
		lon = b.min_lon
	end
	if lon > b.max_lon then
		lon = b.max_lon
	end
	local lat_frac = (lat - b.min_lat) / lat_span
	local lon_frac = (lon - b.min_lon) / lon_span
	if lat_frac < 0 then
		lat_frac = 0
	elseif lat_frac > 1 then
		lat_frac = 1
	end
	if lon_frac < 0 then
		lon_frac = 0
	elseif lon_frac > 1 then
		lon_frac = 1
	end
	local row = math.floor(lat_frac * rows) + 1
	local col = math.floor(lon_frac * cols) + 1
	if row < 1 then
		row = 1
	elseif row > rows then
		row = rows
	end
	if col < 1 then
		col = 1
	elseif col > cols then
		col = cols
	end
	return (row - 1) * cols + col
end

-- Returns clamped best sink estimate using TECS limits.
function SoarNav:sink_best_now()
	local smin = self.Cache.tecs_sink_min or 0.3
	local smax = self.Cache.tecs_sink_max or 3.0
	local s = self.State.sink_best_ema or 0.9
	if s < smin then return smin end
	if s > smax then return smax end
	return s
end

-- ============================================================================
-- CORE LOGIC: GLIDE CONE & POLAR
-- ============================================================================

-- Dynamically updates SOAR_ALT_MIN/MAX/CUTOFF based on the calculated glide cone to home.
function SoarNav:update_dynamic_soar_alt_min()
	local G = self.State
	local Cache = self.Cache
	local P = self.Params
	if G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE or G.manual_override_active then
		return
	end
	local override_active = (G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE) or (G.manual_override_active == true)
	if override_active then
		G.gc_reset_hold_start_ms = nil
		if not G.override_reset_done then
			local init_min = G.initial_soar_alt_min
			local init_cutoff = G.initial_soar_alt_cutoff
			local init_max = G.initial_soar_alt_max
			if init_min and init_cutoff and init_max and P.soar_alt_min and P.soar_alt_cutoff and P.soar_alt_max then
				local cur_min = Cache.soar_alt_min
				local cur_cut = Cache.soar_alt_cutoff
				local cur_max = Cache.soar_alt_max
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
					self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GCONE_RST_OVERRIDE, init_min, init_cutoff, init_max)
				end
			end
			G.override_reset_done = true
		end
		return
	else
		G.override_reset_done = nil
	end

	if
		not Cache.dyn_soalt
		or not Cache.soar_polar_b
		or not Cache.soar_polar_cd0
		or not Cache.airspeed_cruise
		or not P.soar_alt_min
		or not P.soar_alt_max
		or not P.soar_alt_cutoff
		or not Cache.gc_margin
		or not Cache.gc_pad
	then
		return
	end

	local initial_min = G.initial_soar_alt_min
	local initial_cutoff = G.initial_soar_alt_cutoff
	local initial_max = G.initial_soar_alt_max
	local current_min = Cache.soar_alt_min
	if not initial_min or not initial_cutoff or not initial_max or not current_min then
		return
	end

	local dyn_soalt_mode = math.floor((Cache.dyn_soalt or 0) + 0.5)
	if dyn_soalt_mode == 0 or dyn_soalt_mode == 3 then
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

	local polar_b = Cache.soar_polar_b
	local polar_cd0 = Cache.soar_polar_cd0
	local best_glide_airspeed = Cache.airspeed_cruise
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
		if not G.gcone_param_warning_sent then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.GCONE_OFF_NO_PARAMS)
			G.gcone_param_warning_sent = true
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
	local wind_vec = self:get_wind_vector()
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

	local pad = Cache.gc_pad or 0
	local margin = Cache.gc_margin or 0
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
		if not G.gc_reset_hold_start_ms then
			G.gc_reset_hold_start_ms = now
			return
		elseif self:safe_time_diff(now, G.gc_reset_hold_start_ms) >= RESET_HOLD_MS then
			if current_alt > (initial_max + 10) then
				G.gc_reset_hold_start_ms = nil
				return
			end
			target_min = initial_min
			target_cutoff = initial_cutoff
			target_max = initial_max
			G.gc_reset_hold_start_ms = nil
		else
			return
		end
	else
		G.gc_reset_hold_start_ms = nil
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
		if dyn_soalt_mode == 1 or dyn_soalt_mode == 2 then
			if Cache.soar_alt_min ~= target_min then
				P.soar_alt_min:set(target_min)
				changed = true
			end
			if Cache.soar_alt_cutoff ~= target_cutoff then
				P.soar_alt_cutoff:set(target_cutoff)
				changed = true
			end
			if Cache.soar_alt_max ~= target_max then
				P.soar_alt_max:set(target_max)
				changed = true
			end
		end

		if changed then
			local alt_range = G.initial_soar_alt_max - G.initial_soar_alt_min
			if alt_range > 0 then
				local warning_margin_m = alt_range / 10.0
				if current_alt < (target_min + warning_margin_m) then
					self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GCONE_ALT_UPDATE, target_min)
				end
			else
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GCONE_ALT_UPDATE, target_min)
			end
			G.last_gcone_logged_alt = target_min
			G.last_gcone_log_ms = now
		elseif dyn_soalt_mode == 3 then
			local now_ms = millis()
			if not G.last_gcone_log_ms or self:safe_time_diff(now_ms, G.last_gcone_log_ms) > 10000 then
				if not G.last_gcone_logged_alt or G.last_gcone_logged_alt ~= target_min then
					self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GCONE_ALT_UPDATE, target_min)
					G.last_gcone_logged_alt = target_min
					G.last_gcone_log_ms = now_ms
				end
			end
		end
	end
end

-- Dynamically learns the aircraft's glide performance (Polar).
function SoarNav:update_polar_learning()
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	local P = self.Params
	if not G.polar_learn then
		return
	end
	local pe = G.polar_est
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
		G.polar_est = pe
	end

	local throttle = SRV_Channels:get_output_scaled(C.THROTTLE_SRV_CHANNEL) or 0
	if throttle > C.THROTTLE_ACTIVE_THRESHOLD then
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

	local wind = self:get_wind_vector()
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
	local dt_ms = self:safe_time_diff(now, (pe.last_sample_ms or 0))
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

	if self:safe_time_diff(now, pe.first_sample_ms or 0) < 60000 then
		return
	end
	if (pe.n or 0) < 150 then
		return
	end
	if pe.last_fit_ms and self:safe_time_diff(now, pe.last_fit_ms) < 8000 then
		return
	end
	pe.last_fit_ms = now

	local cd0 = Cache.soar_polar_cd0
	local bb = Cache.soar_polar_b
	if not pe.last_saved_cd0 then
		pe.last_saved_cd0 = cd0
	end
	if not pe.last_saved_b then
		pe.last_saved_b = bb
	end

	local v_var = math.max(0, (pe.v2_ema or 0) - (pe.v_ema or 0) ^ 2)
	local v_std = math.sqrt(v_var)
	local vc = Cache.airspeed_cruise or 15
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
		local rl_ok = is_first_commit or (self:safe_time_diff(now, pe.last_commit_ms) >= 90000)
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
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.POLAR_LEARN_UPDATE, new_cd0_q, new_b_q)
			G.polar_learn = false
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
	local rl_ok = is_first_commit or (self:safe_time_diff(now, pe.last_commit_ms) >= 90000)
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
		self:send_gcs_id(self.Msg.MAV_SEVERITY.INFO, self.Msg.MSG_ID.POLAR_LEARN_UPDATE, new_cd0_q, new_b_q)
		G.polar_learn = false
	end
end

-- Checks if a waypoint source is a thermal or thermal street.
function SoarNav:is_thermal_target(source_info)
	local src = tostring(source_info)
	return string.find(src, "Thrm") or (src == self.State.SoarNavWaypointSources.THERMAL_STREET)
end

-- ============================================================================
-- CORE LOGIC: AREA & GRID
-- ============================================================================

-- Announces area details for polygon mode (km², vertex count, max distance).
function SoarNav:announce_polygon_area()
	local G = self.State
	local poly = G.polygon_xy
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

	local prefix = G.using_rally_points and "Rally" or "Poly"
	self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.AREA_INFO_POLY, prefix, area_km2, m, md_m)
	G.area_announced = true
end

-- Announces area details for radius mode (km² and radius).
function SoarNav:announce_radius_area()
	local rad_m = self.Cache.radius_m or 0
	if rad_m > 0 then
		local area_km2 = (math.pi * rad_m * rad_m) * 1.0e-6
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.AREA_INFO_RADIUS, area_km2, rad_m)
		self.State.area_announced = true
	end
end

-- Precomputes EN-metric polygon cache and origin from polygon points.
function SoarNav:prepare_polygon_xy_cache()
	local G = self.State
	if #G.polygon_points < 3 then
		self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.POLY_INVALID_POINTS)
		return
	end
	local poly = G.polygon_points
	if not poly or #poly < 3 then
		return
	end

	local origin = poly[1]:copy()
	G.polygon_origin = origin
	G.polygon_xy = {}

	for i = 1, #poly do
		local xy = origin:get_distance_NE(poly[i])
		G.polygon_xy[i] = { x = xy:x(), y = xy:y() }
	end

	self:build_polygon_edges()
end

-- ============================================================================
-- CORE LOGIC: SCRIPT STATE & STATE MACHINE
-- ============================================================================

-- Assesses the aircraft's energy state (NORMAL, LOW, CRITICAL) based on altitude.
function SoarNav:assess_energy_state()
	local G = self.State
	local Cache = self.Cache
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	if not dist_from_home_3d then
		return "UNKNOWN"
	end

	local current_alt = -dist_from_home_3d:z()
	local now = millis()

	local trend = 0
	if G.last_alt_timestamp then
		local dt = self:safe_time_diff(now, G.last_alt_timestamp) / 1000.0
		if dt > 0.5 then
			trend = (current_alt - G.last_alt) / dt
		end
	end
	G.last_alt = current_alt
	G.last_alt_timestamp = now

	local min_alt = Cache.soar_alt_min or 100
	local max_alt = Cache.soar_alt_max or 400
	local alt_range = max_alt - min_alt
	local raw_alt_factor = 0
	if alt_range > 0 then
		raw_alt_factor = math.min(1, math.max(0, (current_alt - min_alt) / alt_range))
	end

	local filter_alpha = 0.05
	if G.energy_state_transition_ms == 0 then
		G.filtered_alt_factor = raw_alt_factor
		G.energy_state_transition_ms = now
	else
		local dt_filter = self:safe_time_diff(now, G.energy_state_transition_ms) / 1000.0
		G.filtered_alt_factor =
			G.filtered_alt_factor
			+ (raw_alt_factor - G.filtered_alt_factor) * (1 - math.exp(-dt_filter * filter_alpha))
		G.energy_state_transition_ms = now
	end

	local new_state = "NORMAL"
	if G.filtered_alt_factor < 0.25 then
		new_state = "LOW"
	end
	if G.filtered_alt_factor < 0.1 then
		new_state = "CRITICAL"
	end

	if trend < -0.3 then
		if not G.negative_trend_start_ms then
			G.negative_trend_start_ms = now
		end
		if
			self:safe_time_diff(now, G.negative_trend_start_ms) >= self.Const.CRITICAL_TREND_HYSTERESIS_MS
			and new_state ~= "CRITICAL"
		then
			new_state = "CRITICAL"
		end
	else
		G.negative_trend_start_ms = nil
	end

	if G.energy_state ~= new_state then
		G.energy_state = new_state
	end

	return G.energy_state
end

-- Calculates the statistical variance of climb rate samples to determine thermal consistency.
function SoarNav:calculate_thermal_variance(samples)
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

-- Restores the SOAR_ALT parameters to their initial boot-up values.
function SoarNav:restore_boot_alts(_)
	local G = self.State
	if G and G.initial_soar_alt_min and G.initial_soar_alt_max and G.initial_soar_alt_cutoff then
		if self.Params.soar_alt_min then
			self.Params.soar_alt_min:set(G.initial_soar_alt_min)
		end
		if self.Params.soar_alt_max then
			self.Params.soar_alt_max:set(G.initial_soar_alt_max)
		end
		if self.Params.soar_alt_cutoff then
			self.Params.soar_alt_cutoff:set(G.initial_soar_alt_cutoff)
		end
		G.last_glide_cone_update_ms = nil
	end
end

-- Manages script state transitions and associated cleanup actions.
function SoarNav:set_script_state(new_state, reason_id, ...)
	local G = self.State
	if G.script_state ~= new_state then
		local prev_state = G.script_state
		if new_state == G.SCRIPT_STATE.PILOT_OVERRIDE then
			self:restore_boot_alts("Pilot Override")
		end

		if new_state == G.SCRIPT_STATE.IDLE and prev_state ~= G.SCRIPT_STATE.THERMAL_PAUSE then
			self:restore_boot_alts("Entering IDLE")
		end

		if new_state == G.SCRIPT_STATE.IDLE and G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE then
			G.pitch_gesture_state = "idle"
			G.pitch_gesture_count = 0
			G.manual_override_active = false
			G.last_pilot_input_ms = nil
			G.pitch_gesture_start_ms = nil
			G.pitch_gesture_triggered_this_override = false
			G.roll_gesture_state = "idle"
			G.roll_gesture_count = 0
			G.roll_gesture_start_ms = nil
			G.manual_override_active = false
		end
		G.script_state = new_state

		if reason_id then
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, reason_id, ...)
		end

		if new_state == G.SCRIPT_STATE.NAVIGATING then
			if prev_state ~= G.SCRIPT_STATE.NAVIGATING then
				G.target_loc = nil
				G.activation_requested = false
				G.activation_wait_notified = false
				G.navigation_start_delay_counter = 0
			end
		end
	end
end

-- Detects a rapid pitch stick gesture to re-center the search area.
function SoarNav:check_pitch_gesture()
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	local mode = vehicle:get_mode()
	local soaring_switch_high = (rc:get_aux_cached(C.rc_opt_soaring_active) == 2)
	local in_fw_nav_mode = (mode == C.mode_fbwb or mode == C.mode_cruise)

	if G.script_state ~= G.SCRIPT_STATE.PILOT_OVERRIDE or not soaring_switch_high or not in_fw_nav_mode then
		G.pitch_gesture_state = "idle"
		G.pitch_gesture_count = 0
		G.pitch_gesture_start_ms = nil
		G.pitch_gesture_triggered_this_override = false
		return
	end

	if G.pitch_gesture_triggered_this_override then
		return
	end
	if G.script_state ~= G.SCRIPT_STATE.PILOT_OVERRIDE then
		G.pitch_gesture_triggered_this_override = false
	end
	if G.pitch_gesture_triggered_this_override then
		return
	end
	local gesture_timeout = false
	if G.pitch_gesture_start_ms then
		if self:safe_time_diff(millis(), G.pitch_gesture_start_ms) > C.PITCH_GESTURE_TIMEOUT_MS then
			gesture_timeout = true
		end
	end

	if G.pitch_gesture_state ~= "idle" and gesture_timeout then
		G.pitch_gesture_state = "idle"
		G.pitch_gesture_count = 0
	end

	local pitch_chan_obj = rc:get_channel(Cache.rcmap_pitch or 2)
	if not pitch_chan_obj then
		return
	end
	local normalized_pitch = pitch_chan_obj:norm_input()

	if G.pitch_gesture_state == "idle" then
		if math.abs(normalized_pitch) > C.PITCH_GESTURE_THRESHOLD then
			G.pitch_gesture_start_ms = millis()
			G.pitch_gesture_count = 1
			if normalized_pitch > 0 then
				G.pitch_gesture_state = "waiting_for_down"
			else
				G.pitch_gesture_state = "waiting_for_up"
			end
		end
	elseif G.pitch_gesture_state == "waiting_for_down" then
		if normalized_pitch < -C.PITCH_GESTURE_THRESHOLD then
			G.pitch_gesture_count = G.pitch_gesture_count + 1
			G.pitch_gesture_state = "waiting_for_up"
		end
	elseif G.pitch_gesture_state == "waiting_for_up" then
		if normalized_pitch > C.PITCH_GESTURE_THRESHOLD then
			G.pitch_gesture_count = G.pitch_gesture_count + 1
			G.pitch_gesture_state = "waiting_for_down"
		end
	end

	if G.pitch_gesture_count >= C.PITCH_GESTURE_COUNT_TARGET then
		local current_loc = ahrs:get_location()
		if not current_loc then
			return
		end

		if G.use_polygon_area then
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.STICK_CMD_POLY_RECENTER)
			G.polygon_centroid = current_loc:copy()
			local offsets = G.polygon_vertex_offsets
			if not offsets or #offsets < 3 then
				self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.POLY_INVALID_POINTS)
				return
			end
			local new_points = {}
			for _, offset in ipairs(offsets) do
				local new_point = current_loc:copy()
				new_point:lat(current_loc:lat() + offset.lat)
				new_point:lng(current_loc:lng() + offset.lon)
				table.insert(new_points, new_point)
			end
			G.polygon_bounds =
				{ min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7 }
			for _, pt in ipairs(new_points) do
				local lat_i7 = pt:lat()
				local lon_i7 = pt:lng()
				G.polygon_bounds.min_lat = math.min(G.polygon_bounds.min_lat, lat_i7)
				G.polygon_bounds.max_lat = math.max(G.polygon_bounds.max_lat, lat_i7)
				G.polygon_bounds.min_lon = math.min(G.polygon_bounds.min_lon, lon_i7)
				G.polygon_bounds.max_lon = math.max(G.polygon_bounds.max_lon, lon_i7)
			end
			G.polygon_points = new_points
			G.dynamic_center_location = nil
			self:prepare_polygon_xy_cache()
		else
			if (Cache.radius_m or 0) <= 0 then
				return
			end
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.STICK_CMD_RADIUS_RECENTER)
			G.dynamic_center_location = current_loc:copy()
		end

		G.target_loc = nil
		G.grid_initialized = false
		G.force_grid_reinit = true
		G.area_announced = false
		G.pitch_gesture_triggered_this_override = true
		G.pitch_gesture_state = "idle"
		G.pitch_gesture_count = 0

		if G.manual_override_active then
			G.manual_override_active = false
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.STICK_CMD_MANUAL_OFF)
		end
		G.last_pilot_input_ms = nil
		self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.RESUMING_NAV)
	end
end

-- Detects a rapid roll stick gesture to activate the script or toggle manual override.
function SoarNav:check_roll_gesture()
	local G = self.State
	local C = self.Const
	local gesture_timeout = false
	if G.roll_gesture_start_ms then
		if self:safe_time_diff(millis(), G.roll_gesture_start_ms) > C.ROLL_GESTURE_TIMEOUT_MS then
			gesture_timeout = true
		end
	end

	if G.roll_gesture_state ~= "idle" and gesture_timeout then
		G.roll_gesture_state = "idle"
		G.roll_gesture_count = 0
	end

	local roll_chan = self.Cache.rcmap_roll or 1
	local chan1 = rc:get_channel(roll_chan)
	if not chan1 then
		return false
	end
	local normalized_roll = chan1:norm_input()

	if G.roll_gesture_state == "idle" then
		if math.abs(normalized_roll) > C.ROLL_GESTURE_THRESHOLD then
			G.roll_gesture_start_ms = millis()
			G.roll_gesture_count = 1
			if normalized_roll > 0 then
				G.roll_gesture_state = "waiting_for_left"
			else
				G.roll_gesture_state = "waiting_for_right"
			end
		end
	elseif G.roll_gesture_state == "waiting_for_left" then
		if normalized_roll < -C.ROLL_GESTURE_THRESHOLD then
			G.roll_gesture_count = G.roll_gesture_count + 1
			G.roll_gesture_state = "waiting_for_right"
		end
	elseif G.roll_gesture_state == "waiting_for_right" then
		if normalized_roll > C.ROLL_GESTURE_THRESHOLD then
			G.roll_gesture_count = G.roll_gesture_count + 1
			G.roll_gesture_state = "waiting_for_left"
		end
	end

	if G.roll_gesture_count >= C.ROLL_GESTURE_COUNT_TARGET then
		local was_activation_gesture = false
		if G.script_state == G.SCRIPT_STATE.WAITING_FOR_ACTIVATION then
			G.activation_requested = true
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GESTURE_ACTIVATED)

			G.activation_grace_start_ms = millis()
			was_activation_gesture = true
		else
			G.manual_override_active = not G.manual_override_active
			if G.manual_override_active then
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.STICK_CMD_MANUAL_ON)
			else
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.STICK_CMD_MANUAL_OFF)
			end
		end
		G.roll_gesture_state = "idle"
		G.roll_gesture_count = 0
		G.roll_gesture_start_ms = nil
		return was_activation_gesture
	end

	return false
end

-- Kicks off the grid generation process.
function SoarNav:initialize_grid()
	local G = self.State
	G.is_initializing = true
	G.grid_initialized = false
	G.grid_init_step = 1
	G.grid_cells = {}
	G.grid_cell_centers = {}
	G.valid_cell_indices = {}
	G.unvisited_cell_indices = {}
	G.last_cell_index = nil
	G.grid_populate_index = 1
	G.grid_rows = 0
	G.grid_cols = 0
	G.grid_bounds = nil
	G.max_operational_distance = 0
	self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.GRID_INIT_STARTED)
end

-- Manages the multi-step, non-blocking grid generation and validation.
function SoarNav:manage_grid_initialization()
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	if not G.is_initializing then
		return
	end
	local active_center = self:get_active_center_location()
	if not active_center then
		self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.GRID_INIT_FAIL_NO_CENTER)
		G.is_initializing = false
		G.grid_init_step = 0
		return
	end

	if G.grid_init_step == 1 then
		if G.use_polygon_area and G.polygon_bounds then
			G.grid_bounds = G.polygon_bounds
			local max_dist_sq = 0
			local poly_pts = G.polygon_points
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
				G.max_operational_distance = math.sqrt(max_dist_sq)
				if not G.area_announced then
					self:announce_polygon_area()
				end
				self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.POLY_MAX_DIST, G.max_operational_distance)
			end
		else
			local radius_m = Cache.radius_m or 0
			local corner_dist = radius_m * 1.4142
			local sw_corner_loc_calc = active_center:copy()
			sw_corner_loc_calc:offset_bearing(225, corner_dist)
			local ne_corner_loc_calc = active_center:copy()
			ne_corner_loc_calc:offset_bearing(45, corner_dist)
			G.grid_bounds = {
				min_lat = sw_corner_loc_calc:lat(),
				max_lat = ne_corner_loc_calc:lat(),
				min_lon = sw_corner_loc_calc:lng(),
				max_lon = ne_corner_loc_calc:lng(),
			}
			G.max_operational_distance = (radius_m or 0) * 2
			if not G.area_announced then
				self:announce_radius_area()
			end
		end

		local sw_corner_loc = active_center:copy()
		sw_corner_loc:lat(G.grid_bounds.min_lat)
		sw_corner_loc:lng(G.grid_bounds.min_lon)
		local ne_corner_loc = active_center:copy()
		ne_corner_loc:lat(G.grid_bounds.max_lat)
		ne_corner_loc:lng(G.grid_bounds.max_lon)
		local xy = sw_corner_loc:get_distance_NE(ne_corner_loc)
		local width_m = xy:x()
		local height_m = xy:y()

		if height_m < 1 or width_m < 1 then
			self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.GRID_TOO_SMALL)
			G.is_initializing = false
			G.grid_init_step = 0
			return
		end

		local area_size = math.max(width_m, height_m)
		G.effective_cluster_radius = math.min(600, math.max(200, area_size * 0.15))
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.EFF_CLUSTER_RADIUS, G.effective_cluster_radius)

		local area_m2 = height_m * width_m
		local approx_cell_size_m = math.sqrt(area_m2 / C.max_total_grid_cells)
		local final_cell_size_m = math.max(C.min_cell_size_m, approx_cell_size_m)
		G.grid_rows = math.max(1, math.floor(height_m / final_cell_size_m))
		G.grid_cols = math.max(1, math.floor(width_m / final_cell_size_m))
		G.grid_cell_size_m = final_cell_size_m
		G.grid_init_step = 2

		local b = G.grid_bounds
		if b and b.max_lat and b.min_lat and b.max_lon and b.min_lon then
			b.lat_span = b.max_lat - b.min_lat
			b.lon_span = b.max_lon - b.min_lon
		else
			b.lat_span = 0
			b.lon_span = 0
		end
	elseif G.grid_init_step == 2 then
		local total_cells = G.grid_rows * G.grid_cols
		local cells_processed = 0
		local cells_per_call = C.grid_init_cells_per_call
		while G.grid_populate_index <= total_cells and cells_processed < cells_per_call do
			table.insert(G.grid_cells, { visit_count = 0 })
			G.grid_populate_index = G.grid_populate_index + 1
			cells_processed = cells_processed + 1
		end
		if G.grid_populate_index > total_cells then
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.GRID_SCAN_INFO, G.grid_rows, G.grid_cols)
			G.grid_init_row = 1
			G.grid_init_col = 1
			G.grid_init_step = 3
		end
	elseif G.grid_init_step == 3 then
		local cells_processed = 0
		local cell_height_lat = (G.grid_bounds.max_lat - G.grid_bounds.min_lat) / G.grid_rows
		local cell_width_lon = (G.grid_bounds.max_lon - G.grid_bounds.min_lon) / G.grid_cols
		while G.grid_init_row <= G.grid_rows and cells_processed < C.grid_init_cells_per_call do
			local r = G.grid_init_row
			local c = G.grid_init_col
			local cell_center_lat = G.grid_bounds.min_lat + (r - 0.5) * cell_height_lat
			local cell_center_lon = G.grid_bounds.min_lon + (c - 0.5) * cell_width_lon
			local center = ahrs:get_home():copy()
			center:lat(cell_center_lat)
			center:lng(cell_center_lon)
			if self:in_flight_area(center) then
				local cell_index = (r - 1) * G.grid_cols + c
				table.insert(G.valid_cell_indices, cell_index)
				G.grid_cell_centers[cell_index] = center
			end
			cells_processed = cells_processed + 1
			G.grid_init_col = G.grid_init_col + 1
			if G.grid_init_col > G.grid_cols then
				G.grid_init_col = 1
				G.grid_init_row = G.grid_init_row + 1
			end
		end
		if G.grid_init_row > G.grid_rows then
			G.is_initializing = false
			G.grid_initialized = true
			G.grid_init_step = 0
			G.unvisited_cell_indices = {}
			G.unvisited_set = {}
			for _, v in ipairs(G.valid_cell_indices) do
				table.insert(G.unvisited_cell_indices, v)
				G.unvisited_set[v] = true
			end
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.GRID_VALIDATED, #G.valid_cell_indices)
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.GRID_READY)
		end
	end
end

-- Tracks which grid cell the aircraft is currently in and updates its visit count.
function SoarNav:update_visited_cell()
	local G = self.State
	local loc = ahrs:get_location()
	if not loc then
		return
	end
	local current_cell_index = self:get_cell_index_from_location(loc)
	if not current_cell_index or current_cell_index <= 0 or current_cell_index > #G.grid_cells then
		return
	end
	if current_cell_index ~= G.last_cell_index then
		if G.unvisited_set[current_cell_index] then
			G.unvisited_set[current_cell_index] = nil
			local uci = G.unvisited_cell_indices
			local n = #uci
			for i = n, 1, -1 do
				if uci[i] == current_cell_index then
					uci[i] = uci[n]
					uci[n] = nil
					break
				end
			end
		end
		if G.grid_cells[current_cell_index] then
			G.grid_cells[current_cell_index].visit_count = G.grid_cells[current_cell_index].visit_count + 1
		end
		G.last_cell_index = current_cell_index
	end
end

-- ============================================================================
-- CORE LOGIC: THERMAL & NAVIGATION
-- ============================================================================

-- Removes expired thermal hotspots from memory and returns a list of currently valid ones.
function SoarNav:clean_and_get_hotspots()
	local G = self.State
	local Cache = self.Cache
	local now_ms = millis()
	local lifetime_ms = (Cache.tmem_life or 1200) * 1000
	for i = #G.thermal_hotspots, 1, -1 do
		local hotspot = G.thermal_hotspots[i]
		if hotspot and hotspot.timestamp then
			if self:safe_time_diff(now_ms, hotspot.timestamp) >= lifetime_ms then
				table.remove(G.thermal_hotspots, i)
			end
		end
	end
	return G.thermal_hotspots
end

-- Calculates a weighted density score for each hotspot based on nearby thermals.
function SoarNav:calculate_hotspot_density(hotspots)
	if not hotspots or #hotspots < 2 then
		for _, h in ipairs(hotspots or {}) do
			h.cluster_density = 0
		end
		return
	end

	local G = self.State
	local Cache = self.Cache
	local now = millis()
	local cluster_radius = G.effective_cluster_radius
	local cluster_radius_sq = cluster_radius * cluster_radius
	local tmem_life_ms = (Cache.tmem_life or 1200) * 1000
	local n = #hotspots

	for i = 1, n do
		local h1 = hotspots[i]
		local density_score = 0
		local h1_loc = h1.loc
		for j = 1, n do
			if i ~= j then
				local h2 = hotspots[j]
				local dx = h1_loc:lat() - h2.loc:lat()
				local dy = h1_loc:lng() - h2.loc:lng()
				local approx_dist_sq = dx * dx + dy * dy
				if approx_dist_sq < cluster_radius_sq * 2e14 then
					local distance = h1_loc:get_distance(h2.loc)
					if distance < cluster_radius then
						local distance_factor = 1.0 - (distance / cluster_radius)
						local age_ms = self:safe_time_diff(now, h2.timestamp)
						local age_factor = 1.0 - math.min(1.0, age_ms / tmem_life_ms)
						density_score = density_score + (h2.avg_strength or 0) * distance_factor * age_factor
					end
				end
			end
		end
		h1.cluster_density = density_score
	end
end

-- Records a new thermal hotspot in memory after exiting THERMAL mode.
function SoarNav:stop_and_record_thermal()
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	G.is_monitoring_thermal = false
	if not G.current_thermal_stats or not G.current_thermal_stats.core_location or G.current_thermal_stats.avg_strength == 0 then
		self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.THERMAL_EXIT_NO_DATA)
		G.lost_thermal_counter = G.lost_thermal_counter + 1
		G.current_thermal_stats = {}
		return
	end

	local max_strength = G.current_thermal_stats.max_strength
	if (max_strength or 0) < (Cache.tmem_min_s or 0.2) then
		G.current_thermal_stats = {}
		return
	end

	G.lost_thermal_counter = 0

	if G.is_in_focus_mode then
		self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.THERMAL_FOUND_EXIT_FOCUS)
		G.is_in_focus_mode = false
		G.focus_area_center = nil
		G.focus_wp_counter = 0
	end

	local avg_strength = G.current_thermal_stats.avg_strength
	local hotspot_loc = G.current_thermal_stats.core_location:copy()
	local variance = self:calculate_thermal_variance(G.current_thermal_stats.samples)
	local consistency_score = variance < C.THERMAL_CONSISTENCY_VARIANCE_THRESHOLD and 1.0 or 0.5

	if not self:in_flight_area(hotspot_loc) then
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.THERMAL_IGNORED_OOB)
		G.current_thermal_stats = {}
		return
	end

	local timestamp_saved = millis()
	local duration = 0
	if G.current_thermal_stats.start_time then
		duration = self:safe_time_diff(timestamp_saved, G.current_thermal_stats.start_time) / 1000
	end

	local new_hotspot = {
		loc = hotspot_loc,
		timestamp = timestamp_saved,
		entry_time = G.current_thermal_stats.start_time,
		avg_strength = avg_strength,
		max_strength = max_strength,
		consistency_score = consistency_score,
		duration = duration,
		wind_vec = G.current_thermal_stats.wind_at_entry,
		failed_attempts = 0,
		cluster_density = 0,
	}
	local new_cell = self:get_cell_index_from_location(hotspot_loc)
	if not new_cell then
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.CELL_INDEX_FAIL)
		new_cell = -1
	end
	new_hotspot.cell = new_cell
	for _, existing in ipairs(G.thermal_hotspots) do
		local existing_cell = self:get_cell_index_from_location(existing.loc)
		if existing_cell and new_cell and existing_cell == new_cell then
			G.current_thermal_stats = {}
			return
		end
	end
	table.insert(G.thermal_hotspots, new_hotspot)
	G.last_used_hotspot_timestamp = timestamp_saved
	if #G.thermal_hotspots > C.max_hotspots then
		table.sort(G.thermal_hotspots, function(a, b)
			return (a.avg_strength or 0) < (b.avg_strength or 0)
		end)
		table.remove(G.thermal_hotspots, 1)
	end

	self:calculate_hotspot_density(G.thermal_hotspots)

	for _, hotspot in ipairs(G.thermal_hotspots) do
		if hotspot.timestamp:toint() == new_hotspot.timestamp:toint() then
			if
				hotspot.cluster_density
				and hotspot.cluster_density >= (Cache.focus_thr or 1.0)
				and not G.is_in_focus_mode
			then
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.FOCUS_ON_DENSITY)
				G.is_in_focus_mode = true
				G.focus_area_center = new_hotspot.loc:copy()
				G.focus_start_time = millis()
				G.focus_wp_counter = 0
				local total_strength = 0
				local count = 0
				for _, h_cluster in ipairs(G.thermal_hotspots) do
					if h_cluster.loc:get_distance(new_hotspot.loc) < G.effective_cluster_radius then
						total_strength = total_strength + (h_cluster.avg_strength or 0)
						count = count + 1
					end
				end
				local avg_cluster_strength = count > 0 and total_strength / count or 0
				local dynamic_timeout = math.floor(3 + (avg_cluster_strength * 2))
				G.focus_wp_timeout = math.min(10, math.max(3, dynamic_timeout))
			end
			break
		end
	end

	local uncertainty_score = 0
	if (new_hotspot.max_strength or 0) < C.strong_thermal_threshold_mps then
		uncertainty_score = (C.strong_thermal_threshold_mps - (new_hotspot.max_strength or 0)) * 25
		uncertainty_score = math.max(0, math.min(50, uncertainty_score))
	end

	local necessity_score = (1.0 - G.filtered_alt_factor) * (Cache.nec_weight or 50)

	local total_score = uncertainty_score + necessity_score

	if total_score >= (Cache.retry_thr or 30) then
		G.thermal_to_retry = new_hotspot
	end

	G.current_thermal_stats = {}
end

-- Periodically samples the climb rate while the aircraft is in THERMAL mode.
function SoarNav:sample_thermal_strength()
	local G = self.State
	local ned_velocity = ahrs:get_velocity_NED()
	if not ned_velocity then
		return
	end
	local climb_rate = -ned_velocity:z()
	local sample_weight = math.min(1.0, math.abs(climb_rate) / 3.0)
	if G.current_thermal_stats.avg_strength then
		G.current_thermal_stats.avg_strength =
			(G.current_thermal_stats.avg_strength * (1 - sample_weight)) + (climb_rate * sample_weight)
	else
		G.current_thermal_stats.avg_strength = climb_rate
	end
	if climb_rate > G.current_thermal_stats.max_strength then
		G.current_thermal_stats.max_strength = climb_rate
		local current_loc = ahrs:get_location()
		if current_loc then
			G.current_thermal_stats.core_location = current_loc:copy()
		end
	end
	table.insert(G.current_thermal_stats.samples, climb_rate)
	if #G.current_thermal_stats.samples > 10 then
		table.remove(G.current_thermal_stats.samples, 1)
	end
	G.last_thermal_sample_ms = millis()
end

-- Initializes the data structure for monitoring a new thermal.
function SoarNav:start_thermal_monitoring()
	local G = self.State
	local loc = ahrs:get_location()
	if not loc then
		self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.NO_THERMAL_LOC)
		return
	end
	G.is_monitoring_thermal = true
	G.last_thermal_sample_ms = millis()
	G.current_thermal_stats = {
		entry_location = loc,
		core_location = loc:copy(),
		max_strength = -99,
		avg_strength = 0,
		samples = {},
		start_time = millis(),
		wind_at_entry = self:get_wind_vector(),
	}
end

-- Reads a polygon file from the SD card and populates the polygon points list.
function SoarNav:read_polygon_file(filename)
	local G = self.State
	local f = io.open(filename, "r")
	if not f then
		return nil
	end
	G.polygon_points = {}
	G.polygon_bounds =
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
					table.insert(G.polygon_points, point)
					sum_lat = sum_lat + lat_i7
					sum_lon = sum_lon + lon_i7
					count = count + 1
					G.polygon_bounds.min_lat = math.min(G.polygon_bounds.min_lat, lat_i7)
					G.polygon_bounds.max_lat = math.max(G.polygon_bounds.max_lat, lat_i7)
					G.polygon_bounds.min_lon = math.min(G.polygon_bounds.min_lon, lon_i7)
					G.polygon_bounds.max_lon = math.max(G.polygon_bounds.max_lon, lon_i7)
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
	G.polygon_centroid = center_loc
	G.polygon_vertex_offsets = {}
	for _, p in ipairs(G.polygon_points) do
		table.insert(G.polygon_vertex_offsets, { lat = p:lat() - center_lat, lon = p:lng() - center_lon })
	end
	local first_pt = G.polygon_points[1]
	local last_pt = G.polygon_points[#G.polygon_points]
	if first_pt:get_distance(last_pt) > 1 then
		table.insert(G.polygon_points, first_pt:copy())
		table.insert(
			G.polygon_vertex_offsets,
			{ lat = first_pt:lat() - center_lat, lon = first_pt:lng() - center_lon }
		)
	end
	self:prepare_polygon_xy_cache()
	G.using_rally_points = false
	return true
end

-- Reads rally points as a polygon and computes the convex hull.
function SoarNav:read_rally_points_as_polygon()
	local G = self.State
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
	G.polygon_points = ordered

	G.polygon_bounds =
		{ min_lat = 91 * 1e7, max_lat = -91 * 1e7, min_lon = 181 * 1e7, max_lon = -181 * 1e7 }
	local sum_lat, sum_lon = 0, 0
	for _, p in ipairs(G.polygon_points) do
		local lat_i7, lon_i7 = p:lat(), p:lng()
		sum_lat = sum_lat + lat_i7
		sum_lon = sum_lon + lon_i7
		G.polygon_bounds.min_lat = math.min(G.polygon_bounds.min_lat, lat_i7)
		G.polygon_bounds.max_lat = math.max(G.polygon_bounds.max_lat, lat_i7)
		G.polygon_bounds.min_lon = math.min(G.polygon_bounds.min_lon, lon_i7)
		G.polygon_bounds.max_lon = math.max(G.polygon_bounds.max_lon, lon_i7)
	end

	local center_lat = sum_lat / #G.polygon_points
	local center_lon = sum_lon / #G.polygon_points
	local center_loc = home:copy()
	center_loc:lat(center_lat)
	center_loc:lng(center_lon)
	G.polygon_centroid = center_loc

	G.polygon_vertex_offsets = {}
	for _, p in ipairs(G.polygon_points) do
		table.insert(G.polygon_vertex_offsets, { lat = p:lat() - center_lat, lon = p:lng() - center_lon })
	end

	local first_pt = G.polygon_points[1]
	local last_pt = G.polygon_points[#G.polygon_points]
	if first_pt:get_distance(last_pt) > 1 then
		table.insert(G.polygon_points, first_pt:copy())
	end

	G.using_rally_points = true
	self:prepare_polygon_xy_cache()
	return true
end

-- Announces the new navigation target to the GCS.
function SoarNav:log_new_waypoint(dist_to_wp)
	local G = self.State
	if G.is_in_focus_mode and G.focus_wp_counter > 1 then
		return
	end

	local d = tonumber(dist_to_wp) or 0
	local src = G.g_waypoint_source_info

	local current_loc = ahrs:get_location()
	if not current_loc or not G.target_loc then
		return
	end

	local hdg_deg = self:get_wind_corrected_heading_deg(current_loc, G.target_loc)

	local __dirs = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" }
	local __ix = math.floor((hdg_deg / 22.5) + 0.5) + 1
	if __ix > #__dirs then
		__ix = 1
	end
	local hdg_dir = __dirs[__ix]

	local total_valid_cells = #G.valid_cell_indices
	local visited_cells = total_valid_cells - #G.unvisited_cell_indices

	local __show_cells = true
	if
		string.find(src, "Thrm", 1, true)
		or src == G.SoarNavWaypointSources.THERMAL_STREET
		or src == G.SoarNavWaypointSources.RANDOM_FALLBACK
		or src == G.SoarNavWaypointSources.TERRAIN_EVASION
		or src == G.SoarNavWaypointSources.TE_EGRESS
	then
		__show_cells = false
	end
	if src == G.SoarNavWaypointSources.FOCUS_MODE or string.find(src, "Focus", 1, true) then
		__show_cells = false
	end
	if src == G.SoarNavWaypointSources.REENGAGE_ENTRY or src == G.SoarNavWaypointSources.REENGAGE_FLYOUT then
		__show_cells = false
	end

	if __show_cells then
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.NAV_TARGET_CELLS, src, hdg_deg, hdg_dir, d, visited_cells, total_valid_cells)
	else
		local dir_out = hdg_dir
		if (src == G.SoarNavWaypointSources.TERRAIN_EVASION or src == G.SoarNavWaypointSources.TE_EGRESS) and G.TE and G.TE.turn_arrow and G.TE.turn_arrow ~= "" then
			dir_out = hdg_dir .. " " .. G.TE.turn_arrow
		end
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.NAV_TARGET_SIMPLE, src, hdg_deg, dir_out, d)
	end
end

-- Finalizes the selection of a new waypoint by setting global state variables.
function SoarNav:_finalize_waypoint_selection()
	local G = self.State
	G.waypoint_start_time_ms = millis()
	G.waypoint_search_in_progress = false
	G.last_commanded_roll_deg = 0
	G.last_heading_error = 0
	G.heading_err_f = nil
	G.last_heading_error_f = nil
	G.last_progress_check_ms = nil
	G.distance_at_last_check = -1
	G.stuck_counter = 0
	local loc = ahrs:get_location()
	if loc and G.target_loc then
		local dist_to_wp = loc:get_distance(G.target_loc)
		self:log_new_waypoint(dist_to_wp)
		G.initial_distance_to_wp = dist_to_wp
		if G.g_waypoint_source_info == G.SoarNavWaypointSources.TERRAIN_EVASION then
			G.reroute_check_armed = false
		else
			G.reroute_check_armed = true
		end
	end
end

-- Selects the best thermal from memory, applies drift, and validates it.
function SoarNav:select_best_thermal_waypoint()
	local G = self.State
	local Cache = self.Cache
	local valid_hotspots = self:clean_and_get_hotspots()
	if #valid_hotspots == 0 then
		return false
	end

	local good_hotspots = {}
	local good_hotspots_excluding_last = {}
	local loc = ahrs:get_location()
	if not loc then
		return false
	end
	local min_dist_sq = (Cache.wp_radius * 2) ^ 2

	for _, hotspot in ipairs(valid_hotspots) do
		if hotspot.avg_strength and hotspot.avg_strength > 0 then
			local dist_vec = loc:get_distance_NE(hotspot.loc)
			local dist_sq = dist_vec:x() ^ 2 + dist_vec:y() ^ 2
			if dist_sq and dist_sq > min_dist_sq then
				table.insert(good_hotspots, hotspot)
				local last_used_int = 0
				if G.last_used_hotspot_timestamp then
					last_used_int = G.last_used_hotspot_timestamp:toint()
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
	if G.energy_state == "LOW" then
		quality_weight = 0.5
	elseif G.energy_state == "CRITICAL" then
		quality_weight = 0.7
	end

	table.sort(final_hotspot_list, function(a, b)
		local strength_weight = 1.0 - quality_weight
		local score_a =
			(a.avg_strength or 0) * strength_weight
			+ ((a.avg_strength or 0) * (a.consistency_score or 0.5)) * quality_weight
		local score_b =
			(b.avg_strength or 0) * strength_weight
			+ ((b.avg_strength or 0) * (b.consistency_score or 0.5)) * quality_weight
		return score_a > score_b
	end)

	for _, best_hotspot in ipairs(final_hotspot_list) do
		local age_seconds = 0
		if best_hotspot.timestamp then
			age_seconds = self:safe_time_diff(millis(), best_hotspot.timestamp) / 1000.0
		end
		local wind_vec = best_hotspot.wind_vec or self:get_wind_vector()
		local drifted_loc = self:predict_thermal_drift(best_hotspot.loc, wind_vec, age_seconds)

		if self:in_flight_area(drifted_loc) then
			if self:is_path_terrain_safe(drifted_loc) then
				local drift_dist = best_hotspot.loc:get_distance(drifted_loc)
				if drift_dist > 0 then
					G.g_waypoint_source_info =
						string.format(G.SoarNavWaypointSources.THERMAL_MEMORY_DRIFT, best_hotspot.avg_strength)
				else
					G.g_waypoint_source_info =
						string.format(G.SoarNavWaypointSources.THERMAL_MEMORY_NODRIFT, best_hotspot.avg_strength)
				end

				G.current_selected_hotspot = best_hotspot
				self:set_target_safely(drifted_loc)
				G.last_used_hotspot_timestamp = best_hotspot.timestamp
				return true
			end
		end
	end

	return false
end

-- Selects the most upwind cell from a list of candidates.
function SoarNav:score_and_select_best_cell(candidate_indices)
	local G = self.State
	if not candidate_indices or #candidate_indices == 0 then
		return nil
	end

	local loc = ahrs:get_location()
	local cur_idx = loc and self:get_cell_index_from_location(loc) or nil
	if cur_idx then
		local filtered = {}
		for _, idx in ipairs(candidate_indices) do
			if idx ~= cur_idx then
				table.insert(filtered, idx)
			end
		end
		if #filtered > 0 then
			candidate_indices = filtered
		end
	end

	if #candidate_indices == 1 then
		return candidate_indices[1]
	end

	local wind = self:get_wind_vector()
	if not loc or not wind or wind:length() < 1.0 then
		return candidate_indices[math.random(#candidate_indices)]
	end

	local upwind_bearing_rad = math.rad((self:wind_vector_to_bearing_deg(wind) + 180) % 360)

	local best_score = -math.huge
	local best_cell_index = nil

	for _, cell_idx in ipairs(candidate_indices) do
		local cell_center = G.grid_cell_centers[cell_idx]
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
function SoarNav:check_and_use_thermal_street()
	local G = self.State
	local hotspots = self:clean_and_get_hotspots()
	if #hotspots < self.Const.MIN_HOTSPOTS_FOR_STREET then
		return false
	end

	local wind_vec = self:get_wind_vector()
	if not wind_vec or wind_vec:length() < 1.0 then
		return false
	end

	table.sort(hotspots, function(a, b)
		return a.timestamp:toint() > b.timestamp:toint()
	end)

	local h1 = hotspots[1]
	local h2 = hotspots[2]

	if not h1 or not h2 or not h1.loc or type(h1.loc.lat) ~= "function" then
		return false
	end
	if not h2.loc or type(h2.loc.lat) ~= "function" then
		return false
	end

	if h1.loc:get_distance(h2.loc) < 150 then
		return false
	end

	local wind_bearing = self:wind_vector_to_bearing_deg(wind_vec)
	local street_bearing = h2.loc:get_bearing(h1.loc)
	local angle_diff = math.abs(((street_bearing - wind_bearing + 540) % 360) - 180)

	if angle_diff <= (self.Cache.street_tol or 30) then
		local projection_dist = h2.loc:get_distance(h1.loc)
		projection_dist = math.max(500, math.min(projection_dist, 2000))

		local new_target = h1.loc:copy()
		new_target:offset_bearing((wind_bearing + 180) % 360, projection_dist)

		local loc = ahrs:get_location()
		if self:in_flight_area(new_target) and loc then
			local dist_to_target = loc:get_distance(new_target)
			if dist_to_target > (self.Cache.wp_radius * 2) then
				self:set_target_safely(new_target)
				G.g_waypoint_source_info = G.SoarNavWaypointSources.THERMAL_STREET
				return true
			end
		end
	end

	return false
end

-- Core logic for searching and selecting a new waypoint.
function SoarNav:search_for_new_waypoint()
	local G = self.State
	local Cache = self.Cache
	local source_for_log = G.SoarNavWaypointSources.UNKNOWN
	local new_wp_found = false

	if G.force_egress then
		G.force_egress = false
		local loc = ahrs:get_location()
		if loc and G.TE.last_safe_heading then
			local as = (ahrs.airspeed_estimate and ahrs:airspeed_estimate()) or 0
			local vned = ahrs:get_velocity_NED()
			local gs = 0
			if vned then gs = math.sqrt((vned:x() or 0)^2 + (vned:y() or 0)^2) end
			local speed_ref = (as > 3) and as or ((gs > 0) and gs or (Cache.airspeed_cruise or 15))
			local egress_dist_mult = 4.0
			local egress_dist = (Cache.te_look_s or 10.0) * speed_ref * egress_dist_mult
			egress_dist = math.max(250, math.min(10000, egress_dist))
			local new_target = loc:copy()
			new_target:offset_bearing(G.TE.last_safe_heading, egress_dist)
			self:set_target_safely(new_target)
			source_for_log = G.SoarNavWaypointSources.TE_EGRESS
			G.g_waypoint_source_info = source_for_log
			self:_finalize_waypoint_selection()
			return
		else
			G.force_egress = false
		end
	end

	local throttle_output = SRV_Channels:get_output_scaled(self.Const.THROTTLE_SRV_CHANNEL) or 0
	local throttle_on = throttle_output > 1

	if throttle_on then
		G.thermal_to_retry = nil
		G.reengage_final_target = nil
		G.reengage_hold_active = false
		G.reengage_hold_until_ms = nil
	end

	if G.thermal_to_retry and not throttle_on then
		local hotspot_to_retry = G.thermal_to_retry
		G.thermal_to_retry = nil
		local current_loc = ahrs:get_location()
		if hotspot_to_retry and hotspot_to_retry.loc and current_loc then
			local bearing_to_thermal = current_loc:get_bearing(hotspot_to_retry.loc)
			local tas_sensor = ahrs:airspeed_estimate()
			local vned = ahrs:get_velocity_NED()
			local gs_scalar = (vned and math.sqrt((vned:x() or 0) ^ 2 + (vned:y() or 0) ^ 2)) or 0
			local va_param = Cache.airspeed_cruise or 15
			local speed_for_calc =
				(tas_sensor and tas_sensor > 3.0) and tas_sensor or ((gs_scalar > 3.0) and gs_scalar or va_param)
			local reengage_dist = math.max(400, math.min(1000, speed_for_calc * 20.0))
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
			local flyout_point =
				best_p
				or (function()
					local fp = current_loc:copy()
					fp:offset_bearing(((bearing_to_thermal + 180) % 360), reengage_dist)
					return fp
				end)()

			if not self:in_flight_area(flyout_point) then
				flyout_point = self:clamp_inside_polygon(flyout_point, Cache.wp_radius or 50)
			end

			local path_safe = self:is_path_terrain_safe(flyout_point)
			local max_attempts = 5
			local attempt = 1
			local min_flyout_dist = 200
			while not path_safe and attempt <= max_attempts do
				local test_dist = reengage_dist * (0.5 / attempt)
				if test_dist < min_flyout_dist then
					break
				end
				local test_point = current_loc:copy()
				test_point:offset_bearing(bearing_to_thermal + 180, test_dist)
				if not self:in_flight_area(test_point) then
					test_point = self:clamp_inside_polygon(test_point, Cache.wp_radius or 50)
				end
				if self:in_flight_area(test_point) and self:is_path_terrain_safe(test_point) then
					flyout_point = test_point
					path_safe = true
					break
				end
				attempt = attempt + 1
			end

			if path_safe then
				self:set_target_safely(flyout_point)
				local age_s = self:safe_time_diff(millis(), hotspot_to_retry.timestamp) / 1000.0
				local wind = hotspot_to_retry.wind_vec or self:get_wind_vector()
				local predicted = self:predict_thermal_drift(hotspot_to_retry.loc, wind, age_s) or hotspot_to_retry.loc
				local bearing_back = current_loc:get_bearing(predicted)
				local dist_back = current_loc:get_distance(predicted)
				local reentry_cap = reengage_dist * 2.0
				if dist_back > reentry_cap then
					local capped = current_loc:copy()
					capped:offset_bearing(bearing_back, reentry_cap)
					if G.use_polygon_area then
						if not self:in_flight_area(capped) then
							capped = self:clamp_inside_polygon(capped, Cache.wp_radius or 50)
						end
						if not self:segment_stays_inside(current_loc, capped) then
							capped = self:adjust_target_segment(current_loc, capped)
						end
					end
					G.reengage_final_target = capped
				else
					G.reengage_final_target = predicted
				end
				if G.reengage_final_target and not self:is_path_terrain_safe(G.reengage_final_target) then
					G.reengage_final_target = nil
				end
				source_for_log = G.SoarNavWaypointSources.REENGAGE_FLYOUT
				new_wp_found = true
			else
				G.thermal_to_retry = nil
				G.reengage_final_target = nil
				G.force_new_search = true
				return
			end
		end
	end
	if new_wp_found then
		G.g_waypoint_source_info = source_for_log
		self:_finalize_waypoint_selection()
		return
	end

	local force_grid_search = false
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	if dist_from_home_3d then
		local current_alt = -dist_from_home_3d:z()
		local soar_alt_max = Cache.soar_alt_max
		if soar_alt_max and soar_alt_max > 0 and current_alt > soar_alt_max then
			force_grid_search = true
		end
	end
	if G.lost_thermal_counter >= 2 then
		G.force_grid_after_reset = true
		G.lost_thermal_counter = 0
	end

	local try_smart_strategies = true
	if G.force_grid_after_reset then
		G.force_grid_after_reset = false
		try_smart_strategies = false
	end

	if try_smart_strategies then
		if not force_grid_search then
			if G.is_in_focus_mode then
				if G.focus_wp_counter >= (G.focus_wp_timeout or 3) then
					G.is_in_focus_mode = false
				else
					local age_seconds = self:safe_time_diff(millis(), G.focus_start_time) / 1000.0
					local wind = self:get_wind_vector()
					local drifted_focus_center =
						self:predict_thermal_drift(G.focus_area_center, wind, age_seconds)
						or G.focus_area_center
					local focus_radius = G.effective_cluster_radius / 2
					local focus_wp_ok = false
					for _ = 1, 3 do
						local new_target = self:generate_target_around_point(drifted_focus_center, focus_radius)
						if new_target and self:is_path_terrain_safe(new_target) then
							self:set_target_safely(new_target)
							source_for_log = string.format(G.SoarNavWaypointSources.FOCUS_MODE, G.focus_wp_counter + 1)
							G.focus_wp_counter = G.focus_wp_counter + 1
							new_wp_found = true
							focus_wp_ok = true
							break
						end
					end
					if not focus_wp_ok then
						G.is_in_focus_mode = false
					end
				end
			end
			if not new_wp_found then
				local energy_status = self:assess_energy_state()
				if energy_status == "LOW" or energy_status == "CRITICAL" then
					if #self:clean_and_get_hotspots() > 0 and self:select_best_thermal_waypoint() then
						new_wp_found = true
						source_for_log = G.g_waypoint_source_info
					end
				end
			end
			if not new_wp_found and Cache.tmem_enabled and self:select_best_thermal_waypoint() then
				new_wp_found = true
				source_for_log = G.g_waypoint_source_info
			end
		end

		if not new_wp_found and self:try_ridge_target() then
			if G.target_loc and self:is_path_terrain_safe(G.target_loc) then
				new_wp_found = true
				source_for_log = G.SoarNavWaypointSources.RIDGE
			else
				G.target_loc = nil
			end
		end

		if not new_wp_found and self:check_and_use_thermal_street() then
			if G.target_loc and self:is_path_terrain_safe(G.target_loc) then
				new_wp_found = true
				source_for_log = G.g_waypoint_source_info
			else
				G.target_loc = nil
			end
		end
	end

	if not new_wp_found then
		if G.grid_initialized and #G.valid_cell_indices > 0 then
			if #G.unvisited_cell_indices == 0 then
				for _, cell_idx in ipairs(G.valid_cell_indices) do
					if G.grid_cells[cell_idx] then
						G.grid_cells[cell_idx].visit_count = 0
					end
				end
				G.unvisited_cell_indices = {}
				for _, v in ipairs(G.valid_cell_indices) do
					table.insert(G.unvisited_cell_indices, v)
				end
				G.force_grid_after_reset = true
			end

			local recent_success_count = 0
			local now_ms = millis()
			for _, hotspot in ipairs(G.thermal_hotspots) do
				if
					hotspot.entry_time
					and self:safe_time_diff(now_ms, hotspot.entry_time) <= ((Cache.strat_hist or 900) * 1000)
				then
					recent_success_count = recent_success_count + 1
				end
			end

			local chosen_cell_index
			local success_rate_factor = math.min(1, recent_success_count / 3.0)
			local guided_chance = 25 + (50 * success_rate_factor)

			local loc = ahrs:get_location()
			local cur_idx = loc and self:get_cell_index_from_location(loc) or nil

			local function get_filtered_list(list_in)
				if not cur_idx or #list_in <= 1 then
					return list_in
				end
				local filtered = {}
				for _, idx in ipairs(list_in) do
					if idx ~= cur_idx then
						table.insert(filtered, idx)
					end
				end
				if #filtered > 0 then
					return filtered
				end
				return list_in
			end

			if math.random(1, 100) <= guided_chance then
				source_for_log = G.SoarNavWaypointSources.GRID_GUIDED
				if #G.unvisited_cell_indices > 0 then
					chosen_cell_index = self:score_and_select_best_cell(G.unvisited_cell_indices)
				else
					local min_visits = math.huge
					for _, cell_idx in ipairs(G.valid_cell_indices) do
						min_visits = math.min(min_visits, G.grid_cells[cell_idx].visit_count)
					end
					local least_visited_indices = {}
					for _, cell_idx in ipairs(G.valid_cell_indices) do
						if G.grid_cells[cell_idx].visit_count == min_visits then
							table.insert(least_visited_indices, cell_idx)
						end
					end
					chosen_cell_index = self:score_and_select_best_cell(least_visited_indices)
				end
			else
				source_for_log = G.SoarNavWaypointSources.GRID_PURE
				if #G.unvisited_cell_indices > 0 then
					local pool = get_filtered_list(G.unvisited_cell_indices)
					local random_table_index = math.random(1, #pool)
					chosen_cell_index = pool[random_table_index]
				else
					local pool = get_filtered_list(G.valid_cell_indices)
					local random_table_index = math.random(1, #pool)
					chosen_cell_index = pool[random_table_index]
				end
			end

			if chosen_cell_index and G.grid_cell_centers[chosen_cell_index] then
				local precalculated_center = G.grid_cell_centers[chosen_cell_index]
				local current_loc = ahrs:get_location()
				local success = false
				local min_grid_dist = (G.grid_cell_size_m or 50) * 1.5
				for _ = 1, 5 do
					local new_target_loc = precalculated_center:copy()
					local offset_radius = math.floor(G.grid_cell_size_m / 2.5)
					new_target_loc:offset_bearing(math.random() * 360, math.sqrt(math.random()) * offset_radius)
					if
						self:in_flight_area(new_target_loc)
						and current_loc
						and current_loc:get_distance(new_target_loc) > min_grid_dist
						and self:is_path_terrain_safe(new_target_loc)
					then
						self:set_target_safely(new_target_loc)
						success = true
						break
					end
				end
				if success then
					new_wp_found = true
				else
					if
						self:in_flight_area(precalculated_center)
						and current_loc
						and current_loc:get_distance(precalculated_center) > min_grid_dist
						and self:is_path_terrain_safe(precalculated_center)
					then
						self:set_target_safely(precalculated_center)
						new_wp_found = true
					end
				end
			end
		end
	end

	if not new_wp_found then
		self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.ALL_STRAT_FAILED)
		local center_point = self:get_active_center_location()
		if center_point then
			local radius_m = G.max_operational_distance / 2
			if radius_m <= 0 then
				radius_m = Cache.radius_m or 500
			end
			for _ = 1, 10 do
				local fallback_loc = center_point:copy()
				fallback_loc:offset_bearing(math.random() * 360, math.sqrt(math.random()) * radius_m)
				if self:in_flight_area(fallback_loc) then
					self:set_target_safely(fallback_loc)
					source_for_log = G.SoarNavWaypointSources.RANDOM_FALLBACK
					new_wp_found = true
					break
				end
			end
		end
	end

	if new_wp_found then
		G.g_waypoint_source_info = source_for_log
		self:_finalize_waypoint_selection()
	else
		self:log_gcs(self.Msg.MAV_SEVERITY.CRITICAL, 0, self.Msg.MSG_ID.FATAL_NO_WP)
	end
end

-- Calculates a dynamic waypoint timeout based on current wind speed.
function SoarNav:get_wp_timeout()
	local C = self.Const
	local Cache = self.Cache
	local base_timeout = (Cache.wp_timeout or 300) * 1000
	local mode = C.WP_TIMEOUT_WIND_MODE or 0
	if mode == 0 then
		return base_timeout
	end
	local ref = C.WP_TIMEOUT_WIND_REF_MPS or 15
	local minmul = C.WP_TIMEOUT_WIND_MIN_MULT or 0.5
	local maxmul = C.WP_TIMEOUT_WIND_MAX_MULT or 2.5
	local wind_vec = self:get_wind_vector()
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
function SoarNav:check_motor_failure(current_mode, dist_from_home_3d)
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	if G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE or not arming:is_armed() or not dist_from_home_3d then
		return
	end

	local dyn_soalt_mode = Cache.dyn_soalt
	if dyn_soalt_mode == 0 or G.script_state ~= G.SCRIPT_STATE.NAVIGATING or current_mode == C.mode_thermal then
		G.motor_failure_check_active = false
		G.motor_on_start_time_ms = nil
		G.rpm_failure_start_ms = nil
		return
	end

	if dyn_soalt_mode == 3 then
		G.motor_failure_check_active = false
		G.motor_on_start_time_ms = nil
		G.rpm_failure_start_ms = nil
		return
	end

	local wp_radius = Cache.wp_radius or 50
	local suppress_radius = math.max(100, wp_radius * 2)
	if dist_from_home_3d:xy():length() <= suppress_radius then
		return
	end

	local soar_alt_min = Cache.soar_alt_min
	if not soar_alt_min then
		return
	end

	local current_alt = -dist_from_home_3d:z()
	local throttle_output = SRV_Channels:get_output_scaled(C.THROTTLE_SRV_CHANNEL) or 0
	local motor_should_be_on = (current_alt < soar_alt_min) and (throttle_output > C.THROTTLE_ACTIVE_THRESHOLD)

	if not motor_should_be_on then
		G.motor_failure_check_active = false
		G.motor_on_start_time_ms = nil
		G.rpm_failure_start_ms = nil
		return
	end

	local now = millis()
	local rpm_value = RPM and RPM.get_rpm and RPM:get_rpm(0)

	local function trigger_rtl()
		self:log_gcs(self.Msg.MAV_SEVERITY.CRITICAL, 0, self.Msg.MSG_ID.MOTOR_FAIL_RTL)
		vehicle:set_mode(C.RTL_MODE_CODE)
		self:restore_boot_alts("MOTOR Failure RTL")
		self:set_script_state(G.SCRIPT_STATE.ERROR, self.Msg.MSG_ID.MOTOR_FAIL_REASON)
	end

	if rpm_value ~= nil and rpm_value >= 0 then
		G.motor_on_start_time_ms = nil
		local rpm_issue = (throttle_output > C.MOTOR_RPM_FAIL_THROTTLE_PERCENT)
			and (rpm_value < C.MOTOR_RPM_FAIL_RPM_MIN)

		if rpm_issue then
			if not G.rpm_failure_start_ms then
				G.rpm_failure_start_ms = now
			end
		else
			G.rpm_failure_start_ms = nil
		end

		if G.rpm_failure_start_ms and self:safe_time_diff(now, G.rpm_failure_start_ms) > C.MOTOR_RPM_FAIL_DEBOUNCE_MS then
			trigger_rtl()
		end
	else
		G.rpm_failure_start_ms = nil
		if not G.motor_failure_check_active then
			G.motor_failure_check_active = true
			G.motor_on_start_time_ms = now
			G.altitude_at_check_start_m = current_alt
		end

		local time_since_motor_on = self:safe_time_diff(now, G.motor_on_start_time_ms)
		if time_since_motor_on > C.MOTOR_FAILURE_CHECK_DELAY_MS then
			local ned_velocity = ahrs:get_velocity_NED()
			if not ned_velocity then
				return
			end

			local climb_rate = -ned_velocity:z()
			local altitude_gained_ok = -dist_from_home_3d:z() > (G.altitude_at_check_start_m + 1.0)

			if not (climb_rate >= C.MOTOR_FAILURE_CLIMB_RATE_THRESHOLD_MPS) and not altitude_gained_ok then
				trigger_rtl()
			else
				G.motor_failure_check_active = false
				G.motor_on_start_time_ms = nil
			end
		end
	end
end

-- ============================================================================
-- TERRAIN EVASION VELOCITY-ADAPTIVE PARAMETERS
-- ============================================================================

-- Computes velocity-dependent TE parameters (cached for performance).
function SoarNav:_te_compute_params(speed)
	local G = self.State
	G.TE = G.TE or {}
	local te = G.TE
	local now = millis()
	local TC = self.TE_Config
	local v = math.max(TC.SPEED_MIN, math.min(TC.SPEED_MAX, speed or 15))

	if te._tp_cache and te._tp_cache_ms and (now - te._tp_cache_ms) < 500 then
		if math.abs((te._tp_cache.v or 0) - v) < 4 then
			return te._tp_cache
		end
	end

	local T_LOOK = self.Cache.te_look_s or 10
	local k_c = T_LOOK / 10.0
	local buf_min_user = self.Cache.te_buf_min or TC.FLOOR_BUFFER
	local tp = {
		v = v,
		k_c = k_c,
		lookahead_min = math.max(TC.FLOOR_LOOKAHEAD, v * TC.T_LOOKAHEAD_MIN * k_c),
		spike_check = v * TC.T_SPIKE_CHECK * k_c,
		sample_offset = v * TC.T_SAMPLE_OFFSET * k_c,
		early_exit = v * TC.T_EARLY_EXIT * k_c,
		path_skip = v * TC.T_PATH_SKIP * k_c,
		lever_max = math.max(TC.FLOOR_LEVER, v * TC.T_LEVER_MAX * k_c),
		buffer_min = math.max(buf_min_user, v * TC.T_BUFFER_BASE * k_c),
		buffer_max = math.max(buf_min_user * 2, v * TC.T_BUFFER_MAX * k_c),
		t_egress = TC.T_EGRESS_BASE * k_c,
		gate_ms = (TC.D_GATE * k_c / v) * 1000,
		side_cooldown = (TC.D_SIDE_COOLDOWN * k_c / v) * 1000,
		offset_int_ms = (TC.D_OFFSET_INTERVAL / v) * 1000,
		gate_roll_bonus = (TC.D_GATE_ROLL_BONUS * k_c / v) * 1000,
		fan_half_max = math.min(55, TC.K_FAN_HALF_MAX / v),
		fan_half_min = math.max(10, TC.K_FAN_HALF_MIN / v),
		angle_step = math.max(6, TC.K_ANGLE_STEP / v),
		turn_max_base = math.max(20, TC.K_TURN_MAX_BASE / v),
		turn_max_cap = math.min(150, 90 + TC.K_TURN_MAX_CAP / v),
		duplicate_thr = math.max(5, TC.K_DUPLICATE_THR / v),
		min_gain = v * TC.T_MIN_GAIN * k_c,
		min_gain_side = v * TC.T_MIN_GAIN_SIDE * k_c,
		alpha_terrain = TC.ALPHA_TERRAIN_BASE + v / TC.ALPHA_TERRAIN_K,
		alpha_gs = TC.ALPHA_GS_BASE + v / TC.ALPHA_GS_K,
		alpha_vario = TC.ALPHA_VARIO_BASE + v / TC.ALPHA_VARIO_K,
		p_incr_max = TC.P_GAIN_INCR_BASE + v / TC.P_GAIN_INCR_K,
		p_deficit_mult = TC.P_DEFICIT_MULT_BASE + v / TC.P_DEFICIT_MULT_K,
	}
	te._tp_cache = tp
	te._tp_cache_ms = now
	return tp
end

-- ============================================================================
-- NAVIGATION STATE HANDLER (REFACTORED LOGIC)
-- ============================================================================

-- Estimates the current navigation speed (Airspeed or Groundspeed).
function SoarNav:_nav_speed_mps()
	local as = ahrs:airspeed_estimate() or 0
	local vned = ahrs:get_velocity_NED()
	local gs = 0
	if vned then
		gs = math.sqrt((vned:x() or 0) ^ 2 + (vned:y() or 0) ^ 2)
	end
	if as and as > 3 then
		return as
	end
	return gs
end

-- Calculates and smooths the offset between real AGL (Lidar) and database AGL (SRTM).
function SoarNav:update_tevas_terrain_offset(loc, current_amsl_m)
	local G = self.State
	local TC = self.TE_Config
	local ROTATION_PITCH_270 = 25  -- downward-facing orientation
	if self.Cache.dyn_soalt ~= 3 or not rangefinder or not rangefinder:has_orientation(ROTATION_PITCH_270) then
		return
	end

	local speed = self:_nav_speed_mps()
	local tp = self:_te_compute_params(speed)

	local now = millis()
	if self:safe_time_diff(now, G.TE.tevas_last_offset_update_ms) < tp.offset_int_ms then
		return
	end
	G.TE.tevas_last_offset_update_ms = now

	local real_agl = rangefinder:distance_orient(ROTATION_PITCH_270)
	if not real_agl or real_agl <= 0 or real_agl > TC.RANGEFINDER_MAX then
		return
	end

	local terrain_amsl, _ = terrain:height_amsl(loc, true)
	if not terrain_amsl or not current_amsl_m then
		return
	end

	local db_agl = current_amsl_m - terrain_amsl
	local measured_offset = real_agl - db_agl

	local alpha = tp.alpha_terrain
	G.TE.tevas_terrain_offset_m = (G.TE.tevas_terrain_offset_m * (1.0 - alpha)) + (measured_offset * alpha)
end

-- Helper function to check minimum AGL along a path segment.
function SoarNav:get_min_agl_on_segment_fast(
	start_loc,
	bearing_deg,
	current_amsl_m,
	lookahead_s,
	gs_along_path,
	sink_best,
	vario_mps,
	buffer_m,
	wind_vec_xy,
	use_arc_sampling,
	r_min,
	current_heading_rad,
	tp
)
	local G = self.State
	local TC = self.TE_Config
	tp = tp or self:_te_compute_params(gs_along_path)
	local bearing_rad = math.rad(bearing_deg)
	local heading_n, heading_e = math.cos(bearing_rad), math.sin(bearing_rad)
	local align_lee = 0
	if wind_vec_xy then
		local wind_mag = wind_vec_xy:length()
		if wind_mag > 0.1 then
			local wind_n, wind_e = wind_vec_xy:x(), wind_vec_xy:y()
			align_lee = (heading_n * wind_n + heading_e * wind_e) / wind_mag
		end
	end
	align_lee = math.max(0, align_lee)
	local sink_eff = sink_best * (1.0 + TC.SINK_LEE_FACTOR * align_lee)

	local dist_total = math.max(tp.lookahead_min, gs_along_path * lookahead_s)
	local num_samples = TC.NUM_SEGMENT_SAMPLES
	local worst_agl = math.huge
	local data_ok = true

	local arc_center_loc, start_angle_from_center_rad, arc_dir
	if use_arc_sampling then
		local start_bearing_rad = current_heading_rad
		local target_bearing_rad = bearing_rad
		local delta_bearing = (target_bearing_rad - start_bearing_rad + math.pi * 3) % (math.pi * 2) - math.pi
		arc_dir = (delta_bearing >= 0) and 1.0 or -1.0

		local center_bearing_rad = start_bearing_rad + (arc_dir * math.pi / 2.0)
		arc_center_loc = start_loc:copy()
		arc_center_loc:offset_bearing(math.deg(center_bearing_rad), r_min)
		start_angle_from_center_rad = arc_center_loc:get_bearing(start_loc)
	end

	local sample_offset = tp.sample_offset

	local function agl_at(dist)
		local p = start_loc:copy()

		if use_arc_sampling then
			local angle_travelled_rad = (dist / r_min)
			local sample_angle_rad = start_angle_from_center_rad + (angle_travelled_rad * arc_dir)
			p = arc_center_loc:copy()
			p:offset_bearing(math.deg(sample_angle_rad), r_min)
		else
			p:offset_bearing(bearing_deg, dist)
		end

		if G.use_polygon_area then
			if not self:in_flight_area(p) then
				p = self:clamp_inside_polygon(p, sample_offset * 0.5)
			end
		end

		local ter, _ = terrain:height_amsl(p, true)
		if not ter then
			local p_n = p:copy()
			p_n:offset_bearing(0, sample_offset)
			local p_e = p:copy()
			p_e:offset_bearing(90, sample_offset)
			local p_s = p:copy()
			p_s:offset_bearing(180, sample_offset)
			local p_w = p:copy()
			p_w:offset_bearing(270, sample_offset)

			local ter_n, _ = terrain:height_amsl(p_n, true)
			local ter_e, _ = terrain:height_amsl(p_e, true)
			local ter_s, _ = terrain:height_amsl(p_s, true)
			local ter_w, _ = terrain:height_amsl(p_w, true)

			ter = ter_n or ter_e or ter_s or ter_w
			if ter_n and ter_n > ter then
				ter = ter_n
			end
			if ter_e and ter_e > ter then
				ter = ter_e
			end
			if ter_s and ter_s > ter then
				ter = ter_s
			end
			if ter_w and ter_w > ter then
				ter = ter_w
			end
		end

		if not ter then
			data_ok = false
			return math.huge
		end

		local t = dist / math.max(gs_along_path, 1.0)
		local est_amsl = current_amsl_m + vario_mps * t - sink_eff * t
		local ter_corrected = ter - G.TE.tevas_terrain_offset_m
		return est_amsl - ter_corrected
	end

	local spacing = TC.SEGMENT_SPACING
	local spike_check_dist_m = tp.spike_check
	local early_exit_margin = tp.early_exit

	for i = 1, num_samples do
		local d = dist_total * spacing[i]
		local agl = agl_at(d)

		if agl < buffer_m then
			local agl_before = agl_at(math.max(0, d - spike_check_dist_m))
			local agl_after = agl_at(math.min(dist_total, d + spike_check_dist_m))

			if not (agl_before >= buffer_m and agl_after >= buffer_m) then
				if agl < worst_agl then
					worst_agl = agl
				end
				if agl < (buffer_m - early_exit_margin) then
					return agl, data_ok, agl
				end
			end
		else
			if agl < worst_agl then
				worst_agl = agl
			end
		end
	end

	local agl_end_of_segment = agl_at(dist_total)
	if agl_end_of_segment < worst_agl then
		worst_agl = agl_end_of_segment
	end

	return worst_agl, data_ok, agl_end_of_segment
end

-- Checks if the path to a candidate location is safe from terrain.
function SoarNav:is_path_terrain_safe(candidate_loc)
	local G = self.State
	local Cache = self.Cache
	local TC = self.TE_Config

	if Cache.dyn_soalt ~= 3 or not terrain or not terrain:enabled() then
		return true
	end

	local loc = ahrs:get_location()
	local vned = ahrs:get_velocity_NED()
	local home = ahrs:get_home()
	local ned = ahrs:get_relative_position_NED_home()
	if not loc or not vned or not home or not ned then
		return false
	end

	local current_amsl = (home:alt() / 100.0) + (-ned:z())
	local gs_xy = math.sqrt((vned:x() or 0) ^ 2 + (vned:y() or 0) ^ 2)
	local speed_ref = (gs_xy > 1) and gs_xy or (Cache.airspeed_cruise or 12)

	local tp = self:_te_compute_params(speed_ref)

	local current_terrain_amsl, ter_ok = terrain:height_amsl(loc, true)
	if not ter_ok or not current_terrain_amsl then
		return true
	end

	local current_agl = current_amsl - current_terrain_amsl
	local buffer_m = math.max(tp.buffer_min, math.min(tp.buffer_max, speed_ref * tp.k_c * self.TE_Config.T_BUFFER_BASE))

	if current_agl < buffer_m then
		return true
	end

	local sink_best = self:sink_best_now()
	local vario_mps = (G.TE and G.TE.vario_ema_mps) or (-vned:z())
	local safety_buffer = buffer_m * TC.SAFETY_PATH_FACTOR

	local bearing_deg = math.deg(loc:get_bearing(candidate_loc))
	local dist = loc:get_distance(candidate_loc)
	if dist < tp.path_skip then
		return true
	end
	local lookahead_s = dist / math.max(1.0, speed_ref)

	local min_agl, data_ok = self:get_min_agl_on_segment_fast(
		loc,
		bearing_deg,
		current_amsl,
		lookahead_s,
		speed_ref,
		sink_best,
		vario_mps,
		safety_buffer,
		nil,
		false,
		0,
		ahrs:get_yaw_rad(),
		tp
	)

	if not data_ok then
		self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 2, self.Msg.MSG_ID.TERRAIN_DATA_MISSING_WARN)
		return true
	end

	return min_agl >= safety_buffer
end

-- Returns robust HAGL and AMSL using HAGL sensor when available, terrain AMSL as fallback.
-- Input: loc, home, ned; Output: hagl_now (m AGL), amsl (m AMSL).
function SoarNav:_robust_hagl_and_amsl(loc, home, ned)
	local ter_amsl, ter_ok = terrain:height_amsl(loc, true)
	local hagl = ahrs:get_hagl()
	local amsl
	if hagl and ter_ok then
		amsl = ter_amsl + hagl
	else
		amsl = (home:alt()/100.0) + (-ned:z())
	end
	local hagl_now
	if hagl then
		hagl_now = hagl
	elseif ter_ok then
		hagl_now = amsl - ter_amsl
	else
		hagl_now = math.max(0, -ned:z())
	end
	return hagl_now, amsl
end

-- Navigation altitude gate: TE active uses HAGL guard, otherwise relative-home guard.
-- Returns true if altitude conditions for navigation are satisfied.
function SoarNav:_nav_altitude_check(tevas_active, home, ned, loc, initial_min)
	local G = self.State
	local C = self.Const
	local min_alt = initial_min or (self.Cache.soar_alt_min or 100)
	local mult = C.HOME_CYLINDER_MULT
	local cyl_R = math.max(50, min_alt * mult)

	local dist2d = 0
	if ned and ned.xy then
		dist2d = ned:xy():length()
	elseif ahrs and ahrs.get_relative_position_NED_home then
		local n = ahrs:get_relative_position_NED_home()
		if n and n.xy then dist2d = n:xy():length() end
	end

	if dist2d >= cyl_R then
		G._alt_gate_state = true
		return true
	end

	G._alt_gate_state = (G._alt_gate_state == true)
	local hagl_now, _ = self:_robust_hagl_and_amsl(loc, home, ned)
	local have_hagl = (hagl_now ~= nil)
	local use_hagl = (terrain and terrain:enabled() and have_hagl)

	local enter_thr_hagl = 40
	local exit_thr_hagl  = 25
	local alt_rel_home = -(ned and ned:z() or 0)
	local enter_thr_rel = (initial_min ~= nil) and (initial_min - 2) or 20
	local exit_thr_rel  = (initial_min ~= nil) and (initial_min - 10) or 10

	local want_on
	if use_hagl then
		want_on = G._alt_gate_state and (hagl_now > exit_thr_hagl) or (hagl_now > enter_thr_hagl)
	else
		want_on = G._alt_gate_state and (alt_rel_home >= exit_thr_rel) or (alt_rel_home >= enter_thr_rel)
	end
	G._alt_gate_state = want_on
	return G._alt_gate_state
end

-- Returns the nav mode to resume after THERMAL / terrain evasion.
function SoarNav:_get_exit_nav_mode()
	local G = self.State
	local C = self.Const
	local m = G.nav_mode_before_thermal
	if m == C.mode_fbwb or m == C.mode_cruise then
		return m
	end
	return C.mode_cruise
end

-- Builds TE environment struct with all derived params for planner (avoids recalculation).
function SoarNav:_te_build_env(now_ms, loc, target_loc)
	local G = self.State
	local Cache = self.Cache
	local TC = self.TE_Config
	local te = G.TE
	local vned = ahrs:get_velocity_NED()
	local head = ahrs:get_yaw_rad()
	local home = ahrs:get_home()
	local ned = ahrs:get_relative_position_NED_home()
	if not vned or not head or not home or not ned or not loc or not target_loc then
		return nil
	end
	local hagl_now, current_amsl = self:_robust_hagl_and_amsl(loc, home, ned)
	if not hagl_now or not current_amsl then
		return nil
	end
	local gs_xy = math.sqrt(vned:x() * vned:x() + vned:y() * vned:y())
	local as = ahrs.airspeed_estimate and ahrs:airspeed_estimate() or nil
	local speed_ref = math.max(TC.SPEED_MIN,
		(as and as > 3) and as
			or ((gs_xy and gs_xy > 1) and gs_xy
				or (Cache.airspeed_cruise or 15)))

	local tp = self:_te_compute_params(speed_ref)

	te.gs_ema = te.gs_ema and (te.gs_ema + (gs_xy - te.gs_ema) * tp.alpha_gs) or gs_xy
	local vario_raw = -vned:z()
	te.vario_ema_mps = te.vario_ema_mps and (te.vario_ema_mps + (vario_raw - te.vario_ema_mps) * tp.alpha_vario) or vario_raw
	local vario_mps = te.vario_ema_mps or 0

	local T_look = Cache.te_look_s or 10
	local buf_min = tp.buffer_min
	local T_eff = math.max(TC.T_EGRESS_BASE, T_look * tp.k_c)
	local T_egress = math.min(T_eff, tp.t_egress)

	local roll_lim_deg = Cache.roll_limit or 30
	local roll_lim = math.rad(roll_lim_deg)
	local r_min = math.max(TC.FLOOR_LEVER, (speed_ref * speed_ref) / (9.81 * math.tan(roll_lim)))

	local buffer_base = math.max(buf_min, math.min(hagl_now * TC.PCT_BUFFER_HI, tp.buffer_max))
	local buffer_hi = buffer_base * TC.PCT_BUFFER_HI
	local buffer_lo = 0
	local vT = speed_ref * T_eff
	local h_gate = math.max(buf_min * TC.H_GATE_BUF_MULT, vT * TC.H_GATE_T_MULT)
	local lateral_h_gate = math.max(buf_min * TC.LAT_H_GATE_BUF_MULT, vT * TC.LAT_H_GATE_T_MULT)

	local lever_T_factors = TC.LEVER_T_FACTORS
	local lever_T = math.max(lever_T_factors[1], math.min(T_egress * lever_T_factors[2],
		math.max(lever_T_factors[1] * r_min / speed_ref,
			math.max((Cache.loiter_radius or 60) * lever_T_factors[2] / speed_ref,
				(Cache.wp_radius or 50) * lever_T_factors[3] / speed_ref))))
	local lever_m = math.min(speed_ref * lever_T, tp.lever_max)

	local wind = self:get_wind_vector()
	local wind_xy = nil
	if wind and wind:length() > 0.5 then
		wind_xy = wind:xy()
	end
	return {
		now_ms = now_ms,
		loc = loc,
		target_loc = target_loc,
		hagl = hagl_now,
		amsl = current_amsl,
		speed_ref = speed_ref,
		roll_limit = roll_lim_deg,
		r_min = r_min,
		tgt_brg_deg = math.deg(loc:get_bearing(target_loc)),
		current_heading_deg = (math.deg(head) + 360) % 360,
		current_heading_rad = head,
		T_look = T_look,
		T_eff = T_eff,
		T_egress = T_egress,
		buffer_base = buffer_base,
		buffer_min_abs = buf_min,
		buffer_hi = buffer_hi,
		buffer_lo = buffer_lo,
		buffer_front = buffer_base,
		buffer_side = buffer_base * 1.1,
		h_gate = h_gate,
		lateral_h_gate = lateral_h_gate,
		lever_m = lever_m,
		vario_mps = vario_mps,
		sink_best = self:sink_best_now(),
		roll_abs = math.abs(math.deg(ahrs:get_roll_rad() or 0)),
		wind_xy = wind_xy,
		tp = tp,
	}
end

-- Evaluates a heading using sampling (balanced).
function SoarNav:_te_eval_heading(env, hdg_deg, use_arc)
	local TC = self.TE_Config
	local dist = env.lever_m
	local spacing = TC.HEADING_SPACING
	local worst_agl = 1e9
	local data_ok = true
	local start_agl = nil
	local end_agl = nil
	for i = 1, TC.NUM_HEADING_SAMPLES do
		local agl, ok = self:_te_get_agl_at_point(env, hdg_deg, dist * spacing[i])
		if i == 1 and ok then
			start_agl = agl
		end
		if i == TC.NUM_HEADING_SAMPLES and ok then
			end_agl = agl
		end
		if ok and agl < worst_agl then
			worst_agl = agl
		elseif not ok then
			data_ok = false
		end
	end
	if worst_agl == 1e9 then
		return nil, false, end_agl, start_agl
	end
	if not end_agl then
		end_agl = worst_agl
	end
	if not start_agl then
		start_agl = worst_agl
	end
	return worst_agl, data_ok, end_agl, start_agl
end

-- Simple AGL evaluation at a point (for lateral threat detection).
function SoarNav:_te_get_agl_at_point(env, brg_deg, dist_m)
	local G = self.State
	local tp = env.tp
	local p = env.loc:copy()
	p:offset_bearing(brg_deg, dist_m)
	if G.use_polygon_area and not self:in_flight_area(p) then
		p = self:clamp_inside_polygon(p, tp.sample_offset * 0.5)
	end
	local ter = terrain:height_amsl(p, true)
	if not ter then
		return nil, false
	end
	local t_flight = dist_m / math.max(env.speed_ref, 1.0)
	local sink = env.sink_best
	local v_eff = env.vario_mps
	if v_eff > 0 then v_eff = 0 end
	local est_amsl = env.amsl + v_eff * t_flight - sink * t_flight
	local agl = est_amsl - (ter - (G.TE.tevas_terrain_offset_m or 0))
	return agl, true
end

-- Generates candidate headings as a fan around preferred escape direction.
function SoarNav:_te_generate_candidates(env, threat_type, threat_brg, prefer_right, deficit_ratio)
	local TC = self.TE_Config
	local tp = env.tp
	local cur_hdg = env.current_heading_deg
	local tgt_brg = env.tgt_brg_deg
	local fan_half = math.min(tp.fan_half_max, math.max(tp.fan_half_min, env.roll_limit * 0.8))
	local back_brg = (threat_brg + 180) % 360
	local base_escape
	if threat_type == "front" then
		base_escape = back_brg
	elseif threat_type == "left" then
		local delta = math.max(tp.fan_half_min * 1.5, math.min(tp.fan_half_max * 1.5, fan_half + tp.angle_step))
		base_escape = (cur_hdg + delta) % 360
	else
		local delta = math.max(tp.fan_half_min * 1.5, math.min(tp.fan_half_max * 1.5, fan_half + tp.angle_step))
		base_escape = (cur_hdg - delta + 360) % 360
	end
	local cand = {}
	cand[1] = base_escape
	if threat_type == "front" then
		cand[2] = (base_escape + TC.THREAT_FRONT_OFFSETS[1]) % 360
		cand[3] = (base_escape + 360 - TC.THREAT_FRONT_OFFSETS[1]) % 360
	else
		cand[2] = (base_escape + TC.THREAT_SIDE_OFFSETS[1]) % 360
		cand[3] = (base_escape - TC.THREAT_SIDE_OFFSETS[1] + 360) % 360
	end
	if threat_type ~= "front" then
		cand[#cand + 1] = back_brg
	end
	local step = tp.angle_step
	local turn_max = math.max(tp.turn_max_base, math.min(tp.turn_max_cap, tp.turn_max_base + (tp.turn_max_cap - tp.turn_max_base) * math.min(1.0, 0.4 + deficit_ratio * 0.6)))
	local dir = prefer_right and 1 or -1
	for angle = step, turn_max, step do
		local hdg = (tgt_brg + dir * angle) % 360
		local dominated = false
		for j = 1, #cand do
			if math.abs(((hdg - cand[j] + 540) % 360) - 180) < tp.duplicate_thr then
				dominated = true
				break
			end
		end
		if not dominated then
			cand[#cand + 1] = hdg
		end
	end
	return cand, back_brg
end

-- Pure TE planner: takes env, returns decision struct without touching globals.
function SoarNav:_te_plan(env)
	local G = self.State
	local TE_S = G.TE_STATE
	local te = G.TE
	local Cache = self.Cache
	local TC = self.TE_Config
	local tp = env.tp
	local function sdelta(a, b)
		return (a - b + 540) % 360 - 180
	end
	local fan_half = math.min(tp.fan_half_max, math.max(tp.fan_half_min, env.roll_limit * 0.8))
	local fan_angles = {0, -fan_half, fan_half}
	local front_dist_fracs = TC.FRONT_DIST_FRACS
	local dist1 = env.speed_ref * env.T_eff * front_dist_fracs[1]
	local dist2 = env.speed_ref * env.T_eff * front_dist_fracs[2]
	local dist3 = env.speed_ref * env.T_eff * front_dist_fracs[3]
	local front_worst_agl = 1e9
	local front_worst_brg = env.tgt_brg_deg
	local front_ok = true
	for i = 1, #fan_angles do
		local brg = (env.tgt_brg_deg + fan_angles[i] + 360) % 360
		local a1, ok1 = self:_te_get_agl_at_point(env, brg, dist1)
		local a2, ok2 = self:_te_get_agl_at_point(env, brg, dist2)
		local a3, ok3 = self:_te_get_agl_at_point(env, brg, dist3)
		if ok1 and a1 < front_worst_agl then front_worst_agl, front_worst_brg = a1, brg end
		if ok2 and a2 < front_worst_agl then front_worst_agl, front_worst_brg = a2, brg end
		if ok3 and a3 < front_worst_agl then front_worst_agl, front_worst_brg = a3, brg end
		if not ok1 and not ok2 and not ok3 then front_ok = false end
	end
	local margin_front = front_worst_agl ~= 1e9 and (front_worst_agl - env.buffer_front) or 1e9
local lateral_enabled = env.hagl < env.lateral_h_gate
	local margin_right = 1e9
	local margin_left = 1e9
	local right_min_agl = 1e9
	local left_min_agl = 1e9
	local right_brg = (env.current_heading_deg + 90) % 360
	local left_brg = (env.current_heading_deg + 270) % 360
	local lateral_ok = true
	if lateral_enabled then
		local side_r_fracs = TC.SIDE_R_FRACS
		local side_t_fracs = TC.SIDE_T_FRACS
		local side_dist1 = math.max(env.r_min * side_r_fracs[1], env.speed_ref * env.T_eff * side_t_fracs[1])
		local side_dist2 = math.max(env.r_min * side_r_fracs[2], env.speed_ref * env.T_eff * side_t_fracs[2])
		local a_r1, ok_r1 = self:_te_get_agl_at_point(env, right_brg, side_dist1)
		local a_r2, ok_r2 = self:_te_get_agl_at_point(env, right_brg, side_dist2)
		if ok_r1 and a_r1 < right_min_agl then right_min_agl = a_r1 end
		if ok_r2 and a_r2 < right_min_agl then right_min_agl = a_r2 end
		local a_l1, ok_l1 = self:_te_get_agl_at_point(env, left_brg, side_dist1)
		local a_l2, ok_l2 = self:_te_get_agl_at_point(env, left_brg, side_dist2)
		if ok_l1 and a_l1 < left_min_agl then left_min_agl = a_l1 end
		if ok_l2 and a_l2 < left_min_agl then left_min_agl = a_l2 end
		if right_min_agl ~= 1e9 then margin_right = right_min_agl - env.buffer_side end
		if left_min_agl ~= 1e9 then margin_left = left_min_agl - env.buffer_side end
		if not ok_r1 and not ok_r2 and not ok_l1 and not ok_l2 then lateral_ok = false end
	end
	local num_bad = 0
	if margin_front < 0 then num_bad = num_bad + 1 end
	if margin_left < 0 then num_bad = num_bad + 1 end
	if margin_right < 0 then num_bad = num_bad + 1 end
	local is_bowl = (num_bad >= 2 and env.hagl < env.buffer_hi)
	local threat_type = "front"
	local threat_agl = front_worst_agl
	local threat_brg = front_worst_brg
	local buffer_m = env.buffer_front
	local best_margin = margin_front
	if margin_left < best_margin then
		best_margin = margin_left
		threat_type = "left"
		threat_agl = left_min_agl
		threat_brg = left_brg
		buffer_m = env.buffer_side
	end
	if margin_right < best_margin then
		best_margin = margin_right
		threat_type = "right"
		threat_agl = right_min_agl
		threat_brg = right_brg
		buffer_m = env.buffer_side
	end
	if is_bowl then
		buffer_m = buffer_m * 0.7
	end
	local margin = threat_agl ~= 1e9 and (threat_agl - buffer_m) or 1e9
	local deficit = math.max(0, buffer_m - threat_agl)
	local deficit_ratio = math.min(2.0, deficit / math.max(1.0, buffer_m))
	local zone
	if margin >= env.buffer_hi then
		zone = 0
	elseif margin >= env.buffer_lo then
		zone = 1
	else
		zone = 2
	end
	local decision = {
		zone = zone,
		margin = margin,
		deficit = deficit,
		deficit_ratio = deficit_ratio,
		threat_agl = threat_agl,
		threat_type = threat_type,
		threat_brg = threat_brg,
		buffer_m = buffer_m,
		front_margin = margin_front,
		left_margin = margin_left,
		right_margin = margin_right,
		best_margin = best_margin,
		p_gain_scaler = 1.0,
		improvement = 0,
		dir_new = 0,
		dir_old = 0,
		data_fail = not front_ok and not lateral_ok,
	}
	if threat_agl == 1e9 then
		decision.state = TE_S.IDLE
		return decision
	end
	if not te.evasion_active and env.hagl > env.h_gate then
		decision.state = TE_S.IDLE
		decision.p_gain_scaler = 1.0
		return decision
	end
	if not te.evasion_active then
		local safe_margin = math.max(TC.PCT_SAFE_MARGIN * env.buffer_min_abs, TC.PCT_SAFE_MARGIN * buffer_m)
		if margin >= safe_margin then
			decision.state = TE_S.IDLE
			decision.p_gain_scaler = 1.0
			return decision
		end
	end
	local speed_deficit_factor = math.max(0.8, math.min(env.speed_ref / (Cache.airspeed_cruise or 15), 1.3))
	decision.p_gain_scaler = 1.0 + math.min(tp.p_incr_max, deficit_ratio * tp.p_deficit_mult) * speed_deficit_factor
	if deficit <= 0 and te.evasion_active then
		decision.state = TE_S.IDLE
		decision.p_gain_scaler = 1.0
		return decision
	end
	if zone == 1 and not te.evasion_active then
		decision.state = TE_S.MONITORING
		return decision
	end
	local delta_to_threat = sdelta(env.current_heading_deg, threat_brg)
    local prefer_right = (delta_to_threat <= 0)
	local candidates, back_brg = self:_te_generate_candidates(env, threat_type, threat_brg, prefer_right, deficit_ratio)
	if te.evasion_active and te.last_safe_heading then
		candidates[#candidates + 1] = te.last_safe_heading
	else
		candidates[#candidates + 1] = env.current_heading_deg
	end
	local use_arc = math.abs(sdelta(env.current_heading_deg, env.tgt_brg_deg)) > tp.duplicate_thr * 2.5
	local function score_hdg(hdg)
		local agl_min, ok, agl_end = self:_te_eval_heading(env, hdg, use_arc)
		if not ok or not agl_min then
			return -1e9, -1e9, false
		end
		local base = agl_min
		local trend = 0
		if agl_end then
			trend = agl_end - agl_min
		end
		if trend > 0 then
			local k_trend = tp.trend_gain or 0.3
			if is_bowl then
				k_trend = k_trend * 1.5
			end
			local trend_effect = math.min(trend, buffer_m)
			base = base + k_trend * trend_effect
		end
		local d_threat = math.abs(sdelta(hdg, threat_brg))
		local d_tgt = math.abs(sdelta(hdg, env.tgt_brg_deg))
		local d_cur = math.abs(sdelta(hdg, env.current_heading_deg))
		local pen_terrain = 0
		if d_threat < TC.PENALTY_ANGLE_THR then
			pen_terrain = (TC.PENALTY_ANGLE_THR - d_threat) * TC.PENALTY_MULT
		end
		local pen_factor = math.max(0.2, 1.0 - 0.6 * math.min(1.0, deficit_ratio))
		local k_tgt = deficit > 0 and 0.01 or 0.03
		local k_cur = 0.01
		local score = base - pen_terrain - d_tgt * k_tgt * pen_factor - d_cur * k_cur * pen_factor
		return score, agl_min, ok
	end
	local best = { score = -1e9, hdg = env.current_heading_deg, agl = threat_agl, ok = false }
	local fail_count = 0
	for i = 1, #candidates do
		local sc, agl, ok = score_hdg(candidates[i])
		if not ok then fail_count = fail_count + 1 end
		if sc > best.score then
			best.score = sc
			best.hdg = candidates[i]
			best.agl = agl
			best.ok = ok
		end
	end
	if best.agl < buffer_m then
		local uturn_hdg = (env.current_heading_deg + 180) % 360
		local uturn_agl, uturn_ok = self:_te_eval_heading(env, uturn_hdg, false)
		if uturn_ok and uturn_agl and uturn_agl > best.agl then
			best.hdg = uturn_hdg
			best.agl = uturn_agl
			best.ok = uturn_ok
			best.score = uturn_agl
		end
	end
	decision.data_fail = (fail_count > #candidates * 0.5)
	local d_front = math.abs(sdelta(best.hdg, threat_brg))
	if d_front < TC.PENALTY_ANGLE_THR * 0.875 then
		local sc_back, agl_back, ok_back = score_hdg(back_brg)
		if ok_back and agl_back > buffer_m and sc_back > best.score then
			best.hdg = back_brg
			best.agl = agl_back
			best.ok = ok_back
			best.score = sc_back
		end
	end
	if not best.ok or best.agl < 0 then
		decision.state = TE_S.IDLE
		decision.data_fail = true
		decision.p_gain_scaler = 1.0
		return decision
	end
	local curr_hdg = te.last_safe_heading or env.current_heading_deg
	local curr_score, curr_agl, curr_ok = score_hdg(curr_hdg)
	decision.improvement = best.score - curr_score
	decision.dir_new = sdelta(best.hdg, env.tgt_brg_deg) >= 0 and 1 or -1
	decision.dir_old = sdelta(curr_hdg, env.tgt_brg_deg) >= 0 and 1 or -1
	local curr_is_good = curr_ok and curr_agl > buffer_m
	local hysteresis_scale
	if curr_is_good then
		hysteresis_scale = 1.0
	else
		hysteresis_scale = math.max(0.5, 1.0 - 0.4 * math.min(1.0, deficit_ratio))
	end
	local min_gain = math.max(tp.min_gain, TC.PCT_MIN_GAIN * buffer_m) * hysteresis_scale
	local min_gain_side = math.max(tp.min_gain_side, TC.PCT_MIN_GAIN_SIDE * buffer_m) * hysteresis_scale
	local since_switch = self:safe_time_diff(env.now_ms, te.last_side_switch_ms or 0)
	local gate_ms = tp.gate_ms
	if env.roll_abs > TC.ROLL_THR_GATE then
		gate_ms = gate_ms + tp.gate_roll_bonus
	end
	local side_cooldown = tp.side_cooldown
	local since_set = self:safe_time_diff(env.now_ms, te.last_set_target_ms or 0)
	local is_emergency = deficit_ratio >= TC.EMERGENCY_DEFICIT and best.agl > (curr_agl + TC.EMERGENCY_AGL_GAIN)
	if not is_emergency then
		if since_set < gate_ms and decision.improvement < min_gain then
			decision.state = TE_S.EVADING
			decision.new_heading = curr_hdg
			decision.best_agl = curr_agl
			return decision
		end
		if decision.dir_new ~= decision.dir_old and since_switch < side_cooldown and decision.improvement < min_gain_side then
			decision.state = TE_S.EVADING
			decision.new_heading = curr_hdg
			decision.best_agl = curr_agl
			return decision
		end
	end
	decision.state = TE_S.EVADING
	decision.new_heading = best.hdg
	decision.best_agl = best.agl
	local margin_best = math.max(0, best.agl - buffer_m)
	local T_leg = env.lever_m / math.max(env.speed_ref, 5.0)
	local T_hold = math.max(TC.T_HOLD_BASE * env.T_egress, T_leg * 1.2)
	if margin_best > buffer_m * TC.PCT_SAFE_MARGIN then
		T_hold = T_hold + math.min(TC.T_HOLD_BASE * env.T_egress, (margin_best / buffer_m) * 0.8 * env.T_egress)
	end
	if env.roll_abs > TC.ROLL_THR_HOLD then
		T_hold = T_hold + 0.1 * env.T_egress
	end
	T_hold = math.min(TC.T_HOLD_MAX_MULT * env.T_egress, T_hold)
	decision.hold_ms = T_hold * 1000
	return decision
end

-- Applies TE decision to globals and sets target heading.
function SoarNav:_te_apply_decision(env, decision)
	local G = self.State
	local TE_S = G.TE_STATE
	local te = G.TE
	local Cache = self.Cache
	local C = self.Const
	local TC = self.TE_Config
	local function sdelta(a, b)
		return (a - b + 540) % 360 - 180
	end
	te.margin = decision.margin
	te.deficit = decision.deficit
	te.deficit_ratio = decision.deficit_ratio
	te.T_eff = env.T_eff
	te.T_egress = env.T_egress
	te.speed_ref = env.speed_ref
	te.zone = decision.zone
	te.improvement = decision.improvement
	te.dir_new = decision.dir_new
	te.dir_old = decision.dir_old
	te.p_gain_scaler = decision.p_gain_scaler
	te.state = decision.state
	te.threat_type = decision.threat_type
	if decision.data_fail then
		te.data_fail_count = (te.data_fail_count or 0) + 1
		if te.data_fail_count > TC.DATA_FAIL_LIMIT then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.TERRAIN_DATA_UNRELIABLE)
		end
	else
		te.data_fail_count = 0
	end
	local function ms(k) return math.floor(k * env.T_eff * 1000) end
	if decision.state == TE_S.IDLE then
		te.evasion_active = false
		te.scan_phase = nil
		te.target_heading = nil
		te.next_check_ms = env.now_ms + ms(0.1)
		return false
	end
	if decision.state == TE_S.MONITORING then
		te.evasion_active = false
		te.scan_phase = nil
		te.target_heading = nil
		te.next_check_ms = env.now_ms + ms(0.08)
		return false
	end
	local function ensure_exit_thermal()
		local m = vehicle:get_mode()
		if m == C.mode_thermal then
			vehicle:set_mode(self:_get_exit_nav_mode())
		end
	end
	ensure_exit_thermal()
	if decision.deficit > 0 then
		ensure_exit_thermal()
	end
	te.evasion_active = true
	te.scan_phase = "EVADING"
	G.g_waypoint_source_info = G.SoarNavWaypointSources.TERRAIN_EVASION
	te.target_heading = decision.new_heading
	te.target_lever_m = env.lever_m
	te.last_set_target_ms = env.now_ms
	te.last_safe_heading = decision.new_heading
	te.last_evasion_dir = sdelta(decision.new_heading, env.tgt_brg_deg)
	if decision.dir_new ~= decision.dir_old then
		te.last_side_switch_ms = env.now_ms
	end
	local d = te.last_evasion_dir or 0
    te.turn_arrow = d > 0 and "→" or (d < 0 and "←" or "")
    local hold_ms = decision.hold_ms or 0
    if hold_ms <= 0 then
        hold_ms = env.T_egress * 1000
    end
    te.hold_until_ms = env.now_ms + math.floor(hold_ms)
    te.next_check_ms = env.now_ms + ms(0.08)
	if (Cache.log_lvl or 0) >= 2 then
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 2, self.Msg.MSG_ID.TERRAIN_AVOID_DEBUG,
			decision.threat_agl or -999, decision.best_agl or -999, decision.buffer_m or 0,
			math.floor(te.last_evasion_dir or 0), env.lever_m or 0)
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 2, self.Msg.MSG_ID.TERRAIN_AVOID_DEBUG_EXT,
			decision.margin or 0, decision.deficit or 0, decision.deficit_ratio or 0,
			env.T_eff or 0, decision.state or 0, decision.zone or 0)
	end
	return false
end

-- Main terrain evasion state machine (wrapper for planner).
function SoarNav:update_terrain_evasion_sm(now_ms, loc, target_loc)
	local G = self.State
	local TE_S = G.TE_STATE
	G.TE = G.TE or {}
	local te = G.TE
	local Cache = self.Cache
	local TC = self.TE_Config
	local t = now_ms or millis()
	local T_look = Cache.te_look_s or 10
	local function set_arrow()
		if te.last_evasion_dir then
			te.turn_arrow = te.last_evasion_dir > 0 and "→" or (te.last_evasion_dir < 0 and "←" or "")
		end
	end
	if (te.next_check_ms or 0) > t then
		set_arrow()
		return false
	end
	if Cache.dyn_soalt ~= 3 or not terrain or not terrain:enabled() then
		set_arrow()
		te.p_gain_scaler = 1.0
		te.scan_phase = nil
		te.evasion_active = false
		te.state = TE_S.IDLE
		te.target_heading = nil
		te.next_check_ms = t + math.floor(T_look * 800)
		return false
	end
	self:update_tevas_terrain_offset(loc, nil)
	local env = self:_te_build_env(now_ms, loc, target_loc)
	if not env then
		te.next_check_ms = t + math.floor(T_look * 800)
		return false
	end
	local tp = env.tp
	if te.evasion_active and te.last_safe_heading then
		local ev_agl, ev_ok = self:_te_eval_heading(env, te.last_safe_heading, false)
		if ev_ok and ev_agl >= env.buffer_base then
			if t < (te.hold_until_ms or 0) then
				te.state = TE_S.HOLD
				te.next_check_ms = t + math.max(tp.gate_ms * 0.25, (env.roll_abs > TC.ROLL_THR_GATE and tp.gate_ms * 0.4 or tp.gate_ms * 0.6))
				set_arrow()
				return false
			end
		end
	end
	local decision = self:_te_plan(env)
	return self:_te_apply_decision(env, decision)
end

-- Checks the status of the current waypoint (reached, timeout, or reroute trigger).
function SoarNav:check_waypoint_status(loc, current_time_ms)
	local G = self.State
	local Cache = self.Cache
	G.distance_to_wp = loc:get_distance(G.target_loc)

	if
		(G.TE.hold_until_ms <= current_time_ms)
		and G.reroute_check_armed
		and (G.initial_distance_to_wp > 0)
		and (G.distance_to_wp <= (G.initial_distance_to_wp / 2))
	then
		G.reroute_check_armed = false
		if
			G.g_waypoint_source_info ~= G.SoarNavWaypointSources.TERRAIN_EVASION
			and G.g_waypoint_source_info ~= G.SoarNavWaypointSources.TE_EGRESS
			and G.g_waypoint_source_info ~= G.SoarNavWaypointSources.REENGAGE_ENTRY
			and G.g_waypoint_source_info ~= G.SoarNavWaypointSources.REENGAGE_FLYOUT
			and G.g_waypoint_source_info ~= G.SoarNavWaypointSources.REROUTE
			and self:check_tactical_reroute_conditions()
		then
			return "REROUTE"
		end
	end

	local time_on_current_wp = self:safe_time_diff(current_time_ms, G.waypoint_start_time_ms)
	if time_on_current_wp > self:get_wp_timeout() then
		return "TIMEOUT"
	end

	if G.distance_to_wp < Cache.wp_radius then
		return "REACHED"
	end

	return "NAVIGATING"
end

-- Detects if the aircraft is stuck and initiates a repositioning maneuver.
function SoarNav:manage_anti_stuck(loc, current_time_ms)
	local G = self.State
	local C = self.Const
	local Cache = self.Cache

	if G.is_repositioning then
		return false
	end

	if G.g_waypoint_source_info == G.SoarNavWaypointSources.TE_EGRESS then
		return false
	end

	if G.TE and G.TE.evasion_active then
		return false
	end

	if G.TE and (G.TE.hold_until_ms or 0) > current_time_ms then
		return false
	end

	if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_ENTRY then
		return false
	end

	if G.reengage_hold_active and (G.reengage_hold_until_ms or 0) > current_time_ms then
		return false
	end

	local time_on_current_wp = self:safe_time_diff(current_time_ms, G.waypoint_start_time_ms)
	if time_on_current_wp < (Cache.stuck_time * 1000) then
		return false
	end

	if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_FLYOUT then
		if not G.target_loc then
			G.thermal_to_retry = nil
			G.reengage_final_target = nil
			G.force_new_search = true
			return true
		end
	end

	local progress_check_timeout =
		self:safe_time_diff(current_time_ms, G.last_progress_check_ms) > C.STUCK_PROGRESS_CHECK_INTERVAL_MS
	if not G.last_progress_check_ms or progress_check_timeout then
		if G.distance_at_last_check > 0 and G.loc_at_last_check then
			local progress_made = G.distance_at_last_check - G.distance_to_wp
			local distance_traveled = loc:get_distance(G.loc_at_last_check)
			local efficiency_ratio = 0
			if distance_traveled > 5 then
				efficiency_ratio = progress_made / distance_traveled
			end
			if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_FLYOUT then
				if distance_traveled < 30 then
					efficiency_ratio = 1
					progress_made = C.STUCK_MIN_PROGRESS_M
				end
			end
			local min_eff = Cache.stuck_eff
			local min_prog = C.STUCK_MIN_PROGRESS_M
			if progress_made < min_prog or efficiency_ratio < min_eff then
				G.stuck_counter = G.stuck_counter + 1
				local wind_vec_info = self:get_wind_vector()
				if wind_vec_info then
					local wind_bearing_info = self:wind_vector_to_bearing_deg(wind_vec_info)
					self:log_gcs(
						self.Msg.MAV_SEVERITY.WARNING,
						1,
						self.Msg.MSG_ID.NO_PROG_WARN,
						efficiency_ratio,
						wind_vec_info:length(),
						wind_bearing_info
					)
				end
			else
				G.stuck_counter = 0
			end
		end
		G.last_progress_check_ms = current_time_ms
		G.distance_at_last_check = G.distance_to_wp
		G.loc_at_last_check = loc:copy()
	end

	local wind_vec = self:get_wind_vector()
	local wind_speed = wind_vec and wind_vec:length() or 0
	local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 3)))

	if G.stuck_counter >= adaptive_stuck_limit then
		if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_FLYOUT and G.reengage_final_target then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.NAV_TARGET_SIMPLE, "Stuck FlyOut -> Entry", 0, "Skip", 0)
			self:set_target_safely(G.reengage_final_target)
			G.reengage_final_target = nil
			G.g_waypoint_source_info = G.SoarNavWaypointSources.REENGAGE_ENTRY
			local reengage_dwell_ms = (self.Cache.reeng_dwell or 7) * 1000
			G.reengage_hold_until_ms = millis():toint() + reengage_dwell_ms
			G.reengage_hold_active = true
			G.reengage_hold_last_msg_ms = millis():toint()
			self:_finalize_waypoint_selection()
			G.stuck_counter = 0
			return true
		end

		G.original_target_loc = G.target_loc
		G.is_repositioning = true
		if wind_vec and loc then
			local repo_attempts = G.stuck_counter
			local base_dist = math.max(150, math.min(700, wind_speed * 60))
			local repo_dist_m = base_dist + (repo_attempts * C.STUCK_DISTANCE_INCREMENT_M)
			local upwind_offset_deg = (repo_attempts * 45) % 360
			local repo_loc = loc:copy()
			repo_loc:offset_bearing((self:wind_vector_to_bearing_deg(wind_vec) + upwind_offset_deg) % 360, repo_dist_m)
			if self:in_flight_area(repo_loc) then
				self:log_gcs(
					self.Msg.MAV_SEVERITY.WARNING,
					1,
					self.Msg.MSG_ID.STICK_CMD_STUCK,
					repo_dist_m,
					upwind_offset_deg
				)
				self:set_target_safely(repo_loc)
				self:_finalize_waypoint_selection()
			else
				G.target_loc = nil
			end
			return true
		else
			G.is_repositioning = false
			G.target_loc = nil
			return true
		end
	end

	return false
end

-- Calculates and applies the roll command to steer towards the target waypoint or HDG.
function SoarNav:update_navigation_controller(loc, dt_s)
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	local function wrap360(x)
		return (x % 360 + 360) % 360
	end
	local function wrap180(x)
		return (x + 540) % 360 - 180
	end

	local target_heading_deg
	if G.TE.evasion_active and G.TE.target_heading then
		target_heading_deg = G.TE.target_heading
	else
		target_heading_deg = self:get_wind_corrected_heading_deg(loc, G.target_loc)
	end

	local current_heading_deg = wrap360(math.deg(ahrs:get_yaw_rad()))
	local heading_error = wrap180(target_heading_deg - current_heading_deg)

	local tau = (C.HEADING_ERR_TAU_S or 0.35)
	G.heading_err_f = G.heading_err_f or heading_error
	local alpha = dt_s / (tau + dt_s)
	G.heading_err_f = wrap180(G.heading_err_f + alpha * wrap180(heading_error - G.heading_err_f))
	local e_f = G.heading_err_f

	local HEADING_DB = (C.HEADING_DB_DEG or 0.7)
	local ae = math.abs(e_f)
	local e_for_p
	if ae < HEADING_DB then
		local k = ae / HEADING_DB
		e_for_p = e_f * k * k
	else
		e_for_p = (ae - HEADING_DB) * (e_f > 0 and 1 or -1)
	end

	local error_derivative = 0
	if dt_s > 0.01 then
		local last = G.last_heading_error_f or e_f
		error_derivative = wrap180(e_f - last) / dt_s
	end
	G.last_heading_error_f = e_f

	local D_CLAMP = (C.HEADING_D_CLAMP_DEGPS or 40)
	if error_derivative > D_CLAMP then error_derivative = D_CLAMP end
	if error_derivative < -D_CLAMP then error_derivative = -D_CLAMP end

	local p_gain_scaler = (G.TE.evasion_active and G.TE.p_gain_scaler) or 1.0
	local effective_p_gain = (Cache.nav_p or 0.6) * p_gain_scaler

	local p_term = e_for_p * effective_p_gain
	local d_term = error_derivative * (Cache.nav_d or 0)
	local raw_pd_command = p_term + d_term

	if raw_pd_command ~= raw_pd_command then
		raw_pd_command = 0
	end

	if C.HUMANIZE_PILOT == 1 then
		local now_ms_raw = millis()
		local now_ms = (type(now_ms_raw) == "number") and now_ms_raw
			or ((now_ms_raw and now_ms_raw.toint) and now_ms_raw:toint() or 0)

		if not G.human_seeded then
			local seed = now_ms
			if type(seed) ~= "number" or seed <= 0 then
				seed = 1
			end
			seed = (seed % 2147483647) + 1
			math.randomseed(seed)
			math.random(); math.random(); math.random()
			G.human_seeded = true
		end

		G.human_next_ms = G.human_next_ms or 0
		G.human_bias_deg = G.human_bias_deg or 0
		G.human_hold_cmd = G.human_hold_cmd or 0
		local DWELL_MIN = (C.HUMAN_DWELL_MIN_MS or 180)
		local DWELL_MAX = (C.HUMAN_DWELL_MAX_MS or 420)
		local GAIN_JIT = (C.HUMAN_GAIN_JITTER or 0.18)
		local BIAS_RW = (C.HUMAN_BIAS_RW_DEG_PER_S or 0.25)
		local BIAS_MAX = (C.HUMAN_BIAS_MAX_DEG or 1.5)
		local MIN_BANK = (C.HUMAN_MIN_BANK_DEG or 0.35)
		local step = (math.random() * 2 - 1) * BIAS_RW * dt_s
		G.human_bias_deg = math.max(-BIAS_MAX, math.min(BIAS_MAX, G.human_bias_deg + step))
		if now_ms < G.human_next_ms then
			raw_pd_command = G.human_hold_cmd
		else
			local jitter = 1.0 + (math.random() * 2 - 1) * GAIN_JIT
			raw_pd_command = raw_pd_command * jitter + (G.human_bias_deg * effective_p_gain * 0.35)
			if math.abs(raw_pd_command) < MIN_BANK then
				raw_pd_command = 0
			end
			G.human_hold_cmd = raw_pd_command
			G.human_next_ms = now_ms + math.random(DWELL_MIN, DWELL_MAX)
		end
	end

	local smooth_factor = C.ROLL_SMOOTHING_FACTOR
	local abs_smooth_factor = math.abs(smooth_factor)
	local smoothed_command
	if smooth_factor > 0 then
		if math.abs(raw_pd_command) > math.abs(G.last_commanded_roll_deg) then
			smoothed_command = (abs_smooth_factor * raw_pd_command)
				+ ((1.0 - abs_smooth_factor) * G.last_commanded_roll_deg)
		else
			smoothed_command = raw_pd_command
		end
	else
		smoothed_command = (abs_smooth_factor * raw_pd_command)
			+ ((1.0 - abs_smooth_factor) * G.last_commanded_roll_deg)
	end

	local desired_roll_deg = math.max(-Cache.roll_limit, math.min(Cache.roll_limit, smoothed_command))

	local prev_roll = G.last_commanded_roll_deg or 0
	local MAX_ROLL_RATE = (C.MAX_ROLL_RATE_DEGPS or 40)
	local max_step = MAX_ROLL_RATE * dt_s
	if max_step > 0 then
		if desired_roll_deg > prev_roll + max_step then
			desired_roll_deg = prev_roll + max_step
		elseif desired_roll_deg < prev_roll - max_step then
			desired_roll_deg = prev_roll - max_step
		end
	end
	G.last_commanded_roll_deg = desired_roll_deg

	local sys_roll_limit = Cache.sys_roll_limit

	if not sys_roll_limit or sys_roll_limit == 0 then
		sys_roll_limit = 45.0
	end

	local roll_normalized = desired_roll_deg / sys_roll_limit

	roll_normalized = math.max(-1.0, math.min(1.0, roll_normalized))

	local roll_pwm_value
	if roll_normalized > 0 then
		roll_pwm_value = G.rc_roll_trim + roll_normalized * (G.rc_roll_max - G.rc_roll_trim)
	else
		roll_pwm_value = G.rc_roll_trim + roll_normalized * (G.rc_roll_trim - G.rc_roll_min)
	end

	roll_pwm_value = math.max(G.rc_roll_min, math.min(G.rc_roll_max, roll_pwm_value))

	if G.rc_roll_channel then
		G.rc_roll_channel:set_override(math.floor(roll_pwm_value))
	else
		self:set_script_state(G.SCRIPT_STATE.ERROR, self.Msg.MSG_ID.ERR_ROLL_NIL)
	end

	return heading_error
end

-- ============================================================================
-- CORE SCRIPT STATE MACHINE
-- ============================================================================

-- Handles the IDLE state, waiting for navigation conditions to be met.
function SoarNav:handle_idle(can_navigate)
	local G = self.State
	if can_navigate then
		if not G.require_initial_activation_gesture then
			if not G.has_been_activated then
				G.has_been_activated = true
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.AUTO_START)
			end
			self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.NAV_COND_MET)
			return
		end
		if not G.has_been_activated then
			if not G.activation_wait_notified then
				if G.script_state ~= G.SCRIPT_STATE.WAITING_FOR_ACTIVATION then
					self:set_script_state(G.SCRIPT_STATE.WAITING_FOR_ACTIVATION, self.Msg.MSG_ID.AWAITING_ACTIVATION)
				end
				G.activation_wait_notified = true
			end
		else
			self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.NAV_COND_MET)
		end
	end
end

-- Handles the NAVIGATING state, managing the flight to a waypoint.
function SoarNav:handle_navigating(current_time_ms, loc, can_navigate, is_in_thermal_mode, autotune_active, dt_s)
	local G = self.State
	if G.force_new_search then
		G.target_loc = nil
		G.force_new_search = false
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.FORCING_NEW_TARGET)
	end

	if G.navigation_start_delay_counter > 0 then
		G.navigation_start_delay_counter = G.navigation_start_delay_counter - 1
		return
	end

	if G.target_loc then
		local Sched = self.Scheduler
		local Conf = self.Config.Scheduler
		local te_evasion_active = G.TE and G.TE.evasion_active
		if te_evasion_active or current_time_ms >= Sched.tevas_update_ms then
			if not te_evasion_active then
				Sched.tevas_update_ms = current_time_ms + Conf.TEVAS_UPDATE_MS
			end
			if self:update_terrain_evasion_sm(current_time_ms, loc, G.target_loc) then
				if is_in_thermal_mode then
					vehicle:set_mode(self:_get_exit_nav_mode())
				end
				return
			end
		end
	end

	local te_is_escaping = (G.TE.hold_until_ms > current_time_ms)
	if is_in_thermal_mode then
		if te_is_escaping then
			vehicle:set_mode(self:_get_exit_nav_mode())
			return
		else
			self:set_script_state(G.SCRIPT_STATE.THERMAL_PAUSE)
			return
		end
	end
	if (not can_navigate or autotune_active) and not is_in_thermal_mode then
		self:set_script_state(G.SCRIPT_STATE.IDLE, self.Msg.MSG_ID.NAV_COND_NOT_MET)
		return
	end

	do
		local TE = G.TE or {}
		local hold_ms = TE.hold_until_ms or 0
		if TE.evasion_active and (current_time_ms > hold_ms) then
			G.TE.evasion_active = false
			G.TE.hold_until_ms = 0
			G.TE.next_check_ms = 0
			G.TE.target_heading = nil
			G.TE.target_lever_m = nil
			G.TE.last_logged_heading = nil
			G.g_waypoint_source_info = nil
			G.target_loc = nil
			G.waypoint_search_in_progress = false
			G.last_progress_check_ms = nil
			G.distance_at_last_check = -1
			G.stuck_counter = 0

			G.force_egress = true
			G.target_loc = nil
			return
		end
		if G.g_waypoint_source_info ~= G.SoarNavWaypointSources.TERRAIN_EVASION and G.g_waypoint_source_info ~= G.SoarNavWaypointSources.TE_EGRESS then
			G.TE.evasion_active = false
			G.TE.target_heading = nil
			G.TE.target_lever_m = nil
			G.TE.last_logged_heading = nil
		end
	end

	if G.TE.evasion_active and G.TE.target_heading then
		local hdg_deg = G.TE.target_heading
		local hdg_changed = (G.TE.last_logged_heading == nil) or (math.abs(hdg_deg - G.TE.last_logged_heading) > 1)

		if hdg_changed then
			G.TE.last_logged_heading = hdg_deg
			local dist = G.TE.target_lever_m or 0
			local __dirs = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" }
			local __ix = math.floor((hdg_deg / 22.5) + 0.5) + 1
			if __ix > #__dirs then __ix = 1 end
			local hdg_dir = __dirs[__ix]
			local dir_out = hdg_dir
			if G.TE.turn_arrow and G.TE.turn_arrow ~= "" then
				dir_out = hdg_dir .. " " .. G.TE.turn_arrow
			end
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.NAV_TARGET_SIMPLE,
				G.SoarNavWaypointSources.TERRAIN_EVASION, hdg_deg, dir_out, dist)
		end
		self:update_navigation_controller(loc, dt_s)
		return
	end

	while not G.target_loc do
		if not G.waypoint_search_in_progress then
			G.waypoint_search_in_progress = true
		end
		self:search_for_new_waypoint()
		if not G.target_loc then
			return
		end
	end

	local wp_status = self:check_waypoint_status(loc, current_time_ms)

	if wp_status == "REACHED" then
		if G.is_repositioning then
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.REPOSITIONED)
			self:set_target_safely(G.original_target_loc)
			G.is_repositioning = false
			G.original_target_loc = nil
			self:_finalize_waypoint_selection()
		else
			if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_ENTRY then
				local now_ms = millis():toint()
				if G.reengage_hold_active and now_ms < G.reengage_hold_until_ms and not is_in_thermal_mode then
					if not G.reengage_hold_last_msg_ms or ((now_ms - G.reengage_hold_last_msg_ms) >= 2000) then
						G.reengage_hold_last_msg_ms = now_ms
					end
				else
					G.reengage_hold_active = false
					G.reengage_hold_last_msg_ms = nil
				end
			end
			if G.reengage_final_target then
				self:set_target_safely(G.reengage_final_target)
				G.reengage_final_target = nil
				G.g_waypoint_source_info = G.SoarNavWaypointSources.REENGAGE_ENTRY
				local reengage_dwell_ms = (self.Cache.reeng_dwell or 7) * 1000
				G.reengage_hold_until_ms = millis():toint() + reengage_dwell_ms
				G.reengage_hold_active = true
				G.reengage_hold_last_msg_ms = millis():toint()
				self:_finalize_waypoint_selection()
				return
			end
			local hotspot_to_check = G.current_selected_hotspot
			if self:is_thermal_target(G.g_waypoint_source_info) and hotspot_to_check then
				hotspot_to_check.failed_attempts = (hotspot_to_check.failed_attempts or 0) + 1
				if hotspot_to_check.failed_attempts >= 2 then
					for i = #G.thermal_hotspots, 1, -1 do
						local h = G.thermal_hotspots[i]
						if
							h.timestamp
							and hotspot_to_check.timestamp
							and h.timestamp:toint() == hotspot_to_check.timestamp:toint()
						then
							table.remove(G.thermal_hotspots, i)
							break
						end
					end
				end
				G.current_selected_hotspot = nil
			end
			G.target_loc = nil
		end
	elseif wp_status == "REROUTE" then
		local original_target = G.target_loc
		if not original_target then
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.REROUTE_SKIP_NO_TARGET)
			G.target_loc = nil
			return
		end

		local reroute_origin = ahrs:get_location()
		local yaw_rad = ahrs:get_yaw_rad()
		if not reroute_origin or not yaw_rad then
			self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.REROUTE_SKIP_HEADING)
			G.target_loc = nil
			return
		end

		local hdg = (math.deg(yaw_rad) + 360) % 360
		local offset = math.random(self.Cache.reroute_min or 80, self.Cache.reroute_max or 100)
		if math.random() > 0.5 then
			offset = -offset
		end
		local desired_bearing = (hdg + offset + 360) % 360

		local distance = reroute_origin:get_distance(original_target)
		local new_target = reroute_origin:copy()
		new_target:offset_bearing(desired_bearing, distance)

		if not self:in_flight_area(new_target) then
			G.target_loc = nil
			return
		end

		self:set_target_safely(new_target)
		G.g_waypoint_source_info = G.SoarNavWaypointSources.REROUTE
		self:_finalize_waypoint_selection()
		return
	elseif wp_status == "TIMEOUT" then
		if G.g_waypoint_source_info == G.SoarNavWaypointSources.REENGAGE_FLYOUT and G.reengage_final_target then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.NAV_TARGET_SIMPLE, "FlyOut Timeout", 0, "Skip", 0)
			self:set_target_safely(G.reengage_final_target)
			G.reengage_final_target = nil
			G.g_waypoint_source_info = G.SoarNavWaypointSources.REENGAGE_ENTRY
			local reengage_dwell_ms = (self.Cache.reeng_dwell or 7) * 1000
			G.reengage_hold_until_ms = millis():toint() + reengage_dwell_ms
			G.reengage_hold_active = true
			G.reengage_hold_last_msg_ms = millis():toint()
			self:_finalize_waypoint_selection()
		else
			G.target_loc = nil
		end
	end

	if self:manage_anti_stuck(loc, current_time_ms) then
		return
	end

	if G.target_loc then
		if G.grid_initialized and G.grid_cell_size_m > 0 then
			local should_update_cell =
				not G.last_cell_check_loc or loc:get_distance(G.last_cell_check_loc) > (G.grid_cell_size_m / 2)
			if should_update_cell then
				local Sched = self.Scheduler
				local Conf = self.Config.Scheduler
				if current_time_ms >= (Sched.visited_cell_ms or 0) then
					Sched.visited_cell_ms = current_time_ms + Conf.VISITED_CELL_MS
					self:update_visited_cell()
					G.last_cell_check_loc = loc:copy()
				end
			end
		end
		self:update_navigation_controller(loc, dt_s)
	end
end

-- Handles the THERMAL_PAUSE state while the aircraft is in ArduPilot's THERMAL mode.
function SoarNav:handle_thermal_pause(is_in_thermal_mode, current_time_ms, loc)
	local G = self.State
	local MODE_SWITCHED_FROM_TE = "MODE_SWITCHED"

	if (self.Cache.dyn_soalt or 0) == 3 then
		local target_for_tevas = loc:copy()
		local current_heading_rad = ahrs:get_yaw_rad()
		if not current_heading_rad then
			return true
		end
		target_for_tevas:offset_bearing(math.deg(current_heading_rad), 500)

		local Sched = self.Scheduler
		local Conf = self.Config.Scheduler
		local te_evasion_active = G.TE and G.TE.evasion_active
		if te_evasion_active or current_time_ms >= Sched.tevas_update_ms then
			if not te_evasion_active then
				Sched.tevas_update_ms = current_time_ms + Conf.TEVAS_UPDATE_MS
			end
			if self:update_terrain_evasion_sm(current_time_ms, loc, target_for_tevas) then
				vehicle:set_mode(self:_get_exit_nav_mode())
				self:set_script_state(G.SCRIPT_STATE.NAVIGATING)
				return MODE_SWITCHED_FROM_TE
			end
		end
	end

	if not is_in_thermal_mode then
		self:set_script_state(G.SCRIPT_STATE.NAVIGATING)
	end
	return true
end

-- Handles the WAITING_FOR_ACTIVATION state.
function SoarNav:handle_waiting(can_navigate)
	local G = self.State
	if G.has_been_activated and can_navigate then
		self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.NAV_COND_MET)
		return
	end
	if not arming:is_armed() then
		self:set_script_state(G.SCRIPT_STATE.IDLE, self.Msg.MSG_ID.DISARMED)
		return
	end
	if can_navigate then
		if G.activation_requested then
			G.has_been_activated = true
			G.activation_requested = false
			self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.PILOT_ACTIVATION)
		end
	end
end

-- Handles the PILOT_OVERRIDE state, managing stick input and gesture detection.
function SoarNav:handle_pilot_override(
	current_time_ms,
	is_outside_pitch_dz,
	is_outside_yaw_dz,
	is_outside_roll_dz,
	can_navigate
)
	local G = self.State
	local C = self.Const
	local pilot_is_holding_input = is_outside_pitch_dz or is_outside_yaw_dz or is_outside_roll_dz
	if pilot_is_holding_input then
		G.last_pilot_input_ms = current_time_ms
	end

	if not G.manual_override_active then
		local resume_delay_passed =
			self:safe_time_diff(current_time_ms, G.last_pilot_input_ms) > C.PILOT_RESUME_DELAY_MS
		if not pilot_is_holding_input and G.last_pilot_input_ms and resume_delay_passed then
			if can_navigate then
				self:set_script_state(G.SCRIPT_STATE.NAVIGATING, self.Msg.MSG_ID.RESUMING_NAV)
			else
				self:set_script_state(G.SCRIPT_STATE.IDLE, self.Msg.MSG_ID.SET_CRUISE_FBWB)
			end
		end
	end
end

-- Manages RTL override logic when using Rally points as the flight area.
function SoarNav:SoarNav_RTLH_update(now_ms)
	local G = self.State
	local C = self.Const
	if not G.rtlh_en then
		return
	end
	if not G.using_rally_points then
		return
	end

	G.RTLH._mode = vehicle:get_mode()

	if
		G.RTLH.last_mode ~= nil
		and G.RTLH.engaged_guided
		and G.RTLH.last_mode == C.GUIDED_MODE_CODE
		and G.RTLH._mode ~= C.GUIDED_MODE_CODE
	then
		G.RTLH.abort_until_next_rtl = true
		G.RTLH.engaged_guided = false
		self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.RTL_OVERRIDE_RESUME)
	end

	if G.RTLH.last_mode ~= C.RTL_MODE_CODE and G.RTLH._mode == C.RTL_MODE_CODE then
		G.RTLH.abort_until_next_rtl = false
		G.RTLH.active = false
	end

	G.RTLH.last_mode = G.RTLH._mode
	if G.RTLH.abort_until_next_rtl then
		return
	end

	if G.RTLH.engaged_guided and vehicle:get_mode() == C.GUIDED_MODE_CODE then
		local here = ahrs:get_location()
		local home = ahrs:get_home()
		if here and home then
			local d = here:get_distance(home)
			local rtl_radius = param:get("RTL_RADIUS") or 0
			if rtl_radius == 0 then
				rtl_radius = 50
			end
			if d <= rtl_radius then
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.RTL_OVERRIDE_RESUME)
				G.RTLH.abort_until_next_rtl = true
				G.RTLH.engaged_guided = false
			end
		end
	end

	if G.RTLH._mode ~= C.RTL_MODE_CODE then
		if G.RTLH.active then
			G.RTLH.active = false
		end
		return
	end

	if not G.RTLH.active then
		local home = ahrs:get_home()
		local here = ahrs:get_location()

		if not home or not here then
			return
		end

		G.RTLH.active = true
		G.RTLH.t0_ms = now_ms
		G.RTLH.d0_m = here:get_distance(home)
		return
	end

	if not G.RTLH.engaged_guided and (now_ms - G.RTLH.t0_ms) >= C.RTLH_CHECK_DELAY_MS then
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

		if (G.RTLH.d0_m - d_now) < needed_gain then
			local alt_m = param:get("RTL_ALTITUDE") or 0
			local dest = Location()
			dest:lat(home:lat())
			dest:lng(home:lng())
			dest:alt(alt_m * 100)
			dest:change_alt_frame(1)
			vehicle:set_mode(C.GUIDED_MODE_CODE)
			vehicle:set_target_location(dest)
			G.RTLH.engaged_guided = true
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.RTL_STALL_OVERRIDE, math.floor(alt_m + 0.5))
		end
	end
end

-- ============================================================================
-- SCRIPT MAIN LOOP (UPDATE_BODY)
-- ============================================================================

-- Updates the global parameter cache table.
function SoarNav:update_param_cache()
	local P = self.Params
	local Cache = self.Cache
	local G = self.State

	Cache.enable = self:pget(P.enable, 0)
	Cache.radius_m = self:get_safe_param(P.radius_m, 500, 0, nil)
	Cache.dyn_soalt = self:get_safe_param(P.dyn_soalt, 0, 0, 3)

	G._param_full_update_count = (G._param_full_update_count or 0) + 1
	if G._param_full_update_count >= 3 or not Cache.roll_limit then
		G._param_full_update_count = 0
		self:update_param_cache_full()
	end
end

-- Full parameter cache update for non-critical parameters.
function SoarNav:update_param_cache_full()
	local P = self.Params
	local Cache = self.Cache
	Cache.tecs_sink_min = self:get_safe_param(Parameter('TECS_SINK_MIN'), 0.3, 0, nil)
	Cache.tecs_sink_max = self:get_safe_param(Parameter('TECS_SINK_MAX'), 3.0, 0, nil)
	Cache.log_lvl = self:get_safe_param(P.log_lvl, 1, 0, 2)
	Cache.wp_radius = self:get_safe_param(P.wp_radius, 50, 10, 200)
	Cache.tmem_enabled = (self:get_safe_param(P.tmem_enable, 1, 0, 1) == 1)
	Cache.tmem_life = self:pget(P.tmem_life, 1200)
	Cache.soar_alt_min = self:pget(P.soar_alt_min, 100)
	Cache.soar_alt_max = self:pget(P.soar_alt_max, 600)
	Cache.soar_alt_cutoff = self:pget(P.soar_alt_cutoff, 150)
	Cache.rcmap_roll = self:pget(P.rcmap_roll, 1)
	Cache.rcmap_pitch = self:pget(P.rcmap_pitch, 2)
	Cache.rcmap_yaw = self:pget(P.rcmap_yaw, 4)
	Cache.roll_limit = self:get_safe_param(P.roll_limit, 30, 10, 50)
	Cache.nav_p = self:get_safe_param(P.nav_p, 0.6)
	Cache.nav_d = self:get_safe_param(P.nav_d, 0.05)
	Cache.stuck_eff = self:get_safe_param(P.stuck_eff, 0.35, 0.1, 1.0)
	Cache.stuck_time = self:get_safe_param(P.stuck_time, 30, 5, 120)
	Cache.reeng_dwell = self:get_safe_param(P.reeng_dwell, 7, 0, 120)
	Cache.retry_thr = self:get_safe_param(P.retry_thr, 30, 0, 100)
	Cache.street_tol = self:get_safe_param(P.street_tol, 30, 5, 90)
	Cache.reroute_p = self:get_safe_param(P.reroute_p, 50, 0, 100)
	Cache.nec_weight = self:get_safe_param(P.nec_weight, 50, 0, 100)
	Cache.tmem_min_s = self:get_safe_param(P.tmem_min_s, 0.2, 0.0, 5.0)
	Cache.focus_thr = self:get_safe_param(P.focus_thr, 1.0, 0.0, 5.0)
	Cache.strat_hist = self:get_safe_param(P.strat_hist, 900, 60, 3600)
	Cache.wp_timeout = self:get_safe_param(P.wp_timeout, 300, 30, 900)
	Cache.reroute_min = self:get_safe_param(P.reroute_min, 80)
	Cache.reroute_max = self:get_safe_param(P.reroute_max, 100)
	Cache.soar_polar_cd0 = self:pget(P.soar_polar_cd0)
	Cache.soar_polar_b = self:pget(P.soar_polar_b)
	Cache.airspeed_cruise = self:pget(P.airspeed_cruise)
	Cache.gc_margin = self:get_safe_param(P.gc_margin, 25, 0, 150)
	Cache.gc_pad = self:get_safe_param(P.gc_pad, 20, 0, 100)
	Cache.te_look_s = self:get_safe_param(P.te_look_s, 10, 5, 60)
	Cache.te_buf_min = self:get_safe_param(P.te_buf_min, 80, 40, 150)

	-- ArduPilot system parameters
	Cache.loiter_radius = self:get_safe_param(Parameter('WP_LOITER_RAD'), 60, 0, nil)
	local sys_roll = param:get("ROLL_LIMIT_DEG")
	if not sys_roll or sys_roll <= 0 then
		sys_roll = (param:get("LIM_ROLL_CD") or 4500) / 100
	end
	if not sys_roll or sys_roll <= 0 then
		sys_roll = 45
	end
	Cache.sys_roll_limit = sys_roll
end

-- The main logic loop of the script, containing the primary state machine.
function SoarNav:update_body()
	local G = self.State
	local C = self.Const
	local Cache = self.Cache
	local Sched = self.Scheduler
	local Conf = self.Config.Scheduler

	local period = Conf.MAIN_LOOP_SLOW_MS
	if G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE or G.script_state == G.SCRIPT_STATE.NAVIGATING or G.script_state == G.SCRIPT_STATE.WAITING_FOR_ACTIVATION then
		period = Conf.MAIN_LOOP_FAST_MS
	end

	local current_time_ms = millis()

	if current_time_ms >= Sched.cache_update_ms then
		Sched.cache_update_ms = current_time_ms + Conf.CACHE_UPDATE_MS
		self:update_param_cache()
	end

	if G.rpm_check_counter ~= nil and G.rpm_check_counter >= 0 then
		G.rpm_check_counter = G.rpm_check_counter + 1
		if G.rpm_check_counter == 5 then
			local rpm1_type = param:get("RPM1_TYPE") or 0
			local rpm2_type = param:get("RPM2_TYPE") or 0
			if rpm1_type > 0 or rpm2_type > 0 then
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.RPM_SENSOR_SET)
			else
				self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.NO_RPM_FALLBACK)
			end
			G.rpm_check_counter = -1
		end
	end
	local is_armed_now = arming:is_armed()
	if G.last_armed_state == nil then
		G.last_armed_state = is_armed_now
	end
	if (not is_armed_now) and G.last_armed_state == true and not G.restored_on_disarm then
		self:restore_boot_alts("Disarmed")
		G.restored_on_disarm = true
		G.activation_requested = false
		G.activation_wait_notified = false
	end
	if is_armed_now then
		G.restored_on_disarm = false
	end
	G.last_armed_state = is_armed_now
	if G.initial_soar_alt_min == nil and Cache.soar_alt_min ~= nil then
		local min_val = Cache.soar_alt_min or -1
		local max_val = Cache.soar_alt_max or -1
		local cutoff_val = Cache.soar_alt_cutoff or -1
		G.initial_soar_alt_min = min_val
		G.initial_soar_alt_max = max_val
		G.initial_soar_alt_cutoff = cutoff_val
		self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.BASE_ALTS_INFO, min_val, cutoff_val, max_val)
	end
	local dt_s = 0
	if G.last_update_time_ms then
		dt_s = self:safe_time_diff(current_time_ms, G.last_update_time_ms) / 1000.0
	end
	G.last_update_time_ms = current_time_ms
	local current_snav_enable = Cache.enable
	if current_snav_enable == 0 then
		if G.script_state ~= G.SCRIPT_STATE.IDLE then
			self:set_script_state(G.SCRIPT_STATE.IDLE, self.Msg.MSG_ID.SCRIPT_DISABLED)
		end
		return period * 10
	end

	if current_time_ms >= Sched.rally_poll_ms then
		Sched.rally_poll_ms = current_time_ms + Conf.RALLY_POLL_MS
		if (Cache.radius_m or 0) <= 0 and rally and rally.get_rally_location then
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

			local count_changed = (count ~= G.last_rally_point_count)

			if G.rally_signature == nil then
				G.last_rally_point_count = count
				G.rally_signature = signature
			else
				local content_changed = (signature ~= G.rally_signature)
				local became_invalid = G.using_rally_points and (count < 3)
				local became_valid = (not G.using_rally_points) and (count >= 3)

				if count_changed or content_changed or became_invalid or became_valid then
					G.last_rally_point_count = count
					G.rally_signature = signature
					if (Cache.radius_m or 0) <= 0 then
						self:set_script_state(G.SCRIPT_STATE.IDLE, self.Msg.MSG_ID.RALLY_CHANGED_REINIT)
						G.grid_initialized = false
						G.polygon_load_attempted = false
						G.area_announced = false
						G.using_rally_points = false
						G.target_loc = nil
						G.activation_requested = false
						G.activation_wait_notified = false
						G.navigation_start_delay_counter = 0
					end
				end
			end
		end
	end

	local current_snav_radius_m = Cache.radius_m
	if current_time_ms >= Sched.param_check_ms then
		Sched.param_check_ms = current_time_ms + Conf.PARAM_CHECK_MS
		if
			(current_snav_radius_m ~= G.last_snav_radius_m_value)
			or (current_snav_enable ~= G.last_snav_enable_value)
		then
			self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.PARAM_CHANGED_REINIT)
			if current_snav_radius_m > 0 then
				self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.RADIUS_MODE_ACTIVATED, current_snav_radius_m)
				G.area_announced = false
				self:announce_radius_area()
			end
			G.target_loc = nil
			G.is_initializing = false
			G.grid_initialized = false
			G.polygon_load_attempted = false
			G.last_snav_radius_m_value = current_snav_radius_m
			G.last_snav_enable_value = current_snav_enable
			G.use_polygon_area = false
		end
	end

	local autotune_chan = rc:find_channel_for_option(107)
	local autotune_active = (autotune_chan and autotune_chan:get_aux_switch_pos() > 0)
	if not self:validate_params() then
		self:set_script_state(G.SCRIPT_STATE.ERROR, self.Msg.MSG_ID.INVALID_SNAV_PARAMS)
		return period * 50
	end
	if G.script_state == G.SCRIPT_STATE.ERROR then
		return period * 50
	end
	if not G.rc_limits_read and arming:is_armed() then
		local roll_ch_num = Cache.rcmap_roll or 1
		local pitch_ch_num = Cache.rcmap_pitch or 2
		local yaw_ch_num = Cache.rcmap_yaw or 4
		G.rc_roll_channel = rc:get_channel(roll_ch_num)
		G.rc_roll_min = param:get(string.format("RC%u_MIN", roll_ch_num)) or 1000
		G.rc_roll_max = param:get(string.format("RC%u_MAX", roll_ch_num)) or 2000
		G.rc_roll_trim = param:get(string.format("RC%u_TRIM", roll_ch_num)) or 1500
		local rc_pitch_min = param:get(string.format("RC%u_MIN", pitch_ch_num)) or 1000
		local rc_pitch_max = param:get(string.format("RC%u_MAX", pitch_ch_num)) or 2000
		local rc_pitch_trim = param:get(string.format("RC%u_TRIM", pitch_ch_num)) or 1500
		local rc_yaw_min = param:get(string.format("RC%u_MIN", yaw_ch_num)) or 1000
		local rc_yaw_max = param:get(string.format("RC%u_MAX", yaw_ch_num)) or 2000
		local rc_yaw_trim = param:get(string.format("RC%u_TRIM", yaw_ch_num)) or 1500
		if not (G.rc_roll_min < G.rc_roll_trim and G.rc_roll_trim < G.rc_roll_max) then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.RC_R_LIMITS, G.rc_roll_min, G.rc_roll_trim, G.rc_roll_max)
		end
		if not (rc_pitch_min < rc_pitch_trim and rc_pitch_trim < rc_pitch_max) then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.RC_P_LIMITS, rc_pitch_min, rc_pitch_trim, rc_pitch_max)
		end
		if not (rc_yaw_min < rc_yaw_trim and rc_yaw_trim < rc_yaw_max) then
			self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.RC_Y_LIMITS, rc_yaw_min, rc_yaw_trim, rc_yaw_max)
		end
		G.rc_limits_read = true
		self:log_gcs(self.Msg.MAV_SEVERITY.INFO, 1, self.Msg.MSG_ID.RC_LIMITS_READ, roll_ch_num, pitch_ch_num, yaw_ch_num)
	end
	local loc = ahrs:get_location()
	if not loc then
		G.location_error_count = G.location_error_count + 1
		if G.location_error_count > C.MAX_LOCATION_ERRORS and arming:is_armed() then
			self:set_script_state(G.SCRIPT_STATE.ERROR, self.Msg.MSG_ID.PERSISTENT_LOC_ERROR)
		end
		return period
	else
		G.location_error_count = self:safe_decrement(G.location_error_count)
	end
	if arming:is_armed() and not G.grid_initialized and not G.is_initializing then
		local should_init = false
		if G.force_grid_reinit then
			should_init = true
			G.force_grid_reinit = false
		end

		if Cache.radius_m > 0 then
			G.use_polygon_area = false
			G.using_rally_points = false
			should_init = true
			if G.last_snav_radius_m_value <= 0 then
				G.dynamic_center_location = ahrs:get_location():copy()
			end
		else
			if not G.polygon_load_attempted then
				if self:read_rally_points_as_polygon() then
					G.use_polygon_area = true
					G.using_rally_points = true
					should_init = true
				else
					G.using_rally_points = false
					if rally then
						local rally_point_count = 0
						while rally:get_rally_location(rally_point_count) do
							rally_point_count = rally_point_count + 1
						end
						if rally_point_count > 0 and rally_point_count < 3 then
							self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.RPS_NEED_3, rally_point_count)
						end
					end

					local poly_index_to_try = math.min(current_snav_enable, C.max_polygon_index)
					local poly_loaded = false
					local function try_load_polygon(index)
						local base_filename =
							string.format("%s%d%s", C.polygon_filename_prefix, index, C.polygon_filename_suffix)
						local paths_to_try =
							{ "/APM/SCRIPTS/" .. base_filename, "/SCRIPTS/" .. base_filename, base_filename }
						for _, path in ipairs(paths_to_try) do
							if self:read_polygon_file(path) then
								self:log_gcs(self.Msg.MAV_SEVERITY.NOTICE, 1, self.Msg.MSG_ID.LOADED_POLY, path)
								return true
							end
						end
						return false
					end

					if try_load_polygon(poly_index_to_try) then
						poly_loaded = true
					else
						self:log_gcs(self.Msg.MAV_SEVERITY.WARNING, 1, self.Msg.MSG_ID.POLY_FAILED_FALLBACK, poly_index_to_try)
						for i = 1, C.max_polygon_index do
							if try_load_polygon(i) then
								poly_loaded = true
								break
							end
						end
					end

					if poly_loaded then
						G.use_polygon_area = true
						should_init = true
					else
						if rally then
							local rally_point_count = 0
							while rally:get_rally_location(rally_point_count) do
								rally_point_count = rally_point_count + 1
							end
							if rally_point_count == 0 then
								self:log_gcs(self.Msg.MAV_SEVERITY.CRITICAL, 0, self.Msg.MSG_ID.NO_POLY_DISABLED)
							end
						else
							self:log_gcs(self.Msg.MAV_SEVERITY.CRITICAL, 0, self.Msg.MSG_ID.NO_POLY_DISABLED)
						end
						G.use_polygon_area = false
					end
				end
				G.polygon_load_attempted = true
			end
		end
		if should_init and not G.grid_initialized then
			self:initialize_grid()
		end
	end
	if G.is_initializing then
		self:manage_grid_initialization()
		if G.is_initializing then
			return 50
		end
	end
	local roll_ch_obj = rc:get_channel(Cache.rcmap_roll)
	local pitch_ch_obj = rc:get_channel(Cache.rcmap_pitch)
	local yaw_ch_obj = rc:get_channel(Cache.rcmap_yaw)
	if not roll_ch_obj or not pitch_ch_obj or not yaw_ch_obj then
		self:log_gcs(self.Msg.MAV_SEVERITY.ERROR, 0, self.Msg.MSG_ID.INVALID_RC_MAPPING)
		return period
	end
	local roll_in = roll_ch_obj:norm_input_dz()
	local pitch_in = pitch_ch_obj:norm_input_dz()
	local yaw_in = yaw_ch_obj:norm_input_dz()
	local stick_eps = 0.05
	local is_outside_roll_dz  = math.abs(roll_in)  > stick_eps
    local is_outside_pitch_dz = math.abs(pitch_in) > stick_eps
    local is_outside_yaw_dz   = math.abs(yaw_in)   > stick_eps
	local gesture_just_completed = false
	if current_time_ms >= Sched.stick_gesture_ms then
		Sched.stick_gesture_ms = current_time_ms + Conf.STICK_GESTURE_MS
		if G.script_state == G.SCRIPT_STATE.NAVIGATING or G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE then
			gesture_just_completed = self:check_roll_gesture()
			self:check_pitch_gesture()
		elseif G.script_state == G.SCRIPT_STATE.WAITING_FOR_ACTIVATION then
			local __mode = vehicle:get_mode()
			if __mode == C.mode_fbwb or __mode == C.mode_cruise then
				gesture_just_completed = self:check_roll_gesture()
			end
		end
	end

	if
		not gesture_just_completed
		and not G.pitch_gesture_triggered_this_override
		and G.script_state == G.SCRIPT_STATE.NAVIGATING
		and (math.abs(pitch_in) > 0.05 or math.abs(yaw_in) > 0.05)
		and not (G.activation_grace_start_ms and self:safe_time_diff(current_time_ms, G.activation_grace_start_ms) < C.ACTIVATION_OVERRIDE_GRACE_MS)
	then
		self:set_script_state(G.SCRIPT_STATE.PILOT_OVERRIDE, self.Msg.MSG_ID.PILOT_OVERRIDE)
		G.last_pilot_input_ms = current_time_ms
		G.pitch_gesture_state = "idle"
		G.pitch_gesture_count = 0
		G.pitch_gesture_triggered_this_override = false
		G.manual_override_active = false
	end

	local current_mode = vehicle:get_mode()
	local in_nav_mode = (current_mode == C.mode_fbwb or current_mode == C.mode_cruise)
	if in_nav_mode then
		G.last_nav_mode = current_mode
	end
	if current_time_ms >= Sched.rtlh_update_ms then
		Sched.rtlh_update_ms = current_time_ms + Conf.RTLH_UPDATE_MS
		self:SoarNav_RTLH_update(current_time_ms)
	end
	local is_in_thermal_mode = (current_mode == C.mode_thermal)
	if is_in_thermal_mode and not G.was_in_thermal_mode then
		if G.last_nav_mode == C.mode_fbwb or G.last_nav_mode == C.mode_cruise then
			G.nav_mode_before_thermal = G.last_nav_mode
		else
			G.nav_mode_before_thermal = C.mode_cruise
		end
	end
	if Cache.tmem_enabled then
		if is_in_thermal_mode and not G.was_in_thermal_mode then
			self:start_thermal_monitoring()
		elseif not is_in_thermal_mode and G.was_in_thermal_mode then
			if not G.TE.evasion_active then
				self:stop_and_record_thermal()
			end
		end
	end
	G.was_in_thermal_mode = is_in_thermal_mode
	if G.is_monitoring_thermal then
		local time_since_sample_ms = -1
		if G.last_thermal_sample_ms then
			time_since_sample_ms = self:safe_time_diff(current_time_ms, G.last_thermal_sample_ms)
		end
		local strength = G.current_thermal_stats.max_strength or 1.0
		if strength <= 0 then
			strength = 1.0
		end
		local sample_interval = math.max(500, math.min(3000, 2000 / (strength + 0.5)))
		if time_since_sample_ms > 0 and (time_since_sample_ms > sample_interval) then
			self:sample_thermal_strength()
		end
	end

local script_switch_high = (rc:get_aux_cached(C.rc_opt_soaring_active) == 2)
	local home = ahrs:get_home()
	local dist_from_home_3d = ahrs:get_relative_position_NED_home()
	local can_navigate = false

	if
		arming:is_armed()
		and script_switch_high
		and not autotune_active
		and in_nav_mode
		and home
		and dist_from_home_3d
		and ahrs:get_yaw_rad()
		and G.grid_initialized
	then
		local tevas_active = (Cache.dyn_soalt == 3)
		local altitude_check_ok = self:_nav_altitude_check(tevas_active, home, dist_from_home_3d, loc, G.initial_soar_alt_min)

		if altitude_check_ok then
			can_navigate = true
		end
	end

	if
		can_navigate
		and G.initial_soar_alt_min
		and G.script_state ~= G.SCRIPT_STATE.PILOT_OVERRIDE
		and not G.manual_override_active
	then
		local throttle_output = SRV_Channels:get_output_scaled(C.THROTTLE_SRV_CHANNEL) or 0
		local motor_is_off = throttle_output < 10
		if
			motor_is_off
			and current_time_ms >= Sched.glide_cone_ms
		then
			Sched.glide_cone_ms = current_time_ms + Conf.GLIDE_CONE_MS
			self:update_dynamic_soar_alt_min()
			G.last_glide_cone_update_ms = current_time_ms
		end
	end
	if G.script_state == G.SCRIPT_STATE.IDLE then
		self:handle_idle(can_navigate)
	elseif G.script_state == G.SCRIPT_STATE.NAVIGATING then
		self:handle_navigating(current_time_ms, loc, can_navigate, is_in_thermal_mode, autotune_active, dt_s)
	elseif G.script_state == G.SCRIPT_STATE.THERMAL_PAUSE then
		local MODE_SWITCHED_FROM_TE = "MODE_SWITCHED"
		local thermal_result = self:handle_thermal_pause(is_in_thermal_mode, current_time_ms, loc)
		if thermal_result == MODE_SWITCHED_FROM_TE then
			is_in_thermal_mode = false
			self:handle_navigating(current_time_ms, loc, can_navigate, is_in_thermal_mode, autotune_active, dt_s)
		end
	elseif G.script_state == G.SCRIPT_STATE.PILOT_OVERRIDE then
		self:handle_pilot_override(current_time_ms, is_outside_pitch_dz, is_outside_yaw_dz, is_outside_roll_dz, can_navigate)
	elseif G.script_state == G.SCRIPT_STATE.WAITING_FOR_ACTIVATION then
		self:handle_waiting(can_navigate)
	end
	if
		arming:is_armed()
		and G.script_state == G.SCRIPT_STATE.NAVIGATING
		and (SRV_Channels:get_output_scaled(C.THROTTLE_SRV_CHANNEL) or 0) < C.THROTTLE_ACTIVE_THRESHOLD
		and current_mode ~= C.mode_thermal
	then
		if G.polar_learn and current_time_ms >= Sched.polar_learn_ms then
			Sched.polar_learn_ms = current_time_ms + Conf.POLAR_LEARN_MS
			self:update_polar_learning()
		end
	end
	if current_time_ms >= Sched.motor_failure_ms then
		Sched.motor_failure_ms = current_time_ms + Conf.MOTOR_FAILURE_MS
		self:check_motor_failure(current_mode, dist_from_home_3d)
	end
	return period
end

-- Checks if a tactical reroute should be performed based on probability.
function SoarNav:check_tactical_reroute_conditions()
	local reroute_prob = self.Cache.reroute_p or 50
	if reroute_prob <= 0 then
		return false
	end
	return (math.random(1, 100) <= reroute_prob)
end

-- Entry point for ArduPilot’s scheduler; delegates to SoarNav:update_body() and returns (function, period).
function update()
  local ok, res_or_err = pcall(SoarNav.update_body, SoarNav)
  local period
  if not ok or type(res_or_err) ~= "number" then
    period = 100
    if not ok then
      gcs:send_text(SoarNav.Msg.MAV_SEVERITY.CRITICAL, "SoarNav: FATAL LUA ERROR: " .. tostring(res_or_err))
    else
      local log_ok, log_err = pcall(function()
        SoarNav:log_gcs(SoarNav.Msg.MAV_SEVERITY.WARNING, 1, SoarNav.Msg.MSG_ID.PARAM_OUT_OF_RANGE, "UPDATE_PERIOD", ">=0", "n/a", tostring(res_or_err))
      end)
      if not log_ok then
        gcs:send_text(SoarNav.Msg.MAV_SEVERITY.WARNING, "SoarNav: Invalid period (and dynamic log failed): " .. tostring(res_or_err) .. " | log err: " .. tostring(log_err))
      end
    end

  else
    period = res_or_err
  end

  return update, period
end

-- ============================================================================
-- SCRIPT INITIALIZATION
-- ============================================================================
do
  math.randomseed(millis():toint())
  SoarNav:add_params()
  SoarNav:bind_params()
  SoarNav:log_gcs(SoarNav.Msg.MAV_SEVERITY.NOTICE, 1, SoarNav.Msg.MSG_ID.SCRIPT_INITIALIZED)
  return update, SoarNav.Config.Scheduler.MAIN_LOOP_SLOW_MS
end