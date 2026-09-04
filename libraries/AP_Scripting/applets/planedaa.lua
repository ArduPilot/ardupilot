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

 Detect, Alert, Avoid for Plane

 This code implements a DAA function for Plane.
 Detect - relies on MAVLink input from GLOBAL_POSITION_INT, FOLLOW_TARGET or ADSB_VEHICLE
            to provide the location of other aircraft in the vecility
        - This other input can come from a variety of sources including an onboard
            companion computer using vision based AI to generate ADSB_VEHICLE messages
            for detected obstacles in teh vicinity.
        - Also uses ArduPilot GeoFences which can be inclusive or exclusive and can used
            to describe no fly zones such as restricted airspace, or towers, buildings or
            other structures that might not show up in terrain.
Alert - displays GCS text messages which describe threatening vehicles or obstacles
            also sends GCS_THREAT messages which can be displayed by a suitably enabled
            GCS such as Mission Planner or MavProxy
Avoid - implements bendy ruler based heuristic avoidance for most obstacles
            for crewed aircraft it also implements more
            flexible avoidance manoeuvre such as a Standard Right Turn to Altitude.
            The intention is to come up with a standard library of crewed-aircraft avoidance
            manoeuvres, will also allowing users to implement there own due to this
            being implemented as Lua.
--]]

SCRIPT_NAME         = "Plane DAA"
SCRIPT_NAME_SHORT   = "pDAA"
SCRIPT_VERSION      = "4.8.0-104"

STARTUP_DELAY       = 25  -- wait this many seconds for the FC to come up before starting the main loop

PLANE_MODE          = {MANUAL = 0, CIRCLE = 1, STABILIZE = 2, TRAINING = 3, ACRO = 4, FBWA = 5, FBWB = 6, CRUISE = 7,
                        AUTOTUNE = 8, AUTO=10, RTL=11, LOITER=12, TAKEOFF = 13, AVOID_ADSB = 14, GUIDED=15,
                        INITIALISING = 16, QSTABILIZE = 17, QHOVER=18, QLOITER=19, QLAND = 20, QRTL=21,
                        QAUTOTUNE = 22, QACRO = 23, THERMAL = 24, LOITER_ALT_QLAND = 25, AUTOLAND = 26}
-- MAV_DO_REPOSITION_FLAGS: setting CHANGE_MODE makes the reposition switch to GUIDED
-- itself, so the mode change and the target load are one operation rather than two
MAV_DO_REPOSITION_FLAGS = {CHANGE_MODE = 1}

ALT_FRAME           = {GLOBAL = 0, RELATIVE = 1, ORIGIN = 2, TERRAIN = 3}

MAV_SEVERITY        = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
 -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points


-- ADSB Emitter types

local PARAM_TABLE_KEY = 126
local PARAM_TABLE_PREFIX = "DAA_"

-- bind a parameter to a variable
function bind_param(name)
    return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format(' could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 37), SCRIPT_NAME_SHORT .. ' could not add param table: ' .. PARAM_TABLE_PREFIX .. " key: " .. PARAM_TABLE_KEY)

-- Every parameter this applet binds lives in this one table rather than in a global each.
-- The field name IS the string handed to bind_add_param, so the pair costs the parser a
-- single distinct name instead of two - see planedaa.md, "Distinct names are a budget" -
-- and it keeps 45 names out of _ENV.  The cached locals further down are what the code
-- actually reads; these objects are only touched when parameters are refreshed.
local PARAM = {}

--[[
    // @Param: DAA_ACT_FN
    // @DisplayName: DAA Activation
    // @Description: Disable the DAA capability or turn it off (defaults to on)
    // @Range: 300 308
--]]
PARAM.ACT_FN = bind_add_param("ACT_FN", 1, 308)

--[[
  // @Param: DAA_MARGIN_FENCE
  // @DisplayName: fence margin
  // @Description: Avoidance margin (m) kept clear of fences. 0 (default) uses the turn radius WP_LOITER_RAD, so the standoff matches a single loiter circle and fences do not thrash.
  // @Units: m
--]]
PARAM.MARGIN_FENCE = bind_add_param('MARGIN_FENCE', 2, 0)

--[[
  // @Param: DAA_LKAHD_M
  // @DisplayName: avoidance probe distance
  // @Description: How far along each candidate heading the bendy ruler probes for a clear path; the second leg probes a further 2x. Not the detection range (DAA_DETECT_M) nor the commanded target distance (DAA_PLAN_M). Keep it at least 3x the turn radius.
  // @Units: m
  // @Range: 50 2000
  // @User: Standard
--]]
PARAM.LKAHD_M  = bind_add_param('LKAHD_M', 3, 500)

--[[
  // @Param: DAA_UPDATE_RATE
  // @DisplayName: rate to process avoidance
  // @Description: Avoidance processing rate
  // @Units: Hz
--]]
PARAM.UPDATE_RATE  = bind_add_param('UPDATE_RATE', 4, 10.0)

--[[
  // @Param: DAA_MARGIN_CA
  // @DisplayName: Margin for crewed aircraft
  // @Description: Avoidance margin for crewed aircraft (fixed wing, helicopter, eVTOL) over and above the Well Clear margin AVD_WCLR_XY
  // @Units: m
--]]
PARAM.MARGIN_CA  = bind_add_param('MARGIN_CA', 5, 50)

--[[
  // @Param: DAA_MARGIN_UAV
  // @DisplayName: Margin for UAVs/Drones
  // @Description: Avoidance radius for UAV/drone (MAVLink sourced)
  // @Units: m
--]]
PARAM.MARGIN_UAV  = bind_add_param('MARGIN_UAV', 9, 50)

--[[
  // @Param: DAA_MARGIN_AIS
  // @DisplayName: Margin for AIS (ships)
  // @Description: Avoidance radius for AIS (MAVLink sourced)
  // @Units: m
--]]
PARAM.MARGIN_AIS  = bind_add_param('MARGIN_AIS', 10, 50)

--[[
  // @Param: DAA_MARGIN_PRX
  // @DisplayName: Margin for proximity
  // @Description: Avoidance radius for obstacles detected by proximity sensors. Typically pretty close
  // @Units: m
--]]
PARAM.MARGIN_PRX  = bind_add_param('MARGIN_PRX', 11, 50)

--[[
    // @Param: DAA_BR_RATIO
    // @DisplayName: DAA margin ratio for BendyRuler to change bearing significantly
    // @Description:  DAA BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is at least this much.
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
--]]
PARAM.BR_RATIO = bind_add_param('BR_RATIO', 12, 1.5)

--[[
    // @Param: DAA_BR_ANGLE
    // @DisplayName: BendyRuler's bearing change resistance threshold angle
    // @Description:  DAA BendyRuler will resist changing current bearing if the change in bearing is over this angle
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
--]]
PARAM.BR_ANGLE = bind_add_param('BR_ANGLE', 13, 45)

--[[
    // @Param: DAA_AVD_ALT
    // @DisplayName: The altitude to loiter to when avoiding a crewed aircraft
    // @Description:  DAA will loiter and descent to this altitude if a crewed aircraft is detected within DAA_MARGIN_CA of the vehicle. Ignored if zero (0).
    // @Range: 20 5000
    // @Increment: 5
    // @User: Standard
--]]
PARAM.AVD_ALT = bind_add_param('AVD_ALT', 14, 50)

--[[
    // @Param: DAA_AVD_ALT_TP
    // @DisplayName: The frame of the DAA_AVD_ALT
    // @Description:  DAA will loiter and descent to DAA_AVD_ALT in this frame.
    // @Values: 0:Absolute,1:Above Home,2:Above Origin,3:Above Terrain
    // @User: Standard
--]]
PARAM.AVD_ALT_TP = bind_add_param('AVD_ALT_TP', 15, 3)

--[[
    // @Param: DAA_AVD_ALERT
    // @DisplayName: Alert for DAA Avoidance
    // @Description: Alert or not Alert
    // @Values: 0: None, 1: Alert
    // @User: Standard
--]]
PARAM.AVD_ALERT = bind_add_param('AVD_ALERT', 16, 1)

--[[
    // @Param: DAA_AVD_ACTION
    // @DisplayName: Action for DAA Avoidance
    // @Description: Action for DAA Avoidance
    // @Values: 0: None, 1: Avoid
    // @User: Standard
--]]
PARAM.AVD_ACTION = bind_add_param('AVD_ACTION', 17, 1)

--[[
    // @Param: DAA_MARGIN_ALT
    // @DisplayName: Altitude fence avoidance margin
    // @Description: Proactive buffer (in metres) inside the safe altitude-fence limits at which DAA starts clamping the commanded altitude. The plane levels off this far inside the safe limit. Avoidance kicks in for FENCE_ALT_MAX (FENCE_TYPE bit 0) and/or FENCE_ALT_MIN (FENCE_TYPE bit 3) when enabled.
    // @Units: m
    // @Range: 0 200
    // @Increment: 1
    // @User: Standard
--]]
PARAM.MARGIN_ALT = bind_add_param('MARGIN_ALT', 18, 20)

--[[
    // @Param: DAA_ALT_HYST_M
    // @DisplayName: Altitude fence avoidance hysteresis
    // @Description: Hysteresis band (in metres) for altitude-fence avoidance. Once engaged, avoidance stays engaged until the altitude is this far back inside the safe band, preventing chatter as the plane levels off. Also sets how far before the level-off altitude the notice is announced.
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
--]]
PARAM.ALT_HYST_M = bind_add_param('ALT_HYST_M', 19, 10)

--[[
    // @Param: DAA_ALT_COOL_S
    // @DisplayName: Altitude fence alert cooldown
    // @Description: Minimum time (in seconds) between altitude-fence "levelling off" notices, so brief re-engagements do not re-spam the GCS. Does not affect the avoidance itself.
    // @Units: s
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
--]]
PARAM.ALT_COOL_S = bind_add_param('ALT_COOL_S', 20, 15)

--[[
    // @Param: DAA_HEADING_INC
    // @DisplayName: Avoidance heading search increment
    // @Description: Angular step (in degrees) used when searching candidate headings around the target bearing for a collision-free path. Smaller values search more finely but cost more CPU per avoidance update.
    // @Units: deg
    // @Range: 0.5 45
    // @Increment: 0.5
    // @User: Advanced
--]]
DEFAULT_HEADING_INC_DEG = 1.5
PARAM.HEADING_INC = bind_add_param('HEADING_INC', 21, DEFAULT_HEADING_INC_DEG)

--[[
    // @Param: DAA_WIND_MIN
    // @DisplayName: Minimum wind speed for wind-aware avoidance
    // @Description: Minimum wind speed (in m/s) before the avoidance look-ahead accounts for wind-driven drift while turning onto a candidate heading. Below this the still-air path is used, so calm-air behaviour is unchanged. Set very high to disable the wind-aware path entirely.
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.5
    // @User: Advanced
--]]
PARAM.WIND_MIN = bind_add_param('WIND_MIN', 22, 2.0)

--[[
    // @Param: DAA_WIND_MARG
    // @DisplayName: Wind-scaled fence margin
    // @Description: Extra fence avoidance margin added per m/s of wind above DAA_WIND_MIN, in metres per m/s. This widens the commanded standoff from fences in wind so the controller has buffer to absorb cross-track drift and is less likely to be blown across the boundary. 0 disables wind scaling. The extra margin is DAA_WIND_MARG * max(0, wind_speed - DAA_WIND_MIN).
    // @Units: s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
PARAM.WIND_MARG = bind_add_param('WIND_MARG', 23, 5.0)

--[[
    // @Param: DAA_SLEW_DPS
    // @DisplayName: Avoidance heading slew rate
    // @Description: Maximum rate the commanded avoidance heading is allowed to change, in degrees per second. Rate-limiting the avoidance heading smooths the oscillation a per-cycle bendy ruler produces against moving obstacles and near fences. Bypassed when the estimated time-to-conflict is below DAA_SLEW_URG so an urgent manoeuvre keeps full authority. 0 disables the slew limit.
    // @Units: deg/s
    // @Range: 0 90
    // @Increment: 1
    // @User: Advanced
--]]
PARAM.SLEW_DPS = bind_add_param('SLEW_DPS', 24, 20)

--[[
    // @Param: DAA_SLEW_URG
    // @DisplayName: Avoidance slew urgency time
    // @Description: If the estimated time-to-conflict with a moving obstacle is below this, the DAA_SLEW_DPS heading slew limit is bypassed so the aircraft can turn at full authority. 0 always applies the slew limit.
    // @Units: s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
PARAM.SLEW_URG = bind_add_param('SLEW_URG', 25, 4)

--[[
    // @Param: DAA_SIDE_HOLD
    // @DisplayName: Avoidance side hold time
    // @Description: Once a left/right avoidance side is committed for an obstacle, the opposite side must be preferred by the bendy ruler for at least this long before the aircraft is allowed to switch sides. This stops the left/right flip-flop when avoiding a moving obstacle. 0 disables side commitment.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
--]]
PARAM.SIDE_HOLD = bind_add_param('SIDE_HOLD', 26, 3)

--[[
    // @Param: DAA_CPA_MIN
    // @DisplayName: Avoidance minimum closing speed
    // @Description: Minimum closing speed for a moving obstacle to be treated as a conflict. CPA (Closest Point of Approach) is the predicted minimum separation between the vehicle and a moving obstacle on their current tracks. A moving obstacle whose CPA stays beyond the well-clear distance and which is opening range faster than this is not avoided (it is leaving). 0 avoids regardless of closing speed.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
PARAM.CPA_MIN = bind_add_param('CPA_MIN', 27, 2)

--[[
    // @Param: DAA_TRAP_ACT
    // @DisplayName: Trapped-failsafe action
    // @Description: What to do when the vehicle has been compromised continuously for DAA_TRAP_S - a fence breach, or a crewed aircraft inside the AVD_NMAC_XY/Z near-miss volume. A hung avoidance - running but making no progress toward the navigation target for DAA_HUNG_ALRT_S - is also a compromise, and unlike the fence branch it is never stood down. (Detecting that the bendy ruler is boxed in - no clear heading at all - is a different condition and is not implemented yet.) 0 disables the trapped-failsafe entirely (avoidance just keeps trying). For a VTOL, QLOITER stops forward flight and hovers (zero turn radius) - the safest way out of a tight space. If the aircraft has no VTOL (Q_ENABLE=0) the VTOL options fall back to RTL. Trapped by a fixed obstacle (fence) is sticky (held until the pilot changes mode); trapped by a moving obstacle (a crewed aircraft) recovers to the previous mode after DAA_TRAP_CLR_S. NOTE: the fence branch of the trap stands down when the core fence library will refuse a scripted mode change - i.e. FENCE_ACTION is non-zero AND FENCE_OPTIONS bit0 (DISABLE_MODE_CHANGE) is set - because in that case the core already handles the breach and the trap's mode change would only be denied. The aircraft near-miss branch (AVD_NMAC_XY/Z) is unaffected.
    // @Values: 0:Disabled,1:RTL,2:QRTL,3:QLOITER,4:QLAND
    // @User: Standard
--]]
PARAM.TRAP_ACT = bind_add_param('TRAP_ACT', 28, 1)

--[[
    // @Param: DAA_TRAP_S
    // @DisplayName: Trapped-failsafe trigger time
    // @Description: The vehicle must be compromised continuously for this long before the DAA_TRAP_ACT failsafe fires - a fence breach, or a crewed aircraft inside the AVD_NMAC_XY/Z near-miss volume. Prevents transient clutter from triggering it.
    // @Units: s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Standard
--]]
PARAM.TRAP_S = bind_add_param('TRAP_S', 29, 5)

--[[
    // @Param: DAA_TRAP_CLR_S
    // @DisplayName: Trapped-failsafe recover time
    // @Description: For a trap caused by a MOVING obstacle (a crewed aircraft), resume the previous mode this long after the failsafe fired, on the assumption the obstacle has passed (if it has not, forward flight simply re-triggers the failsafe). A trap caused by a fixed obstacle (fence) is not auto-recovered - it is held until the pilot changes mode.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
--]]
PARAM.TRAP_CLR_S = bind_add_param('TRAP_CLR_S', 30, 4)

--[[
    // @Param: DAA_TRAP_ESC_ACT
    // @DisplayName: Trapped-failsafe escalation action
    // @Description: Action to take when the DAA_TRAP_ACT action would leave the aircraft in the mode it is ALREADY in (e.g. DAA_TRAP_ACT=RTL and the aircraft is already in RTL) - commanding the same mode again would do nothing, so escalate to this instead. Typically the aircraft is trapped mid-RTL and this stops it: QRTL (VTOL return, zero turn radius) or QLOITER (stop & hover) or QLAND. VTOL actions fall back to RTL if there is no VTOL. Set equal to DAA_TRAP_ACT to disable escalation.
    // @Values: 1:RTL,2:QRTL,3:QLOITER,4:QLAND
    // @User: Standard
--]]
PARAM.TRAP_ESC_ACT = bind_add_param('TRAP_ESC_ACT', 31, 2)

--[[
    // @Param: DAA_STALE_S
    // @DisplayName: Traffic-feed staleness warning threshold
    // @Description: When avoiding a network-sourced moving obstacle (ADS-B drone/aircraft) whose position has not been updated for longer than this, a "traffic stale" warning is sent to the GCS - the DAA is acting on lagged data, e.g. from an intermittent telemetry/ADS-B link. A "lost" warning is sent if such an obstacle then disappears (pruned). Fences are on-board and never go stale. Set to 0 to disable the warnings.
    // @Units: s
    // @Range: 0 30
    // @User: Standard
--]]
PARAM.STALE_S = bind_add_param('STALE_S', 32, 3)

--[[
    // @Param: DAA_MARGIN_CA_Z
    // @DisplayName: Vertical margin for crewed aircraft
    // @Description: Vertical avoidance margin for crewed aircraft, over and above the Well Clear vertical separation AVD_WCLR_Z. An aircraft is detected (and the loiter-to-altitude triggered) only while the altitude difference between it and the vehicle is less than AVD_WCLR_Z + this margin. This is the vertical mirror of the horizontal DAA_MARGIN_CA.
    // @Units: m
    // @Range: 0 200
    // @User: Standard
--]]
PARAM.MARGIN_CA_Z = bind_add_param('MARGIN_CA_Z', 33, 30)

--[[
    // @Param: DAA_LTR_COOL_S
    // @DisplayName: Aircraft loiter release delay
    // @Description: Time the aircraft loiter-to-altitude is held after the aircraft was last detected, before releasing back to the mission. Acts as hysteresis so a briefly-dropped or laggy ADS-B feed cannot thrash the vehicle between GUIDED (loiter) and AUTO (mission). Set to 0 to release as soon as the aircraft is no longer detected.
    // @Units: s
    // @Range: 0 60
    // @User: Standard
--]]
PARAM.LTR_COOL_S = bind_add_param('LTR_COOL_S', 34, 10)

--[[
    // @Param: DAA_HUNG_ALRT_S
    // @DisplayName: Hung-avoidance alert time
    // @Description: Avoidance is "hung" when it has run this long without the vehicle getting closer to its navigation target, e.g. a waypoint inside an exclusion-fence standoff that can never be reached while avoiding. Raises a GCS alert, then counts as a compromise for DAA_TRAP_ACT after the DAA_TRAP_S dwell, so with DAA_TRAP_ACT=0 it is alert-only. Never stood down by FENCE_ACTION/FENCE_OPTIONS. Released when the mission moves on or the pilot changes mode. 0 disables. Detects lack of PROGRESS, not being boxed in.
    // @Units: s
    // @Range: 0 300
    // @Increment: 5
    // @User: Standard
--]]
PARAM.HUNG_ALRT_S = bind_add_param('HUNG_ALRT_S', 35, 60)

--[[
    // @Param: DAA_DETECT_M
    // @DisplayName: Obstacle detection horizon
    // @Description: Range within which obstacles are detected and announced. Shortening it costs detection of fences and drones. Crewed traffic is unaffected (AVD_WCLR_XY + DAA_MARGIN_CA). Also sets the altitude-fence anticipation horizon.
    // @Units: m
    // @Range: 100 5000
    // @User: Standard
--]]
PARAM.DETECT_M = bind_add_param('DETECT_M', 36, 1000)

--[[
    // @Param: DAA_PLAN_M
    // @DisplayName: Commanded avoidance target distance
    // @Description: Minimum distance along the chosen bearing at which the commanded avoidance target is placed. It replaces next_WP_loc, which the mission also uses to decide it has reached or passed the waypoint, so shortening it makes the mission skip waypoints and costs fence clearance. Raise before lowering - see planedaa.md.
    // @Units: m
    // @Range: 100 2000
    // @User: Standard
--]]
PARAM.PLAN_M = bind_add_param('PLAN_M', 37, 250)

PARAM.AVD_ENABLE                  = bind_param("AVD_ENABLE")
PARAM.AVD_WCLR_XY                 = bind_param("AVD_WCLR_XY")
PARAM.AVD_WCLR_Z                  = bind_param("AVD_WCLR_Z")
PARAM.AVD_UAV_XY                  = bind_param("AVD_UAV_XY")
PARAM.AVD_NMAC_XY                 = bind_param("AVD_NMAC_XY")
PARAM.AVD_NMAC_Z                  = bind_param("AVD_NMAC_Z")
PARAM.ROLL_LIMIT_DEG              = bind_param("ROLL_LIMIT_DEG")
PARAM.WP_LOITER_RAD               = bind_param("WP_LOITER_RAD")
PARAM.WP_RADIUS                   = bind_param("WP_RADIUS")
-- Roll-rate bound for the bank-aware reversal transition (daacore.lua's
-- location_for_candidate()) - RLL2SRV_RMAX when the operator has set it, otherwise
-- daageo's roll_limit_deg/RLL2SRV_TCONST fallback, since ArduPlane ships RLL2SRV_RMAX at
-- 0 ("rate limit disabled") by default.
PARAM.RLL2SRV_RMAX                = bind_param("RLL2SRV_RMAX")
PARAM.RLL2SRV_TCONST              = bind_param("RLL2SRV_TCONST")

-- Cached locals below are kept ONLY where this file reads the value itself (a startup
-- sanity check in DAA.warnings(), a hot-path comparison, or - like margin_fence_m below -
-- self-referential derivation).  A parameter that exists solely to be forwarded into a
-- module's configure() call is fetched straight from PARAM there instead: see
-- configure_modules() below, and project_planedaa_param_cache_cleanup in memory.
local lookahead_param_m     = PARAM.LKAHD_M:get()
local detect_m              = PARAM.DETECT_M:get()
-- daageo's turn_radius_m()/max_turn_rate_dps() are stateless and take this directly (see
-- fence_margin_fallback_m() below, DAA.warnings(), hung_update(), and DAAV.TrR logging) -
-- no longer module-only now that this file is a direct consumer too, not just a forwarder
-- to a geometry.configure() call.
local roll_limit_deg        = PARAM.ROLL_LIMIT_DEG:get()
-- Fallback ("0 => use the turn radius") is resolved further down, once turn_radius_m
-- exists - see fence_margin_fallback_m() and its call right after.
local margin_fence_m        = PARAM.MARGIN_FENCE:get()
-- refresh_period_ms is the loop period in ms; DAA_UPDATE_RATE is in Hz (floored at 1 Hz to avoid /0)
local refresh_period_ms     = 1000.0 / math.max(PARAM.UPDATE_RATE:get(), 1.0)
local bendy_ratio           = PARAM.BR_RATIO:get()
-- WP_LOITER_RAD is signed: negative selects a counter-clockwise loiter.  Every use here
-- wants the magnitude - as a distance, as a reposition radius, and as a turn-radius stand-in -
-- and the loiter direction is carried separately, so take the sign out once, at the source.
local wp_loiter_rad_m       = math.abs(PARAM.WP_LOITER_RAD:get())
local wp_radius_m           = math.abs(PARAM.WP_RADIUS:get())
local crewed_avoid_alt_m    = PARAM.AVD_ALT:get()
local crewed_avoid_alt_frame = PARAM.AVD_ALT_TP:get()
local daa_alert             = PARAM.AVD_ALERT:get()
local daa_action            = PARAM.AVD_ACTION:get()
local well_clear_xy         = PARAM.AVD_WCLR_XY:get()
local well_clear_z          = PARAM.AVD_WCLR_Z:get()
local uav_clear_xy          = PARAM.AVD_UAV_XY:get()
local near_miss_xy          = PARAM.AVD_NMAC_XY:get()
local near_miss_z           = PARAM.AVD_NMAC_Z:get()
local slew_dps              = PARAM.SLEW_DPS:get()
local side_hold_s           = PARAM.SIDE_HOLD:get()
local trap_act              = PARAM.TRAP_ACT:get()
local trap_s                = PARAM.TRAP_S:get()
local trap_clr_s            = PARAM.TRAP_CLR_S:get()
local trap_esc_act          = PARAM.TRAP_ESC_ACT:get()
local stale_s               = PARAM.STALE_S:get()
local margin_crewed_m       = PARAM.MARGIN_CA:get()
local hung_alrt_s           = PARAM.HUNG_ALRT_S:get()

GRAVITY_MSS = 9.80665
-- A new closest approach to the navigation target has to beat the previous one by at least
-- this much to count as progress, when the turn radius is not usable (no airspeed yet).
-- It only has to be larger than position noise: real progress closes hundreds of metres.
MIN_HUNG_PROGRESS_M = 10.0
-- How long after an avoidance target is dropped a waypoint completion is still credited to
-- it.  The vehicle finishes the waypoint on the NEXT navigation tick after we hand the
-- target back, so requiring avoidance to still be live missed 3 of 4 real skips.
SKIP_AVOID_GRACE_MS = 2000


-- Load a module, reporting a failure in a form that survives the 64-character cap on a
-- logged STATUSTEXT.  Lua's own message is "./scripts/modules/daaobs.lua:12: <text>" and
-- the path alone eats the whole budget before any of the text arrives, so re-order it to
-- line number, then file, then message.  This can only ever cover the MODULES: a syntax
-- error in this file is raised before a line of it runs, and nothing here can catch that.
local function need(name)
    local ok, mod = pcall(require, name)
    if ok then
        return mod
    end
    local msg  = tostring(mod)
    local file, line, rest = msg:match("([^/\\]+%.lua):(%d+):%s*(.*)")
    if file ~= nil then
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s %s %s", line, file, rest))
    else
        -- no file:line in it: a module that was not found, or one that would not fit.
        -- Report what Lua actually said rather than guessing - "not enough memory" and
        -- "module not found" need completely different answers from the operator.
        gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: %s", name, msg:sub(1, 40)))
    end
    error(name .. " load failed", 0)
end

-- Geometry, airframe capability and obstacle lookup live in modules, so that THIS file
-- stays the one an integrator edits to change avoidance policy (see planedaa.md).  Each
-- is aliased into a local below: every call site stays unchanged, and stays a fast upvalue
-- call rather than a table index inside the candidate-heading sweep.
-- assigned once get_mode_string exists, which it needs; configure_modules() below
-- refers to it, so it has to be in scope from here
local loiteralt
local core

-- daageo is stateless (every function is a pure function of its arguments, see that file),
-- so it is required directly rather than instantiated - there is no shared state to keep
-- consistent between this file, daacore and daaobs.
local geometry  = need("daageo")
local obstacles = need("daaobs").new()

-- The obstacle taxonomy belongs to the module that classifies obstacles, so it is defined
-- there and read back here.  This file only names the four members it actually uses, which
-- keeps the other twelve - and the whole twenty-member ADSB_EMITTER table - out of this
-- chunk's parser budget.  See planedaa.md, "Distinct names are a budget".
local OBSTACLE_TYPE = obstacles.OBSTACLE_TYPE

local max_turn_rate_dps         = geometry.max_turn_rate_dps
local turn_radius_m             = geometry.turn_radius_m
local wrap_180                  = geometry.wrap_180
local locations_equal           = geometry.locations_equal
local pretty_obstacle_type      = obstacles.pretty_obstacle_type
local obstacle_report_distance  = obstacles.obstacle_report_distance

-- Real achievable turn radius at ROLL_LIMIT_DEG and AIRSPEED_CRUISE - the physical margin
-- "DAA_MARGIN_FENCE = 0" promises ("use the turn radius so the fence standoff = one turn").
-- WP_LOITER_RAD was used for this until 2026-09-03: it is a loiter-geometry setting, not an
-- avoidance one, and can be far larger than the achievable turn radius (measured 150 m vs a
-- real turn radius under 50 m on one aircraft) - large enough to silently swallow a waypoint
-- placed well outside any margin the operator actually intended, with no warning at all.
-- Falls back to wp_loiter_rad_m only when no cruise speed is configured yet (turn_radius_m
-- returns 0 for airspeed <= 0), so the standoff is never literally zero.  Shared by the
-- initial value below, the 5 s parameter refresh, and DAA.warnings()'s own turn-radius
-- sanity checks - one formula, not three copies of it.
local function fence_margin_fallback_m()
    local achievable_turn_radius_m = turn_radius_m(param:get('AIRSPEED_CRUISE') or 0, roll_limit_deg)
    if achievable_turn_radius_m <= 0 then achievable_turn_radius_m = wp_loiter_rad_m end
    return achievable_turn_radius_m
end
if margin_fence_m <= 0 then margin_fence_m = fence_margin_fallback_m() end

-- See daageo's roll_rate_dps() for the rationale (RLL2SRV_RMAX, falling back to
-- roll_limit_deg / RLL2SRV_TCONST since RMAX ships at 0).  Refreshed alongside
-- roll_limit_deg every 5 s below, since the fallback depends on it.
local roll_rate_dps = geometry.roll_rate_dps(PARAM.RLL2SRV_RMAX:get(), roll_limit_deg, PARAM.RLL2SRV_TCONST:get())

local bearing_inc_deg = PARAM.HEADING_INC:get() or DEFAULT_HEADING_INC_DEG
if bearing_inc_deg <= 0 then
    bearing_inc_deg = DEFAULT_HEADING_INC_DEG
end

-- Push the cached parameter values into the modules.  Called at startup and from the 5 s
-- parameter refresh, so an operator changing a margin in flight reaches the modules
-- exactly as it reaches the rest of the applet.
-- Parameters read ONLY here (nowhere else in this file) are fetched straight from PARAM
-- inline rather than kept as a module-level cached local - one fewer distinct name for the
-- parser budget, at the cost of one :get() call every 5 s instead of an upvalue read
-- (measured: :get() is a C binding call, but this only runs on the 5 s parameter refresh,
-- never per-cycle or per-probe, so the runtime cost is immaterial).  See
-- project_planedaa_param_cache_cleanup in memory.
local function configure_modules()
    obstacles.configure({
        margin_fence_m      = margin_fence_m,
        margin_crewed_m     = margin_crewed_m,
        margin_uav_m        = PARAM.MARGIN_UAV:get(),
        margin_ais_m        = PARAM.MARGIN_AIS:get(),
        margin_proximity_m  = PARAM.MARGIN_PRX:get(),
        well_clear_xy       = well_clear_xy,
        uav_clear_xy        = uav_clear_xy,
        wind_min_ms         = PARAM.WIND_MIN:get(),
        wind_margin_per_ms  = PARAM.WIND_MARG:get(),
    })
    core.configure({
        alt_cool_ms       = PARAM.ALT_COOL_S:get() * 1000,
        alt_hyst_m        = PARAM.ALT_HYST_M:get(),
        bearing_inc_deg   = bearing_inc_deg,
        bendy_angle       = PARAM.BR_ANGLE:get(),
        bendy_ratio       = bendy_ratio,
        cpa_min_ms        = PARAM.CPA_MIN:get(),
        detect_m          = detect_m,
        margin_alt_m      = PARAM.MARGIN_ALT:get(),
        margin_crewed_m   = margin_crewed_m,
        margin_fence_m    = margin_fence_m,
        margin_vertical_m = PARAM.MARGIN_CA_Z:get(),
        plan_m            = PARAM.PLAN_M:get(),
        roll_limit_deg    = roll_limit_deg,
        roll_rate_dps     = roll_rate_dps,
        side_hold_s       = side_hold_s,
        slew_dps          = slew_dps,
        slew_urg_s        = PARAM.SLEW_URG:get(),
        well_clear_xy     = well_clear_xy,
        well_clear_z      = well_clear_z,
        wp_loiter_rad_m   = wp_loiter_rad_m,
        lookahead_param_m = lookahead_param_m,
    })
    loiteralt.configure({
        loiter_cool_ms  = PARAM.LTR_COOL_S:get() * 1000,
        wp_loiter_rad_m = wp_loiter_rad_m,
    })
end

-- Floor for SCR_VM_I_COUNT below which DAA.warnings() complains. Overrunning the VM
-- instruction budget kills the script outright (and then blocks arming), so this is a
-- safety-relevant setting, not a tuning one. See planedaa.md for the recommended value.
MIN_VM_I_COUNT = 150000

-------------------------------------------------------------------------------
--- Vehicle State stored in local variables to reduce api calls
-------------------------------------------------------------------------------

local current_loc           = ahrs:get_position()
local current_mode          = vehicle:get_mode()
local in_fw_flight          = true

local now_ms                = millis()
local now_params_ms         = now_ms
local now_avoiding_ms       = now_ms
local now_obstacle_ms       = now_ms
local now_aircraft_ms       = now_ms
local now_loitering_ms      = now_ms

-------------------------------------------------------------------------------
--- Lua Modules
-------------------------------------------------------------------------------
---
---
local mavlink_wrappers = require("mavlink_wrappers")

-------------------------------------------------------------------------------
--- Utility methods
-------------------------------------------------------------------------------
---
---

-- Mode number -> name, as a table rather than a chain of twenty comparisons: one index
-- instead of up to twenty tests, and the mapping is readable as data.  A mode that is not
-- listed reports its number, which is what the pilot needs to look it up.
local MODE_NAME = {
    [PLANE_MODE.MANUAL]             = "Manual",
    [PLANE_MODE.CRUISE]             = "Cruise",
    [PLANE_MODE.FBWA]               = "FBWA",
    [PLANE_MODE.FBWB]               = "FBWB",
    [PLANE_MODE.AUTOTUNE]           = "Autotune",
    [PLANE_MODE.TAKEOFF]            = "Takeoff",
    [PLANE_MODE.AUTO]               = "Auto",
    [PLANE_MODE.RTL]                = "RTL",
    [PLANE_MODE.LOITER]             = "Loiter",
    [PLANE_MODE.GUIDED]             = "Guided",
    [PLANE_MODE.THERMAL]            = "Thermal",
    [PLANE_MODE.AVOID_ADSB]         = "Avoid ADSB",
    [PLANE_MODE.AUTOLAND]           = "Autoland",
    [PLANE_MODE.QSTABILIZE]         = "Q Stabilize",
    [PLANE_MODE.QHOVER]             = "Q Hover",
    [PLANE_MODE.QLOITER]            = "Q Loiter",
    [PLANE_MODE.QLAND]              = "Q Land",
    [PLANE_MODE.QAUTOTUNE]          = "Q Autotune",
    [PLANE_MODE.LOITER_ALT_QLAND]   = "Loiter Alt Q Land",
}

local function get_mode_string(mode)
    return MODE_NAME[mode] or string.format("mode: %d", mode)
end

-- the QuadPlane singleton exists only on quadplane builds (nil on a plain plane).
-- in_vtol_mode()/in_assisted_flight() are available()-guarded, so they are safe to
-- call even at Q_ENABLE=0; requiring Q_ENABLE also treats a Q_ENABLE=0 plane as
-- always fixed-wing.
local quadplane_enabled = quadplane ~= nil and (param:get('Q_ENABLE') or 0) > 0

-- keep local copies of parameter values that the user might change so update ever 5 seconds
local function get_vehicle_state()

    current_loc         = ahrs:get_position()
    -- the obstacle module reports ranges from the aircraft, so it needs where we are
    obstacles.update_state(current_loc)
    current_mode        = vehicle:get_mode()
    if quadplane_enabled then
        -- avoid only in clean fixed-wing forward flight: NOT a VTOL mode and NOT under
        -- VTOL assist. During a transition or Q_ASSIST the vehicle flies forward but the
        -- transition state reports TRANSITION_TO_FW, where hijacking the nav target is
        -- unsafe; in_assisted_flight() covers both. (== the old MAV_VTOL_STATE == FW.)
        in_fw_flight    = not quadplane:in_vtol_mode() and not quadplane:in_assisted_flight()
    else
        -- not a quadplane, so we are always in fixed wing flight
        in_fw_flight    = true
    end

    now_ms = millis()

    -- after current_mode and now_ms are current, not before: the loiter watches for the
    -- pilot leaving GUIDED and times its cool-down off these
    loiteralt.update_state(current_loc, current_mode, now_ms)

    -- refresh parameters every 5 seconds, its not that urgent we know about changs
    if (now_ms - now_params_ms) > 5000 then
        lookahead_param_m     = PARAM.LKAHD_M:get()
        detect_m              = PARAM.DETECT_M:get()
        roll_limit_deg        = PARAM.ROLL_LIMIT_DEG:get()
        roll_rate_dps         = geometry.roll_rate_dps(PARAM.RLL2SRV_RMAX:get(), roll_limit_deg, PARAM.RLL2SRV_TCONST:get())
        margin_fence_m        = PARAM.MARGIN_FENCE:get()
        refresh_period_ms     = 1000.0 / math.max(PARAM.UPDATE_RATE:get(), 1.0)
        bendy_ratio           = PARAM.BR_RATIO:get()
        wp_loiter_rad_m       = math.abs(PARAM.WP_LOITER_RAD:get())
        wp_radius_m           = math.abs(PARAM.WP_RADIUS:get())
        -- after wp_loiter_rad_m above, since the fallback's own secondary fallback reads it
        if margin_fence_m <= 0 then margin_fence_m = fence_margin_fallback_m() end
        crewed_avoid_alt_m    = PARAM.AVD_ALT:get()
        crewed_avoid_alt_frame  = PARAM.AVD_ALT_TP:get()
        daa_alert             = PARAM.AVD_ALERT:get()
        daa_action            = PARAM.AVD_ACTION:get()

        bearing_inc_deg       = PARAM.HEADING_INC:get() or DEFAULT_HEADING_INC_DEG
        if bearing_inc_deg <= 0 then
            bearing_inc_deg   = DEFAULT_HEADING_INC_DEG
        end

        well_clear_xy         = PARAM.AVD_WCLR_XY:get()
        well_clear_z          = PARAM.AVD_WCLR_Z:get()
        uav_clear_xy          = PARAM.AVD_UAV_XY:get()
        near_miss_xy          = PARAM.AVD_NMAC_XY:get()
        near_miss_z           = PARAM.AVD_NMAC_Z:get()
        slew_dps              = PARAM.SLEW_DPS:get()
        side_hold_s           = PARAM.SIDE_HOLD:get()
        trap_act              = PARAM.TRAP_ACT:get()
        trap_s                = PARAM.TRAP_S:get()
        trap_clr_s            = PARAM.TRAP_CLR_S:get()
        trap_esc_act          = PARAM.TRAP_ESC_ACT:get()
        stale_s               = PARAM.STALE_S:get()
        hung_alrt_s           = PARAM.HUNG_ALRT_S:get()
        margin_crewed_m       = PARAM.MARGIN_CA:get()

        -- the modules cache these too, so push the new values through - LAST, after every
        -- local above is refreshed.  This used to run partway through the block, before
        -- bearing_inc_deg (among others) was refreshed, so core.configure() was handed a
        -- one-refresh-cycle-stale bearing_inc_deg every time after the first; moving the
        -- call to the end fixes that incidentally.
        configure_modules()

        now_params_ms         = now_ms
    end
end




-------------------------------------------------------------------------------
-- LOITER ALTITUDE - Loiter right or left to (usually) lose altitude to avoid an obstacle (usually a crewed aircraft)
-------------------------------------------------------------------------------
-- The avoidance MECHANISM.  It answers "where can I safely go?" and decides nothing about
-- what to do with the answer - alerting, commanding and the failsafes all stay in this
-- file, which is what keeps this the only file an integrator has to edit.
core = need("daacore").new({
    obstacles           = obstacles,
    MAV_SEVERITY        = MAV_SEVERITY,
})

-- The altitude loiter is a POLICY implementation living behind a small seam: five members
-- (.active, start, stop, update, aircraft_seen).  Point this at a different module - your
-- own daaltr2 - and nothing else in this file changes.  See planedaa.md.
loiteralt = need("daaltr").new({
    PLANE_MODE              = PLANE_MODE,
    ALT_FRAME               = ALT_FRAME,
    MAV_DO_REPOSITION_FLAGS = MAV_DO_REPOSITION_FLAGS,
    MAV_SEVERITY            = MAV_SEVERITY,
    get_mode_string         = get_mode_string,
    mavlink_wrappers        = mavlink_wrappers,
})

-- Now that every module exists, push the cached parameter values into all of them.  The
-- 5 s refresh in get_vehicle_state() calls this again whenever a parameter changes.
configure_modules()

-------------------------------------------------------------------------------
--- DAA (Detect, Alert, Avoid) management class
-------------------------------------------------------------------------------
local DAA = {
   enabled = false,
}
(function ()
    local navigating            = false;
    local active                = true;
    local current_loc           = ahrs:get_position() -- luacheck: ignore current_loc
    -- Placeholder values only: get_vehicle_state()/DAA.get_vehicle_state() overwrite all
    -- six of these on the first real cycle, in one place, before anything reads them -
    -- see the comment there.  Kept here only because Lua requires an initial value for
    -- each upvalue; wind is left literally unmeasured (0.0/0.0) rather than fetched
    -- again from ahrs, so there is exactly one fetch site for wind/airspeed/groundspeed/
    -- roll in the whole file, not two.
    local groundspeed_ms        = 0.0
    local airspeed_ms           = 0.0
    local ground_course_deg     = 0.0
    local wind_dir_rad          = 0.0
    local wind_speed            = 0.0
    local current_roll_deg      = 0.0
    local obstacle_avoiding     = nil
    local aircraft_avoiding     = nil
    local trap_active           = false         -- trapped-failsafe is currently controlling the vehicle
    local trap_since_ms         = uint32_t(0)   -- when the trap condition began (for DAA_TRAP_S)
    local trap_trigger_ms       = uint32_t(0)   -- when the failsafe fired (for DAA_TRAP_CLR_S recovery)
    local trap_dynamic          = false         -- trap from a moving obstacle (recoverable) vs a fence (sticky)
    local trap_prev_mode        = -1            -- mode to restore on recovery
    local trap_fs_mode          = -1            -- the failsafe mode we commanded
    local trap_hung             = false         -- trap from a hung avoidance (released when the mission moves on)
    local hung_active           = false         -- avoidance has made no progress for DAA_HUNG_ALRT_S
    local hung_best_m           = nil           -- closest we have been to the navigation target this episode
    local hung_since_ms         = uint32_t(0)   -- when that closest approach last improved
    local hung_target_loc       = nil           -- the navigation target the progress is measured against
    local hung_nav_index        = -1            -- mission index when a hung trap fired (release when it changes)
    local skip_nav_index        = -1            -- mission index last cycle, to notice the mission moving on
    local skip_target_loc       = nil           -- navigation target last cycle, to measure how short we left it
    local skip_avoid_ms         = uint32_t(0)   -- last time an avoidance target was commanded
    local previous_label        = ""
    local avoiding_label        = ""
    -- laggy/dropped traffic-feed watchdog (network-fed moving obstacles carry an update
    -- timestamp; a fence's is always fresh).  Threshold is DAA_STALE_S (0 disables).
    local feed_watch_label      = ""            -- label of the moving obstacle we are tracking ("" = none)
    local feed_is_stale         = false         -- its last-seen update was stale
    local feed_stale_warn_ms    = uint32_t(0)   -- throttle for the "traffic stale" GCS text
    local LoWC_active           = false
    local LoWC_label            = ""
    local NMAC_active           = false
    local NMAC_label            = ""

    local update_target_location_save_loc = nil -- this is the saved current_target for use by update_target_location ONLY
    local navigation_target_loc = nil           -- this is where the vehicle is trying to get to (i.e. next waypoint if no avoidance)
    local daa_target_loc        = nil           -- this is where the DAA is currently trying to go in order to avoid obstacles (nil if not avoiding)
    -- Crosstrack (horizontal path-following) state saved from just before avoidance
    -- disabled it, restored when avoidance ends - see set_avoid_location() below.
    -- Defaults true (ArduPlane's normal AUTO/GUIDED behaviour) so a firmware without
    -- the set_crosstrack_enabled()/get_crosstrack_enabled() bindings degrades to
    -- "always leave crosstrack alone" rather than an undefined saved value.
    local saved_crosstrack_enabled = true

    -- Last DAA_LKAHD_M this file announced a change for - the working value the sweep
    -- actually probes at is core.update_state()'s own copy (kept in sync via
    -- lookahead_param_m in configure_modules()); this is only a "did it change" sentinel
    -- for the GCS message below.
    local lookahead_set_m   = lookahead_param_m

    -- Speed and heading of the horizontal wind, from the AHRS wind estimate.  Computed
    -- directly from the x/y components rather than through an intermediate Vector2f:
    -- this runs every active cycle and a userdata allocation here bought nothing that
    -- math.sqrt/math.atan don't already give for free.
    local function calculate_windspeed()
        local wind_3d = ahrs:get_wind()
        if wind_3d == nil then                  -- get_wind returns nil when there is no valid estimate: treat as calm
            return 0.0, 0.0
        end
        local wx, wy = wind_3d:x(), wind_3d:y()
        return math.sqrt(wx * wx + wy * wy), math.atan(wy, wx)
    end

    -- methods to log DAA results DAAD = Detect, DAAA = Alert, DAAV = aVoid

    local function log_avoid(obstacle, target_loc)
        -- Both can be nil independently: avoid_obstacle()'s "done" branch calls this with
        -- obstacle == nil on the very cycle avoidance ends, before daa_target_loc is cleared
        -- (that happens in set_avoid_location(nil), called separately) - so target_loc alone
        -- being non-nil does not guarantee obstacle is too.  Reproduced live: "Excl. Circle
        -- done" followed immediately by "attempt to index a nil value (local 'obstacle')" at
        -- the old obstacle.distance_m read below, which aborted the rest of that cycle's
        -- DAA.avoid() - including any state bookkeeping still to run after this call.
        if target_loc == nil or obstacle == nil then
            return
        end
        -- TrR records the turn radius the sizing assumed at this moment, so a flight log can
        -- be checked against what the aircraft actually flew: compare it with the achieved
        -- radius over the same window (airspeed / turn rate, or ATT.Roll).  That is the open
        -- question behind using the roll limit rather than WP_LOITER_RAD for the standoffs.
        local status, err = pcall(logger.write, logger, "DAAV",
            'DstO,TLat,TLng,TAlt,TFra,DstH,DstZ,ObjT,Age,TrR,Hung',
            'fLLfBffBffB',                      -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mm-sm-',                      -- Units (D=lat deg, U=lng deg, m=meter, s=second)
            '-GG--------',                      -- Multipliers (G=1e-7 for L types)
            obstacle.distance_m,                -- DstO - Distance to found obstacle in meters
            target_loc:lat(),                   -- TLat - Latitude of DAA target in degrees
            target_loc:lng(),                   -- TLng - Longitude of DAA target in degrees
            target_loc:alt() * 0.01,            -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame(),         -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            obstacle.distance_xy,               -- DstH - Horizontal distance to the obstacle
            obstacle.distance_z,                -- DstZ - Vertical distance to the aircraft (+ve is up),
            obstacle.type,                      -- ObjT - the type of the obstacle as an OBSTACLE_TYPE
            obstacle.timestamp_ms and ((now_ms:tofloat() - obstacle.timestamp_ms) * 0.001) or 0,  -- Age - obstacle position age in s (0 = fresh/on-board)
            turn_radius_m(airspeed_ms, roll_limit_deg), -- TrR - achievable turn radius at ROLL_LIMIT_DEG and the current airspeed (0 = no usable airspeed)
            hung_active and 1 or 0              -- Hung - avoidance has made no progress toward the navigation target for DAA_HUNG_ALRT_S
        )
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log avoid:" .. tostring(err) )
        end
    end

    -- DAAS: per-cycle avoidance-smoothing trace, logged every avoidance cycle for a
    -- non-fixed (moving) obstacle.  It captures the bearing at each smoothing stage so
    -- a flight log shows the raw wiggle and whether the smoothing damps it:
    --   HdD = direct bearing to the target
    --   HdR = raw bendy-ruler bearing (per-cycle, un-smoothed)
    --   HdS = after clearance hysteresis (this is the pre-smoothing command)
    --   HdC = final commanded bearing we fly (after side-commit + slew limit)
    -- plus the decision state (Sid committed side, Flp side-flip pending, Urg slew
    -- bypassed) and the motion assessment (Cls closing speed, CPA horizontal miss,
    -- TTC time-to-conflict, PsB pass-behind side, Dst obstacle range, Typ type).
    -- Compare HdR vs HdS vs HdC across time to see the wiggle and the smoothing effect.

    -- Sanity-check the DAA parameters against each other and related vehicle params, and
    -- flag disabled features. Advisory only (GCS text, nothing is changed). Fires on every
    -- enable (incl. switch flips). Messages are kept short (STATUSTEXT truncates ~50 chars).
    function DAA.warnings()
        local function warn(sev, msg)
            gcs:send_text(sev, SCRIPT_NAME_SHORT .. ": " .. msg)
        end
        local W = MAV_SEVERITY.WARNING
        local I = MAV_SEVERITY.NOTICE
        local function vtol_act(a) return a == 2 or a == 3 or a == 4 end
        local have_vtol   = (param:get('Q_ENABLE') or 0) > 0
        local adsb_type   = param:get('ADSB_TYPE') or 0
        local cruise_ms   = param:get('AIRSPEED_CRUISE') or 0

        -- The turn radius these standoffs are compared against is the achievable one at the
        -- roll limit, not WP_LOITER_RAD: a standoff only has to clear the turn the aircraft
        -- will actually fly while avoiding, and it is not loitering when it does that.
        -- Same formula (and the same WP_LOITER_RAD fallback for no cruise speed set) as
        -- fence_margin_fallback_m() above, which is what margin_fence_m itself may already
        -- be - one shared computation, not two that could quietly drift apart.
        local achievable_turn_radius_m = fence_margin_fallback_m()
        if achievable_turn_radius_m > 0 and margin_fence_m < achievable_turn_radius_m then
            warn(W, string.format("MARGIN_FENCE %.0f < turn %.0f: fences may thrash", margin_fence_m, achievable_turn_radius_m)) end
        if achievable_turn_radius_m > 0 and uav_clear_xy < achievable_turn_radius_m then
            warn(W, string.format("AVD_UAV_XY %.0f < turn %.0f: tight drone avoid", uav_clear_xy, achievable_turn_radius_m)) end
        -- margin ordering: the aircraft near-miss (NMAC) must sit inside the aircraft
        -- well-clear standoff. NMAC is an aircraft-only boundary, so it is NOT compared
        -- against the drone standoff (AVD_UAV_XY).
        if near_miss_xy >= well_clear_xy then
            warn(W, "NMAC_XY >= WCLR_XY: nearmiss outside wellclr") end
        if near_miss_z >= well_clear_z then
            warn(W, "NMAC_Z >= WCLR_Z: vert nearmiss>wellclr") end
        -- lookahead must give room to react
        if achievable_turn_radius_m > 0 and lookahead_param_m < 3 * achievable_turn_radius_m then
            warn(W, string.format("LKAHD_M %.0f < 3x turn %.0f: reacts late", lookahead_param_m, achievable_turn_radius_m)) end
        if bendy_ratio > 1.8 then
            warn(W, string.format("BR_RATIO %.1f > 1.8: fence-follow unstable", bendy_ratio)) end
        -- trapped-failsafe consistency
        if trap_act ~= 0 and not have_vtol and (vtol_act(trap_act) or vtol_act(trap_esc_act)) then
            warn(W, "trap VTOL action but Q_ENABLE=0 -> RTL") end
        if trap_act ~= 0 and trap_esc_act == trap_act then
            warn(I, "TRAP_ESC_ACT = TRAP_ACT: no escalation") end
        -- slew limit that can never bind (exceeds the achievable turn rate)
        if cruise_ms > 1 and slew_dps > 0 then
            local turn_rate = max_turn_rate_dps(cruise_ms, roll_limit_deg)
            if turn_rate > 0 and slew_dps > turn_rate * 1.5 then
                warn(I, string.format("SLEW_DPS %.0f > turn rate %.0f: no effect", slew_dps, turn_rate)) end
        end
        -- disabled features
        if (PARAM.AVD_ENABLE:get() or 0) ~= 1 then
            warn(W, "AVD_ENABLE != 1: traffic avoidance OFF")
        elseif adsb_type == 0 then
            warn(W, "ADSB_TYPE = 0: no traffic source")
        end
        -- Lua VM instruction budget. The bendy-ruler sweep is by far the most expensive thing
        -- this script does, and its worst case (boxed in, every candidate heading probed) is
        -- hit exactly when avoidance matters most. Overrunning SCR_VM_I_COUNT does not skip a
        -- cycle - the VM kills the script, so avoidance is gone for the rest of the flight and
        -- the sticky error then fails the pre-arm check. Warn while the pilot can still fix it.
        local vm_i_count = param:get('SCR_VM_I_COUNT') or 0
        if vm_i_count < MIN_VM_I_COUNT then
            -- keep this inside the 50-char STATUSTEXT limit (6 of which are the "pDAA: " prefix)
            warn(W, string.format("SCR_VM_I_COUNT %.0f < %.0f: may stop DAA", vm_i_count, MIN_VM_I_COUNT)) end
        if daa_action == 0 then warn(W, "AVD_ACTION = 0: avoidance manoeuvres OFF") end
        if trap_act == 0 then warn(I, "TRAP_ACT = 0: trap failsafe OFF") end
        if slew_dps == 0 and side_hold_s == 0 then warn(I, "smoothing OFF (SLEW_DPS=0 SIDE_HOLD=0)") end

    end

    function DAA.disable()
        DAA.enabled = false
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " disabled")
    end
    function DAA.enable()
        DAA.enabled = true
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " enabled")
        DAA.warnings()
    end

    -- Take the mechanism's threat report.  This has to be a method rather than an
    -- assignment in update(): update() runs at file scope, so assigning there would create
    -- globals while every policy function below reads these class locals.
    function DAA.take_report(target_loc, obstacle, aircraft)
        obstacle_avoiding = obstacle
        aircraft_avoiding = aircraft
        return target_loc
    end

    --return true if we are in a state where DAA can apply
    function DAA.isactive()
        return DAA.enabled and active and arming:is_armed()
    end

    -- populate some local values with a static/consistent picture of the vehicle state.
    -- fetched_current_loc is the position get_vehicle_state() (file scope) already read
    -- this same cycle - passed in rather than re-fetched, so ahrs:get_position() is only
    -- ever called once per cycle instead of once per get_vehicle_state function.
    function DAA.get_vehicle_state(fetched_current_loc)
        local current_target_loc = vehicle:get_target_location()

        active      = true;
        current_loc = fetched_current_loc

        -- get_vehicle_state() re-reads DAA_LKAHD_M into lookahead_param_m every 5 s;
        -- announce it to the GCS only when it actually changes.
        if lookahead_param_m ~= lookahead_set_m then
            lookahead_set_m = lookahead_param_m
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(
                ": probe distance now %.0f m", lookahead_param_m))
        end

        if OAScripting == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " OAScripting object is nil!")
            active = false
            return
        end

        if current_loc == nil or current_target_loc == nil then
            -- no position or not navigating
            navigation_target_loc   = nil
            daa_target_loc          = nil
            if navigating then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "NOT NAVIGATING")
                navigating = false
            end
            active = false
            return
        end

        -- if we got here we have a current location (AHRS active) and a current navigation target
        -- No :copy() here: current_target_loc is a fresh Location this vehicle:get_target_location()
        -- call built for us, nothing else this cycle holds a reference to it, and
        -- update_target_location_save_loc is only ever READ (:get_alt_frame(), passed as the
        -- "current" argument into vehicle:update_target_location()) - it is never the object that
        -- gets change_alt_frame()'d in place, so aliasing it costs nothing.  Contrast
        -- navigation_target_loc below: that copy IS load-bearing, see its own comment.
        update_target_location_save_loc = current_target_loc

        -- if the navigation target has changed to some other target not the DAA target, it must be vehicle navigation
        if navigation_target_loc == nil or
            (not locations_equal(navigation_target_loc, current_target_loc) and
                not locations_equal(daa_target_loc, current_target_loc)) then
            -- the vehicle navigation code has changed it's target
            -- :copy() IS needed here (unlike update_target_location_save_loc above):
            -- navigation_target_loc persists across many cycles, and set_avoid_location()'s
            -- revert path calls update_target_location(navigation_target_loc), which mutates
            -- whatever object is passed in via :change_alt_frame() - so navigation_target_loc
            -- must be its own private object, not an alias shared with anything else.
            navigation_target_loc = current_target_loc:copy()
        end

        -- one fetch, not two: :length() and :angle() both read the same vector
        local groundspeed_vec        = ahrs:groundspeed_vector()
        groundspeed_ms              = groundspeed_vec:length()
        ground_course_deg           = wrap_180(math.deg(groundspeed_vec:angle()))
        airspeed_ms                 = ahrs:airspeed_EAS() or groundspeed_ms
        -- Calculate wind direction and speed
        wind_speed, wind_dir_rad    = calculate_windspeed()
        -- get_roll() is deprecated - use the _rad form.  Needed for the fence
        -- reversal-in-progress latch (project_planedaa_reversal_awareness): the sweep
        -- must know which way the aircraft is ACTUALLY banked, not just which way it was
        -- last told to go.
        current_roll_deg            = math.deg(ahrs:get_roll_rad())

        -- hand the mechanism the picture it searches against, positionally: it keeps its
        -- own copies as locals rather than reading ours through a table, and the sweep
        -- touches them on every one of a hundred-plus probes a cycle, so a table literal
        -- here would be built and thrown away on every single active cycle.
        core.update_state(current_loc, navigation_target_loc, airspeed_ms, groundspeed_ms,
                          ground_course_deg, wind_speed, wind_dir_rad, now_ms, current_roll_deg)
    end


    local function alert_obstacle(alert_target_loc)
        if obstacle_avoiding ~= nil and
                (obstacle_avoiding.type == OBSTACLE_TYPE.FENCE_ALT_MAX or
                 obstacle_avoiding.type == OBSTACLE_TYPE.FENCE_ALT_MIN) then
            -- altitude fences emit their own throttled, edge-triggered notice from
            -- detect_altitude_fence(); don't also raise the generic obstacle ALERT
            return
        end
        if obstacle_avoiding == nil or alert_target_loc == nil then
            previous_label = ""
            return
        end
        -- Report the real range where we can get one.  A fence carries no single location,
        -- so distance_xy falls back to the bendy-ruler PROJECTED distance for fences;
        -- obstacle_report_distance() asks C++ for the true edge distance instead.  This is
        -- announce-time only, so the per-call fence search stays out of the sweep.
        local report_m = obstacle_report_distance(obstacle_avoiding) or obstacle_avoiding.distance_xy
        if report_m > detect_m then
            previous_label = ""
            return
        end
        if obstacle_avoiding.label == previous_label then
            -- still near the same object, no need to spam the user
            return
        end

        if (now_ms - now_obstacle_ms) > 5000 then
            gcs:send_named_string("DAA-ALERT", "obstacle")
            gcs:send_named_string("DAA-OBSTCL", obstacle_avoiding.label)
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT: %s %s %.0fm",
                                obstacle_avoiding.label, pretty_obstacle_type(obstacle_avoiding.type, obstacle_avoiding.sysid),
                                report_m))
            gcs:send_named_float("DAA-DISTXY", report_m)
            gcs:send_named_float("DAA-DISTZ", obstacle_avoiding.distance_z)
            previous_label  = obstacle_avoiding.label
            now_obstacle_ms = now_ms
        end
    end

    local function NMAC_triggered(nmac_obstacle)
        NMAC_active = true
        NMAC_label  = nmac_obstacle.label
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Near Miss: %.0fm/%.0fm",
                        nmac_obstacle.label, nmac_obstacle.distance_xy, nmac_obstacle.distance_z ))
        gcs:send_named_string("DAA-NMAC", "aircraft")
    end

    local function NMAC_cleared()
        if not NMAC_active then
            return
        end
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Near Miss CLEAR",
                        NMAC_label ))
        gcs:send_named_string("DAA-NMACOK", "aircraft")
        NMAC_active = false
        NMAC_label  = ""
    end

    local function LoWC_triggered(lowc_obstacle)
        LoWC_active = true
        LoWC_label  = lowc_obstacle.label
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Loss of Well Clear",
                    lowc_obstacle.label ))
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: LoWC: %.0f/%.0fm",
                    lowc_obstacle.distance_xy, lowc_obstacle.distance_z ))
        gcs:send_named_string("DAA-LOWC", "aircraft")
    end

    local function LoWC_cleared()
        if not LoWC_active then
            return
        end
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Well Clear", LoWC_label) )
        gcs:send_named_string("DAA-LOWCOK", "aircraft")
        LoWC_active = false
        LoWC_label  = ""
    end

    local function notify_aircraft_nearby(aircraft_obstacle)
        gcs:send_named_string("DAA-NEARBY", aircraft_obstacle.label)
        gcs:send_named_float("DAA-DISTXY", aircraft_obstacle.distance_xy)
        gcs:send_named_float("DAA-DISTZ", aircraft_obstacle.distance_z)
    end

    -- is the contact inside one of the two nested alert volumes?  Both axes have to be
    -- inside: an aircraft directly overhead but 1000 m above is neither a near miss nor a
    -- loss of well clear.
    local function aircraft_inside(aircraft_obstacle, limit_xy, limit_z)
        return aircraft_obstacle.distance_xy < limit_xy and aircraft_obstacle.distance_z < limit_z
    end

    local function alert_aircraft()
        if aircraft_avoiding == nil then
            NMAC_cleared()
            LoWC_cleared()
            return
        end

        local in_nmac = aircraft_inside(aircraft_avoiding, near_miss_xy, near_miss_z)
        local in_lowc = aircraft_inside(aircraft_avoiding, well_clear_xy, well_clear_z)

        -- each state clears on its OWN boundary, the moment the contact leaves it - not only
        -- when the contact disappears from the feed altogether.  Clearing sits outside the 5 s
        -- alert throttle below because it is an edge, not a repeat: leaving the pilot looking
        -- at a near miss that is already over is worse than one extra message.
        if not in_nmac then
            NMAC_cleared()
        end
        if not in_lowc then
            LoWC_cleared()
        end

        if (now_ms - now_aircraft_ms) <= 5000 then
            return
        end
        if in_nmac then
            NMAC_triggered(aircraft_avoiding)
        elseif in_lowc then
            LoWC_triggered(aircraft_avoiding)
        else
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s %.0f m",
                            aircraft_avoiding.label, aircraft_avoiding.distance_xy ))
            gcs:send_named_string("DAA-ALERT", "aircraft")
        end
        notify_aircraft_nearby(aircraft_avoiding)
        now_aircraft_ms = now_ms
    end

    -- alert the pilot about any obstacles found, alert_target_loc is the suggested new target location (if applicable)
    function DAA.alert(alert_target_loc)
        if daa_alert == 0 then
            return  -- parameter DAA_ALERT can be used to turn off alerting
        end
        alert_obstacle(alert_target_loc)
        alert_aircraft()
    end

    -- a wrapper around vehicle:update_target_location()
    local function update_target_location(new_target_loc)
        if new_target_loc == nil or update_target_location_save_loc == nil then
            return false
        end
        -- enforce the altitude fences on every commanded target (clamp-and-continue)
        core.clamp_alt_to_fence(new_target_loc)
        new_target_loc:change_alt_frame(update_target_location_save_loc:get_alt_frame())
        local updated_location = vehicle:update_target_location(update_target_location_save_loc, new_target_loc)
        if updated_location then
            return true
        end
        return false
    end

    local function set_avoid_location(new_target_loc)
        -- if the "new" target is nil then revert back to the original navigation target which should be the current waypoint
        if new_target_loc == nil then
            if update_target_location(navigation_target_loc) then
                if daa_target_loc ~= nil then
                    -- Leaving avoidance: restore whatever crosstrack state was in
                    -- effect before it was disabled below, now that the mission/
                    -- guided target is back in charge of the leg.
                    vehicle:set_crosstrack_enabled(saved_crosstrack_enabled)
                end
                daa_target_loc = nil
            end
            return false
        end
        -- if the "new" target is different from the current DAA target then lets try to go there
        if not locations_equal(daa_target_loc, new_target_loc) then
            local entering_avoidance = (daa_target_loc == nil)
            if update_target_location(new_target_loc) then
                if entering_avoidance then
                    -- Entering avoidance: fly the direct current-position-to-target
                    -- line DAA itself evaluated when it chose this bearing, not the
                    -- stale prev_WP_loc -> target leg - a stale crosstrack reference
                    -- manufactured a large XTE and could command a bank direction
                    -- that contradicted DAA's own current_loc -> bearing evaluation
                    -- (confirmed live 2026-09-04, log 00000178.BIN: NTUN.XT jumped
                    -- from +7m to -22.6m the instant DAA changed HdgB). Unlike
                    -- set_crosstrack_start(), this never touches prev_WP_loc, so the
                    -- vertical glide-slope calculation that also depends on it is
                    -- undisturbed - see project_planedaa_reversal_awareness in
                    -- memory for why that distinction matters here.
                    local enabled = vehicle:get_crosstrack_enabled()
                    if enabled ~= nil then
                        saved_crosstrack_enabled = enabled
                    end
                    vehicle:set_crosstrack_enabled(false)
                end
                daa_target_loc = new_target_loc:copy()
                return true
            end
        end
        return false
    end

    -- Revert any in-progress avoidance and hand navigation back to the vehicle's real
    -- target (the current waypoint / RTL-to-home).  Called from the main loop whenever
    -- DAA is inactive (e.g. the pilot switched it off) while an avoidance target is still
    -- commanded, so the vehicle does not keep flying to a now-stale avoidance waypoint.
    -- Without this, disabling DAA mid-avoidance leaves the hijacked target in place and
    -- the vehicle flies to it instead of home (e.g. RTL flying straight past home).
    function DAA.clear_avoidance()
        -- An aircraft loiter hijacks the flight MODE (it commands GUIDED directly rather
        -- than going through set_avoid_location), so it leaves daa_target_loc nil and the
        -- check below would return without touching it.  Nothing else can stop it either:
        -- do_loitering(), and with it loiteralt.update(), only runs while DAA.isactive().
        -- Switching DAA off mid-loiter therefore used to leave the vehicle circling in
        -- GUIDED indefinitely.
        if loiteralt.active then
            -- if the pilot has already left GUIDED, update() drops the loiter without
            -- commanding a mode; only force the restore while we still hold GUIDED.
            loiteralt.update()
            if loiteralt.active then
                -- force past the DAA_LTR_COOL_S hold: it exists so a flapping traffic
                -- feed cannot thrash the mode, and means nothing once DAA is off.
                -- stop() restores the mode saved when the loiter began (and correctly
                -- leaves us in GUIDED if that is what we were in beforehand).
                loiteralt.stop(true)
            end
        end

        if daa_target_loc == nil then
            return
        end
        -- set_avoid_location(nil) reverts to navigation_target_loc and, on success,
        -- clears daa_target_loc so this only fires once.
        set_avoid_location(nil)
        if daa_target_loc == nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": avoidance cleared")
            gcs:send_named_string("DAA-AVOID", "")
            gcs:send_named_string("DAA-OBSTCL", "")
            avoiding_label = ""
        end
    end

    local function avoid_obstacle(new_target_loc, obstacle)
        if obstacle == nil then
            -- no obstacle: reset the target back to the original target
            new_target_loc = nil
        end
        -- if we have a new target - update it if it's different from our current target otherwise revert to the original target
        if set_avoid_location(new_target_loc) and navigation_target_loc ~= nil then
            local avoid_dist = navigation_target_loc:get_distance(new_target_loc)
            if (now_ms - now_avoiding_ms) > 5000 and obstacle ~= nil and avoid_dist > 5 then
                local report_dist = obstacle_report_distance(obstacle)
                if report_dist ~= nil then
                    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s dist: %.0fm", obstacle.label, report_dist))
                    gcs:send_named_float("DAA-DIST", report_dist)
                else
                    -- distance not simply knowable for this obstacle (e.g. Lua fence) - label only
                    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s", obstacle.label))
                end
                avoiding_label = obstacle.label
                gcs:send_named_string("DAA-AVOID", "obstacle")
                gcs:send_named_string("DAA-OBSTCL", avoiding_label)
                now_avoiding_ms = now_ms
            end
        else
            if avoiding_label ~= "" then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s done", avoiding_label))
                gcs:send_named_string("DAA-AVOID", "")
                gcs:send_named_string("DAA-OBSTCL", "")
                avoiding_label = ""
            end
        end
        log_avoid(obstacle, daa_target_loc)
    end

    local function do_loitering()
        if current_loc == nil then
            -- no current position to measure against, keep loitering until we have one
            return
        end
        -- Release the loiter only when the aircraft is no longer detected at all. The
        -- "gone" distance must match the DETECTION distance (well_clear_xy + margin_crewed_m),
        -- NOT the bare margin_crewed_m: a plane sitting steadily inside the detection volume
        -- (e.g. 200 m out, well within ~660 m but far beyond 50 m) would otherwise satisfy
        -- both "still detected" (start) and "far enough to stop" (stop) every cycle and flip
        -- GUIDED<->AUTO. Hysteresis is provided by the 10 s aircraft_seen() dwell in
        -- loiteralt.stop(false), so a plane at the boundary cannot thrash the mode.
        if aircraft_avoiding == nil or (current_loc:get_distance(aircraft_avoiding.location) > (well_clear_xy + margin_crewed_m)) then
            if loiteralt.stop(false) then
                return
            end
        end
        if aircraft_avoiding ~= nil then
            loiteralt.aircraft_seen()
            if (now_ms - now_loitering_ms) > 5000 then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITERING to %.0f m for AIRCRAFT: %s", crewed_avoid_alt_m, aircraft_avoiding.label))
                now_loitering_ms = now_ms
            end
        end
        loiteralt.update()
    end

    -- execute avoidance maneuvers depending on the nature of the obstacle
    function DAA.avoid(new_target_loc)
        if daa_action == 0 then
            return              -- parameter DAA_AVOID can be used to disable avoidance
        end

        if loiteralt.active then
            do_loitering()
        -- Crewed traffic outranks whatever we are already avoiding, so the loiter trigger is
        -- tested before falling through to ordinary bendy-ruler avoidance below.  An aircraft
        -- that appears while a fence or drone avoidance is already running must still be able
        -- to take over.
        -- DAA_AVD_ALT = 0 disables the loiter-to-altitude (see above).  Tested before
        -- assess_obstacle_motion() so the CPA work is skipped when the loiter is off.
        elseif aircraft_avoiding ~= nil and crewed_avoid_alt_m > 0
                and core.assess_obstacle_motion(aircraft_avoiding).is_conflict then
            -- CONSERVATIVE CPA gate on the loiter trigger: an aircraft inside the well-clear
            -- radius is always a conflict (assess_obstacle_motion's range check), so the loiter
            -- still fires unconditionally at close range - safer-first. Only a plane in the outer
            -- detection band that will miss beyond well-clear AND is not closing is skipped, and it
            -- then falls through to normal monitoring. A missing/uncertain velocity => conflict.
            if loiteralt.start(crewed_avoid_alt_m, crewed_avoid_alt_frame, true, airspeed_ms) then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITER AIRCRAFT: %s", aircraft_avoiding.label))

                gcs:send_named_string("DAA-AVOID", "LOITER")
                gcs:send_named_float("DAA-LOITER", crewed_avoid_alt_m)
                gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
                gcs:send_named_float("DAA-DIST", aircraft_avoiding.distance_m)

                return
            end
            -- the loiter did not start: fall through to ordinary bendy-ruler avoidance below.
        end
        avoid_obstacle(new_target_loc, obstacle_avoiding)
    end

    -- a moving obstacle (drone/aircraft/bird/AIS/proximity) may move out of the way;
    -- a fence will not, so a fence trap is held until the pilot intervenes
    local function obstacle_is_dynamic(obstacle_type)
        if obstacle_type == nil then return true end
        -- obstacles.is_fence_obstacle() covers the horizontal fence types; the altitude
        -- fences are handled separately there (see its own comment) but are also fixed,
        -- not dynamic, so this caller ORs them in explicitly.
        local is_fence = obstacles.is_fence_obstacle(obstacle_type)
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN
        return not is_fence
    end

    -- Watchdog for a laggy/dropped traffic feed.  A network-sourced moving obstacle (ADS-B
    -- drone/aircraft) carries an update timestamp; if we are avoiding one whose position has
    -- gone stale (> DAA_STALE_S) we warn the pilot that the DAA is acting on old data, and warn
    -- again if it then disappears (feed dropped / pruned).  Fences are on-board and never stale.
    function DAA.feed_watch()
        if stale_s <= 0 then return end
        local mv = aircraft_avoiding
        if mv == nil and obstacle_avoiding ~= nil and obstacle_is_dynamic(obstacle_avoiding.type) then
            mv = obstacle_avoiding
        end
        if mv ~= nil and mv.timestamp_ms ~= nil then
            local age_s = (now_ms:tofloat() - mv.timestamp_ms) / 1000.0
            feed_is_stale = age_s > stale_s
            if feed_is_stale and (now_ms - feed_stale_warn_ms) > 3000 then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": traffic stale %.0fs", age_s))
                feed_stale_warn_ms = now_ms
            end
            feed_watch_label = mv.label
        else
            -- the monitored moving obstacle is gone: if it was stale when last seen, the feed
            -- dropped it (pruned) rather than the plane clearing it
            if feed_watch_label ~= "" and feed_is_stale then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": lost " .. feed_watch_label)
            end
            feed_watch_label = ""
            feed_is_stale = false
        end
    end

    -- map DAA_TRAP_ACT to a flight mode; VTOL modes fall back to RTL if there is no VTOL
    local function trap_act_to_mode(act)
        local have_vtol = (param:get('Q_ENABLE') or 0) > 0
        if act == 1 then return PLANE_MODE.RTL
        elseif act == 2 then return have_vtol and PLANE_MODE.QRTL or PLANE_MODE.RTL
        elseif act == 3 then return have_vtol and PLANE_MODE.QLOITER or PLANE_MODE.RTL
        elseif act == 4 then return have_vtol and PLANE_MODE.QLAND or PLANE_MODE.RTL
        end
        return PLANE_MODE.RTL
    end

    -- the mode the trap should command: the DAA_TRAP_ACT action, but if that would just
    -- re-command the mode we are already in (e.g. RTL while already RTL - a no-op), escalate
    -- to DAA_TRAP_ESC_ACT (default QRTL) so a mid-RTL trap actually stops the aircraft
    local function resolve_trap_mode(mode_now)
        local mode = trap_act_to_mode(trap_act)
        if mode == mode_now then
            mode = trap_act_to_mode(trap_esc_act)
        end
        return mode
    end

    -- Drop the progress tracker.  Called whenever measuring progress is meaningless: not
    -- avoiding, not in forward flight, deliberately loitering, or the navigation target
    -- moved - a waypoint advancing IS progress, so the measurement starts again.
    local function hung_reset()
        if hung_active then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": HUNG clear")
            gcs:send_named_string("DAA-AVOID", "")
        end
        hung_active     = false
        hung_best_m     = nil
        hung_since_ms   = uint32_t(0)
        hung_target_loc = nil
    end

    -- Avoidance is "hung" when it is running but the vehicle is not closing on its navigation
    -- target.  This is neither a breach nor a boxed-in sweep: the bendy ruler keeps finding
    -- headings and the plane keeps flying them, it simply never arrives.  The observed case is
    -- a waypoint inside an exclusion-fence standoff, where the mission pull and the fence push
    -- are irreconcilable, so the vehicle orbits until fuel or some other failsafe ends it.
    -- Progress is a new CLOSEST approach to the target, not a shrinking range: a legitimate
    -- detour flies away from the waypoint for a while and must not read as no progress.  The
    -- new best has to beat the old by a turn radius - the distance scale the avoidance geometry
    -- itself works at - so position noise cannot keep resetting the timer.
    local function hung_update()
        if hung_alrt_s <= 0 or not in_fw_flight
            or daa_target_loc == nil or navigation_target_loc == nil or current_loc == nil
            or loiteralt.active then
            hung_reset()
            return
        end
        if not locations_equal(hung_target_loc, navigation_target_loc) then
            hung_reset()
            hung_target_loc = navigation_target_loc:copy()
        end
        local range_m    = current_loc:get_distance(navigation_target_loc)
        local progress_m = math.max(turn_radius_m(airspeed_ms, roll_limit_deg), MIN_HUNG_PROGRESS_M)
        if hung_best_m == nil or range_m < (hung_best_m - progress_m) then
            hung_best_m   = range_m
            hung_since_ms = now_ms
            return
        end
        if hung_active or (now_ms - hung_since_ms) < (hung_alrt_s * 1000) then
            return
        end
        hung_active = true
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
            " HUNG: no progress %.0fs, %.0fm to go", hung_alrt_s, range_m))
        gcs:send_named_string("DAA-AVOID", "HUNG")
    end

    -- A "compromise" = the aircraft has penetrated an obstacle's inner danger line, which is
    -- tighter than planedaa's avoidance standoff (so normal close-skirting avoidance never
    -- reaches it, hence no false-trigger):
    --   * fence    -> an actual AC_Fence breach (the real boundary at FENCE_MARGIN, well inside
    --                 planedaa's DAA_MARGIN_FENCE standoff)
    --   * aircraft -> a real aircraft inside near-miss (AVD_NMAC_XY/Z), i.e. well-clear lost.
    --                 NMAC is an aircraft-only boundary (matches alert_aircraft): drones,
    --                 proximity, AIS and birds are avoided but do not trip the trap here.
    -- Sustained (DAA_TRAP_S) this is a genuine trap. Altitude fences are vertical
    -- (clamp-and-continue) and are covered by get_breaches, not the aircraft near-miss check.
    --   * hung     -> avoidance running with no progress toward the navigation target for
    --                 DAA_HUNG_ALRT_S (see hung_update).  Nothing has been penetrated at all -
    --                 that is the point: this is the failure the other two cannot see.
    -- Returns (compromised, cause).  The cause decides recovery: a real aircraft inside NMAC
    -- can move out of the way, a fence breach cannot, and a hung avoidance is released only
    -- when the mission it could not fly moves on.  The
    -- cause must be reported from here because it cannot be recovered afterwards: the trap
    -- used to classify itself from obstacle_avoiding, the bendy ruler's current closest
    -- threat, which need not be what caused the compromise.  With simultaneous threats that
    -- filed an aircraft near-miss as a sticky fence trap, or let a fence breach inherit a
    -- moving obstacle's auto-resume.
    local function daa_compromised_now()
        -- A fence breach is a compromise UNLESS the core fence library is already handling it
        -- in a way that will refuse the trap's mode change: FENCE_ACTION ~= 0 (core takes a mode
        -- action) AND FENCE_OPTIONS bit0 (DISABLE_MODE_CHANGE) set. In that pairing set_mode is
        -- denied "in fence recovery", so firing the trap only spams the pilot with a
        -- TRAPPED -> denied -> released burst and achieves nothing. Stand the fence branch down;
        -- DAA.warnings() flags the pairing at enable. The trap still fires for a report-only fence
        -- (FENCE_ACTION=0) or when mode changes are allowed, and the aircraft near-miss branch
        -- below is unaffected (a real NMAC still traps regardless of fence config).
        if fence ~= nil and fence:get_breaches() ~= 0 then
            local fence_act  = param:get('FENCE_ACTION') or 0
            local fence_opts = param:get('FENCE_OPTIONS') or 0
            if not (fence_act ~= 0 and (math.floor(fence_opts) % 2) == 1) then
                return true, "fence"    -- a fence will not move out of the way: static trap
            end
        end
        if aircraft_avoiding ~= nil
            and aircraft_avoiding.distance_xy ~= nil
            and aircraft_avoiding.distance_xy < near_miss_xy
            and aircraft_avoiding.distance_z < near_miss_z then
            return true, "moving"       -- a real aircraft can move away: recoverable trap
        end
        -- A hung avoidance is deliberately NOT stood down by the FENCE_ACTION/FENCE_OPTIONS
        -- pairing above: it never breaches, so the core fence library never acts and there is
        -- nothing to defer to.  That pairing is exactly what left the aircraft with no escape.
        if hung_active then
            return true, "hung"
        end
        return false, nil
    end

    -- Arm the trapped failsafe: latch the cause, commit the mode change, announce it.
    -- Returns true if the failsafe took control, false if the mode change was refused.
    local function trap_engage(mode_now, cause)
        -- classify from the cause the compromise check reported, not from
        -- whatever the bendy ruler happens to be closest to right now
        trap_dynamic    = (cause == "moving")
        trap_hung       = (cause == "hung")
        if trap_hung then
            -- the mission index at the moment we gave up on it: any change means the geometry
            -- that hung us is no longer what the vehicle is being asked to fly
            hung_nav_index = mission:get_current_nav_index()
        end
        trap_prev_mode  = mode_now
        trap_fs_mode    = resolve_trap_mode(mode_now)
        trap_trigger_ms = now_ms
        if not vehicle:set_mode(trap_fs_mode) then
            -- do not claim a failsafe that never engaged: trap_active would be released
            -- again next cycle by the mode-changed check, which would blame a pilot who
            -- never touched anything
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
                ": TRAPPED - %s refused", get_mode_string(trap_fs_mode)))
            trap_prev_mode  = -1
            trap_since_ms   = uint32_t(0)
            return false
        end
        trap_active     = true
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
            ": TRAPPED (%s) -> %s", cause, get_mode_string(trap_fs_mode)))
        gcs:send_named_string("DAA-AVOID", "TRAPPED")
        return true
    end

    -- Hand control back to the mode the trap interrupted.  Shared by the moving-obstacle
    -- auto-clear and the hung release, so both report and reset identically.
    local function trap_release(reason)
        local resumed = (trap_prev_mode >= 0) and vehicle:set_mode(trap_prev_mode)
        if resumed then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(
                ": trap clear (%s) -> resume %s", reason, get_mode_string(trap_prev_mode)))
        else
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
                ": trap clear but resume %s REFUSED", get_mode_string(trap_prev_mode)))
        end
        gcs:send_named_string("DAA-AVOID", "")
        trap_active   = false
        trap_since_ms = uint32_t(0)
    end

    -- Tell the pilot when the mission has moved on from a waypoint the vehicle never
    -- reached.  Skipping one is the right outcome - containment beats mission fidelity -
    -- but it is otherwise silent, and the pilot is owed the fact: ArduPlane's
    -- past-the-waypoint test is taken against next_WP_loc, which we have replaced with the
    -- avoidance target, so a laterally offset target rotates that finish line and the
    -- waypoint completes early (measured at 100-177 m short on a real flight).
    --
    -- Only a FORWARD move counts.  A GCS can DO_JUMP the mission backwards while we happen
    -- to be avoiding, and that is not ours to report.
    local function mission_skip_update()
        local was_index = skip_nav_index
        local was_loc   = skip_target_loc
        skip_nav_index  = mission:get_current_nav_index()
        skip_target_loc = (navigation_target_loc ~= nil) and navigation_target_loc:copy() or nil

        if daa_target_loc ~= nil then
            skip_avoid_ms = now_ms
        end
        if current_loc == nil or was_loc == nil then
            return                              -- nothing to compare against
        end
        -- Recently avoiding, not necessarily still avoiding.  The mission completes the
        -- waypoint on the tick AFTER the target is handed back, so "AVOIDING ... done"
        -- lands about 0.1 s ahead of it: testing daa_target_loc directly reported 1 of the
        -- 4 skips in a 404 s flight and which one it caught was a race.
        if skip_avoid_ms == uint32_t(0) or (now_ms - skip_avoid_ms) > SKIP_AVOID_GRACE_MS then
            return
        end
        if was_index < 0 or skip_nav_index <= was_index then
            return                              -- no move, or a jump backwards
        end
        -- WP_RADIUS is the arrival test the vehicle itself used, near enough: its acceptance
        -- distance is WP_RADIUS scaled by EAS2TAS squared and capped at the L1 distance, so
        -- this can miss a skip by a few metres at altitude.  Erring that way is right - a
        -- false "skipped" on a genuine arrival is the more misleading of the two.
        local short_m = current_loc:get_distance(was_loc)
        if short_m <= wp_radius_m then
            return                              -- close enough that this was a real arrival
        end
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
            " WP%d skipped by %.0fm while avoiding", was_index, short_m))
        gcs:send_named_string("DAA-AVOID", "WPSKIP")
    end

    -- Trapped-failsafe state machine. Returns true while the failsafe is controlling the
    -- vehicle (so update() skips normal avoidance). See DAA_TRAP_ACT/S/CLR_S.
    function DAA.trap_update()
        -- track progress even when the failsafe action is disabled: DAA_TRAP_ACT=0 with
        -- DAA_HUNG_ALRT_S set is the alert-only configuration, and it is the default
        hung_update()
        mission_skip_update()
        if trap_act == 0 then
            trap_active = false
            trap_since_ms = uint32_t(0)
            return false
        end
        local mode_now = vehicle:get_mode()

        if not trap_active then
            local compromised, cause = daa_compromised_now()
            if not compromised then
                trap_since_ms = uint32_t(0)
                return false
            end
            if trap_since_ms == uint32_t(0) then trap_since_ms = now_ms end
            if (now_ms - trap_since_ms) < (trap_s * 1000) then
                return false        -- still inside the DAA_TRAP_S dwell
            end
            return trap_engage(mode_now, cause)
        end

        -- failsafe active
        if mode_now ~= trap_fs_mode then
            -- the pilot (or another failsafe) changed mode: release, never fight it
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": trap released (mode changed)")
            trap_active = false
            trap_since_ms = uint32_t(0)
            return false
        end
        if trap_dynamic and (now_ms - trap_trigger_ms) >= (trap_clr_s * 1000) then
            -- transient moving-obstacle squeeze: resume the mission; if the obstacle is
            -- still there, forward flight will simply re-trigger the failsafe
            trap_release("moving")
            return false
        end
        -- A hung trap is released when the mission moves on - the pilot advancing it, or a new
        -- mission - because that is the only thing that changes the geometry we could not fly.
        -- Progress toward the target cannot be the release test: the trap's own action retargets
        -- the vehicle (RTL goes to home), which would read as instant progress and drop us
        -- straight back into the stall.
        if trap_hung and mission:get_current_nav_index() ~= hung_nav_index then
            trap_release("mission moved")
            hung_reset()
            return false
        end
        -- fence (static) trap: hold the failsafe mode until the pilot intervenes
        return true
    end

end) () -- DAA management class

-------------------------------------------------------------------------------
--- Main script execution and update loop including RC on/off management
-------------------------------------------------------------------------------
local last_switch_state = 0
local no_DAA_displayed  = false

local function update()
    -- Both state-fetch calls first, back to back, before anything else runs this cycle:
    -- position/mode (get_vehicle_state) then target/wind/airspeed/groundspeed
    -- (DAA.get_vehicle_state) - so every decision made later in this same cycle, in
    -- either function, sees one consistent snapshot rather than values fetched at
    -- scattered points across the cycle.  Neither depends on the switch-function check
    -- below.
    get_vehicle_state()
    DAA.get_vehicle_state(current_loc)

    local switch_function = PARAM.ACT_FN:get()
    if switch_function == nil then
        if not no_DAA_displayed then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " no DAA function")
            no_DAA_displayed = true
        end
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
        if switch_state == 0 then -- switch Low to disarm - so defaults to on
            DAA.enable()
        elseif switch_state >= 1 then -- switch High to turn off
            DAA.disable()
        end
        last_switch_state = switch_state
    end

    if DAA.isactive() then
        local suggested_target_loc = DAA.take_report(core.detect())
        DAA.alert(suggested_target_loc)
        DAA.feed_watch()
        -- when avoidance can't find a way out, the trapped-failsafe takes over (and we
        -- stop issuing avoidance); otherwise avoid, but only in FW forward flight
        local trapped = DAA.trap_update()
        if not trapped and in_fw_flight then
            DAA.avoid(suggested_target_loc)
        end
    else
        -- DAA is inactive (e.g. the pilot switched it off).  If it was mid-avoidance,
        -- revert the commanded target so the vehicle resumes its real navigation
        -- (e.g. RTL-to-home) instead of flying on to a stale avoidance waypoint.
        DAA.clear_avoidance()
    end
end

-- wrapper around update(). This re-schedules update() every refresh_period_ms ms (= 1000 / DAA_UPDATE_RATE Hz)
-- and if update faults then an error is displayed, but the script is not stopped
function Protected_Wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. tostring(err))
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, refresh_period_ms
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    -- DAA defaults to on but can be disabled using a scripting aux function
    DAA.enable()
    return Protected_Wrapper()
end

-- wait a bit for AP to come up cleanly then start running update loop, unless armed
if arming:is_armed() then
    return Delayed_Startup()
else
    return Delayed_Startup, 1000 * STARTUP_DELAY
end
