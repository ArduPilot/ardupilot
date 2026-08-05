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
            for GA (General Aviation) or Crude Aircraft also implements more
            flexible avoidance manoeuvre such as a Standard Right Turn to Altitude.
            The intention is to come up with a standard library of GA avoidance 
            manoeuvres, will also allowing users to implement there own due to this 
            being implemented as Lua.            
--]]

SCRIPT_NAME         = "Plane DAA"
SCRIPT_NAME_SHORT   = "pDAA"
SCRIPT_VERSION      = "4.8.0-046"

STARTUP_DELAY       = 25  -- wait this many seconds for the FC to come up before starting the main loop

PLANE_MODE          = {CIRCLE = 1, STABILIZE = 2, TRAINING = 3, ACRO = 4, FBWA = 4, FBWB = 6, CRUISE = 7, 
                        AUTOTUNE = 8, AUTO=10, RTL=11, LOITER=12, TAKEOFF = 13, AVOID_ADSB = 14, GUIDED=15, 
                        QSTABILIZE = 17,  QHOVER=18, QLOITER=19, QLAND = 20, QRTL=21, QAUTOTUNE = 22, QACRO = 23, 
                        THERMAL = 24, LOITER_ALT_QLAND = 25, AUTOLAND = 26}

ALT_FRAME           = {GLOBAL = 0, RELATIVE = 1, ORIGIN = 2, TERRAIN = 3}

MAV_SEVERITY        = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_CMD_INT         = {DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                        GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002}
MAV_SPEED_TYPE      = {AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3}
MAV_HEADING_TYPE    = {COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

MAV_VTOL_STATE      = {UNDEFINED = 0, TRANSITION_TO_FW = 1, TRANSITION_TO_MC = 2, MC = 3, FW = 4 }

-- MAV_COLLISION_THREAT_LEVEL
MAV_COLLISION_THREAT_LEVEL = {
    NONE                        = 0,    -- Not a threat
    LOW                         = 1,    -- Mild concern about this threat
    HIGH                        = 2,    -- Craft is panicking and may take action to avoid
    ENUM_END                    = 3     -- End of enum
}
-- MAV_COLLISION_SRC
MAV_COLLISION_SRC = {
    ADSB                        = 0,    -- Source is ADSB_VEHICLE packets
    MAVLINK_GPS_GLOBAL_INT      = 1,    -- Source is MAVLink GPS_GLOBAL_INT
    ENUM_END                    = 2     -- End of enum
}

MAV_COLLISION_ACTION = {
    NONE                        = 0,    -- Ignore any potential collisions 
    REPORT                      = 1,    -- Report potential collision 
    ASCEND_OR_DESCEND           = 2,    -- Ascend or Descend to avoid threat 
    MOVE_HORIZONTALLY           = 3,    -- Move horizontally to avoid threat 
    MOVE_PERPENDICULAR          = 4,    -- Aircraft to move perpendicular to the collision's velocity vector 
    RTL                         = 5,    -- Aircraft to fly directly back to its launch point 
    HOVER                       = 6,    -- Aircraft to stop in place 
    LOITERTURN                  = 7,    -- Aircraft to do a loiter turn left or right to lose altitude
}

OBSTACLE_TYPE = {
    GENERAL                     = 0,    -- generic obstacle, we don't really know what it is
    MAV_SYSID                   = 1,    -- another MAVLINK drone with a MAV_SYSID
    GENERAL_AVIATION            = 2,    -- crude aircraft, usually with an ICAO ADSB identifier
    WEATHER                     = 3,
    BIRD_MIGRATORY              = 4,    -- typically one or more Canada Geese
    BIRD_OF_PREY                = 5,    -- a bird that might attack the vehicle
    FENCE_HOME                  = 6,    -- all fixed/unmovable fences
    FENCE_CIRCLE_INCLUSION      = 7,
    FENCE_CIRCLE_EXCLUSION      = 8,
    FENCE_POLYGON_INCLUSION     = 9,
    FENCE_POLYGON_EXCLUSION     = 10,
    FENCE_LUA                   = 11,
    PROXIMITY                   = 12,   -- detected by a proximty sensor, typically quite close
    AIS                         = 13,   -- Automatic Identification System for ship (maritime) vehicles
    FENCE_ALT_MAX               = 14,   -- max altitude fence (AC_FENCE_TYPE_ALT_MAX, FENCE_TYPE bit 0)
    FENCE_ALT_MIN               = 15,   -- min altitude fence (AC_FENCE_TYPE_ALT_MIN, FENCE_TYPE bit 3)
}

-- ADSB Emitter types
ADSB_EMITTER = {
    NO_INFO                     = 0,
    LIGHT                       = 1,
    SMALL                       = 2,
    LARGE                       = 3,
    HIGH_VORTEX_LARGE           = 4,
    HEAVY                       = 5,
    HIGHLY_MANUV                = 6,
    ROTOCRAFT                   = 7,    -- this is Helicopter not a drone
    -- 8 Unassigned
    GLIDER                      = 9,
    LIGHTER_AIR                 = 10,
    PARACHUTE                   = 11,
    ULTRA_LIGHT                 = 12,
    AIRCRAFT_HIGH               = 13,
    UAV                         = 14,   -- this is drones
    SPACE                       = 15,   -- this is rockets
    --16 Unassigned

    -- Surface types
    EMERGENCY_SURFACE           = 17,
    SERVICE_SURFACE             = 18,

    -- Obstacle types
    POINT_OBSTACLE              = 19,
    CLUSTER_OBSTACLE            = 20,
    LINE_OBSTACLE               = 21,
    -- 22 - 39 Reserved

}

-- luacheck: ignore DAA_active
local DAA_active = true;

local PARAM_TABLE_KEY = 125
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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 39), SCRIPT_NAME_SHORT .. ' could not add param table: ' .. PARAM_TABLE_PREFIX .. " key: " .. PARAM_TABLE_KEY)

--[[
    // @Param: DAA_ACT_FN
    // @DisplayName: DAA Activation
    // @Description: Disable the DAA capability or turn it off (defaults to on)
    // @Range: 300 307
--]]
DAA_ACT_FN = bind_add_param("ACT_FN", 1, 308)

--[[
  // @Param: DAA_MARGIN_FENCE
  // @DisplayName: fence margin 
  // @Description: Avoidance margin (m) kept clear of fences. 0 (default) uses the turn radius WP_LOITER_RAD, so the standoff matches a single loiter circle and fences do not thrash.
  // @Units: m
--]]
DAA_MARGIN_FENCE = bind_add_param('MARGIN_FENCE', 2, 0)

--[[
  // @Param: DAA_MARGIN_DYN
  // @DisplayName: dynamic margin
  // @Description: Avoidance margin for dynamic objects
  // @Units: m
--]]
DAA_MARGIN_DYN   = bind_add_param('MARGIN_DYN', 3, 20)

--[[
  // @Param: DAA_MARGIN_EXCL
  // @DisplayName: exclusion zone margin
  // @Description: Avoidance margin for exclusion zones
  // @Units: m
--]]
DAA_MARGIN_EXCL   = bind_add_param('MARGIN_EXCL', 4, 20)

--[[
  // @Param: DAA_MARGIN_WIDE
  // @DisplayName: wide avoidance margin
  // @Description: Avoidance margin for wide avoidance
  // @Units: m
--]]
DAA_MARGIN_WIDE   = bind_add_param('MARGIN_WIDE', 5, 30)

--[[
  // @Param: DAA_MARGIN_HGT
  // @DisplayName: height avoidance margin
  // @Description: Avoidance margin for height avoidance 
  // @Units: m
--]]
DAA_MARGIN_HGT   = bind_add_param('MARGIN_HGT', 6, 60)

--[[
  // @Param: DAA_LKAHD
  // @DisplayName: avoidance lookahead distance
  // @Description: Avoidance lookahead distance
  // @Units: m
--]]
DAA_LKAHD  = bind_add_param('LKAHD', 7, 1000)

--[[
  // @Param: DAA_UPDATE_RATE
  // @DisplayName: rate to process avoidance
  // @Description: Avoidance processing rate
  // @Units: Hz
--]]
DAA_UPDATE_RATE  = bind_add_param('UPDATE_RATE', 8, 10.0)

--[[
  // @Param: DAA_HEIGHT_USE
  // @DisplayName: Include height differences
  // @Description: Avoidance will consider height differences when calculating collisions
  // @Values: 0:Use Height,1:Ignore Height
--]]
DAA_HEIGHT_USE  = bind_add_param('HEIGHT_USE', 9, 0)

--[[
  // @Param: DAA_MARGIN_GA
  // @DisplayName: Margin for General Aviation
  // @Description: Avoidance margin for Fixed Wing aircraft/General Aviation (Helicopters? eVTOL?) over and above the Well Clear margin AVD_WCLR_XY
  // @Units: m
--]]
DAA_MARGIN_GA  = bind_add_param('MARGIN_GA', 10, 50)

--[[
  // @Param: DAA_MARGIN_WTH
  // @DisplayName: Radius for Weather
  // @Description: Avoidance radius for Weather/Clouds/Rain 
  // @Units: m
--]]
DAA_MARGIN_WTH  = bind_add_param('MARGIN_WTH', 11, 173)

--[[
  // @Param: DAA_MARGIN_BIRD
  // @DisplayName: Margin for Birds
  // @Description: Avoidance margin for Migratory Birds 
  // @Units: m
--]]
DAA_MARGIN_BIRD  = bind_add_param('MARGIN_BIRD', 12, 100)

--[[
  // @Param: DAA_MARGIN_PREY
  // @DisplayName: Radius for Birds of Prey
  // @Description: Avoidance radius for Birds of Prey
  // @Units: m
--]]
DAA_MARGIN_PREY  = bind_add_param('MARGIN_PREY', 13, 200)

--[[
  // @Param: DAA_MARGIN_UAV
  // @DisplayName: Margin for UAVs/Drones
  // @Description: Avoidance radius for UAV/drone (MAVLink sourced)
  // @Units: m
--]]
DAA_MARGIN_UAV  = bind_add_param('MARGIN_UAV', 14, 50)

--[[
  // @Param: DAA_MARGIN_AIS
  // @DisplayName: Margin for AIS (ships)
  // @Description: Avoidance radius for AIS (MAVLink sourced)
  // @Units: m
--]]
DAA_MARGIN_AIS  = bind_add_param('MARGIN_AIS', 15, 50)

--[[
  // @Param: DAA_MARGIN_PRX
  // @DisplayName: Margin for proximity 
  // @Description: Avoidance radius for obstacles detected by proximity sensors. Typically pretty close
  // @Units: m
--]]
DAA_MARGIN_PRX  = bind_add_param('MARGIN_PRX', 16, 50)

--[[
    // @Param: DAA_BR_RATIO
    // @DisplayName: DAA margin ratio for BendyRuler to change bearing significantly
    // @Description:  DAA BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is at least this much.
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
--]]
DAA_BR_RATIO = bind_add_param('BR_RATIO', 17, 1.5)

--[[
    // @Param: DAA_BR_ANGLE
    // @DisplayName: BendyRuler's bearing change resistance threshold angle
    // @Description:  DAA BendyRuler will resist changing current bearing if the change in bearing is over this angle
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
--]]
DAA_BR_ANGLE = bind_add_param('BR_ANGLE', 18, 45)

--[[
    // @Param: DAA_AVD_ALT
    // @DisplayName: The altitude to loiter to when avoiding a crude aircraft
    // @Description:  DAA will loiter and descent to this altitude if a crude aircraft is detected within DAA_MARGIN_GA of the vehicle. Ignored if zero (0).
    // @Range: 20 5000
    // @Increment: 5
    // @User: Standard
--]]
DAA_AVD_ALT = bind_add_param('AVD_ALT', 19, 50)

--[[
    // @Param: DAA_AVD_ALT_TP
    // @DisplayName: The frame of the DAA_AVD_ALT
    // @Description:  DAA will loiter and descent to DAA_AVD_ALT in this frame. 0: Absolute, 1: Above Home, 2: Above Origin, 3: Above Terrain (default)
    // @Range: 20 5000
    // @Increment: 5
    // @User: Standard
--]]
DAA_AVD_ALT_TP = bind_add_param('AVD_ALT_TP', 20, 3)

--[[
    // @Param: DAA_AVD_ALERT
    // @DisplayName: Alert for DAA Avoidance
    // @Description: Alert or not Alert 
    // @Values: 0: None, 1: Alert
    // @User: Standard
--]]
DAA_AVD_ALERT = bind_add_param('AVD_ALERT', 21, 1)

--[[
    // @Param: DAA_AVD_ACTION
    // @DisplayName: Action for DAA Avoidance
    // @Description: Action for DAA Avoidance
    // @Values: 0: None, 1: Avoid
    // @User: Standard
--]]
DAA_AVD_ACTION = bind_add_param('AVD_ACTION', 22, 1)

--[[
    // @Param: DAA_MARGIN_ALT
    // @DisplayName: Altitude fence avoidance margin
    // @Description: Proactive buffer (in metres) inside the safe altitude-fence limits at which DAA starts clamping the commanded altitude. The plane levels off this far inside the safe limit. Avoidance kicks in for FENCE_ALT_MAX (FENCE_TYPE bit 0) and/or FENCE_ALT_MIN (FENCE_TYPE bit 3) when enabled.
    // @Units: m
    // @Range: 0 200
    // @Increment: 1
    // @User: Standard
--]]
DAA_MARGIN_ALT = bind_add_param('MARGIN_ALT', 23, 20)

--[[
    // @Param: DAA_ALT_HYST_M
    // @DisplayName: Altitude fence avoidance hysteresis
    // @Description: Hysteresis band (in metres) for altitude-fence avoidance. Once engaged, avoidance stays engaged until the altitude is this far back inside the safe band, preventing chatter as the plane levels off. Also sets how far before the level-off altitude the notice is announced.
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
--]]
DAA_ALT_HYST_M = bind_add_param('ALT_HYST_M', 24, 10)

--[[
    // @Param: DAA_ALT_COOL_S
    // @DisplayName: Altitude fence alert cooldown
    // @Description: Minimum time (in seconds) between altitude-fence "levelling off" notices, so brief re-engagements do not re-spam the GCS. Does not affect the avoidance itself.
    // @Units: s
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
--]]
DAA_ALT_COOL_S = bind_add_param('ALT_COOL_S', 25, 15)

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
DAA_HEADING_INC = bind_add_param('HEADING_INC', 26, DEFAULT_HEADING_INC_DEG)

--[[
    // @Param: DAA_WIND_MIN
    // @DisplayName: Minimum wind speed for wind-aware avoidance
    // @Description: Minimum wind speed (in m/s) before the avoidance look-ahead accounts for wind-driven drift while turning onto a candidate heading. Below this the still-air path is used, so calm-air behaviour is unchanged. Set very high to disable the wind-aware path entirely.
    // @Units: m/s
    // @Range: 0 50
    // @Increment: 0.5
    // @User: Advanced
--]]
DAA_WIND_MIN = bind_add_param('WIND_MIN', 27, 2.0)

--[[
    // @Param: DAA_WIND_MARG
    // @DisplayName: Wind-scaled fence margin
    // @Description: Extra fence avoidance margin added per m/s of wind above DAA_WIND_MIN, in metres per m/s. This widens the commanded standoff from fences in wind so the controller has buffer to absorb cross-track drift and is less likely to be blown across the boundary. 0 disables wind scaling. The extra margin is DAA_WIND_MARG * max(0, wind_speed - DAA_WIND_MIN).
    // @Units: s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
DAA_WIND_MARG = bind_add_param('WIND_MARG', 28, 5.0)

--[[
    // @Param: DAA_SLEW_DPS
    // @DisplayName: Avoidance heading slew rate
    // @Description: Maximum rate the commanded avoidance heading is allowed to change, in degrees per second. Rate-limiting the avoidance heading smooths the oscillation a per-cycle bendy ruler produces against moving obstacles and near fences. Bypassed when the estimated time-to-conflict is below DAA_SLEW_URG so an urgent manoeuvre keeps full authority. 0 disables the slew limit.
    // @Units: deg/s
    // @Range: 0 90
    // @Increment: 1
    // @User: Advanced
--]]
DAA_SLEW_DPS = bind_add_param('SLEW_DPS', 29, 20)

--[[
    // @Param: DAA_SLEW_URG
    // @DisplayName: Avoidance slew urgency time
    // @Description: If the estimated time-to-conflict with a moving obstacle is below this, the DAA_SLEW_DPS heading slew limit is bypassed so the aircraft can turn at full authority. 0 always applies the slew limit.
    // @Units: s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
DAA_SLEW_URG = bind_add_param('SLEW_URG', 30, 4)

--[[
    // @Param: DAA_SIDE_HOLD
    // @DisplayName: Avoidance side hold time
    // @Description: Once a left/right avoidance side is committed for an obstacle, the opposite side must be preferred by the bendy ruler for at least this long before the aircraft is allowed to switch sides. This stops the left/right flip-flop when avoiding a moving obstacle. 0 disables side commitment.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
--]]
DAA_SIDE_HOLD = bind_add_param('SIDE_HOLD', 31, 3)

--[[
    // @Param: DAA_CPA_MIN
    // @DisplayName: Avoidance minimum closing speed
    // @Description: Minimum closing speed for a moving obstacle to be treated as a conflict. A moving obstacle whose predicted closest approach stays beyond the well-clear distance and which is opening range faster than this is not avoided (it is leaving). 0 avoids regardless of closing speed.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.5
    // @User: Advanced
--]]
DAA_CPA_MIN = bind_add_param('CPA_MIN', 32, 2)

--[[
    // @Param: DAA_TRAP_ACT
    // @DisplayName: Trapped-failsafe action
    // @Description: What to do when avoidance cannot find a way out (boxed in, or unable to keep clear of an obstacle for DAA_TRAP_S). 0 disables the trapped-failsafe entirely (avoidance just keeps trying). For a VTOL, QLOITER stops forward flight and hovers (zero turn radius) - the safest way out of a tight space. If the aircraft has no VTOL (Q_ENABLE=0) the VTOL options fall back to RTL. Trapped by a fixed obstacle (fence) is sticky (held until the pilot changes mode); trapped by a moving obstacle (drone/aircraft) recovers to the previous mode after DAA_TRAP_CLR_S.
    // @Values: 0:Disabled,1:RTL,2:QRTL,3:QLOITER,4:QLAND
    // @User: Standard
--]]
DAA_TRAP_ACT = bind_add_param('TRAP_ACT', 33, 0)

--[[
    // @Param: DAA_TRAP_S
    // @DisplayName: Trapped-failsafe trigger time
    // @Description: Avoidance must be unable to find a way out continuously for this long before the DAA_TRAP_ACT failsafe fires. Prevents transient clutter from triggering it.
    // @Units: s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Standard
--]]
DAA_TRAP_S = bind_add_param('TRAP_S', 34, 5)

--[[
    // @Param: DAA_TRAP_CLR_S
    // @DisplayName: Trapped-failsafe recover time
    // @Description: For a trap caused by a MOVING obstacle (drone/aircraft), resume the previous mode this long after the failsafe fired, on the assumption the obstacle has passed (if it has not, forward flight simply re-triggers the failsafe). A trap caused by a fixed obstacle (fence) is not auto-recovered - it is held until the pilot changes mode.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
--]]
DAA_TRAP_CLR_S = bind_add_param('TRAP_CLR_S', 35, 4)

--[[
    // @Param: DAA_TRAP_ESC_ACT
    // @DisplayName: Trapped-failsafe escalation action
    // @Description: Action to take when the DAA_TRAP_ACT action would leave the aircraft in the mode it is ALREADY in (e.g. DAA_TRAP_ACT=RTL and the aircraft is already in RTL) - commanding the same mode again would do nothing, so escalate to this instead. Typically the aircraft is trapped mid-RTL and this stops it: QRTL (VTOL return, zero turn radius) or QLOITER (stop & hover) or QLAND. VTOL actions fall back to RTL if there is no VTOL. Set equal to DAA_TRAP_ACT to disable escalation.
    // @Values: 1:RTL,2:QRTL,3:QLOITER,4:QLAND
    // @User: Standard
--]]
DAA_TRAP_ESC_ACT = bind_add_param('TRAP_ESC_ACT', 36, 2)

--[[
    // @Param: DAA_STALE_S
    // @DisplayName: Traffic-feed staleness warning threshold
    // @Description: When avoiding a network-sourced moving obstacle (ADS-B drone/aircraft) whose position has not been updated for longer than this, a "traffic stale" warning is sent to the GCS - the DAA is acting on lagged data, e.g. from an intermittent telemetry/ADS-B link. A "lost" warning is sent if such an obstacle then disappears (pruned). Fences are on-board and never go stale. Set to 0 to disable the warnings.
    // @Units: s
    // @Range: 0 30
    // @User: Standard
--]]
DAA_STALE_S = bind_add_param('STALE_S', 37, 3)

--[[
    // @Param: DAA_MARGIN_GA_Z
    // @DisplayName: Vertical margin for General Aviation
    // @Description: Vertical avoidance margin for Fixed Wing aircraft/General Aviation, over and above the Well Clear vertical separation AVD_WCLR_Z. An aircraft is detected (and the loiter-to-altitude triggered) only while the altitude difference between it and the vehicle is less than AVD_WCLR_Z + this margin. This is the vertical mirror of the horizontal DAA_MARGIN_GA.
    // @Units: m
    // @Range: 0 200
    // @User: Standard
--]]
DAA_MARGIN_GA_Z = bind_add_param('MARGIN_GA_Z', 38, 30)

--[[
    // @Param: DAA_LTR_COOL_S
    // @DisplayName: Aircraft loiter release delay
    // @Description: Time the aircraft loiter-to-altitude is held after the aircraft was last detected, before releasing back to the mission. Acts as hysteresis so a briefly-dropped or laggy ADS-B feed cannot thrash the vehicle between GUIDED (loiter) and AUTO (mission). Set to 0 to release as soon as the aircraft is no longer detected.
    // @Units: s
    // @Range: 0 60
    // @User: Standard
--]]
DAA_LTR_COOL_S = bind_add_param('LTR_COOL_S', 39, 10)

WARN_DIST_XY                = bind_param("AVD_W_DIST_XY")
WARN_ACTION                 = bind_param("AVD_W_ACTION")
AVD_ENABLE                  = bind_param("AVD_ENABLE")
AVD_WCLR_XY                 = bind_param("AVD_WCLR_XY")
AVD_WCLR_Z                  = bind_param("AVD_WCLR_Z")
AVD_UAV_XY                  = bind_param("AVD_UAV_XY")
AVD_NMAC_XY                 = bind_param("AVD_NMAC_XY")
AVD_NMAC_Z                  = bind_param("AVD_NMAC_Z")
ROLL_LIMIT_DEG              = bind_param("ROLL_LIMIT_DEG")
WP_LOITER_RAD               = bind_param("WP_LOITER_RAD")

local roll_limit_deg        = ROLL_LIMIT_DEG:get()
local lookahead_param_m       = DAA_LKAHD:get()
local margin_fence_m          = DAA_MARGIN_FENCE:get()
if margin_fence_m <= 0 then margin_fence_m = math.abs(WP_LOITER_RAD:get()) end   -- 0 => use the turn radius so the fence standoff = one loiter circle
local margin_alt_m            = DAA_MARGIN_ALT:get()
local alt_hyst_m            = DAA_ALT_HYST_M:get()
local alt_cool_ms           = DAA_ALT_COOL_S:get() * 1000
local loiter_cool_ms        = DAA_LTR_COOL_S:get() * 1000
local margin_aircraft_m       = DAA_MARGIN_GA:get()
local margin_vertical_m       = DAA_MARGIN_GA_Z:get()
-- margin_bird_m/prey/weather are parked for planned obstacle types (see the
-- commented-out block in find_closest_obstacle); read but not yet consumed.
-- luacheck: ignore margin_bird_m margin_prey_m margin_weather_m
local margin_bird_m           = DAA_MARGIN_BIRD:get()
local margin_prey_m           = DAA_MARGIN_PREY:get()
local margin_uav_m            = DAA_MARGIN_UAV:get()
local margin_weather_m        = DAA_MARGIN_WTH:get()
local margin_ais_m            = DAA_MARGIN_AIS:get()
local margin_proximity_m      = DAA_MARGIN_PRX:get()
local refresh_rate          = 1000.0 / DAA_UPDATE_RATE:get()
local bendy_ratio           = DAA_BR_RATIO:get()
local bendy_angle           = DAA_BR_ANGLE:get()
local wp_loiter_rad_m         = WP_LOITER_RAD:get()
local ga_avoid_alt_m          = DAA_AVD_ALT:get()
local ga_avoid_alt_frame    = DAA_AVD_ALT_TP:get()
local daa_alert             = DAA_AVD_ALERT:get()
local daa_action            = DAA_AVD_ACTION:get()
local wind_min_ms           = DAA_WIND_MIN:get()
local wind_margin_per_ms    = DAA_WIND_MARG:get()
local well_clear_xy         = AVD_WCLR_XY:get()
local well_clear_z          = AVD_WCLR_Z:get()
local uav_clear_xy          = AVD_UAV_XY:get()
local near_miss_xy          = AVD_NMAC_XY:get()
local near_miss_z           = AVD_NMAC_Z:get()
local slew_dps              = DAA_SLEW_DPS:get()
local slew_urg_s            = DAA_SLEW_URG:get()
local side_hold_s           = DAA_SIDE_HOLD:get()
local cpa_min_ms            = DAA_CPA_MIN:get()
local trap_act              = DAA_TRAP_ACT:get()
local trap_s                = DAA_TRAP_S:get()
local trap_clr_s            = DAA_TRAP_CLR_S:get()
local trap_esc_act          = DAA_TRAP_ESC_ACT:get()
local stale_s               = DAA_STALE_S:get()

GRAVITY_MSS = 9.80665
LOCATION_SCALING_FACTOR_INV = 89.83204953368922

local bearing_inc_deg = DAA_HEADING_INC:get() or DEFAULT_HEADING_INC_DEG
if bearing_inc_deg <= 0 then
    bearing_inc_deg = DEFAULT_HEADING_INC_DEG
end

COLLISION_DETECTED = false

FLT_MAX = 3.402823466e+38

-------------------------------------------------------------------------------
--- Vehicle State stored in local variables to reduce api calls
-------------------------------------------------------------------------------

local current_loc           = ahrs:get_position()
local current_mode          = vehicle:get_mode()
local vtol_state            = MAV_VTOL_STATE.UNDEFINED

local now_ms                = millis()
local now_params_ms         = now_ms
local now_debug_ms          = now_ms
local now_avoiding_ms       = now_ms
local now_obstacle_ms       = now_ms
local now_aircraft_ms       = now_ms
local now_loitering_ms      = now_ms
local aircraft_seen_now_ms  = now_ms

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

local function get_mode_string(mode)
    if mode == PLANE_MODE.AUTO then
        return "Auto"
    elseif mode == PLANE_MODE.RTL then
        return "RTL" 
    elseif mode == PLANE_MODE.LOITER then
        return "Loiter"
    elseif mode == PLANE_MODE.GUIDED then
        return "Guided"
    elseif mode == PLANE_MODE.QSTABILIZE then
        return "Q Stabilize"
    elseif mode == PLANE_MODE.QHOVER then
        return "Q Hover"
    elseif mode == PLANE_MODE.QLOITER then
        return "Q Loiter"
    elseif mode == PLANE_MODE.QLAND then
        return "Q Land"
    elseif mode == PLANE_MODE.QAUTOTUNE then
        return "Q Autotune"
    elseif mode == PLANE_MODE.FBWA then
        return "FBWA"
    elseif mode == PLANE_MODE.FBWB then
        return "FBWB"
    elseif mode == PLANE_MODE.MANUAL then
        return "Manual"
    elseif mode == PLANE_MODE.CRUISE then
        return "Cruise"
    elseif mode == PLANE_MODE.AUTOTUNE then
        return "Autotune"
    elseif mode == PLANE_MODE.TAKEOFF then
        return "Takeoff"
    elseif mode == PLANE_MODE.AVOID_ADSB then
        return "Avoid ADSB"
    elseif mode == PLANE_MODE.THERMAL then
        return "Thermal"
    elseif mode == PLANE_MODE.LOITER_ALT_QLAND then
        return "Loiter Alt Q Land"
    elseif mode == PLANE_MODE.AUTOLAND then
        return "Autoland"
    end

    return string.format("mode: %d", mode)
end

-- the quadplane singleton is non-nil even when Q_ENABLE is 0, but calling
-- get_mav_vtol_state() in that state dereferences a null pointer and crashes,
-- so also require Q_ENABLE to be set
local quadplane_enabled = quadplane ~= nil and (param:get('Q_ENABLE') or 0) > 0

-- keep local copies of parameter values that the user might change so update ever 5 seconds
local function get_vehicle_state()

    current_loc         = ahrs:get_position()
    current_mode        = vehicle:get_mode()
    if quadplane_enabled then
        vtol_state      = quadplane:get_mav_vtol_state()
    else
        -- not a quadplane, so we are always in fixed wing flight
        vtol_state      = MAV_VTOL_STATE.FW
    end

    now_ms = millis()

    -- refresh parameters every 5 seconds, its not that urgent we know about changs
    if (now_ms - now_params_ms) > 5000 then
        --warn_act = WARN_ACTION:get()
        roll_limit_deg      = ROLL_LIMIT_DEG:get()
        lookahead_param_m     = DAA_LKAHD:get()
        margin_fence_m        = DAA_MARGIN_FENCE:get()
        if margin_fence_m <= 0 then margin_fence_m = math.abs(WP_LOITER_RAD:get()) end   -- 0 => use the turn radius
        margin_alt_m          = DAA_MARGIN_ALT:get()
        alt_hyst_m          = DAA_ALT_HYST_M:get()
        alt_cool_ms         = DAA_ALT_COOL_S:get() * 1000
        loiter_cool_ms      = DAA_LTR_COOL_S:get() * 1000
        margin_aircraft_m     = DAA_MARGIN_GA:get()
        margin_vertical_m     = DAA_MARGIN_GA_Z:get()
        margin_bird_m         = DAA_MARGIN_BIRD:get()
        margin_prey_m         = DAA_MARGIN_PREY:get()
        margin_uav_m          = DAA_MARGIN_UAV:get()
        margin_weather_m      = DAA_MARGIN_WTH:get()
        margin_ais_m          = DAA_MARGIN_AIS:get()
        margin_proximity_m    = DAA_MARGIN_PRX:get()
        refresh_rate        = 1000.0 / DAA_UPDATE_RATE:get()
        bendy_ratio         = DAA_BR_RATIO:get()
        bendy_angle         = DAA_BR_ANGLE:get()
        wp_loiter_rad_m       = WP_LOITER_RAD:get()
        ga_avoid_alt_m        = DAA_AVD_ALT:get()
        ga_avoid_alt_frame  = DAA_AVD_ALT_TP:get()
        daa_alert           = DAA_AVD_ALERT:get()
        daa_action          = DAA_AVD_ACTION:get()
        wind_min_ms         = DAA_WIND_MIN:get()
        wind_margin_per_ms  = DAA_WIND_MARG:get()

        bearing_inc_deg     = DAA_HEADING_INC:get() or DEFAULT_HEADING_INC_DEG
        if bearing_inc_deg <= 0 then
            bearing_inc_deg = DEFAULT_HEADING_INC_DEG
        end

        well_clear_xy        = AVD_WCLR_XY:get()
        well_clear_z         = AVD_WCLR_Z:get()
        uav_clear_xy         = AVD_UAV_XY:get()
        near_miss_xy         = AVD_NMAC_XY:get()
        near_miss_z          = AVD_NMAC_Z:get()
        slew_dps             = DAA_SLEW_DPS:get()
        slew_urg_s           = DAA_SLEW_URG:get()
        side_hold_s          = DAA_SIDE_HOLD:get()
        cpa_min_ms           = DAA_CPA_MIN:get()
        trap_act             = DAA_TRAP_ACT:get()
        trap_s               = DAA_TRAP_S:get()
        trap_clr_s           = DAA_TRAP_CLR_S:get()
        trap_esc_act         = DAA_TRAP_ESC_ACT:get()
        stale_s              = DAA_STALE_S:get()

        now_params_ms       = now_ms
    end
end

-----Auxiliary functions
local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
     if res < 0 then
         res = res + 360.0
     end
     return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

local function length_squared(w)
    return (w:x() * w:x()) + (w:y() * w:y())
end

local function dot_product_2vector(p,w)
    return (w:x() * p:x()) + (w:y() * p:y())
end

-- luacheck: ignore closest_point
local function closest_point(p, w)
    local l2 = length_squared(w)
    if l2 < 1e-6 then
        return w  -- case v == w
    end
    local t = dot_product_2vector(p,w) / l2
    if t <= 0 then
        local Vector2=Vector2f()
        Vector2:x(0)
        Vector2:y(0)
        return Vector2
    elseif t >= 1 then
        return w
    else
        w:x(w:x()*t)
        w:y(w:y()*t)
        return w
    end
end

local function longitude_scale(lat)
    local DEG_TO_RAD = math.pi / 180
    local scale = math.cos(lat * (1.0e-7 * DEG_TO_RAD))
    return math.max(scale, 0.01)
end

local function limit_latitude(lat)
    if lat > 900000000 then
        lat = 1800000000 - lat
    elseif lat < -900000000 then
        lat = -(1800000000 + lat)
    end
    return lat
end

local function wrap_longitude(lon)
    if lon > 1800000000 then
        lon = lon - 3600000000
    elseif lon < -1800000000 then
        lon = lon + 3600000000
    end
    return lon
end

 -- Extrapolate latitude/longitude given bearing and distance
-- luacheck: ignore offset_bearing
local function offset_bearing(lat, lng, bearing_deg, distance)
    --local radians = math.rad
    local ofs_north = math.cos(math.rad(bearing_deg)) * distance
    local ofs_east  = math.sin(math.rad(bearing_deg)) * distance
    local dlat = ofs_north * LOCATION_SCALING_FACTOR_INV
    local dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat + dlat / 2.0)
    lat = lat + dlat
    lat = limit_latitude(lat)
    lng = wrap_longitude(dlng + lng)
    return lat, lng
end

--[[
    return true if two locations are identical
--]]
local function locations_equal(loc1, loc2)
    if loc1 == nil and loc2 == nil then
        return true
    end
    if (loc1 == nil and loc2 ~= nil) or (loc1 ~= nil and loc2 == nil) then
        return false
    end
    return (loc1:lat() == loc2:lat()) and (loc1:lng() == loc2:lng())
            and (loc1:alt() == loc2:alt())
            and (loc1:get_alt_frame() == loc2:get_alt_frame())
end

-- Project forward from loc1 to a newlocation in the direction bearing_deg and distance m
-- the altitude of the new projected location should be based on alt_target_loc, including frame
local function location_project(loc1, bearing_deg, distance, alt_target_loc)
    -- Create a copy of the location projected distance meters in bearing_deg direction
    -- the projection should be in the frame project_in_frame
    --local loc2 = Location()
    --loc2=loc1
    --loc2:offset_bearing(bearing_deg, distance)
    -- local loc2 = alt_target_loc:copy()
    -- can now do this with Location
    --local lat, lon = offset_bearing(loc1:lat(), loc1:lng(), bearing_deg, distance)
    --loc2:lat(lat)
    --loc2:lng(lon)

    local loc2 = loc1:copy()
    loc2:offset_bearing(bearing_deg, distance)
    loc2:copy_alt_from(alt_target_loc)

--    void offset_bearing(ftype bearing_deg, ftype distance);

    -- use the altitude/frame copied from loc1
    --loc2:alt(loc1:alt())
    --gcs:send_text(0, string.format("IN: Lat: %.2f, Lon: %.2f", loc1:lat(),loc1:lng()))
    --gcs:send_text(0, string.format("Dist: %.2f, Bear: %.2f", distance,bearing_deg)) 
    --gcs:send_text(0, string.format("Out: Lat: %.2f, Lon: %.2f", loc2:lat(),loc2:lng()))  
    --gcs:send_text(0, string.format("Frame: IN: %d, OUT: %d", loc1:get_alt_frame(),loc2:get_alt_frame()))
    return loc2
end

-- make obstacle labels a bit more meaningful for user especially for crude aircraft and MAVLink vehicles
local function pretty_label(script_obstacle)
    local obstacle_type = script_obstacle:obstacle_type()
    local emitter_type  = script_obstacle:emitter_type()

    -- a MAVLink drone (GLOBAL_POSITION_INT/FOLLOW_TARGET) carries a small MAV system id in
    -- src_id; an ADSB-sourced drone (emitter 14) carries a 24-bit ICAO address there instead,
    -- so show that in hex rather than a meaningless decimal "SYSID" (0xBFFF matches the C++ split)
    if script_obstacle:is_drone() == true or emitter_type == ADSB_EMITTER.UAV then
        if script_obstacle:src_id() > 0xBFFF then
            return string.format("Drone:%06X", script_obstacle:icao_code())
        end
        return string.format("SYSID:%d", script_obstacle:src_id())

    -- this will have arrived as an ADSB_VEHICLE
    elseif script_obstacle:is_aircraft() == true or emitter_type == 100
            or (emitter_type >= ADSB_EMITTER.LIGHT and emitter_type <= ADSB_EMITTER.AIRCRAFT_HIGH) then
        return string.format("%06X", script_obstacle:icao_code())

    -- fake generated obstacles from mavproxy_genobstacles have these special case "emitters" for SITL/testing
    elseif emitter_type == 99 then
        return "Obstacle"
    elseif emitter_type == 101 then
        return "Drone"
    elseif emitter_type == 102 then
        return "Weather"
    elseif emitter_type == 103 then
        return "Migratory Bird"
    elseif emitter_type == 104 then
        return "Bird of Prey"

    -- these obstacle types are returned by AP_OAScripting for fences
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
        return "Excl. Circle"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
        return "Incl. Circle"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION then
        return "Excl. Polygon"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION then
        return "Incl. Polygon"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_HOME then
        return "Tin Can"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_LUA then
        return "Lua Fence"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX then
        return "Alt Max Fence"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN then
        return "Alt Min Fence"
    end
    -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": UNKNOWN: " .. script_obstacle:icao_code() .. " drone? " .. script_obstacle:is_drone() .. " aircraft:" .. script_obstacle:is_aircraft() .. " type: " .. obstacle_type)
    return "Unknown"
end

local function pretty_obstacle_type(type, src_id)
    if type == OBSTACLE_TYPE.GENERAL then
        return "general"
    end
    if type == OBSTACLE_TYPE.MAV_SYSID then
        -- a small src_id is a real MAVLink drone; a 24-bit ICAO (>0xBFFF) is an
        -- ADS-B drone (matches the label split in pretty_label)
        if src_id ~= nil and src_id > 0xBFFF then
            return "adsbdrone"
        end
        return "mavdrone"
    end
    if type == OBSTACLE_TYPE.GENERAL_AVIATION then 
        return "aircraft"
    end
    if type == OBSTACLE_TYPE.WEATHER then
        return "weather"
    end
    if type == OBSTACLE_TYPE.BIRD_MIGRATORY then
        return "bird"
    end
    if type == OBSTACLE_TYPE.BIRD_OF_PREY then
        return "predator"
    end
    if type == OBSTACLE_TYPE.FENCE_HOME then
        return "fence:home"
    end
    if type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
        return "fence:circle-inc"
    end
    if type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
        return "fence:circle-exc"
    end
    if type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION then
        return "fence:poly-inc"
    end
    if type == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION then
        return "fence:poly-exc"
    end
    if type == OBSTACLE_TYPE.FENCE_LUA then
        return "fence:lua"
    end
    if type == OBSTACLE_TYPE.FENCE_ALT_MAX then
        return "fence:alt-max"
    end
    if type == OBSTACLE_TYPE.FENCE_ALT_MIN then
        return "fence:alt-min"
    end
    if type == OBSTACLE_TYPE.PROXIMITY then
        return "proximity"
    end
    if type == OBSTACLE_TYPE.AIS then
        return "ship"
    end
    return "unknown"
end

local function populate_obstacle(distance_m, any_obstacle)
    local obstacle = {}
    obstacle.distance_m   = distance_m                      -- this is the Projected distance based on lookahead
    obstacle.sysid        = any_obstacle:src_id()
    obstacle.icao_code    = any_obstacle:icao_code()
    obstacle.type         = any_obstacle:obstacle_type()
    obstacle.timestamp_ms = any_obstacle:timestamp_ms()    -- last update time (millis); stale = laggy feed
    obstacle.label        = pretty_label(any_obstacle)
    obstacle.location     = any_obstacle:location()
    obstacle.pos_NED_m    = any_obstacle:position_NED_m()
    obstacle.vel_NED_ms   = any_obstacle:velocity_NED_ms()
    -- these are the actual distances based on current location with no lookahead
    -- note that for polygons there is no "location", so it's not easy to find the simple distance, so we just use the OA distance
    if obstacle.location == nil then
        obstacle.distance_xy = obstacle.distance_m
        obstacle.distance_z  = 0
    else
        obstacle.distance_xy  = obstacle.location:get_distance(current_loc)
        obstacle.distance_z   = math.abs(obstacle.location:get_distance_NED(current_loc):z())
        if obstacle.type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
            obstacle.distance_xy = obstacle.distance_xy - any_obstacle:radius_m() - any_obstacle:margin_m()
        elseif obstacle.type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
            -- not too sure about the math here
            obstacle.distance_xy = any_obstacle:radius_m() - obstacle.distance_xy - any_obstacle:margin_m()
        end
    end

    return obstacle
end

-- Real current horizontal distance (m) to the physical obstacle, for the AVOIDING message.
-- distance_m is the bendy-ruler PROJECTED distance (along the lookahead ray), not a real range.
-- Returns nil when we can't determine it simply (Lua fence, alt fence, no location) so the
-- message omits the distance rather than mislead.  Called ONLY at announce time (not in the
-- bendy-ruler sweep) so the per-call fence_distance loop stays out of the hot path.
local function obstacle_report_distance(obstacle)
    if current_loc == nil then return nil end
    local t = obstacle.type
    if t == OBSTACLE_TYPE.FENCE_HOME
        or t == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION
        or t == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION
        or t == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or t == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION
        or t == OBSTACLE_TYPE.FENCE_LUA then
        -- horizontal fences carry no single usable "location"; ask C++ for the real edge distance
        -- to a fence of THIS type (so an "Excl. Circle" label reports the nearest exclusion
        -- circle, not a nearer polygon).  Returns the distance in metres, or nil if none.
        return OAScripting:fence_distance(current_loc, t)
    elseif t ~= OBSTACLE_TYPE.FENCE_ALT_MAX and t ~= OBSTACLE_TYPE.FENCE_ALT_MIN
            and obstacle.location ~= nil then
        -- point/traffic obstacle (aircraft/drone/bird/AIS/proximity): location is a real position
        return obstacle.location:get_distance(current_loc)
    end
    return nil
end

-- Keep-out ("well clear") radius (m) for the CPA conflict test, per obstacle type. This is the
-- miss distance below which a moving obstacle is treated as a conflict; because the range check in
-- assess_obstacle_motion uses the same value, it is also the range inside which avoidance is
-- unconditional (the conservative "safer is better" floor). Distinct from the detection margin in
-- find_closest_obstacle, which adds a look-ahead buffer on top so obstacles are picked up earlier.
-- Aircraft/drone/plane share one CPA calculation; only this standoff differs.
local function get_standoff(obstacle_type)
    if obstacle_type == OBSTACLE_TYPE.MAV_SYSID then
        return uav_clear_xy                 -- drone/UAV: AVD_UAV_XY
    elseif obstacle_type == OBSTACLE_TYPE.BIRD_MIGRATORY then
        return margin_bird_m
    elseif obstacle_type == OBSTACLE_TYPE.BIRD_OF_PREY then
        return margin_prey_m
    elseif obstacle_type == OBSTACLE_TYPE.WEATHER then
        return margin_weather_m
    elseif obstacle_type == OBSTACLE_TYPE.PROXIMITY then
        return margin_proximity_m
    end
    -- GENERAL_AVIATION, AIS and anything else: the aircraft well-clear radius (AVD_WCLR_XY)
    return well_clear_xy
end

local function find_closest_obstacle(loc1, loc2, lookahead_m, wind_ms)
    -- By projecting 1m along the line we avoid a problem with the
    -- exclusion avoidance being happy to skirt along a line parallel
    -- to an exclusion zone
    local bearing_deg   = math.deg(loc1:get_bearing(loc2))
    local loc1_shifted  = location_project(loc1, bearing_deg, 1, loc2)
    local obstacle

    local distance_m, any_obstacle, _, _, _  =
            OAScripting:find_threats(loc1_shifted, loc2, lookahead_m)

    if distance_m == nil then
        return FLT_MAX, nil
    end

    if any_obstacle == nil then
        return FLT_MAX, nil
    end

    local obstacle_type_val = any_obstacle:obstacle_type()

    local obstacle_margin = 0;
    --[[
    These are currently handled inside find_threats, it would be better if they could be parameterized
    if obstacle_type_val == OBSTACLE_TYPE.GENERAL_AVIATION then
        obstacle_margin = margin_aircraft_m
    elseif obstacle_type_val == OBSTACLE_TYPE.MAV_SYSID then
        obstacle_margin = margin_uav_m
    elseif obstacle_type_val == OBSTACLE_TYPE.BIRD_MIGRATORY then
        obstacle_margin = margin_bird_m
    elseif obstacle_type_val == OBSTACLE_TYPE.BIRD_OF_PREY then
        obstacle_margin = margin_prey_m
    elseif obstacle_type_val == OBSTACLE_TYPE.WEATHER then
        obstacle_margin = margin_weather_m
    else
    --]]
    if obstacle_type_val == OBSTACLE_TYPE.GENERAL_AVIATION then
        obstacle_margin = well_clear_xy + margin_aircraft_m
    elseif obstacle_type_val == OBSTACLE_TYPE.MAV_SYSID then
        -- drone/UAV (ADSB emitter 14): mirrors the GA line but with the UAV horizontal
        -- reference (AVD_UAV_XY) instead of the aircraft "well clear" (AVD_WCLR_XY), and
        -- no aircraft loiter-to-alt; bendy ruler handles it via obstacle_avoiding
        obstacle_margin = uav_clear_xy + margin_uav_m
    elseif obstacle_type_val == OBSTACLE_TYPE.AIS then
        obstacle_margin = well_clear_xy + margin_ais_m
    elseif obstacle_type_val == OBSTACLE_TYPE.PROXIMITY then
        obstacle_margin = margin_proximity_m
    elseif obstacle_type_val == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_HOME
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_LUA
        then
        obstacle_margin = margin_fence_m
        -- widen the standoff in wind so the controller has buffer to absorb cross-track
        -- drift and is less likely to be blown across the fence (DAA_WIND_MARG = 0 disables)
        if wind_ms ~= nil and wind_ms > wind_min_ms then
            obstacle_margin = obstacle_margin + wind_margin_per_ms * (wind_ms - wind_min_ms)
        end
    end

    -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle:label() .. " : " .. distance_m)
    if distance_m > obstacle_margin then
        -- we are further away from the obstacle than we care about
        return FLT_MAX, nil
    end

    -- If any fence is currently breached, skip all fence avoidance.
    -- When inside an exclusion zone or outside an inclusion zone the bendy ruler sees every
    -- exit/return path as "blocked", trapping the plane. Let it navigate freely instead.
    local is_fence_type = obstacle_type_val == OBSTACLE_TYPE.FENCE_HOME
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION
        or obstacle_type_val == OBSTACLE_TYPE.FENCE_LUA
    if is_fence_type and fence ~= nil and fence:get_breaches() ~= 0 then
        return FLT_MAX, nil
    end

    obstacle = populate_obstacle(distance_m, any_obstacle)

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle:icao_code() .. " drone? " .. any_obstacle:is_drone() .. " aircraft:" .. any_obstacle:is_aircraft())

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle.: .. " :" .. obstacle.label)

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle.obstacle_type() .. " :" .. obstacle.label)

    return distance_m, obstacle
end

--[[
    calculate what our ground speed would be in a given direction, using wind estimate
--]]
local function effective_groundspeed(airspeed, bearing_deg, wind_dir_rad, wind_speed)
    -- Ensure airspeed is at least 1.0
    airspeed = math.max(airspeed, 1.0)
    -- Convert bearing to radians
    local bearing_rad = math.rad(bearing_deg)    
    -- Calculate the angle between wind direction and bearing
    local temp = math.pi - (wind_dir_rad - bearing_rad)
    local dangle = wind_speed * math.sin(temp) / airspeed
    -- If dangle is out of valid range, return 0
    if dangle > 1.0 or dangle < -1.0 then
        return 0
    end
    -- Calculate the angle alpha using arcsine
    local alpha = math.asin(dangle)    
    -- Calculate yaw
    local yaw = bearing_rad - alpha    
    -- Calculate beta, the angle between wind direction and yaw
    local beta = math.pi - (wind_dir_rad - yaw)    
    -- Calculate ground speed squared (gs2)
    local gs2 = airspeed^2 + wind_speed^2 - 2 * airspeed * wind_speed * math.cos(beta)    
    -- If gs2 is negative or zero, return 0
    if gs2 <= 0 then
        return 0
    end
    -- Calculate the final ground speed
    local gs = math.sqrt(gs2)
    --gcs:send_text(0, string.format("as:%.1f bear:%.1f wind_dir:%.1f ws:%.1f -> gs:%.1f", airspeed, bearing_deg, math.deg(wind_dir_rad), wind_speed, gs)) 
    return gs
end

-------------------------------------------------------------------------------
-- LOITER ALTITUDE - Loiter right or left to (usually) lose altitude to avoid an obstacle (usually a crude aircraft)
-------------------------------------------------------------------------------
local loiteralt = {
    active = false,
}
(function ()
    local pre_loiteralt_heading_deg = -1.0
    local previous_mode = -1
    local target_alt_m = nil
    local target_alt_frame = ALT_FRAME.GLOBAL

    function loiteralt.start(new_alt_m, new_alt_frame, direction_right, _speed_ms)
        local direction

        if loiteralt.active then
            -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": loiteralt ALREADY ACTIVE: ")
            return nil
        end

        -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": loiteralt starting: " .. target_alt_m)

        if current_loc == nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": loiteralt no current_location")
            return nil
        end
        pre_loiteralt_heading_deg   = math.deg(ahrs:get_yaw_rad())
        target_alt_frame            = new_alt_frame
        target_alt_m                = new_alt_m

        --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt start hdg pre %.0f now %.0f dif %0.1f alt %.0f",
        --                    pre_loiteralt_heading_deg, current_heading_deg
        --                    , math.abs(pre_loiteralt_heading_deg - current_heading_deg), target_alt_m) )

        -- use the configured loiter radius (a groundspeed-based "standard turn"
        -- radius, (60.0 * speed) / math.pi, was tried previously but not used)
        local radius_m = wp_loiter_rad_m
        local loiteralt_loc = current_loc:copy()
        if direction_right then
            direction = "right"
            loiteralt_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg + 90), radius_m)
        else
            direction = "left"
            loiteralt_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg - 90), radius_m)
        end
        --local new_target_loc, direction = start_loiter(loiteralt.target_alt_agl_m, direction_right, groundspeed_ms)
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": LOITER %s to %.0f/%.0f(%.0f) alt radius %.0f m",
                direction, target_alt_m, target_alt_frame, mavlink_wrappers.alt_frame_to_mavlink(target_alt_frame), radius_m ))

        previous_mode = vehicle:get_mode()
        vehicle:set_mode(PLANE_MODE.GUIDED)

        if mavlink_wrappers.set_vehicle_target_location({lat    = loiteralt_loc:lat(),
                                                        lng     = loiteralt_loc:lng(),
                                                        alt     = target_alt_m,
                                                        frame   = target_alt_frame,
                                                        radius  = radius_m,
                                                        yaw     = 0 }) then
            loiteralt.active = true
        else
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop set_vehicle FAILED" ))
            loiteralt.stop()
        end

        return nil
    end

    function loiteralt.aircraft_seen()
        aircraft_seen_now_ms = now_ms
    end

    function loiteralt.stop(force_stop)
        if not force_stop then
            -- hold the loiter for DAA_LTR_COOL_S after the aircraft was last seen, so a
            -- briefly-dropped or laggy feed cannot thrash GUIDED<->AUTO
            if (now_ms - aircraft_seen_now_ms) < loiter_cool_ms then
                return false
            end
        end
        if previous_mode > 0 and previous_mode ~= PLANE_MODE.GUIDED then
            vehicle:set_mode(previous_mode)
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Loiter Done set mode: %s", get_mode_string(previous_mode) ))
            gcs:send_named_string("DAA-AVOID", "")
            gcs:send_named_float("DAA-LOITER", 0.0)
        end
        previous_mode = -1
        loiteralt.active = false
        return true
    end

    -- should be called regularly if loiteralt is active
    function loiteralt.update()
        if loiteralt.active and current_mode ~= PLANE_MODE.GUIDED then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Pilot changed from GUIDED to: %.0f", current_mode ))
            previous_mode = -1
            loiteralt.stop(true)
            return
        end
        if not loiteralt.active or current_loc == nil then
            return
        end

        -- if we have achieved the target altitude exit immediately and we are pointing to the next WP
        --local current_agl_m = current_location_agl:alt() * 0.01

        --[[if (math.abs(current_agl_m - loiteralt.target_alt_agl_m)) < 10 and 
                (math.abs(pre_loiteralt_heading_deg - current_heading_deg) < 45.0) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt STOP alt curr %.0f trg %.0f max %0.1f",
                        current_altitude_m, current_amsl_m, altitude_max) )
            loiteralt.stop()
        else
            --airspeed_desired = airspeed_cruise
            --mavlink.set_vehicle_speed({speed=airspeed_desired})
        end
        --]]
    end
end)()

-------------------------------------------------------------------------------
--- DAA (Detect, Alert, Avoid) management class
-------------------------------------------------------------------------------
local DAA = {
   enabled = false,
}
(function ()
    local active            = true;
    local navigating        = false;
    local current_loc       = ahrs:get_position() -- luacheck: ignore current_loc
    local groundspeed_ms    = ahrs:groundspeed_vector():length()
    local airspeed_ms       = ahrs:airspeed_EAS() or groundspeed_ms
    local ground_course_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
    local wind_dir_rad      = 0.0
    local wind_speed        = 0.0
    local obstacle_avoiding = nil
    local aircraft_avoiding = nil
    local last_aircraft_ts_ms = nil     -- timestamp_ms of the last aircraft fix acted on (detect de-bounce)
    local last_aircraft_obstacle = nil  -- obstacle populated from it, reused between fresh fixes
    local last_avoid_bearing_deg = nil
    local committed_side_sign = 0       -- +1 = right of the direct bearing, -1 = left, 0 = not committed
    local side_flip_pending  = false    -- true while the opposite side is being preferred (debounce a flip)
    local side_flip_want_ms  = uint32_t(0)  -- time the opposite side first became preferred
    local last_cmd_bearing_ms = uint32_t(0) -- time we last issued an avoidance heading (for the slew-rate dt)
    local trap_active       = false         -- trapped-failsafe is currently controlling the vehicle
    local trap_since_ms     = uint32_t(0)   -- when the trap condition began (for DAA_TRAP_S)
    local trap_trigger_ms   = uint32_t(0)   -- when the failsafe fired (for DAA_TRAP_CLR_S recovery)
    local trap_dynamic      = false         -- trap from a moving obstacle (recoverable) vs a fence (sticky)
    local trap_prev_mode    = -1            -- mode to restore on recovery
    local trap_fs_mode      = -1            -- the failsafe mode we commanded
    local previous_label    = ""
    local avoiding_label    = ""
    -- laggy/dropped traffic-feed watchdog (network-fed moving obstacles carry an update
    -- timestamp; a fence's is always fresh).  Threshold is DAA_STALE_S (0 disables).
    local feed_watch_label  = ""            -- label of the moving obstacle we are tracking ("" = none)
    local feed_is_stale     = false         -- its last-seen update was stale
    local feed_stale_warn_ms = uint32_t(0)  -- throttle for the "traffic stale" GCS text
    -- luacheck: ignore previous_aircraft
    local previous_aircraft = ""
    local STATE             = {monitoring = 1, avoiding = 2, loitering = 3, loitering_avoiding = 4, hovering = 5, landing  = 6}
    local current_state     = STATE.monitoring
    local LoWC_active       = false
    local LoWC_label        = ""
    local NMAC_active       = false
    local NMAC_label        = ""

    local update_target_location_save_loc = nil                 -- this is the saved current_target for use by update_target_location ONLY
    local navigation_target_loc = nil                           -- this is where the vehicle is trying to get to (i.e. next waypoint if no avoidance)
    local daa_target_loc = nil                                  -- this is where the DAA is currently trying to go in order to avoid obstacles (nil if not avoiding)

    -- the distance we look ahead is adjusted dynamically based on avoidance results
    local current_lookahead = lookahead_param_m

    local function calculate_windspeed()
                -- Get wind estimate and convert to 2D
        local wind_3d = ahrs:get_wind()
        local wind_2d = Vector2f()
        if wind_3d ~= nil then          -- get_wind returns nil when there is no valid estimate: treat as calm
            wind_2d:x(wind_3d:x())
            wind_2d:y(wind_3d:y())
        end

        return  wind_2d:length(), wind_2d:angle()
    end

    -- methods to log DAA results DAAD = Detect, DAAA = Alert, DAAV = aVoid
    local function log_detect_result(obstacle_found, distance_found_m, distance_to_target_m, best_bearing_deg, target_loc, obstacle_type)
        --    log_detect_result(false, distance_found_m, distance_to_target_m, best_bearing_deg, target_loc)
        if target_loc == nil or distance_found_m == nil or distance_to_target_m == nil or best_bearing_deg == nil then
            -- we can't be avoiding if no target, so no loggin required
            return
        end
        --print(distance_found_m)
        --print(distance_to_target_m)
        --print(target_loc:lat())
        --print(target_loc:lng())
        --print(target_loc:alt())

        --logger:write('DAAD',                -- D for Detect
        local status, err = pcall(logger.write, logger, "DAAD",
            'Obs,DstF,DstT,HdgB,Tfnd,TLat,TLng,TAlt,TFra,ObjT',
            'BffffLLfBI',                   -- Formats (L for Lat/Lng, f for Alt)
            '-mmmdDUm--',                   -- Units (D=lat deg, U=lng deg, m=meter)
            '-----GG---',                   -- Multipliers (G=1e-7 for L types)
            (obstacle_found and 1 or 0),    -- Obs - Obstacle found true/false
            distance_found_m,               -- DstF - Distance to found obstacle in meters
            distance_to_target_m,           -- DstT - Distance to proposed new target to avoid the obstacle
            wrap_360(best_bearing_deg),     -- HdgB - Best bearing found to avoid obstacles (0-360 deg)
            (target_loc ~= nil and 1 or 0), -- TFnd - Target found
            target_loc:lat(),               -- TLat - Latitude of proposed new target in degrees
            target_loc:lng(),               -- TLng - Longitude of proposed new target in degrees
            target_loc:alt() * 0.01,        -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame(),     -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            obstacle_type)                  -- ObjT - the OBSTACLE_TYPE of the object detected

        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log detect:" .. tostring(err) )
        end
    end

    local function log_detect_aircraft(aircraft)
        -- a position-less contact (e.g. bearing-only ADS-B) has no location to log
        if aircraft == nil or aircraft.location == nil then
            return
        end

        --logger:write('DAAG',                    -- G for GA = General Aviation
        local status, err = pcall(logger.write, logger, "DAAG",
            'DstF,TLat,TLng,TAlt,TFra,DstH,DstZ,ICAO',
            'fLLfBffI',                          -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mmh',                          -- Units (D=lat deg, U=lng deg, m=meter)
            '-GG-----',                          -- Multipliers (G=1e-7 for L types)
            aircraft.distance_m,                -- DstF - Distance to found aircraft in meters
            aircraft.location:lat(),            -- TLat - Latitude of proposed new target in degrees
            aircraft.location:lng(),            -- TLng - Longitude of proposed new target in degrees
            aircraft.location:alt() * 0.01,     -- TAlt - Alitude of proposed new target in meters
            aircraft.location:get_alt_frame(),  -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            aircraft.distance_xy,               -- DstH - Horizontal distance to the aircraft
            aircraft.distance_z,                -- DstZ - Vertical distance to the aircraft (+ve is up)
            aircraft.icao_code                  -- ICAO - the integer value of the ICAO code of the aircraft if available
        )
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log aircraft:" .. tostring(err) )
        end
    end

    -- luacheck: ignore log_alert
    local function log_alert()
    end

    local function log_avoid(obstacle, target_loc)
        if target_loc == nil then
            return
        end
        local status, err = pcall(logger.write, logger, "DAAV",
        --logger:write('DAAV',                        -- V for aVoid
            'DstO,TLat,TLng,TAlt,TFra,DstH,DstZ,ObjT,Age',
            'fLLfBffBf',                            -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mm-s',                            -- Units (D=lat deg, U=lng deg, m=meter, s=second)
            '-GG------',                            -- Multipliers (G=1e-7 for L types)
            obstacle.distance_m,                    -- DstO - Distance to found obstacle in meters
            target_loc:lat(),                       -- TLat - Latitude of DAA target in degrees
            target_loc:lng(),                       -- TLng - Longitude of DAA target in degrees
            target_loc:alt() * 0.01,       -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame(),    -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            obstacle.distance_xy,                   -- DstH - Horizontal distance to the obstacle
            obstacle.distance_z,                    -- DstZ - Vertical distance to the aircraft (+ve is up),
            obstacle.type,                          -- ObjT - the type of the obstacle as an OBSTACLE_TYPE
            obstacle.timestamp_ms and ((now_ms:tofloat() - obstacle.timestamp_ms) * 0.001) or 0  -- Age - obstacle position age in s (0 = fresh/on-board)
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
    local function log_smoothing(direct_deg, raw_deg, resisted_deg, final_deg, side, flip, urgent, motion, obstacle)
        local status, err = pcall(logger.write, logger, "DAAS",
            'HdD,HdR,HdS,HdC,Sid,Flp,Urg,Cls,CPA,TTC,PsB,Dst,Typ',
            'ffffbBBfffbfB',                        -- Formats
            'dddd---nms-m-',                        -- Units (d=deg, n=m/s, m=metre, s=second)
            '-------------',                        -- Multipliers
            wrap_360(direct_deg),                   -- HdD - direct bearing to target
            wrap_360(raw_deg),                      -- HdR - raw bendy-ruler bearing
            wrap_360(resisted_deg),                 -- HdS - after clearance hysteresis (pre-smoothing)
            wrap_360(final_deg),                    -- HdC - final commanded bearing (flown)
            side,                                   -- Sid - committed side (-1 left / 0 / +1 right)
            (flip and 1 or 0),                      -- Flp - side-flip debounce pending
            (urgent and 1 or 0),                    -- Urg - slew limit bypassed (urgent)
            motion.closing_speed,                   -- Cls - closing speed
            motion.cpa_miss,                        -- CPA - predicted horizontal miss distance
            math.min(motion.ttc, 999.0),            -- TTC - time to closest approach (capped)
            motion.pass_behind,                     -- PsB - side that passes behind the obstacle
            obstacle.distance_m,                    -- Dst - range to the obstacle
            obstacle.type)                          -- Typ - OBSTACLE_TYPE
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log smoothing:" .. tostring(err))
        end
    end

    -- Sanity-check the DAA parameters against each other and related vehicle params, and
    -- flag disabled features. Advisory only (GCS text, nothing is changed). Fires on every
    -- enable (incl. switch flips). Messages are kept short (STATUSTEXT truncates ~50 chars).
    function DAA.warnings()
        local turn_r = math.abs(wp_loiter_rad_m)
        local function warn(sev, msg)
            gcs:send_text(sev, SCRIPT_NAME_SHORT .. ": " .. msg)
        end
        local W = MAV_SEVERITY.WARNING
        local I = MAV_SEVERITY.NOTICE
        local function vtol_act(a) return a == 2 or a == 3 or a == 4 end
        local have_vtol   = (param:get('Q_ENABLE') or 0) > 0
        local fence_act   = param:get('FENCE_ACTION') or 0
        local adsb_type   = param:get('ADSB_TYPE') or 0
        local cruise_ms   = param:get('AIRSPEED_CRUISE') or 0

        -- turn radius vs avoidance standoffs (WP_LOITER_RAD is the turn radius)
        if turn_r > 0 and margin_fence_m < turn_r then
            warn(W, string.format("MARGIN_FENCE %.0f < turn %.0f: fences may thrash", margin_fence_m, turn_r)) end
        if turn_r > 0 and uav_clear_xy < turn_r then
            warn(W, string.format("AVD_UAV_XY %.0f < turn %.0f: tight drone avoid", uav_clear_xy, turn_r)) end
        -- margin ordering: the aircraft near-miss (NMAC) must sit inside the aircraft
        -- well-clear standoff. NMAC is an aircraft-only boundary, so it is NOT compared
        -- against the drone standoff (AVD_UAV_XY).
        if near_miss_xy >= well_clear_xy then
            warn(W, "NMAC_XY >= WCLR_XY: nearmiss outside wellclr") end
        if near_miss_z >= well_clear_z then
            warn(W, "NMAC_Z >= WCLR_Z: vert nearmiss>wellclr") end
        -- lookahead must give room to react
        if turn_r > 0 and lookahead_param_m < 3 * turn_r then
            warn(W, string.format("LKAHD %.0f < 3x turn %.0f: reacts late", lookahead_param_m, turn_r)) end
        if bendy_ratio > 1.8 then
            warn(W, string.format("BR_RATIO %.1f > 1.8: fence-follow unstable", bendy_ratio)) end
        -- trapped-failsafe consistency
        if trap_act ~= 0 and not have_vtol and (vtol_act(trap_act) or vtol_act(trap_esc_act)) then
            warn(W, "trap VTOL action but Q_ENABLE=0 -> RTL") end
        if trap_act ~= 0 and fence_act ~= 0 then
            warn(I, string.format("TRAP + FENCE_ACTION %.0f: FA pre-empts trap", fence_act)) end
        if trap_act ~= 0 and trap_esc_act == trap_act then
            warn(I, "TRAP_ESC_ACT = TRAP_ACT: no escalation") end
        -- slew limit that can never bind (exceeds the achievable turn rate)
        if turn_r > 0 and cruise_ms > 1 and slew_dps > 0 then
            local turn_rate = math.deg(cruise_ms / turn_r)
            if slew_dps > turn_rate * 1.5 then
                warn(I, string.format("SLEW_DPS %.0f > turn rate %.0f: no effect", slew_dps, turn_rate)) end
        end
        -- disabled features
        if (AVD_ENABLE:get() or 0) ~= 1 then
            warn(W, "AVD_ENABLE != 1: traffic avoidance OFF")
        elseif adsb_type == 0 then
            warn(W, "ADSB_TYPE = 0: no traffic source")
        end
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

    --return true if we are in a state where DAA can apply
    function DAA.isactive()
        return DAA.enabled and active and arming:is_armed()
    end

    -- populate some local values with a static/consistent picture of the vehicle state
    function DAA.get_vehicle_state()
        local current_target_loc = vehicle:get_target_location()

        active      = true;
        current_loc = ahrs:get_position()

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
        update_target_location_save_loc = current_target_loc:copy()

        -- if the navigation target has changed to some other target not the DAA target, it must be vehicle navigation 
        if navigation_target_loc == nil or
            (not locations_equal(navigation_target_loc, current_target_loc) and
                not locations_equal(daa_target_loc, current_target_loc)) then
            -- the vehicle navigation code has changed it's target
            --[[ debug only
            if navigation_target_loc == nil then
                --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target NAV")
            elseif daa_target_loc == nil then
                --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target DAA")
            elseif not locations_equal(daa_target_loc, current_target_loc) then
                --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target diff")
            end
            --]]
            navigation_target_loc = current_target_loc:copy()
        end

        groundspeed_ms              = ahrs:groundspeed_vector():length()
        airspeed_ms                 = ahrs:airspeed_EAS() or groundspeed_ms
        -- Calculate wind direction and speed
        wind_speed, wind_dir_rad    = calculate_windspeed()
        ground_course_deg           = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
    end

    local function calc_avoidance_distance(avoid_step1_m, target_distance)
        -- test for flying past the waypoint, so if we are close, we have room to dodge after the waypoint
        return math.min(avoid_step1_m, target_distance + math.min(margin_fence_m / 2, 100))
    end

    --[[
    This function is called when BendyRuler has found a bearing which is obstacles free at at least lookahead_step1_dist and  then lookahead_step2_dist from the present location
    In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
    It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
    Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees, 
    unless the new margin is atleast _bendy_ratio times better than the margin with previously calculated bearing.
    We return true if we have resisted the change and will follow the last calculated bearing. 
    --]]
    local function resist_bearing_change(bearing_orig_deg, avoid_step1_m, bearing_deg, distance_found_m)
        if bearing_orig_deg == nil then
            -- no prior commitment, accept the proposed bearing
            return bearing_deg
        end
        if distance_found_m == 0 then
            -- obstacle is immediate, must manoeuvre regardless
            return bearing_deg
        end
        if math.abs(wrap_180(bearing_orig_deg - bearing_deg)) < bendy_angle then
            -- proposed change is small enough, no resistance needed
            return bearing_deg
        end
        if current_loc == nil then
            -- no current position to measure against, accept the proposed bearing
            return bearing_deg
        end
        -- measure clearance in the previously committed direction
        local test_loc = current_loc:copy()
        test_loc:offset_bearing(bearing_orig_deg, avoid_step1_m)
        local distance_previous_m, _ = find_closest_obstacle(current_loc, test_loc, avoid_step1_m, wind_speed)
        -- Only switch sides if the new direction is significantly better: POSITIVE clearance
        -- and bendy_ratio times clearer than continuing.  The positive-clearance requirement is
        -- what makes this negative-aware: when hugging a boundary both clearances read near-zero
        -- or negative and a plain ratio test flip-flops every cycle (the fence-skirt oscillation),
        -- so we hold the committed escape direction through the hug.  It still switches away from
        -- a committed side that is itself breaching (distance_previous_m < 0) towards a side that
        -- actually clears (distance_found_m > 0), so containment is preserved.
        if distance_found_m > 0 and distance_found_m >= bendy_ratio * distance_previous_m then
            return bearing_deg
        end
        -- new direction is not significantly better — stay the course
        return bearing_orig_deg
    end

    --[[
    Velocity-aware assessment of a (possibly moving) obstacle. Uses the obstacle's
    ADS-B velocity plus our own velocity to reason about the encounter over time
    rather than from its instantaneous position (which is what makes bendy ruler
    wiggle against a moving target). Returns:
      is_conflict   - false when a moving obstacle is opening range and its predicted
                      closest approach stays beyond the well-clear distance (it is leaving)
      pass_behind   - +1/-1 the side of the direct bearing that passes behind the
                      obstacle's track (0 when it is not usefully moving)
      ttc_s         - estimated time to closest approach, for the slew-rate urgency test
    Static obstacles (fences, ~zero velocity) return (true, 0, closing-based ttc) so
    their behaviour is unchanged.
    --]]
    local function assess_obstacle_motion(obstacle)
        if obstacle == nil or current_loc == nil or obstacle.location == nil then
            -- no geometry to assess: treat as a conflict (the safe default is to avoid)
            return { is_conflict = true, closing_speed = 0.0, cpa_miss = 0.0, ttc = FLT_MAX, pass_behind = 0 }
        end
        local rel = current_loc:get_distance_NED(obstacle.location)  -- N,E,D metres to the obstacle
        local rn, re = rel:x(), rel:y()
        local range_h = math.sqrt(rn * rn + re * re)

        local ov = obstacle.vel_NED_ms
        local own = ahrs:get_velocity_NED()
        local ovn = (ov ~= nil) and ov:x() or 0.0
        local ove = (ov ~= nil) and ov:y() or 0.0
        local rvn = ovn - ((own ~= nil) and own:x() or 0.0)   -- relative velocity (obstacle - own), North
        local rve = ove - ((own ~= nil) and own:y() or 0.0)   -- East

        local rel_dot_rv = rn * rvn + re * rve                -- < 0 => range decreasing (closing)
        local rv2 = rvn * rvn + rve * rve
        local closing_speed = (range_h > 0.1) and (-rel_dot_rv / range_h) or 0.0

        -- horizontal closest point of approach
        local t_cpa = (rv2 > 1e-4) and math.max(0.0, -rel_dot_rv / rv2) or 0.0
        local miss_n = rn + rvn * t_cpa
        local miss_e = re + rve * t_cpa
        local cpa_miss_h = math.sqrt(miss_n * miss_n + miss_e * miss_e)
        local ttc_s = (closing_speed > 0.1) and (range_h / closing_speed) or FLT_MAX

        -- Type-aware keep-out radius: the miss distance below which this obstacle is a conflict.
        -- The range check uses the same value, so any obstacle already inside the keep-out radius
        -- is unconditionally a conflict (the conservative floor); only one that will miss beyond it
        -- AND is not closing AND is already beyond it is treated as leaving (no manoeuvre needed).
        local standoff_m = get_standoff(obstacle.type)
        local is_conflict = true
        if cpa_miss_h > standoff_m and closing_speed < cpa_min_ms and range_h > standoff_m then
            is_conflict = false
        end

        -- side of the direct bearing that passes behind the obstacle's track
        local pass_behind = 0
        if ov ~= nil and (math.abs(ovn) + math.abs(ove)) > 0.5 then
            local cross = rn * ove - re * ovn                 -- (rel x obstacle_vel) vertical component
            if cross > 0.0 then pass_behind = -1 elseif cross < 0.0 then pass_behind = 1 end
        end

        return {
            is_conflict   = is_conflict,
            closing_speed = closing_speed,
            cpa_miss      = cpa_miss_h,
            ttc           = ttc_s,
            pass_behind   = pass_behind,
        }
    end

    --[[
    Post-process the raw bendy-ruler heading into a smooth command for a MOVING
    obstacle, WITHOUT ever overriding a turn the sweep needs to clear an obstacle.

    resist_bearing_change() already produces a bearing that clears ALL obstacles
    (fences included) and only makes a large change when the new side is clearly
    clearer. So:
      * A large change from the last command (> DAA_BR_ANGLE), or an urgent
        encounter, is a genuine avoidance turn -> obey it exactly, no smoothing.
        (This is the safety fix: previously the side-commit could mirror such a
        turn onto the committed side and the slew limit could throttle it, flying
        the aircraft into the very fence the sweep was turning away from.)
      * Only SMALL residual changes (the left/right jitter) are damped, via a side
        commitment and a heading slew-rate limit. When holding a committed side we
        keep the LAST FLOWN bearing (known clear) rather than a mirrored one that
        was never clearance-checked.
    --]]
    local function refine_avoidance_bearing(direct_bearing_deg, raw_bearing_deg, raw_distance_m, motion, obstacle)
        local pass_behind = motion.pass_behind
        local ttc_s = motion.ttc

        -- (1) clearance hysteresis: the safe, multi-obstacle, anti-flip baseline
        local resisted = resist_bearing_change(last_avoid_bearing_deg, current_lookahead, raw_bearing_deg, raw_distance_m)
        local bearing = resisted

        local urgent = (ttc_s ~= nil) and (ttc_s < slew_urg_s)
        local change = (last_avoid_bearing_deg ~= nil) and math.abs(wrap_180(resisted - last_avoid_bearing_deg)) or 999.0

        if last_avoid_bearing_deg == nil or urgent or change > bendy_angle then
            -- necessary avoidance turn (or first cycle): obey the sweep exactly and
            -- (re)commit the side to it; never smooth this away
            committed_side_sign = 0
            side_flip_pending = false
        else
            -- small adjustment only: damp the jitter
            -- (2) side commitment
            local off = wrap_180(bearing - direct_bearing_deg)
            local side = 0
            if off > 1.0 then side = 1 elseif off < -1.0 then side = -1 end
            if side_hold_s > 0 then
                if committed_side_sign == 0 then
                    -- fresh episode: commit; prefer passing behind a moving obstacle
                    committed_side_sign = (pass_behind ~= 0) and pass_behind or side
                    side_flip_pending = false
                elseif side ~= 0 and side ~= committed_side_sign then
                    -- sweep wants the opposite side: only honour it once it has persisted;
                    -- meanwhile hold the last flown (clearance-proven) bearing, do NOT mirror
                    if not side_flip_pending then
                        side_flip_pending = true
                        side_flip_want_ms = now_ms
                    end
                    if (now_ms - side_flip_want_ms) < (side_hold_s * 1000) then
                        bearing = last_avoid_bearing_deg
                    else
                        committed_side_sign = side
                        side_flip_pending = false
                    end
                else
                    side_flip_pending = false
                end
            end

            -- (3) heading slew-rate limit (small changes only; large turns bypassed above)
            if slew_dps > 0 and last_avoid_bearing_deg ~= nil then
                local dt = (now_ms - last_cmd_bearing_ms):tofloat() / 1000.0
                local max_step = slew_dps * dt
                if max_step > 0 then
                    local d = wrap_180(bearing - last_avoid_bearing_deg)
                    if d > max_step then d = max_step elseif d < -max_step then d = -max_step end
                    bearing = wrap_360(last_avoid_bearing_deg + d)
                end
            end
        end
        last_cmd_bearing_ms = now_ms

        log_smoothing(direct_bearing_deg, raw_bearing_deg, resisted, bearing,
                      committed_side_sign, side_flip_pending, urgent, motion, obstacle)
        return bearing
    end

    -- calculates the second step of the bendy ruler test - look foward a 2nd "full_distance" to see if we can still avoid obstacles
    local function test_step2(loc_test, target_bearing, avoid_step2_m, _full_distance)
        local test_bearings = { 0, 45, -45 }
        local target_loc = loc_test:copy()
        target_loc:offset_bearing(target_bearing, avoid_step2_m)

        -- local obstacle_found = {}
        local bearing_found = target_bearing
        local closest_distance_m  = FLT_MAX

        for _, delta in ipairs(test_bearings) do
            local bearing_test = target_bearing + delta

            local target_distance   = loc_test:get_distance(target_loc)
            local distance          = calc_avoidance_distance(avoid_step2_m, target_distance)
            local loc_test2         = location_project(loc_test, bearing_test, distance, target_loc)

            local distance_m, _     = find_closest_obstacle(loc_test2, target_loc, current_lookahead, wind_speed)

            if distance_m > current_lookahead then
                -- return immediately - no obstacles in this direction
                return bearing_test, distance_m
            end
            if distance_m < closest_distance_m then
                -- return the bearing to the nearest obstacle
                bearing_found       = bearing_test
                closest_distance_m  = distance_m
            end
        end

        return bearing_found, closest_distance_m
    end

    -- This method calculates the projected location in the desired direction taking account of airspeed, windspeed and the time it takes to turn
    local function location_after_course_change(from_loc, course_deg, to_loc)
        local course_change_deg = wrap_180(course_deg - ground_course_deg)
        local ground_speed_ms = effective_groundspeed(airspeed_ms, course_deg, wind_dir_rad, wind_speed) -- ground speed based on the new bearing (accounting for wind)
        local rate_of_turn_dps = math.deg(GRAVITY_MSS * math.tan(math.rad(roll_limit_deg * 0.6)) / (airspeed_ms + 0.1))

        if math.abs(course_change_deg) > 170 then
            -- Skip 180-degree turns as we can't predict the turn direction
            return from_loc
        end
        -- Calculate how long it will take to change course
        local turn_time_s = math.abs(course_change_deg / rate_of_turn_dps)
        -- Approximate turn by flying forward for half of the turn time
        local projected_loc = location_project(from_loc, ground_course_deg, ground_speed_ms * turn_time_s * 0.5, to_loc)
        -- If turning more than 90 degrees, add sideways movement
        if math.abs(course_change_deg) > 90 then
            local direction = course_change_deg > 0 and (ground_course_deg + 90) or (ground_course_deg - 90)
            local proportion = math.sin(math.rad(math.abs(course_change_deg) - 90))
            projected_loc = location_project(projected_loc, direction, ground_speed_ms * proportion * turn_time_s * 0.5, to_loc)
        end

        return projected_loc
    end

    -- Core clearance probe for an explicit candidate course bearing_test_deg. Returns
    -- (distance_found_m, bearing_test_deg, obstacle_found); a clear course returns
    -- FLT_MAX with obstacle_found == nil. allow_straight lets the unobstructed
    -- straight-ahead path short-circuit (only meaningful for the i == 0 candidate).
    local function probe_bearing(bearing_test_deg, bearing_deg, full_distance, target_loc, allow_straight)
        local avoid_step1_m     = current_lookahead
        local avoid_step2_m     = current_lookahead * 2.0

        -- Start the look-ahead from where we will actually be after turning onto this
        -- candidate course. In wind this leads the carrot, so the search both penalises
        -- headings that drift toward the obstacle and prefers wind-favourable escapes.
        -- Gated on wind speed so calm-air behaviour (and the autotests) is unchanged.
        local adjusted_loc      = current_loc
        if wind_speed > wind_min_ms then
            adjusted_loc = location_after_course_change(current_loc, bearing_test_deg, target_loc)
        end

        -- Position after one step from where we think we will be after turning to bearing_test_deg
        local avoidance_distance_m  = calc_avoidance_distance(avoid_step1_m, full_distance)
        local test_loc              = location_project(adjusted_loc, bearing_test_deg, avoidance_distance_m, target_loc)

        local distance_found_m, obstacle_found = find_closest_obstacle(adjusted_loc, test_loc, current_lookahead, wind_speed)
        if distance_found_m == nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "closest returned NIL ")
            return FLT_MAX, bearing_deg, nil -- no avoidance required
        end
        if distance_found_m > current_lookahead then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            local bearing2_deg, distance2_m = test_step2(test_loc, bearing_test_deg, avoid_step2_m, current_lookahead)
            if distance2_m >= current_lookahead then
                if allow_straight and bearing2_deg == bearing_deg then
                    -- means we have a direct unobstructed path for step1 and step2
                    return FLT_MAX, bearing_deg, nil -- no avoidance required
                end
                -- we've found at least one direction where there is no obstacle at least for 2 steps out
                distance_found_m = distance_found_m + distance2_m
            end
        end

        return distance_found_m, bearing_test_deg, obstacle_found
    end

    -- This method checks whether we will collide with any obstacle if we fly at a given bearing bearing_deg + i * bearing_inc_deg
    local function test_step1(full_distance, bearing_deg, i, target_loc)
        local bearing_delta_deg = i * bearing_inc_deg / 2.0
        if i % 2 == 1 then
            -- Alternate between left and right of the target
            bearing_delta_deg = -bearing_delta_deg
        end
        local bearing_test_deg = wrap_180(bearing_deg + bearing_delta_deg)
        return probe_bearing(bearing_test_deg, bearing_deg, full_distance, target_loc, i == 0)
    end

    -- if the plane is currently pointing far away from the target, then assume that we 
    -- will be turning sharply, so we don't look too far ahead for obstacles
    local function limit_distance(from_loc, to_loc, bearing_deg)
        local distance_to_target_m = from_loc:get_distance(to_loc)

        if (math.abs(wrap_180(bearing_deg - ground_course_deg)) > bendy_angle * 2) then
            distance_to_target_m = wp_loiter_rad_m * 3
        end

        return distance_to_target_m
    end

    -- AC_FENCE_TYPE bits (see AC_Fence.h) for the altitude fences we handle here
    local FENCE_TYPE_ALT_MAX = 1    -- FENCE_TYPE bit 0
    local FENCE_TYPE_ALT_MIN = 8    -- FENCE_TYPE bit 3

    -- Clamp a target location's altitude into the safe altitude-fence band, leaving a DAA_MARGIN_ALT buffer
    -- inside the fence's own safe limits. This is the "continue" half of clamp-and-continue: the horizontal
    -- path is untouched, only the commanded altitude is corrected. Applied to every target we command via
    -- update_target_location(), so it also enforces the band while avoiding a horizontal obstacle.
    local function clamp_alt_to_fence(loc)
        if loc == nil or fence == nil then
            return
        end
        local enabled = fence:get_enabled_fences()
        if (enabled & FENCE_TYPE_ALT_MAX) ~= 0 then
            local safe_max, frame_max = fence:get_safe_alt_max()
            local ceiling = safe_max - margin_alt_m
            local cur = loc:get_alt_m(frame_max)
            if cur ~= nil and cur > ceiling then
                loc:set_alt_m(ceiling, frame_max)
            end
        end
        if (enabled & FENCE_TYPE_ALT_MIN) ~= 0 then
            local safe_min, frame_min = fence:get_safe_alt_min()
            local floor_alt = safe_min + margin_alt_m
            local cur = loc:get_alt_m(frame_min)
            if cur ~= nil and cur < floor_alt then
                loc:set_alt_m(floor_alt, frame_min)
            end
        end
    end

    -- altitude fences have no horizontal location, so build a lightweight obstacle for alerting/telemetry only.
    -- headroom_m is the (positive) distance from the current altitude to the safe fence limit, reported in the alert.
    local function make_alt_fence_obstacle(otype, label_str, headroom_m)
        local obstacle = {}
        obstacle.distance_m  = headroom_m
        obstacle.sysid       = 0
        obstacle.icao_code   = 0
        obstacle.type        = otype
        obstacle.label       = label_str
        obstacle.location    = nil
        obstacle.pos_NED_m   = nil
        obstacle.vel_NED_ms  = nil
        obstacle.distance_xy = headroom_m
        obstacle.distance_z  = headroom_m
        return obstacle
    end

    -- latch + hysteresis for the altitude-fence trigger. The proactive projection crosses the limit
    -- intermittently as the plane climbs toward then levels off at the clamp altitude; without a latch
    -- the trigger toggles and re-alerts every few seconds. Once engaged we stay engaged (so the alert
    -- de-dupes to one message and the clamp holds steady) until the plane is clearly back in safe air.
    local alt_fence_active = false
    local alt_fence_near = false
    local last_alt_alert_ms = uint32_t(0)

    -- Proactively detect that we are approaching (or projected to cross) an altitude fence.
    -- Only kicks in for the fences enabled in FENCE_TYPE: bit 0 (ALT_MAX) and/or bit 3 (ALT_MIN).
    -- The vertical position is projected forward using the current climb rate over the time it takes to fly
    -- the lookahead distance (capped to a sane vertical horizon) so we level off before the band is reached.
    -- Returns a synthetic obstacle while corrective action is needed (latched), otherwise nil.
    local function detect_altitude_fence()
        if fence == nil or current_loc == nil then
            alt_fence_active = false
            alt_fence_near = false
            return nil
        end
        local enabled = fence:get_enabled_fences()
        local alt_max_on = (enabled & FENCE_TYPE_ALT_MAX) ~= 0
        local alt_min_on = (enabled & FENCE_TYPE_ALT_MIN) ~= 0
        if not alt_max_on and not alt_min_on then
            alt_fence_active = false
            alt_fence_near = false
            return nil
        end

        -- climb rate (m/s, positive up) for proactive projection
        local climb_rate_ms = 0.0
        local vel_ned = ahrs:get_velocity_NED()
        if vel_ned ~= nil then
            climb_rate_ms = -vel_ned:z()
        end
        -- project over the time to fly the lookahead distance, capped to a sensible vertical horizon
        local horizon_s = lookahead_param_m / math.max(groundspeed_ms, 1.0)
        horizon_s = math.min(math.max(horizon_s, 1.0), 20.0)

        -- pick whichever enabled altitude fence currently needs (or is already taking) action.
        -- enter when current or projected altitude is past the clamp limit; while latched, only release
        -- once we are DAA_ALT_HYST_M clear of the limit on both current and projected altitude.
        local otype, label_str, headroom_m
        if alt_max_on then
            local safe_max, frame_max = fence:get_safe_alt_max()
            local cur = current_loc:get_alt_m(frame_max)
            if cur ~= nil then
                local ceiling = safe_max - margin_alt_m
                local projected = cur + climb_rate_ms * horizon_s
                local enter = cur > ceiling or projected > ceiling
                local clear = cur < ceiling - alt_hyst_m and projected < ceiling - alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MAX, "Alt Max Fence"
                    headroom_m = safe_max - cur     -- metres below the safe ceiling
                end
            end
        end
        if label_str == nil and alt_min_on then
            local safe_min, frame_min = fence:get_safe_alt_min()
            local cur = current_loc:get_alt_m(frame_min)
            if cur ~= nil then
                local floor_alt = safe_min + margin_alt_m
                local projected = cur + climb_rate_ms * horizon_s
                local enter = cur < floor_alt or projected < floor_alt
                local clear = cur > floor_alt + alt_hyst_m and projected > floor_alt + alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MIN, "Alt Min Fence"
                    headroom_m = cur - safe_min     -- metres above the safe floor
                end
            end
        end

        local now_active = label_str ~= nil

        -- Announce once when we actually level off near the limit (within the clamp band), not while
        -- merely projecting a distant crossing. The "near" latch + cooldown collapses the brief
        -- trigger drop-outs during a long climb/descent into a single notice; the steady clamp is silent.
        -- The reported distance is the steady-state clearance the plane settles at (DAA_MARGIN_ALT),
        -- not the trigger headroom.
        local near = now_active and headroom_m ~= nil and headroom_m <= (margin_alt_m + alt_hyst_m)
        if near and not alt_fence_near and (now_ms - last_alt_alert_ms) > alt_cool_ms then
            gcs:send_named_string("DAA-ALERT", "alt-fence")
            gcs:send_named_string("DAA-OBSTCL", label_str)
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" levelling off %.0fm from %s",
                                margin_alt_m, label_str))
            gcs:send_named_float("DAA-DISTZ", margin_alt_m)
            last_alt_alert_ms = now_ms
        end

        alt_fence_near = near
        alt_fence_active = now_active
        if not now_active then
            return nil
        end
        return make_alt_fence_obstacle(otype, label_str, math.max(headroom_m, 0.0))
    end

    -- crude aircraft are a special case. We do specific things if there is an aircraft nearby so we need to know the nearest one
    local function detect_aircraft()
    --[[ local obstacle = {}

        obstacle.distance_m,
        obstacle.type,
        obstacle.label,
        obstacle.sysid,
        obstacle.location,
        obstacle.post_NED_m,
        obstacle.velocity
    --]]

        if current_loc == nil then
            aircraft_avoiding = nil
            last_aircraft_obstacle = nil
            last_aircraft_ts_ms = nil
            return
        end

        -- search out to the well clear distance (plus the GA margin), matching the GA
        -- treatment in the bendy ruler path, so aircraft are detected and logged at a
        -- useful range rather than only once they are within DAA_MARGIN_GA of us
        -- pass the full gate distance for each axis (computed here, applied in C++): the
        -- horizontal gate is well_clear_xy + margin_aircraft_m, the vertical gate is
        -- well_clear_z + margin_vertical_m
        local distance_m, aircraft_obstacle = OAScripting:find_aircraft(current_loc, well_clear_xy + margin_aircraft_m, well_clear_z + margin_vertical_m)

        if distance_m == nil or aircraft_obstacle == nil then
            aircraft_avoiding = nil
            last_aircraft_obstacle = nil
            last_aircraft_ts_ms = nil
            return
        end

        -- De-bounce the oversampled ADS-B feed: AP_Avoidance re-reports the same fix many
        -- times between genuine updates (~63% of DAAG records were duplicate lat/lng in
        -- log_102). Act only on a fresh fix (new timestamp_ms); on a repeat, reuse the last
        -- obstacle so the loiter latch holds without re-populating or re-logging every
        -- cycle. The timestamp change is the true (~1 Hz) fix rate.
        local ts_ms = aircraft_obstacle:timestamp_ms()
        if last_aircraft_obstacle ~= nil and ts_ms == last_aircraft_ts_ms then
            aircraft_avoiding = last_aircraft_obstacle
            return
        end

        local obstacle = populate_obstacle(distance_m, aircraft_obstacle)
        --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" FOUND AIRCRAFT: %s dist: %.0f m alt: %.0f m", obstacle.label, obstacle.distance_m, obstacle.location:alt() * 0.01 ))

        aircraft_avoiding = obstacle
        last_aircraft_obstacle = obstacle
        last_aircraft_ts_ms = ts_ms

        log_detect_aircraft(aircraft_avoiding)
        --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" DETECTED AIRCRAFT: xy %.0f z %.0f", aircraft_avoiding.distance_xy,aircraft_avoiding.distance_z) )
    end


    -- detect flying objects or fences when flying towards navigation_target_loc
    function DAA.detect()
        -- TODO be smarter about re-populating this
        local obstacle_distance_m = FLT_MAX
        obstacle_avoiding = nil
        aircraft_avoiding = nil

        -- we want to calculate avoidance towards the current NAVIGATION TARGET (navigation_target_loc) - coping to target_loc to avoid changing the copy/pasted code
        if navigation_target_loc == nil or current_loc == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, " AVOIDING: NO TARGET ")
            return
        end
        local target_loc = navigation_target_loc:copy()

        --gcs:send_text(0, "got current lookahead")

        -- local bearing_cd = math.deg(current_loc:get_bearing(target_loc))*100
        local bearing_deg       = math.deg(current_loc:get_bearing(target_loc))
        local best_bearing_deg  = bearing_deg
        local best_distance_m   = -FLT_MAX

        -- get the current ground course
        -- gcs:send_text(0, string.format("bearing_deg : %.2f",bearing_deg ))

        --local distance_to_target_m = current_loc:get_distance(target_loc)
        local distance_to_target_m = limit_distance(current_loc, target_loc, bearing_deg)
        -- If the full distance is less than 20m, no avoidance is needed
        if distance_to_target_m < 20 then
            return nil
        end

        -- Try increments around a circle, alternating left and right. The first heading
        -- that clears all obstacles for two look-ahead steps wins (a bounded downwind
        -- preference is applied afterwards, once we know we are avoiding).
        for i = 0, (360 / bearing_inc_deg) do
            local distance_found_m, bearing_found_deg, obstacle_found = test_step1(distance_to_target_m, bearing_deg, i, target_loc)
            if distance_found_m > best_distance_m then
                best_distance_m = distance_found_m
                best_bearing_deg = bearing_found_deg
            end
            if obstacle_found == nil then -- found a path with no obstacles - done!
                goto continue
            end
            if distance_found_m < obstacle_distance_m then
                obstacle_avoiding = obstacle_found
                obstacle_distance_m = distance_found_m
            end
        end
        ::continue::

        -- we need to independently detect aircraft because even if an aircraft may not be the closest obstacle found by bendy ruler, we may still need to deal with it
        -- in other words, sometimes aircraft have higher priority than any other obstacles
        detect_aircraft()

        -- proactively check the altitude fences (vertical clamp-and-continue)
        local alt_obstacle = detect_altitude_fence()

        if obstacle_avoiding == nil then
            last_avoid_bearing_deg = nil
            committed_side_sign = 0
            side_flip_pending = false
            if alt_obstacle ~= nil then
                -- no horizontal threat, but we are approaching an altitude fence: keep heading to the
                -- waypoint and let update_target_location() clamp the commanded altitude into the safe band
                obstacle_avoiding = alt_obstacle
                local alt_target_loc = navigation_target_loc:copy()
                clamp_alt_to_fence(alt_target_loc)
                return alt_target_loc
            end
            --gcs:send_text(MAV_SEVERITY.ERROR, "NO OBSTACLE")
            --log_detect_result(false, obstacle_distance_m, distance_to_target_m, best_bearing_deg, target_loc, -1)
            return nil -- no avoidance required
        end

        if (now_ms - now_debug_ms) > 2000 then
            --gcs:send_text(MAV_SEVERITY.ERROR, string.format("DETECTED: %s distance: %.0f m", obstacle_avoiding.label, obstacle_avoiding.distance_m))
            now_debug_ms = now_ms
        end

        local obstacle_type = obstacle_avoiding.type
        local is_fence = obstacle_type ~= nil and (
            (obstacle_type >= OBSTACLE_TYPE.FENCE_HOME and obstacle_type <= OBSTACLE_TYPE.FENCE_LUA)
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN)

        if is_fence then
            -- Fences are fixed and containment is safety-critical: a heading slew limit or a
            -- committed side could delay/deflect the turn at a hard boundary and breach it.
            -- Keep the responsive bendy-ruler behaviour (clearance hysteresis only).
            best_bearing_deg = resist_bearing_change(last_avoid_bearing_deg, current_lookahead, best_bearing_deg, best_distance_m)
            last_avoid_bearing_deg = best_bearing_deg
            committed_side_sign = 0
            side_flip_pending = false
        else
            -- Non-fixed obstacles (aircraft, drones, birds, AIS, ...): velocity-aware smoothing.
            -- First decide whether the obstacle is actually a conflict: one that is opening range
            -- and whose predicted closest approach stays beyond well-clear is leaving, resume nav.
            local motion = assess_obstacle_motion(obstacle_avoiding)
            if not motion.is_conflict then
                -- the obstacle is leaving (opening range, predicted miss beyond its keep-out
                -- radius): drop it so avoid_obstacle() does not steer or announce for it. Any
                -- avoidance already in progress reverts cleanly (avoid_obstacle(nil)). This is
                -- re-decided every cycle from current geometry (no hold) so a manoeuvring obstacle
                -- is always tracked on fresh data; near a marginal crossing that can cost a few
                -- extra (slew-limited) heading reversals, which is the safe trade.
                obstacle_avoiding = nil
                last_avoid_bearing_deg = nil
                committed_side_sign = 0
                side_flip_pending = false
                return nil
            end
            -- Otherwise commit a side and slew-limit the heading so we track a smooth path
            -- instead of wiggling as the obstacle (and the instantaneous geometry) moves.
            -- refine_avoidance_bearing() also logs the DAAS smoothing trace each cycle.
            best_bearing_deg = refine_avoidance_bearing(bearing_deg, best_bearing_deg, best_distance_m, motion, obstacle_avoiding)
            last_avoid_bearing_deg = best_bearing_deg
        end

        local proj_distance = math.max(distance_to_target_m, current_lookahead)   -- fix bug 2
        local new_target_loc = location_project(current_loc, best_bearing_deg, proj_distance, target_loc)
        log_detect_result(true, obstacle_avoiding.distance_m, distance_to_target_m,
                          best_bearing_deg, new_target_loc, obstacle_avoiding.type)
        return new_target_loc
    end

    local function alert_obstacle(alert_target_loc)
        if obstacle_avoiding ~= nil and
                (obstacle_avoiding.type == OBSTACLE_TYPE.FENCE_ALT_MAX or
                 obstacle_avoiding.type == OBSTACLE_TYPE.FENCE_ALT_MIN) then
            -- altitude fences emit their own throttled, edge-triggered notice from
            -- detect_altitude_fence(); don't also raise the generic obstacle ALERT
            return
        end
        if obstacle_avoiding == nil or alert_target_loc == nil  or obstacle_avoiding.distance_xy > lookahead_param_m then
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
                                obstacle_avoiding.distance_xy))
            gcs:send_named_float("DAA-DISTXY", obstacle_avoiding.distance_xy)
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
        gcs:send_named_string("DAA-LOWC", "aircraft")
        LoWC_active = false
        LoWC_label  = ""
    end

    local function notify_aircraft_nearby(aircraft_obstacle)
        gcs:send_named_string("DAA-NEARBY", aircraft_obstacle.label)
        gcs:send_named_float("DAA-DISTXY", aircraft_obstacle.distance_xy)
        gcs:send_named_float("DAA-DISTZ", aircraft_obstacle.distance_z)
    end

    local function alert_aircraft()
        if aircraft_avoiding == nil then
            NMAC_cleared()
            LoWC_cleared()
            return
        end
        if (now_ms - now_aircraft_ms) > 5000 then
            --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: xy %.0f z %.0f", aircraft_avoiding.distance_xy,aircraft_avoiding.distance_z) )
            if aircraft_avoiding.distance_xy < near_miss_xy and aircraft_avoiding.distance_z < near_miss_z then
                NMAC_triggered(aircraft_avoiding)
            elseif aircraft_avoiding.distance_xy < well_clear_xy and aircraft_avoiding.distance_z < well_clear_z then
                LoWC_triggered(aircraft_avoiding)
            else
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s %.0f m", 
                                aircraft_avoiding.label, aircraft_avoiding.distance_xy ))
                gcs:send_named_string("DAA-ALERT", "aircraft")
            end
            notify_aircraft_nearby(aircraft_avoiding)
            now_aircraft_ms = now_ms
        end
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
            --gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: UPDATE FAILED nil")
            return false
        end
        -- enforce the altitude fences on every commanded target (clamp-and-continue)
        clamp_alt_to_fence(new_target_loc)
        new_target_loc:change_alt_frame(update_target_location_save_loc:get_alt_frame())
        local updated_location = vehicle:update_target_location(update_target_location_save_loc, new_target_loc)
        if updated_location then
            return true
        end
        --gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: UPDATE FAILED")
        return false
    end

    local function set_avoid_location(new_target_loc)
        -- if the "new" target is nil then revert back to the original navigation target which should be the current waypoint
        if new_target_loc == nil then
            if update_target_location(navigation_target_loc) then
                daa_target_loc = nil
                --gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: REVERT target to navigation target")
            end
            return false
        end
        -- if the "new" target is different from the current DAA target then lets try to go there
        if not locations_equal(daa_target_loc, new_target_loc) then
            if update_target_location(new_target_loc) then
                daa_target_loc = new_target_loc:copy()
                --gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: AVOID set new target")
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
            current_state = STATE.monitoring
        end
    end

    local function avoid_obstacle(new_target_loc, obstacle)
        if obstacle == nil then -- no obstacle, so clear any specific avoidance we might have been doing
            if current_state == STATE.loitering  and false then
                if loiteralt.stop(false) then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop NO OBSTACLE" ))
                    current_state = STATE.monitoring
                end
            end
            -- reset the target back to the original target
            new_target_loc = nil
        elseif obstacle.type == OBSTACLE_TYPE.GENERAL_AVIATION and false then
            -- depending on the obstacle we might do different things. Specifically if the obstacle is a crude aircraft
            -- in Canada we want to do a "Right 2" circuit descending to XXX altitude
            -- which for now we are doing by simply doing a loiter to alt in guided mode

            -- we might already be doing a loiter because of this aircraft. As long as it's far enough away, thats all we need to do
            if obstacle == aircraft_avoiding then
                return
            end

            loiteralt.start(ga_avoid_alt_m, ga_avoid_alt_frame, true, airspeed_ms)
            current_state = STATE.loitering

            gcs:send_named_string("DAA-AVOID", "loiter")
            if aircraft_avoiding ~= nil then
                gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
            end
            gcs:send_named_float("DAA-LOITER", ga_avoid_alt_m)

            return
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
                current_state = STATE.avoiding
            end
        else
            if avoiding_label ~= "" then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s done", avoiding_label))
                gcs:send_named_string("DAA-AVOID", "")
                gcs:send_named_string("DAA-OBSTCL", "")
                avoiding_label = ""
                current_state = STATE.monitoring
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
        -- "gone" distance must match the DETECTION distance (well_clear_xy + margin_aircraft_m),
        -- NOT the bare margin_aircraft_m: a plane sitting steadily inside the detection volume
        -- (e.g. 200 m out, well within ~660 m but far beyond 50 m) would otherwise satisfy
        -- both "still detected" (start) and "far enough to stop" (stop) every cycle and flip
        -- GUIDED<->AUTO. Hysteresis is provided by the 10 s aircraft_seen() dwell in
        -- loiteralt.stop(false), so a plane at the boundary cannot thrash the mode.
        if aircraft_avoiding == nil or (current_loc:get_distance(aircraft_avoiding.location) > (well_clear_xy + margin_aircraft_m)) then
            if loiteralt.stop(false) then
                -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop NO aircraft" ))
                current_state = STATE.monitoring
                return
            end
        end
        if aircraft_avoiding ~= nil then
            loiteralt.aircraft_seen()
            if (now_ms - now_loitering_ms) > 5000 then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITERING to %.0f m for AIRCRAFT: %s", ga_avoid_alt_m, aircraft_avoiding.label))
                now_loitering_ms = now_ms
            end
        end
        loiteralt.update()
    end

    -- execute avoidance maneuvers depending on the nature of the obstacle
    function DAA.avoidstates(new_target_loc)
        if daa_action == 0 then
            return              -- parameter DAA_AVOID can be used to disable avoidance
        end
        if current_state == STATE.loitering or current_state == STATE.loitering_avoiding then
            do_loitering()
            if obstacle_avoiding ~= nil then
                current_state = STATE.loitering_avoiding
                avoid_obstacle(new_target_loc,obstacle_avoiding)
            else
                current_state = STATE.loitering
            end
            return
        elseif current_state == STATE.hovering or current_state == STATE.avoiding or current_state == STATE.hovering or current_state == STATE.landing then -- luacheck: ignore 542
            -- do nothing for now
        elseif aircraft_avoiding ~= nil then
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITER AIRCRAFT: %s", aircraft_avoiding.label))
            loiteralt.start(ga_avoid_alt_m, ga_avoid_alt_frame, true, airspeed_ms)
            current_state = STATE.loitering

            gcs:send_named_string("DAA-AVOID", "LOITER")
            gcs:send_named_float("DAA-LOITER", ga_avoid_alt_m)
            gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
            gcs:send_named_float("DAA-DIST", aircraft_avoiding.distance_m)

            return
        else
            current_state = STATE.monitoring
        end
        avoid_obstacle(new_target_loc, obstacle_avoiding)
        --if loiteralt.active then
        --    current_state = STATE.loitering -- deal with avoiding an obstacle (e.g. a drone) while currently in a loiter (to avoid a plane)
        --end
    end

    -- execute avoidance maneuvers depending on the nature of the obstacle
    function DAA.avoid(new_target_loc)
        if daa_action == 0 then
            return              -- parameter DAA_AVOID can be used to disable avoidance
        end
        if loiteralt.active then
            current_state = STATE.loitering
            do_loitering()
        elseif current_state == STATE.hovering or current_state == STATE.avoiding or current_state == STATE.hovering or current_state == STATE.landing then -- luacheck: ignore 542
            -- do nothing for now
        elseif aircraft_avoiding ~= nil and assess_obstacle_motion(aircraft_avoiding).is_conflict then
            -- CONSERVATIVE CPA gate on the loiter trigger: an aircraft inside the well-clear
            -- radius is always a conflict (assess_obstacle_motion's range check), so the loiter
            -- still fires unconditionally at close range - safer-first. Only a plane in the outer
            -- detection band that will miss beyond well-clear AND is not closing is skipped, and it
            -- then falls through to normal monitoring. A missing/uncertain velocity => conflict.
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITER AIRCRAFT: %s", aircraft_avoiding.label))
            loiteralt.start(ga_avoid_alt_m, ga_avoid_alt_frame, true, airspeed_ms)
            current_state = STATE.loitering

            gcs:send_named_string("DAA-AVOID", "LOITER")
            gcs:send_named_float("DAA-LOITER", ga_avoid_alt_m)
            gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
            gcs:send_named_float("DAA-DIST", aircraft_avoiding.distance_m)

            return
        else
            current_state = STATE.monitoring
        end
        avoid_obstacle(new_target_loc, obstacle_avoiding)
        if loiteralt.active then
            current_state = STATE.loitering -- deal with avoiding an obstacle (e.g. a drone) while currently in a loiter (to avoid a plane)
        end
    end

    -- a moving obstacle (drone/aircraft/bird/AIS/proximity) may move out of the way;
    -- a fence will not, so a fence trap is held until the pilot intervenes
    local function obstacle_is_dynamic(obstacle_type)
        if obstacle_type == nil then return true end
        local is_fence = (obstacle_type >= OBSTACLE_TYPE.FENCE_HOME and obstacle_type <= OBSTACLE_TYPE.FENCE_LUA)
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
    local function daa_compromised_now()
        if fence ~= nil and fence:get_breaches() ~= 0 then
            return true
        end
        return aircraft_avoiding ~= nil
            and aircraft_avoiding.distance_xy ~= nil
            and aircraft_avoiding.distance_xy < near_miss_xy
            and aircraft_avoiding.distance_z < near_miss_z
    end

    -- Trapped-failsafe state machine. Returns true while the failsafe is controlling the
    -- vehicle (so update() skips normal avoidance). See DAA_TRAP_ACT/S/CLR_S.
    function DAA.trap_update()
        if trap_act == 0 then
            trap_active = false
            trap_since_ms = uint32_t(0)
            return false
        end
        local mode_now = vehicle:get_mode()

        if not trap_active then
            if daa_compromised_now() then
                if trap_since_ms == uint32_t(0) then trap_since_ms = now_ms end
                if (now_ms - trap_since_ms) >= (trap_s * 1000) then
                    -- a fence breach can nil obstacle_avoiding (breach-escape) yet still be a
                    -- compromise; treat "no current obstacle" as a static (fence) trap
                    trap_dynamic    = (obstacle_avoiding ~= nil) and obstacle_is_dynamic(obstacle_avoiding.type) or false
                    trap_prev_mode  = mode_now
                    trap_fs_mode    = resolve_trap_mode(mode_now)
                    trap_trigger_ms = now_ms
                    vehicle:set_mode(trap_fs_mode)
                    trap_active     = true
                    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
                        ": TRAPPED (%s) -> %s", trap_dynamic and "moving" or "fence", get_mode_string(trap_fs_mode)))
                    gcs:send_named_string("DAA-AVOID", "TRAPPED")
                    return true
                end
            else
                trap_since_ms = uint32_t(0)
            end
            return false
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
            if trap_prev_mode > 0 then vehicle:set_mode(trap_prev_mode) end
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(
                ": trap clear -> resume %s", get_mode_string(trap_prev_mode)))
            gcs:send_named_string("DAA-AVOID", "")
            trap_active = false
            trap_since_ms = uint32_t(0)
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
    get_vehicle_state()


    local switch_function = DAA_ACT_FN:get()
    if switch_function == nil then
        if not no_DAA_displayed then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " no DAA function")
            no_DAA_displayed = true
        end
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
	    -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " switch:"..switch_state)
        if switch_state == 0 then -- switch Low to disarm - so defaults to on
            DAA.enable()
        elseif switch_state >= 1 then -- switch High to turn off
            DAA.disable()
        end
        last_switch_state = switch_state
    end

    DAA.get_vehicle_state()
    if DAA.isactive() then
        local suggested_target_loc = DAA.detect()
        DAA.alert(suggested_target_loc)
        DAA.feed_watch()
        -- when avoidance can't find a way out, the trapped-failsafe takes over (and we
        -- stop issuing avoidance); otherwise avoid, but only in FW forward flight
        local trapped = DAA.trap_update()
        if not trapped and vtol_state == MAV_VTOL_STATE.FW then
            DAA.avoid(suggested_target_loc)
        end
    else
        -- DAA is inactive (e.g. the pilot switched it off).  If it was mid-avoidance,
        -- revert the commanded target so the vehicle resumes its real navigation
        -- (e.g. RTL-to-home) instead of flying on to a stale avoidance waypoint.
        DAA.clear_avoidance()
    end
end

-- wrapper around update(). This calls update() at REFRESHRATE Hz, i.e. every 1000/REFRESH_RATE milliseconds
-- and if update faults then an error is displayed, but the script is not stopped
function Protected_Wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, 1000 / refresh_rate
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


--[[

  Calculates Modified Tau (tau_mod) for DAA logic.
  r: Current horizontal range (e.g., in feet)
  r_dot: Horizontal range rate (e.g., in feet per second). 
         Note: r_dot must be negative for aircraft that are closing.
  dmod: Distance Modification threshold (e.g., 4000 or 2000 feet)

function calculate_tau_mod(r, r_dot, dmod)
    -- If range rate is zero or positive, aircraft are not closing.
    -- Modified Tau is mathematically undefined or infinite (safe).
    if r_dot >= 0 then
        return math.huge
    end

    -- If range is already within the DMOD buffer, tau_mod is 0.
    if r <= dmod then
        return 0
    end

    -- RTCA DO-365C Modified Tau Formula:
    -- tau_mod = -(r^2 - dmod^2) / (r * r_dot)
    local tau_mod = -(math.pow(r, 2) - math.pow(dmod, 2)) / (r * r_dot)
    
    return tau_mod
end

-- Example Usage (En Route Scenario):
local current_range = 15000 -- 15,000 feet away
local closure_rate = -150   -- Closing at 150 feet per second
local dmod_enroute = 4000   -- 4,000 feet threshold

local result = calculate_tau_mod(current_range, closure_rate, dmod_enroute)

print(string.format("Modified Tau: %.2f seconds", result))


-- DO-365C DAA Alerting Script for ArduPilot
-- Thresholds for "Warning" alert (25s) and "Corrective" alert (55s)
local TAU_WARNING    = 25
local TAU_CORRECTIVE = 55
local DMOD_FEET      = 4000 -- En-Route DMOD
local H_THRESHOLD    = 450  -- Vertical threshold in feet

local FEET_TO_METERS = 0.3048
local METERS_TO_FEET = 3.28084

function update()
    -- Get UAS altitude (meters to feet)
    local my_pos = ahrs:get_location()
    if not my_pos then return update, 1000 end
    local my_alt_ft = my_pos:alt() * 0.01 * METERS_TO_FEET 

    -- Iterate through all ADSB intruders
    local num_vehicles = adsb:get_num_vehicles()
    for i = 0, num_vehicles - 1 do
        local vehicle = adsb:get_vehicle_info(i)
        
        if vehicle then
            -- 1. Get Geometry
            local r_meters = vehicle:get_distance() -- Horizontal distance
            local r_ft = r_meters * METERS_TO_FEET
            local r_dot = vehicle:get_horiz_velocity() -- Relative horizontal speed
            
            -- In ArduPilot, horiz_velocity is often positive for closure
            -- but for Tau formula we need it negative for closure.
            -- We assume the library returns relative closure speed.
            local r_dot_ft = r_dot * METERS_TO_FEET
            
            -- 2. Vertical Separation Check
            local int_alt_ft = vehicle:get_altitude() * METERS_TO_FEET
            local dh = math.abs(my_alt_ft - int_alt_ft)

            -- 3. Calculate Modified Tau
            local tau_mod = 1000 -- Safe default
            
            if r_ft <= DMOD_FEET then
                tau_mod = 0 -- Inside the 'hockey puck'
            elseif r_dot > 0 then
                -- Formula: tau_mod = -(r^2 - dmod^2) / (r * r_dot)
                -- We use r_dot as positive closure rate here to avoid double negative
                tau_mod = (math.pow(r_ft, 2) - math.pow(DMOD_FEET, 2)) / (r_ft * r_dot_ft)
            else
                tau_mod = 1000 -- Moving away
            end

            -- 4. Decision Logic (Vertical AND Horizontal/Temporal threat)
            if dh < H_THRESHOLD then
                if tau_mod <= TAU_WARNING then
                    gcs:send_text(0, string.format("DAA WARNING: Int %s at %.0fs", vehicle:callsign(), tau_mod))
                    -- Optional: Trigger automated avoidance here
                elseif tau_mod <= TAU_CORRECTIVE then
                    gcs:send_text(6, string.format("DAA Corrective: Int %s at %.0fs", vehicle:callsign(), tau_mod))
                end
            end
        end
    end

    return update, 500 -- Run at 2Hz
end

gcs:send_text(6, "DAA DO-365C Script Loaded")
return update, 1000

--]]
