--[[
    The avoidance MECHANISM for planedaa.lua: "where can I safely go?"

    This module decides nothing about what to do with an obstacle.  It gathers the threat
    picture and searches for a heading - the bendy-ruler sweep with its turn lead, the
    closest-point-of-approach assessment, aircraft and altitude-fence detection, and the
    DAAD/DAAS/DAAG logging that records those decisions.  Every judgement about what to SAY
    and what to COMMAND stays in planedaa.lua, so an integrator adding their own avoidance
    action edits that one file.  See planedaa.md.

    detect() returns a report rather than setting shared state, which is what lets the two
    halves live in different files:

        { target_loc, obstacle, aircraft, best_distance_m, distance_to_target_m }

    Everything from the applet is pushed in: new() takes the collaborators and constants,
    configure() the cached parameters, update_state() the per-cycle vehicle state.  They
    land in instance locals rather than fields because the candidate-heading sweep reads
    them well over a hundred times per cycle.
--]]

local DAAcore = {}

DAAcore.SCRIPT_VERSION = "4.8.0-024"
DAAcore.SCRIPT_NAME = "DAA core"
DAAcore.SCRIPT_NAME_SHORT = "DAAcore"

-- Load-banner severity only - NOT exported, and not what the instance logs with below.
-- This module is a tightly-coupled collaborator of planedaa.lua, not a standalone library
-- like pid.lua (whose SCRIPT_NAME/SCRIPT_VERSION pattern this follows) - MAV_SEVERITY stays
-- an injected dependency below so there is one shared severity table, not one per module
-- that could theoretically drift. MAV_SEVERITY.INFO is a fixed MAVLink wire value (6),
-- unlike that shared table, so a private literal here carries no drift risk of its own.
local BANNER_SEVERITY = 6

-- daageo is stateless (every function is a pure function of its arguments, see that file),
-- so it is required directly rather than injected as a constructed/configured collaborator -
-- there is no shared state to keep consistent between callers.  roll_limit_deg, the one real
-- input its turn-radius functions need, is this module's own cached parameter instead (see
-- configure() below), passed as an explicit argument each call.
local geometry = require("daageo")

-- Implementation constants: how THIS module works, not a runtime-configurable behaviour,
-- so they live here rather than being injected from planedaa.lua - Codex's review of the
-- constructor deps tables and Tim's "the complexity of making it a deps doesn't seem worth
-- it" (re: FLT_MAX) both landed on the same conclusion.  If one of these ever needs to be
-- configurable it should become a real DAA_ parameter, not a constructor argument.
local FLT_MAX             = 3.402823466e+38
-- The candidate-heading sweep in DAA.detect() runs coarse-to-fine: it steps at
-- COARSE_SWEEP_MULT * DAA_HEADING_INC, then refines around the winner at the full
-- DAA_HEADING_INC. The final resolution is unchanged; the worst case (boxed in, no
-- heading clears, so every candidate is probed) costs ~COARSE_SWEEP_MULT times less.
-- Not a parameter: DAA_HEADING_INC already exposes the resolution-vs-CPU trade, and
-- this only sets how the same search is scheduled.
local COARSE_SWEEP_MULT   = 4
-- minimum length of the bendy-ruler second-leg probe, so a plane sitting almost on top
-- of its waypoint still tests a sane segment (mirrors OA_BENDYRULER_LOOKAHEAD_STEP2_MIN)
local MIN_STEP2_M         = 2.0
-- Shortest turn chord worth its own obstacle probe.  Below this the post-turn point is on
-- top of us and the bearing to it is noise, so probe_turn_arc() declines.
local MIN_TURN_CHORD_M    = 5.0
-- Clamp for the clearances written to DAAD: "no obstacle at all" is FLT_MAX internally and
-- would wreck the autoscaling of any plot it shares an axis with.
local LOG_CLEARANCE_MAX_M = 9999.0
-- assess_aircraft_conflict()'s filter constants - module scope, not inside DAAcore.new(),
-- since they are true constants with no per-instance state (see the 100-local-per-function
-- ceiling note in project_lua_binding_gotchas in memory).
local AIRCRAFT_TAU_FILTER_S = 8.0
local AIRCRAFT_TAU_GAP_S    = 5.0
-- How far ahead validate_horizontal_release() projects the aircraft's CURRENT bank
-- before deciding a fence is genuinely clear - long enough to matter physically, short
-- enough to stay a "where am I actually about to be" check rather than a second sweep.
local VALIDATE_PROJECTION_S = 2.0

function DAAcore.new(deps)
    local self = {}

    -- collaborators, fixed for the life of the instance
    local obstacles             = deps.obstacles
    -- obstacles already owns this taxonomy (it is the code that classifies obstacles into
    -- it) - reading it back through obstacles rather than a second injected copy means
    -- there is exactly one OBSTACLE_TYPE table, not two that could disagree.
    local OBSTACLE_TYPE         = obstacles.OBSTACLE_TYPE
    local MAV_SEVERITY          = deps.MAV_SEVERITY
    local SCRIPT_NAME_SHORT     = DAAcore.SCRIPT_NAME_SHORT
    -- ahrs, gcs, logger, fence and OAScripting are ArduPilot singletons: global in every chunk

    -- the module helpers this file leans on, bound once so the sweep reaches them as
    -- upvalues rather than through a table on every probe
    local wrap_360                  = geometry.wrap_360
    local wrap_180                  = geometry.wrap_180
    local location_project          = geometry.location_project
    local max_turn_rate_dps         = geometry.max_turn_rate_dps
    local arc_projection            = geometry.arc_projection
    local effective_groundspeed     = geometry.effective_groundspeed
    local find_closest_obstacle     = obstacles.find_closest_obstacle
    local populate_obstacle         = obstacles.populate_obstacle
    local get_standoff              = obstacles.get_standoff
    local is_grounded_traffic       = obstacles.is_grounded_traffic

    -- cached parameters, pushed in by configure()
    local alt_cool_ms, alt_hyst_m, bearing_inc_deg, bendy_angle
    local bendy_ratio, cpa_min_ms, detect_m, margin_alt_m
    local margin_crewed_m, margin_fence_m, margin_vertical_m, plan_m
    local roll_limit_deg, roll_rate_dps
    local side_hold_s, slew_dps, slew_urg_s, well_clear_xy
    local well_clear_z, wp_loiter_rad_m, lookahead_param_m

    -- per-cycle vehicle state, pushed in by update_state()
    local current_loc, navigation_target_loc, airspeed_ms, groundspeed_ms, ground_course_deg, wind_speed, wind_dir_rad, now_ms, current_roll_deg

    -- the sweep's own state: nothing outside this module reads any of it
    local obstacle_avoiding         = nil
    local aircraft_avoiding         = nil
    local last_aircraft_obstacle    = nil
    local last_aircraft_ts_ms       = nil
    -- MEASURED range history for assess_aircraft_conflict()'s modified-tau test - see its
    -- own comment for why this is tracked separately from assess_obstacle_motion()'s
    -- instantaneous velocity-vector CPA. Reset alongside last_aircraft_obstacle whenever
    -- the tracked aircraft changes or is lost, so history never leaks across encounters.
    -- Bare GLOBALS, along with tau_s below (removed from the cached-parameters block
    -- above) - not `local`, to stay under Lua's 100-per-function ceiling on this already
    -- near-full function (see project_lua_binding_gotchas in memory). Same caveat as
    -- assess_aircraft_conflict() itself: safe only because there is one DAAcore instance.
    last_aircraft_range_m     = nil
    aircraft_closure_rate_ms  = 0.0
    -- False until the first genuine two-fix measurement exists. assess_aircraft_conflict()
    -- treats "no measurement yet" as a conflict (safer-is-better, matching
    -- assess_obstacle_motion()'s own "no geometry => conflict" default) rather than
    -- silently reading the initial 0.0 as "not closing" for a brand new contact.
    aircraft_closure_rate_valid = false
    local last_avoid_bearing_deg    = nil
    local last_cmd_bearing_ms       = nil
    local committed_side_sign       = 0
    local side_flip_pending         = false
    local side_flip_want_ms         = uint32_t(0)
    local current_lookahead         = 0
    local lookahead_set_m           = nil
    -- reversal-in-progress latch (see resist_bearing_change): nil, or the bank sign
    -- (+1/-1) a fence-avoidance reversal is currently committed to reaching
    local reversal_target_sign      = nil
    local reversal_since_ms         = uint32_t(0)
    -- Last bearing committed specifically to a FENCE, kept independent of
    -- last_avoid_bearing_deg (which a moving-obstacle episode overwrites/clears via
    -- reset_horizontal_avoidance()).  Set only by resolve_fence_bearing(), cleared only
    -- once validate_horizontal_release() confirms the fence is genuinely clear - so a
    -- fence's relevance survives a moving obstacle briefly outranking it in the same-cycle
    -- single-winner obstacle choice.  See validate_horizontal_release()'s own comment.
    local fence_hold_bearing_deg    = nil

    local function configure(settings)
        alt_cool_ms            = settings.alt_cool_ms
        alt_hyst_m             = settings.alt_hyst_m
        bearing_inc_deg        = settings.bearing_inc_deg
        bendy_angle            = settings.bendy_angle
        bendy_ratio            = settings.bendy_ratio
        cpa_min_ms             = settings.cpa_min_ms
        detect_m               = settings.detect_m
        margin_alt_m           = settings.margin_alt_m
        margin_crewed_m        = settings.margin_crewed_m
        margin_fence_m         = settings.margin_fence_m
        margin_vertical_m      = settings.margin_vertical_m
        plan_m                 = settings.plan_m
        roll_limit_deg         = settings.roll_limit_deg
        roll_rate_dps          = settings.roll_rate_dps
        side_hold_s            = settings.side_hold_s
        slew_dps               = settings.slew_dps
        slew_urg_s             = settings.slew_urg_s
        tau_s                  = settings.tau_s
        well_clear_xy          = settings.well_clear_xy
        well_clear_z           = settings.well_clear_z
        wp_loiter_rad_m        = settings.wp_loiter_rad_m
        lookahead_param_m      = settings.lookahead_param_m
        -- the working look-ahead follows the parameter, announcing the change once
        if lookahead_param_m ~= lookahead_set_m then
            lookahead_set_m   = lookahead_param_m
            current_lookahead = lookahead_param_m
        end
    end

    -- Positional, not a table: this runs every cycle the applet is active, and the sweep
    -- alone can call find_closest_obstacle() over a hundred times in one cycle, so a
    -- table literal here would be the single most frequent allocation in the script.
    local function update_state(new_current_loc, new_navigation_target_loc, new_airspeed_ms,
                                 new_groundspeed_ms, new_ground_course_deg,
                                 new_wind_speed, new_wind_dir_rad, new_now_ms, new_current_roll_deg)
        current_loc            = new_current_loc
        navigation_target_loc  = new_navigation_target_loc
        airspeed_ms            = new_airspeed_ms
        groundspeed_ms         = new_groundspeed_ms
        ground_course_deg      = new_ground_course_deg
        wind_speed             = new_wind_speed
        wind_dir_rad           = new_wind_dir_rad
        now_ms                 = new_now_ms
        current_roll_deg       = new_current_roll_deg
    end

    local function log_detect_result(distance_found_m, best_distance_m, distance_to_target_m, best_bearing_deg, target_loc, obstacle_type)
        if target_loc == nil or distance_found_m == nil or distance_to_target_m == nil or best_bearing_deg == nil then
            -- we can't be avoiding if no target, so no loggin required
            return
        end
        local status, err = pcall(logger.write, logger, "DAAD",
            'Obs,DstF,DstB,DstT,HdgB,Tfnd,TLat,TLng,TAlt,TFra,ObjT',
            'BfffffLLfBI',                  -- Formats (L for Lat/Lng, f for Alt)
            '-mmmmdDUm--',                  -- Units (D=lat deg, U=lng deg, m=meter)
            '------GG---',                  -- Multipliers (G=1e-7 for L types)
            -- Obs - Obstacle found true/false.  This is the only caller, and it is
            -- only ever reached once an obstacle has already been found, so this is
            -- always 1 - kept as a logged field (not removed) since existing log
            -- analysis tooling reads it, but no longer a parameter here.
            1,
            distance_found_m,               -- DstF - clearance of the WORST heading in the sweep
            -- DstB - clearance of the heading we CHOSE (HdgB).  A heading that clears every
            -- obstacle reports FLT_MAX, so clamp it to something a log viewer can scale.
            math.max(math.min(best_distance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
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

        local status, err = pcall(logger.write, logger, "DAAG",
            'DstF,TLat,TLng,TAlt,TFra,DstH,DstZ,ICAO',
            'fLLfBffI',                         -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mmh',                         -- Units (D=lat deg, U=lng deg, m=meter)
            '-GG-----',                         -- Multipliers (G=1e-7 for L types)
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

    local function log_smoothing(direct_deg, raw_deg, resisted_deg, final_deg, side, flip, urgent, motion, obstacle)
        local status, err = pcall(logger.write, logger, "DAAS",
            'HdD,HdR,HdS,HdC,Sid,Flp,Urg,Cls,CPA,TTC,PsB,Dst,Typ',
            'ffffbBBfffbfB',                    -- Formats
            'dddd---nms-m-',                    -- Units (d=deg, n=m/s, m=metre, s=second)
            '-------------',                    -- Multipliers
            wrap_360(direct_deg),               -- HdD - direct bearing to target
            wrap_360(raw_deg),                  -- HdR - raw bendy-ruler bearing
            wrap_360(resisted_deg),             -- HdS - after clearance hysteresis (pre-smoothing)
            wrap_360(final_deg),                -- HdC - final commanded bearing (flown)
            side,                               -- Sid - committed side (-1 left / 0 / +1 right)
            (flip and 1 or 0),                  -- Flp - side-flip debounce pending
            (urgent and 1 or 0),                -- Urg - slew limit bypassed (urgent)
            motion.closing_speed,               -- Cls - closing speed
            motion.cpa_miss,                    -- CPA - predicted horizontal miss distance
            math.min(motion.ttc, 999.0),        -- TTC - time until the keep-out boundary is crossed (capped)
            motion.pass_behind,                 -- PsB - side that passes behind the obstacle
            obstacle.distance_m,                -- Dst - range to the obstacle
            obstacle.type)                      -- Typ - OBSTACLE_TYPE
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log smoothing:" .. tostring(err))
        end
    end

    local function calc_avoidance_distance(avoid_step1_m, target_distance)
        -- test for flying past the waypoint, so if we are close, we have room to dodge after the waypoint
        return math.min(avoid_step1_m, target_distance + math.min(margin_fence_m / 2, 100))
    end

    -- Forward-declared: resist_bearing_change() and resist_fence_bearing_change() (defined
    -- here, ahead of probe_bearing in the file) need to call it, and need the SAME
    -- turn-lead-aware measurement probe_bearing gives every other candidate in the sweep.
    -- Assigned without "local" at its usual
    -- location further down; this is the upvalue resist_bearing_change closes over.
    local probe_bearing

    -- Sign of the bank the aircraft is CURRENTLY, physically holding - not the sign of any
    -- commanded/desired roll.  A deadband near wings-level reads as "uncommitted" (0):
    -- right at a reversal's crossing point the true roll flickers around zero, and calling
    -- that noise a side would thrash the reversal latch below on exactly the transition it
    -- exists to protect.  nil (no roll telemetry) also reads as 0 - fails toward the
    -- pre-latch behaviour (Stage 1 alone) rather than toward a lock that can never release.
    local ROLL_DEADBAND_DEG = 10.0
    local function bank_sign(roll_deg)
        if roll_deg == nil or math.abs(roll_deg) < ROLL_DEADBAND_DEG then
            return 0
        end
        return (roll_deg > 0) and 1 or -1
    end
    -- Safety valve for the reversal latch below: release it unconditionally if it has been
    -- held this long without the aircraft's own roll confirming the reversal happened -
    -- degrades to Stage 1 alone rather than a lock that can never clear, if roll telemetry
    -- or the sign convention assumed here ever disagrees with reality.  Generous relative
    -- to any reversal this airframe class should need (a few seconds), not tight.
    local MAX_REVERSAL_LATCH_MS = 6000

    --[[
    This function is called when BendyRuler has found a bearing which is obstacles free at at least lookahead_step1_dist and  then lookahead_step2_dist from the present location
    In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
    It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
    Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees,
    unless the new margin is atleast _bendy_ratio times better than the margin with previously calculated bearing.
    We return true if we have resisted the change and will follow the last calculated bearing.

    Used by the MOVING-obstacle path only (refine_avoidance_bearing()) - the fence path has
    its own resist_fence_bearing_change() below, with different hysteresis rules.  A moving
    obstacle's geometry keeps changing independently of the aircraft, so re-optimizing
    toward the clearest available heading each cycle is appropriate here in a way it is not
    for a fixed fence; refine_avoidance_bearing() also layers its own side-commit and
    slew-rate smoothing on top of whatever this returns.

    Returns (bearing, distance): the distance is the clearance of WHICHEVER bearing is
    returned, not of the candidate that was proposed.
    --]]
    local function resist_bearing_change(bearing_orig_deg, bearing_deg, distance_found_m, target_loc)
        if bearing_orig_deg == nil then
            -- no prior commitment, accept the proposed bearing
            return bearing_deg, distance_found_m
        end
        if distance_found_m == 0 then
            -- obstacle is immediate, must manoeuvre regardless
            return bearing_deg, distance_found_m
        end
        if math.abs(wrap_180(bearing_orig_deg - bearing_deg)) < bendy_angle then
            -- proposed change is small enough, no resistance needed
            return bearing_deg, distance_found_m
        end
        if current_loc == nil then
            -- no current position to measure against, accept the proposed bearing
            return bearing_deg, distance_found_m
        end
        local distance_previous_m = probe_bearing(bearing_orig_deg, bearing_orig_deg,
                                                   FLT_MAX, target_loc, false)
        -- Only switch sides if the new direction is significantly better: POSITIVE clearance
        -- and bendy_ratio times clearer than continuing. The positive-clearance requirement is
        -- what makes this negative-aware: when hugging a boundary both clearances read near-zero
        -- or negative and a plain ratio test flip-flops every cycle. It still switches away from
        -- a committed side that is itself breaching (distance_previous_m < 0) towards a side that
        -- actually clears (distance_found_m > 0), so containment is preserved.
        if distance_found_m > 0 and distance_found_m >= bendy_ratio * distance_previous_m then
            return bearing_deg, distance_found_m
        end
        return bearing_orig_deg, distance_previous_m
    end

    --[[
    Fence-specific hysteresis: persistence (Stage 1), a reversal-in-progress latch
    (Stage 1b), and a bank-aware reversal transition (Stage 2, see
    location_for_candidate() above) - see project_planedaa_reversal_awareness in memory
    for the full field report, SITL reproductions and log analyses behind this (three
    separate live breaches, 2026-09-04, each closing a different gap in what came
    before it).

    Persistence rests on TWO signals, not one, because they catch different failures:

    1. held_is_clear (obstacle_found == nil from probe_bearing() - the same
       authoritative, wind-aware signal every other candidate in the sweep is judged
       by): true only when the held bearing is FULLY clear.  Gates the reversal hold
       below - a genuine bank reversal is only resisted while the held bearing remains
       fully safe; the moment it is not, the latch below must not keep flying an
       unsafe path just to avoid a second reversal.
    2. The CONTESTED_ZONE_M comparison below: while still within a few standoffs of a
       boundary, a fresh candidate - reversal or same-direction - is refused whenever
       it offers LESS clearance than the currently held one, unless the held bearing
       has itself crossed into an actual violation.  held_is_clear alone cannot do this
       job: every retest in this function passes allow_straight=false, so even a fully
       safe candidate returns a large FINITE sum (see probe_bearing()'s own comment),
       never the FLT_MAX sentinel - meaning held_is_clear is essentially always false
       throughout an active, still-safe approach, not just at its unsafe end.  Gating
       persistence on it alone (an earlier version of this fix did exactly that) left
       a same-direction refinement completely unresisted for the WHOLE eroding
       approach: one shrinking-clearance "best available" replacing the last, walking
       achieved clearance from tens of metres down to a live breach with no reversal
       (and so no Stage 1b latch) ever entering the picture - confirmed on two separate
       SITL flights on the SAME day as the fix that turned out not to close this gap.

    Both checks are needed: held_is_clear (1) is the right bar for "is a reversal
    actually required," since instantaneous roll-based whipsaw detection has nothing to
    do with distance; the numeric comparison (2) is the right bar for "is this specific
    change making things worse," which a boolean cannot express and which does not
    care whether the change is a reversal or not.  CONTESTED_ZONE_M bounds (2) to the
    genuinely close-in case: far-field retest numbers fluctuate cycle to cycle purely
    from position noise (two "hundreds of metres clear" evaluations are never exactly
    equal), and comparing those directly would eventually reproduce the earlier,
    already-fixed 16-23s-straight-flight regression this whole function exists to
    avoid (an unconditional hold-while-clear, with no same-direction escape at all).

    Only a MATERIAL course change (>= bendy_angle, the same DAA_BR_ANGLE threshold used
    elsewhere in this file) is ever classified as needing a reversal at all - a small
    correction with the opposite arithmetic sign is not itself a whipsaw and must not
    arm or re-arm the latch.  Once a genuine reversal is accepted, its bank direction is
    latched until the aircraft's own roll reaches THAT side - merely passing back
    through wings-level does not confirm the reversal happened, it only confirms the
    old bank was released, and releasing the latch there let a second, opposite
    reversal back in before the first one completed (a real field breach, not a
    hypothetical). Stage 2 (location_for_candidate() above) gives the turn-lead model a
    bank-aware transition so a genuinely-required reversal's own safety assessment
    accounts for the time the transition itself takes, rather than assuming the target
    bank is established instantaneously - but per review this latch MUST still be
    breakable the moment the held bearing itself stops being clear (a lock that can
    hold an unsafe path is worse than no lock).

    NEEDS TESTING against a fresh live flight before this is trusted alone - the first
    two "fixes" here each looked complete against their own reproduction and were not;
    see the memory file for what to measure (minimum achieved fence clearance across
    several consecutive laps, not just whether one particular symptom disappears).

    Returns (bearing, distance): the distance is the clearance of WHICHEVER bearing is
    returned, not of the candidate that was proposed.  Getting this right matters for
    logging: DAAD.DstB used to be left as the proposed candidate's distance even on the
    "stay the course" path, so it could read as fully clear while the bearing actually
    being flown was not - see project_planedaa_standoff_not_achieved in memory.
    --]]
    local function resist_fence_bearing_change(bearing_orig_deg, bearing_deg, distance_found_m, target_loc)
        if bearing_orig_deg == nil then
            -- no prior commitment, accept the proposed bearing
            reversal_target_sign = nil
            return bearing_deg, distance_found_m
        end
        if distance_found_m == 0 then
            -- obstacle is immediate, must manoeuvre regardless - overrides any latch
            reversal_target_sign = nil
            return bearing_deg, distance_found_m
        end
        if current_loc == nil then
            -- no current position to measure against, accept the proposed bearing
            return bearing_deg, distance_found_m
        end

        -- One retest of the committed bearing, reused below for both the latch's own
        -- safety check and the general persistence check - never two probes in one call,
        -- and never let a multi-value tail call balloon this function's own return past
        -- its documented (bearing, distance) contract.
        local distance_previous_m, _, obstacle_previous = probe_bearing(
                bearing_orig_deg, bearing_orig_deg, FLT_MAX, target_loc, false)
        local held_is_clear = (obstacle_previous == nil)

        if reversal_target_sign ~= nil then
            local holding_sign = bank_sign(current_roll_deg)
            if holding_sign == reversal_target_sign
                    or (now_ms - reversal_since_ms) > MAX_REVERSAL_LATCH_MS then
                -- Roll has actually reached the requested target bank side (not merely
                -- wings-level - crossing back through the deadband confirms the OLD bank
                -- was released, not that the new one was established, and releasing here
                -- let a second, opposite reversal land before the first one completed),
                -- or the safety valve fired: free to re-evaluate normally below.
                reversal_target_sign = nil
            elseif held_is_clear then
                -- still mid-reversal AND the held bearing is still genuinely clear: hold
                -- it regardless of what a fresh sweep proposes.
                return bearing_orig_deg, distance_previous_m
            else
                -- Still mid-reversal, but the held bearing is no longer clear: the latch
                -- must not keep flying an unsafe path just to avoid a second reversal.
                -- Break it and fall through to a fresh decision below, even though that
                -- risks re-triggering the exact whipsaw this latch exists to prevent -
                -- this is the known gap Stage 2 is meant to close.
                reversal_target_sign = nil
            end
        end

        -- What bank direction the fresh candidate needs, and whether that differs from
        -- the bank the aircraft is currently, physically holding - computed up front
        -- because it now gates BOTH branches below, not just the "no longer clear" one.
        -- A candidate within bendy_angle of the current ground course is a small
        -- correction, not a turn with a bank direction of its own - the sign of such a
        -- tiny change is noise and must not be classified as needing a reversal (the
        -- original fault was 150-220 deg jumps, not sub-bendy_angle corrections).
        local course_change_deg = wrap_180(bearing_deg - ground_course_deg)
        local current_sign = bank_sign(current_roll_deg)
        local needs_reversal = false
        local needed_sign = current_sign
        if math.abs(course_change_deg) >= bendy_angle then
            needed_sign   = (course_change_deg >= 0) and 1 or -1
            needs_reversal = current_sign ~= 0 and current_sign ~= needed_sign
        end

        if held_is_clear and needs_reversal then
            -- Still genuinely clear, but the only reason to move off it is a fresh
            -- candidate that would reverse bank direction - hold the committed bearing
            -- rather than risk the servo-overshoot whipsaw this exists to prevent.
            return bearing_orig_deg, distance_previous_m
        end

        -- Contested-zone persistence: this is what actually stops a same-direction
        -- refinement from creeping achieved clearance down cycle by cycle with no
        -- single cycle ever individually alarming - confirmed live 2026-09-04 (two
        -- separate SITL flights, both after the reversal-only fix above): a same-
        -- direction candidate is essentially NEVER "held_is_clear" once genuinely close
        -- to a boundary (a retest here always uses allow_straight=false, so even a
        -- fully-safe candidate returns a large FINITE sum, not probe_bearing()'s FLT_MAX
        -- short-circuit - see probe_bearing()'s own comment), so gating persistence on
        -- that boolean left same-direction refinement completely unresisted for the
        -- entire eroding approach, not just its final unsafe moment.
        --
        -- So: while still within a few standoffs of the boundary, refuse a fresh
        -- candidate that offers LESS clearance than the currently held one, whatever
        -- its direction - unless the held bearing has itself crossed into an actual
        -- violation (distance_previous_m <= 0), which still overrides everything, same
        -- as the reversal case above. Bounded to CONTESTED_ZONE_M so it never touches
        -- the far-field case the numeric retest is noisy in (two "fully safe, hundreds
        -- of metres clear" evaluations from slightly different positions are never
        -- exactly equal, and comparing them directly out there would eventually
        -- reproduce the original 16-23s-straight-flight regression this whole function
        -- exists to avoid).
        local CONTESTED_ZONE_M = margin_fence_m * 4
        if distance_previous_m > 0 and distance_previous_m < CONTESTED_ZONE_M
                and distance_found_m < distance_previous_m then
            return bearing_orig_deg, distance_previous_m
        end

        -- Either the committed bearing is no longer clear/has crossed into violation (a
        -- change is required, whatever its size), or the fresh candidate is at least as
        -- good as what is already held - accepted every cycle rather than holding the
        -- aircraft on one fixed bearing indefinitely just because it happens to stay
        -- clear (that held a SITL aircraft on a single straight heading for 16-23s past
        -- a small exclusion circle - see project_planedaa_reversal_awareness in memory).
        -- If this transition itself needs a reversal, latch that direction so a fresh
        -- sweep result next cycle cannot reverse it again before the aircraft responds.
        if needs_reversal then
            reversal_target_sign = needed_sign
            reversal_since_ms    = now_ms
        end
        return bearing_deg, distance_found_m
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

        -- Type-aware keep-out radius: the miss distance below which this obstacle is a conflict.
        -- The range check uses the same value, so any obstacle already inside the keep-out radius
        -- is unconditionally a conflict (the conservative floor); only one that will miss beyond it
        -- AND is not closing AND is already beyond it is treated as leaving (no manoeuvre needed).
        local standoff_m = get_standoff(obstacle.type)

        -- Time until the obstacle crosses that keep-out boundary: the earlier root of
        -- |rel + rv*t| = standoff_m.  This was range_h / closing_speed, which is the time to
        -- ZERO range - a point crossing traffic never reaches, so it read later than the
        -- encounter really was and left the slew limiter damping turns that had no time left.
        local ttc_s = FLT_MAX
        if rv2 > 1e-4 then
            local c_term = range_h * range_h - standoff_m * standoff_m
            if c_term <= 0.0 then
                ttc_s = 0.0                     -- already inside the keep-out radius
            else
                local disc = rel_dot_rv * rel_dot_rv - rv2 * c_term
                if disc >= 0.0 then
                    local t_enter = (-rel_dot_rv - math.sqrt(disc)) / rv2
                    if t_enter >= 0.0 then
                        ttc_s = t_enter         -- negative => boundary behind us, receding
                    end
                end
            end
        end
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
    Modified-tau conflict test for a CREWED AIRCRAFT specifically (RTCA DO-365C style) -
    NOT used for drones/UAVs, which keep assess_obstacle_motion()'s instantaneous
    velocity-vector CPA test (appropriate for their close range and genuinely erratic
    motion). A crewed aircraft is typically detected much further out, where a small error
    in an ESTIMATED velocity DIRECTION swings a projected miss-distance by hundreds of
    metres - and a circling/loitering aircraft's real instantaneous velocity genuinely does
    point toward us for part of every lap without it ever actually closing (confirmed live,
    log_130_2026-9-8: DstH held 590-660m the whole time a Brolga loitered, while the
    velocity-vector CPA swung 370-660m and repeatedly tripped the loiter). Modified tau
    sidesteps this by never estimating a velocity vector at all - it uses only range and the
    MEASURED, FILTERED rate of change of range itself (aircraft_closure_rate_ms, updated in
    detect_aircraft()), a scalar that is far less sensitive to the aircraft's instantaneous
    heading than a projected miss-distance is.

    tau_mod = (r^2 - dmod^2) / (r * closure_rate)   -- closure_rate > 0 means closing
    r <= dmod            => already inside the keep-out radius: unconditional conflict,
                            matching assess_obstacle_motion()'s own close-range floor.
    closure_rate <= 0    => not closing on average: no conflict regardless of range.
    tau_mod <= DAA_TAU_S => predicted to cross the keep-out radius soon enough to conflict.

    Declared as a bare GLOBAL, not `local function`, specifically to avoid costing
    DAAcore.new() one more of Lua's 100 per-function locals (see the "declare the new
    helper as a bare global" note in project_lua_binding_gotchas in memory) - it is still
    a real closure over this DAAcore instance's current_loc/aircraft_closure_rate_ms/etc,
    exactly like a local function would be; only where the function VALUE is stored
    differs. Safe here because this whole applet only ever constructs one DAAcore
    instance - if that ever changes, a second instance would silently redefine this same
    global over the first one's.
    --]]
    function assess_aircraft_conflict(obstacle)
        if obstacle == nil or current_loc == nil or obstacle.location == nil then
            return { is_conflict = true, tau_mod_s = 0.0, range_m = 0.0, closure_rate_ms = 0.0 }
        end
        -- Recomputed live from the CURRENT position, not obstacle.distance_xy (which is
        -- frozen at whatever it was when this obstacle's last ADS-B fix arrived, ~1 Hz -
        -- our own continued movement between fixes matters at cruise speed). The range
        -- HISTORY used to derive aircraft_closure_rate_ms in detect_aircraft() is anchored
        -- to real fix instants on purpose, which is unaffected by this.
        local range_m    = current_loc:get_distance(obstacle.location)
        local standoff_m = get_standoff(obstacle.type)

        if range_m <= standoff_m then
            return { is_conflict = true, tau_mod_s = 0.0, range_m = range_m,
                     closure_rate_ms = aircraft_closure_rate_ms }
        end
        if not aircraft_closure_rate_valid then
            -- No two-fix measurement yet for this contact (it just appeared - at ~1 Hz
            -- ADS-B fix rate this lasts about one cycle). Fall back to the existing
            -- instantaneous velocity-vector test rather than hardcoding a conflict here:
            -- that briefly reintroduces the noise this function exists to avoid, but only
            -- for one short-lived warm-up cycle, and it is what already correctly told a
            -- genuinely diverging aircraft (declared velocity available from its very
            -- first report) apart from a converging one before this function existed.
            -- Confirmed the hard way: hardcoding "conflict" here engaged the loiter once
            -- during warm-up and then never released it, because do_loitering()'s own
            -- release condition is proximity-only, not conflict-based, and was never
            -- designed to correct an early wrong answer (PlaneDAAAircraftCpaGate,
            -- 2026-09-08). Re-shaped into this function's own return table (not
            -- assess_obstacle_motion()'s) so callers - including the DAAT logging above -
            -- see a consistent set of fields regardless of which branch answered.
            local fallback = assess_obstacle_motion(obstacle)
            return { is_conflict = fallback.is_conflict, tau_mod_s = -1.0, range_m = range_m,
                     closure_rate_ms = aircraft_closure_rate_ms }
        end
        if aircraft_closure_rate_ms <= 0.0 then
            return { is_conflict = false, tau_mod_s = FLT_MAX, range_m = range_m,
                     closure_rate_ms = aircraft_closure_rate_ms }
        end
        local tau_mod_s = (range_m * range_m - standoff_m * standoff_m)
                / (range_m * aircraft_closure_rate_ms)
        return {
            is_conflict     = tau_mod_s <= tau_s,
            tau_mod_s       = tau_mod_s,
            range_m         = range_m,
            closure_rate_ms = aircraft_closure_rate_ms,
        }
    end

    --[[
    Post-process the raw bendy-ruler heading into a smooth command for a MOVING
    obstacle, WITHOUT ever overriding a turn the sweep needs to clear an obstacle.

    resist_bearing_change() already produces a bearing that clears the moving obstacle
    and only makes a large change when the new side is clearly clearer. So:
      * An urgent encounter, or a large change there is no time to damp, is a
        genuine avoidance turn -> obey it exactly, no smoothing.  (This is the
        safety fix: previously the side-commit could mirror such a turn onto the
        committed side and the slew limit could throttle it, flying the aircraft
        into the very fence the sweep was turning away from.)
      * "No time to damp" is a question about TIME, not about size.  It used to be
        size alone - any change over DAA_BR_ANGLE bypassed the smoothing - which
        meant the one case the smoothing was built for was the one case that
        reached the servos completely unfiltered: a ~180 degree side flip is always
        over DAA_BR_ANGLE.  With two obstacles in range the single-obstacle sweep
        alternates between them, and every alternation looked like a "necessary"
        reversal.  Flight log_87 (2026-08-27) commanded 34 changes over 45 degrees
        in one sortie, the worst 180 degrees in 0.26 s, none of them urgent.
        Asking instead whether the slew limit can deliver the change before the
        conflict leaves only the 3 that really were out of time.
      * Only SMALL residual changes (the left/right jitter) are damped, via a side
        commitment and a heading slew-rate limit. When holding a committed side we
        keep the LAST FLOWN bearing (known clear) rather than a mirrored one that
        was never clearance-checked.
    --]]
    -- (2) Side commitment: once a side is chosen, hold it until the sweep has wanted the
    -- other one continuously for DAA_SIDE_HOLD_S.  Returns the bearing to fly.
    local function apply_side_commitment(bearing, side, pass_behind)
        if side_hold_s <= 0 then
            return bearing
        end
        if committed_side_sign == 0 then
            -- fresh episode: commit; prefer passing behind a moving obstacle
            committed_side_sign = (pass_behind ~= 0) and pass_behind or side
            side_flip_pending = false
            return bearing
        end
        if side == 0 or side == committed_side_sign then
            side_flip_pending = false
            return bearing
        end
        -- sweep wants the opposite side: only honour it once it has persisted; meanwhile
        -- hold the last flown (clearance-proven) bearing, do NOT mirror
        if not side_flip_pending then
            side_flip_pending = true
            side_flip_want_ms = now_ms
        end
        if (now_ms - side_flip_want_ms) < (side_hold_s * 1000) then
            return last_avoid_bearing_deg
        end
        committed_side_sign = side
        side_flip_pending = false
        return bearing
    end

    -- (3) Heading slew-rate limit, for small changes only - large turns bypass this.
    local function apply_slew_limit(bearing)
        -- last_avoid_bearing_deg is shared with the fence-avoidance path, so it can
        -- already be non-nil the FIRST time a moving obstacle ever triggers this
        -- function in a flight (a fence was avoided earlier, this is not the "first
        -- cycle" refine_avoidance_bearing() thinks it is) - last_cmd_bearing_ms is
        -- only ever set below, in THIS function's own caller, so it genuinely is
        -- still nil then and must be checked on its own, not inferred from
        -- last_avoid_bearing_deg.  Without this a real flight crashed here
        -- (2026-09-04, log 00000183.BIN) the first time a drone avoidance ran after
        -- an earlier fence episode, aborting that cycle's avoidance decision
        -- entirely - a fence breach followed ~10s later.
        if slew_dps <= 0 or last_avoid_bearing_deg == nil or last_cmd_bearing_ms == nil then
            return bearing
        end
        local dt = (now_ms - last_cmd_bearing_ms):tofloat() / 1000.0
        local max_step = slew_dps * dt
        if max_step <= 0 then
            return bearing
        end
        local d = wrap_180(bearing - last_avoid_bearing_deg)
        if d > max_step then d = max_step elseif d < -max_step then d = -max_step end
        return wrap_360(last_avoid_bearing_deg + d)
    end

    local function refine_avoidance_bearing(direct_bearing_deg, raw_bearing_deg, raw_distance_m,
                                            motion, obstacle, target_loc)
        local pass_behind = motion.pass_behind
        local ttc_s = motion.ttc

        -- (1) clearance hysteresis: the safe, multi-obstacle, anti-flip baseline.  The
        -- returned distance is only accurate for `resisted` itself - apply_side_commitment
        -- and apply_slew_limit below can move the bearing further without a matching
        -- re-measurement, so a bearing damped by either is logged against this value as
        -- the best available figure, not a guaranteed-exact one.
        local resisted, resisted_distance_m =
                resist_bearing_change(last_avoid_bearing_deg, raw_bearing_deg, raw_distance_m, target_loc)
        local bearing = resisted

        local urgent = (ttc_s ~= nil) and (ttc_s < slew_urg_s)
        local change = (last_avoid_bearing_deg ~= nil) and math.abs(wrap_180(resisted - last_avoid_bearing_deg)) or 999.0

        -- which side of the direct bearing the sweep wants this cycle
        local off = wrap_180(resisted - direct_bearing_deg)
        local side = 0
        if off > 1.0 then side = 1 elseif off < -1.0 then side = -1 end

        -- Obey a large change exactly ONLY when damping it would make us late: if
        -- slewing at DAA_SLEW_DPS cannot cover the change before the conflict, there
        -- is no time left to be smooth.  ttc_s is FLT_MAX when nothing is closing, so
        -- a manoeuvre with time in hand always damps.
        local no_time_to_damp = change > bendy_angle and slew_dps > 0
                                and (ttc_s * slew_dps) < change

        if last_avoid_bearing_deg == nil or urgent or no_time_to_damp then
            -- first cycle, or a turn there is no time to damp: obey the sweep exactly.
            -- Record the side we are going round, so that a later reversal is seen as
            -- a flip to be debounced rather than as another "necessary" turn; leave it
            -- uncommitted on the very first cycle so pass_behind can choose below.
            committed_side_sign = (last_avoid_bearing_deg == nil) and 0 or side
            side_flip_pending = false
        else
            -- small adjustment only: damp the jitter ('side' was computed above from this
            -- same bearing)
            bearing = apply_side_commitment(bearing, side, pass_behind)
            bearing = apply_slew_limit(bearing)
        end
        last_cmd_bearing_ms = now_ms

        log_smoothing(direct_bearing_deg, raw_bearing_deg, resisted, bearing,
                      committed_side_sign, side_flip_pending, urgent, motion, obstacle)
        return bearing, resisted_distance_m
    end

    -- Second step of the bendy ruler test: having flown one step out to loc_test, look a
    -- further avoid_step2_m ahead to check this heading does not lead into a dead end.
    -- Mirrors AP_OABendyRuler::search_xy_path(): the three probes fan +/-45 degrees around
    -- the bearing from loc_test to the real destination (not around the step-1 heading),
    -- the leg is capped by the distance still to run, and the segment tested starts AT
    -- loc_test so it is contiguous with step 1, which covered current_loc -> loc_test.
    -- Returns the clearance found and whether the probe that produced it was the straight
    -- one (delta == 0); the caller needs that to decide "no avoidance required" without
    -- comparing recomputed bearings for equality.
    -- Reuse these offsets: test_step2() may run for many candidate headings in one
    -- update, so building this table in the function adds repeated heap churn.
    local test_bearings = { 0, 45, -45 }
    local function test_step2(loc_test, avoid_step2_m, destination_loc)
        local bearing_to_dest_deg   = math.deg(loc_test:get_bearing(destination_loc))
        local distance2_m           = math.max(math.min(avoid_step2_m,
                                                       loc_test:get_distance(destination_loc)), MIN_STEP2_M)

        local closest_distance_m    = FLT_MAX
        local closest_obstacle      = nil
        local straight              = false

        for _, delta in ipairs(test_bearings) do
            local bearing_test  = wrap_180(bearing_to_dest_deg + delta)
            local loc_test2     = location_project(loc_test, bearing_test, distance2_m, destination_loc)

            local distance_m, obstacle = find_closest_obstacle(loc_test, loc_test2, detect_m, wind_speed)

            if distance_m > detect_m then
                -- return immediately - no obstacles in this direction
                return distance_m, (delta == 0), nil
            end
            if distance_m < closest_distance_m then
                -- remember the worst blocker we saw, and whether the probe that hit it
                -- was the straight-at-the-destination one
                closest_distance_m  = distance_m
                closest_obstacle    = obstacle
                straight            = (delta == 0)
            end
        end

        return closest_distance_m, straight, closest_obstacle
    end

    -- Where the vehicle will actually BE once it has turned onto course_deg.  A candidate
    -- course is not flown from where we are now: getting onto it costs a turn, and the arc
    -- of that turn carries the vehicle up to 2R towards whatever lies on the inside of it.
    -- At 25 m/s and 60 deg of roll R is ~35 m, so a reversal displaces ~70 m - more than a
    -- fence standoff - and a heading judged from the present position can read as clear and
    -- still fly the vehicle through the fence.
    --
    -- Displacement around a circular arc of turn angle th is R*sin(th) along the current
    -- ground course plus R*(1 - cos(th)) to the side of the turn.  R is derived from the arc
    -- length the vehicle will actually cover (ground speed x turn time) rather than from
    -- airspeed, so the wind-corrected ground speed carries through and a near-zero turn
    -- degrades to "carry straight on".
    local function location_after_course_change(from_loc, course_deg, to_loc)
        local course_change_deg = wrap_180(course_deg - ground_course_deg)
        local ground_speed_ms   = effective_groundspeed(airspeed_ms, course_deg, wind_dir_rad, wind_speed)
        local rate_of_turn_dps  = max_turn_rate_dps(airspeed_ms, roll_limit_deg)

        if rate_of_turn_dps <= 0 or ground_speed_ms <= 0 then
            return from_loc                 -- no usable speed to turn at
        end
        local arc_length_m  = ground_speed_ms * (math.abs(course_change_deg) / rate_of_turn_dps)
        local turn_rad      = math.rad(math.abs(course_change_deg))
        if turn_rad < 1e-3 then
            return location_project(from_loc, ground_course_deg, arc_length_m, to_loc)
        end
        local radius_m      = arc_length_m / turn_rad
        -- Which way we turn: a reversal has no short way round, so the side is whatever
        -- wrap_180 gives it.  The sweep tries both sides of the target bearing, so the
        -- opposite reversal is still costed - as its own candidate, with its own bulge.
        local side_deg      = (course_change_deg >= 0) and (ground_course_deg + 90) or (ground_course_deg - 90)

        local projected_loc = location_project(from_loc, ground_course_deg, radius_m * math.sin(turn_rad), to_loc)
        return location_project(projected_loc, side_deg, radius_m * (1.0 - math.cos(turn_rad)), to_loc)
    end

    -- Stage 2: where the vehicle will actually be for a candidate that requires REVERSING
    -- bank direction from what it is currently, physically holding.  location_after_
    -- course_change() above assumes the target bank is established instantaneously, which
    -- is fine for a same-direction turn but is exactly what let a genuinely-required
    -- reversal get judged safe by a transition that cannot physically happen that fast -
    -- confirmed live (two SITL fence breaches, 2026-09-04) even after the reversal-latch
    -- fix (daacore.lua's resist_fence_bearing_change): the aircraft still had to
    -- momentarily fly through the OLD bank, wings-level, and the NEW bank before the
    -- constant-arc turn the old model assumed from the first instant.
    --
    -- Modelled as three bounded segments (current bank -> wings level -> target bank ->
    -- constant-bank arc), each a call to geometry.arc_projection() - no new obstacle
    -- probes here, only a more honest ANCHOR for the existing ones: probe_bearing()'s own
    -- step1/step2/turn-arc checks run unchanged against whatever this returns, so a
    -- reversal candidate costs the same number of obstacle-database queries as before,
    -- just against a physically achievable point instead of an unreachable one.
    --
    -- roll_rate_dps (RLL2SRV_RMAX, falling back to ROLL_LIMIT_DEG / RLL2SRV_TCONST when
    -- RMAX is left at its ArduPlane default of 0 - "rate limit disabled") bounds how fast
    -- each phase can happen; see DAAgeometry.roll_rate_dps().  Degrades to the plain
    -- single-arc model whenever there is nothing to unload (no reversal needed, or no roll
    -- telemetry/rate bound), so the ordinary case costs nothing extra.
    local function location_for_candidate(bearing_test_deg, target_loc)
        local course_change_deg = wrap_180(bearing_test_deg - ground_course_deg)
        local needed_sign       = (course_change_deg >= 0) and 1 or -1
        local current_sign      = bank_sign(current_roll_deg)
        if current_sign == 0 or current_sign == needed_sign or roll_rate_dps <= 0 then
            return location_after_course_change(current_loc, bearing_test_deg, target_loc)
        end

        local ground_speed_ms   = effective_groundspeed(airspeed_ms, bearing_test_deg, wind_dir_rad, wind_speed)
        local rate_at_limit_dps = max_turn_rate_dps(airspeed_ms, roll_limit_deg)
        if ground_speed_ms <= 0 or rate_at_limit_dps <= 0 then
            return location_after_course_change(current_loc, bearing_test_deg, target_loc)
        end

        -- Phase A: unload the CURRENT bank to wings-level, still turning the OLD way at
        -- whatever roll the aircraft actually holds right now - it does not teleport to
        -- wings-level, that costs |current_roll_deg| / roll_rate_dps seconds, banked the
        -- old way for all of it.
        local unload_time_s = math.abs(current_roll_deg) / roll_rate_dps
        local mid_loc, mid_heading_deg = arc_projection(current_loc, ground_course_deg, current_sign,
                unload_time_s, airspeed_ms, ground_speed_ms, math.abs(current_roll_deg), target_loc)

        -- Phase B: roll INTO the new direction up to the full roll limit - the target bank
        -- is not established instantly either.
        local establish_time_s = roll_limit_deg / roll_rate_dps
        local established_loc, established_heading_deg = arc_projection(mid_loc, mid_heading_deg, needed_sign,
                establish_time_s, airspeed_ms, ground_speed_ms, roll_limit_deg, target_loc)

        -- Phase C: whatever course change is still outstanding, at the now-established
        -- bank - the same constant-bank model every other candidate is judged by, just
        -- starting from where the transition actually leaves the aircraft rather than
        -- from where it is right now.
        local remaining_deg    = wrap_180(bearing_test_deg - established_heading_deg)
        local remaining_time_s = math.abs(remaining_deg) / rate_at_limit_dps
        return (arc_projection(established_loc, established_heading_deg,
                (remaining_deg >= 0) and 1 or -1, remaining_time_s,
                airspeed_ms, ground_speed_ms, roll_limit_deg, target_loc))
    end

    -- The straight leg above starts where the turn ENDS, so on its own it never looks at the
    -- ground the turn itself covers.  One extra probe along the chord from here to that point
    -- closes the gap: against log 161's exclusion circles it caught every case that sampling
    -- the arc in eight segments caught, and none were missed.  Only worth its cost once the
    -- turn is big enough to bow away from the leg - below DAA_BR_ANGLE the chord lies along
    -- it - which also skips it for the small-deflection candidates the sweep usually wins on.
    local function probe_turn_arc(adjusted_loc, bearing_test_deg, distance_found_m, obstacle_found)
        if math.abs(wrap_180(bearing_test_deg - ground_course_deg)) <= bendy_angle then
            return distance_found_m, obstacle_found
        end
        if current_loc:get_distance(adjusted_loc) < MIN_TURN_CHORD_M then
            return distance_found_m, obstacle_found  -- too short to have a meaningful bearing
        end
        local turn_distance_m, turn_obstacle =
                find_closest_obstacle(current_loc, adjusted_loc, detect_m, wind_speed)
        if turn_distance_m ~= nil and turn_distance_m < distance_found_m then
            return turn_distance_m, turn_obstacle
        end
        return distance_found_m, obstacle_found
    end

    -- Core clearance probe for an explicit candidate course bearing_test_deg. Returns
    -- (distance_found_m, bearing_test_deg, obstacle_found); a clear course returns
    -- FLT_MAX with obstacle_found == nil. allow_straight lets the unobstructed
    -- straight-ahead path short-circuit (only meaningful for the i == 0 candidate).
    -- (forward-declared above, for resist_bearing_change's benefit)
    probe_bearing = function(bearing_test_deg, bearing_deg, full_distance, target_loc, allow_straight)
        local avoid_step1_m     = current_lookahead
        local avoid_step2_m     = current_lookahead * 2.0

        -- Start the look-ahead from where we will actually be after turning onto this
        -- candidate course.  This used to be applied only in wind, as a way of leading the
        -- carrot downwind; it is really a TURN lead and calm air needs it just as much.
        -- Without it the probe assumes the vehicle is already on the candidate course, so a
        -- heading that needs a reversal is judged against a path the turn never flies.
        -- location_for_candidate() upgrades this further for a candidate that reverses
        -- bank direction from what the aircraft is currently, physically holding - see
        -- its own comment for why the plain single-arc model is not safe there.
        local adjusted_loc          = location_for_candidate(bearing_test_deg, target_loc)

        -- Position after one step from where we think we will be after turning to bearing_test_deg
        local avoidance_distance_m  = calc_avoidance_distance(avoid_step1_m, full_distance)
        local test_loc              = location_project(adjusted_loc, bearing_test_deg, avoidance_distance_m, target_loc)

        local distance_found_m, obstacle_found = find_closest_obstacle(adjusted_loc, test_loc, detect_m, wind_speed)
        if distance_found_m == nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "closest returned NIL ")
            return FLT_MAX, bearing_deg, nil -- no avoidance required
        end
        distance_found_m, obstacle_found = probe_turn_arc(adjusted_loc, bearing_test_deg,
                                                         distance_found_m, obstacle_found)
        if distance_found_m > detect_m then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            local distance2_m, straight2, obstacle2 = test_step2(test_loc, avoid_step2_m, target_loc)
            if distance2_m >= detect_m then
                if allow_straight and straight2 then
                    -- means we have a direct unobstructed path for step1 and step2
                    return FLT_MAX, bearing_deg, nil -- no avoidance required
                end
                -- we've found at least one direction where there is no obstacle at least for 2 steps out
                distance_found_m = distance_found_m + distance2_m
            elseif obstacle2 ~= nil then
                -- All three second-leg probes are blocked: this heading is clear for one
                -- step but leads into a dead end.  Report the blocker so self.detect() keeps
                -- sweeping (it ends the sweep on a nil obstacle, so without this the step-2
                -- result was discarded and the heading was accepted as clear).
                return distance2_m, bearing_test_deg, obstacle2
            end
        end

        return distance_found_m, bearing_test_deg, obstacle_found
    end

    -- This method checks whether we will collide with any obstacle if we fly at a given bearing bearing_deg + i * inc_deg
    -- inc_deg is the sweep step: the coarse pass of self.detect() passes a multiple of
    -- DAA_HEADING_INC, and the refine pass probes explicit bearings via probe_bearing().
    local function test_step1(full_distance, bearing_deg, i, target_loc, inc_deg)
        local bearing_delta_deg = i * inc_deg / 2.0
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
            local safe_max_alt_m, max_alt_frame = fence:get_safe_alt_max()
            local ceiling_m = safe_max_alt_m - margin_alt_m
            local current_alt_m = loc:get_alt_m(max_alt_frame)
            if current_alt_m ~= nil and current_alt_m > ceiling_m then
                loc:set_alt_m(ceiling_m, max_alt_frame)
            end
        end
        if (enabled & FENCE_TYPE_ALT_MIN) ~= 0 then
            local safe_min_alt_m, min_alt_frame = fence:get_safe_alt_min()
            local floor_alt_m = safe_min_alt_m + margin_alt_m
            local current_alt_m = loc:get_alt_m(min_alt_frame)
            if current_alt_m ~= nil and current_alt_m < floor_alt_m then
                loc:set_alt_m(floor_alt_m, min_alt_frame)
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
    local alt_fence_active  = false
    local alt_fence_near    = false
    local last_alt_alert_ms = uint32_t(0)

    -- Proactively detect that we are approaching (or projected to cross) an altitude fence.
    -- Only kicks in for the fences enabled in FENCE_TYPE: bit 0 (ALT_MAX) and/or bit 3 (ALT_MIN).
    -- The vertical position is projected forward using the current climb rate over the time it takes to fly
    -- the lookahead distance (capped to a sane vertical horizon) so we level off before the band is reached.
    -- Returns a synthetic obstacle while corrective action is needed (latched), otherwise nil.
    local function detect_altitude_fence()
        if fence == nil or current_loc == nil then
            alt_fence_active    = false
            alt_fence_near      = false
            return nil
        end
        local enabled = fence:get_enabled_fences()
        local alt_max_on = (enabled & FENCE_TYPE_ALT_MAX) ~= 0
        local alt_min_on = (enabled & FENCE_TYPE_ALT_MIN) ~= 0
        if not alt_max_on and not alt_min_on then
            alt_fence_active    = false
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
        local horizon_s = detect_m / math.max(groundspeed_ms, 1.0)
        horizon_s = math.min(math.max(horizon_s, 1.0), 20.0)

        -- pick whichever enabled altitude fence currently needs (or is already taking) action.
        -- enter when current or projected altitude is past the clamp limit; while latched, only release
        -- once we are DAA_ALT_HYST_M clear of the limit on both current and projected altitude.
        local otype, label_str, headroom_m
        if alt_max_on then
            local safe_max_alt_m, max_alt_frame = fence:get_safe_alt_max()
            local current_alt_m = current_loc:get_alt_m(max_alt_frame)
            if current_alt_m ~= nil then
                local ceiling_m = safe_max_alt_m - margin_alt_m
                local projected_alt_m = current_alt_m + climb_rate_ms * horizon_s
                local enter = current_alt_m > ceiling_m or projected_alt_m > ceiling_m
                local clear = current_alt_m < ceiling_m - alt_hyst_m and projected_alt_m < ceiling_m - alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MAX, "Alt Max Fence"
                    headroom_m = safe_max_alt_m - current_alt_m     -- metres below the safe ceiling
                end
            end
        end
        if label_str == nil and alt_min_on then
            local safe_min_alt_m, min_alt_frame = fence:get_safe_alt_min()
            local current_alt_m = current_loc:get_alt_m(min_alt_frame)
            if current_alt_m ~= nil then
                local floor_alt_m = safe_min_alt_m + margin_alt_m
                local projected_alt_m = current_alt_m + climb_rate_ms * horizon_s
                local enter = current_alt_m < floor_alt_m or projected_alt_m < floor_alt_m
                local clear = current_alt_m > floor_alt_m + alt_hyst_m and projected_alt_m > floor_alt_m + alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MIN, "Alt Min Fence"
                    headroom_m = current_alt_m - safe_min_alt_m     -- metres above the safe floor
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

        alt_fence_near      = near
        alt_fence_active    = now_active
        if not now_active then
            return nil
        end
        return make_alt_fence_obstacle(otype, label_str, math.max(headroom_m, 0.0))
    end

    -- crewed aircraft are a special case. We do specific things if there is an aircraft nearby so we need to know the nearest one
    local function detect_aircraft()
        if current_loc == nil then
            aircraft_avoiding = nil
            last_aircraft_obstacle = nil
            last_aircraft_ts_ms = nil
            last_aircraft_range_m = nil
            aircraft_closure_rate_ms = 0.0
            aircraft_closure_rate_valid = false
            return
        end

        -- search out to the well clear distance (plus the crewed-aircraft margin), matching the
        -- treatment in the bendy ruler path, so aircraft are detected and logged at a
        -- useful range rather than only once they are within DAA_MARGIN_CA of us
        -- pass the full gate distance for each axis (computed here, applied in C++): the
        -- horizontal gate is well_clear_xy + margin_crewed_m, the vertical gate is
        -- well_clear_z + margin_vertical_m
        local distance_m, aircraft_obstacle = OAScripting:find_aircraft(current_loc, well_clear_xy + margin_crewed_m, well_clear_z + margin_vertical_m)

        -- a parked/taxiing aircraft (DAA_GND_ALT_M/DAA_GND_SPD_MS) is not a threat regardless of
        -- range - treated the same as no aircraft found, so it never latches the loiter/NMAC
        -- state below. See is_grounded_traffic() in daaobs.lua for why this exists.
        if distance_m ~= nil and aircraft_obstacle ~= nil
                and is_grounded_traffic(aircraft_obstacle:obstacle_type(), aircraft_obstacle) then
            distance_m = nil
            aircraft_obstacle = nil
        end

        if distance_m == nil or aircraft_obstacle == nil then
            aircraft_avoiding       = nil
            last_aircraft_obstacle  = nil
            last_aircraft_ts_ms     = nil
            last_aircraft_range_m   = nil
            aircraft_closure_rate_ms = 0.0
            aircraft_closure_rate_valid = false
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

        -- Update assess_aircraft_conflict()'s MEASURED range-rate filter from this fresh
        -- fix, before last_aircraft_range_m/last_aircraft_ts_ms are overwritten below - see
        -- that function's own comment for why this is tracked instead of a velocity vector.
        if last_aircraft_range_m ~= nil and last_aircraft_ts_ms ~= nil then
            -- OAObstacle's timestamp_ms is bound as int32_t (a plain Lua number), NOT the
            -- boxed uint32_t_ud millis() returns - no :tofloat() here, that is a userdata
            -- method and this is a plain number.
            local dt_s = (ts_ms - last_aircraft_ts_ms) / 1000.0
            if dt_s > 0.0 and dt_s < AIRCRAFT_TAU_GAP_S then
                local raw_rate_ms = (last_aircraft_range_m - obstacle.distance_xy) / dt_s
                local alpha = math.min(1.0, dt_s / AIRCRAFT_TAU_FILTER_S)
                aircraft_closure_rate_ms = aircraft_closure_rate_ms
                        + alpha * (raw_rate_ms - aircraft_closure_rate_ms)
                aircraft_closure_rate_valid = true
            else
                aircraft_closure_rate_ms = 0.0     -- gap too large to trust
                aircraft_closure_rate_valid = false
            end
        end
        last_aircraft_range_m  = obstacle.distance_xy

        aircraft_avoiding       = obstacle
        last_aircraft_obstacle  = obstacle
        last_aircraft_ts_ms     = ts_ms

        log_detect_aircraft(aircraft_avoiding)

        local motion = assess_aircraft_conflict(obstacle)
        local status, err = pcall(logger.write, logger, "DAAT",
            'TauS,ClsR,Rng,Con',
            'fffB',
            'snm-',
            '----',
            math.min(motion.tau_mod_s, 999.0),   -- TauS - modified-tau time to keep-out (capped)
            motion.closure_rate_ms,              -- ClsR - filtered measured closure rate
            motion.range_m,                      -- Rng  - current horizontal range
            motion.is_conflict and 1 or 0)        -- Con  - conflict verdict this cycle
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log tau:" .. tostring(err) )
        end
    end


    -- Coarse pass of the sweep: step at coarse_inc_deg around the full circle (test_step1
    -- alternates left/right) until a clear heading is found or every candidate is exhausted.
    -- Returns clear_delta_deg (the signed deflection of the first clear heading, or nil if
    -- boxed in - nothing cleared) and obstacle_distance_m (the worst blocker seen, needed by
    -- the boxed-in refine below), plus the possibly-improved best_distance_m/best_bearing_deg
    -- (every probe can improve these, clear or not).  A separate function rather than a
    -- goto-out-of-loop: returning on the first clear heading is the same short-circuit,
    -- without a jump target.
    local function coarse_sweep(bearing_deg, distance_to_target_m, target_loc, coarse_inc_deg,
                                best_distance_m, best_bearing_deg)
        local obstacle_distance_m = FLT_MAX
        for i = 0, math.floor(360 / coarse_inc_deg) do
            local distance_found_m, bearing_found_deg, obstacle_found =
                    test_step1(distance_to_target_m, bearing_deg, i, target_loc, coarse_inc_deg)
            if distance_found_m > best_distance_m then
                best_distance_m     = distance_found_m
                best_bearing_deg    = bearing_found_deg
            end
            if obstacle_found == nil then -- found a path with no obstacles - done!
                return wrap_180(bearing_found_deg - bearing_deg), obstacle_distance_m,
                       best_distance_m, best_bearing_deg
            end
            if distance_found_m < obstacle_distance_m then
                obstacle_avoiding   = obstacle_found
                obstacle_distance_m = distance_found_m
            end
        end
        return nil, obstacle_distance_m, best_distance_m, best_bearing_deg
    end

    -- Probe one refine candidate at centre_deg + delta_deg, shared by both refine loops in
    -- sweep_for_heading() below.  They differ in what centre and delta sequence they walk
    -- (see the comments at each call site) but do the same thing with each candidate: adopt
    -- it if it clears, otherwise fold it into the running worst-blocker distance so the
    -- boxed-in case still has a best-available fallback.  Returns cleared (true/false) and,
    -- only when cleared, the distance/bearing to adopt; obstacle_distance_m is always
    -- returned since the caller's running total must carry across candidates that don't clear.
    local function probe_refine_candidate(centre_deg, delta_deg, direct_bearing_deg,
                                          distance_to_target_m, target_loc, obstacle_distance_m)
        local distance_found_m, bearing_found_deg, obstacle_found =
                probe_bearing(wrap_180(centre_deg + delta_deg), direct_bearing_deg,
                             distance_to_target_m, target_loc, false)
        if obstacle_found == nil then
            return true, distance_found_m, bearing_found_deg, obstacle_distance_m
        end
        if distance_found_m < obstacle_distance_m then
            obstacle_avoiding   = obstacle_found
            obstacle_distance_m = distance_found_m
        end
        return false, nil, nil, obstacle_distance_m
    end

    -- Sweep for the heading that best clears the obstacles between here and target_loc.
    -- Returns the updated best_distance_m and best_bearing_deg; obstacle_avoiding is an
    -- upvalue and is updated in place as closer obstacles are found.
    -- Third return value `swept` is for DAAR diagnostics only (RawOK) - true iff the sweep
    -- actually ran and best_distance_m/best_bearing_deg are its real answer, false on the
    -- under-20m early return below, where they are still just the caller's initial values
    -- (best_distance_m in particular is the -FLT_MAX sentinel, not a genuine "everything is
    -- unsafe" reading).
    local function sweep_for_heading(bearing_deg, distance_to_target_m, target_loc,
                                     best_distance_m, best_bearing_deg)
        -- Under 20 m to the target there is nothing useful to sweep for.  Only the sweep
        -- declines: detect_aircraft() and detect_altitude_fence() are independent of it, and
        -- returning from self.detect() here - as this check used to - cleared aircraft_avoiding
        -- and then suppressed traffic alerts, NMAC, the trapped failsafe, the aircraft loiter
        -- and the altitude clamp for as long as the target stayed close.
        if distance_to_target_m < 20 then
            return best_distance_m, best_bearing_deg, false
        end
        -- Try increments around a circle, alternating left and right. The first heading
        -- that clears all obstacles for two look-ahead steps wins (a bounded downwind
        -- preference is applied afterwards, once we know we are avoiding).
        --
        -- The sweep is coarse-to-fine. A full-resolution sweep is 360/DAA_HEADING_INC
        -- candidates (241 at the 1.5 deg default) and it only exits early when a heading
        -- clears, so the boxed-in case - no heading clears at all - runs every candidate
        -- and each one costs an obstacle probe. That worst case can exceed SCR_VM_I_COUNT,
        -- which does not merely skip a cycle: the VM kills the script outright, mid
        -- avoidance, and it stays dead for the rest of the flight. Sweeping at
        -- COARSE_SWEEP_MULT x the increment and refining only around the winner keeps the
        -- final angular resolution while cutting the worst case by ~COARSE_SWEEP_MULT.
        local coarse_inc_deg  = bearing_inc_deg * COARSE_SWEEP_MULT
        local clear_delta_deg, obstacle_distance_m
        clear_delta_deg, obstacle_distance_m, best_distance_m, best_bearing_deg =
                coarse_sweep(bearing_deg, distance_to_target_m, target_loc, coarse_inc_deg,
                            best_distance_m, best_bearing_deg)

        -- Refine. The clear coarse heading sits one coarse step beyond the last blocked one,
        -- so the smallest deflection that actually clears lies inside that window. Walk the
        -- window at the full DAA_HEADING_INC resolution, nearest-to-target first, and take the
        -- first heading that still clears - which restores the "least deflection that works"
        -- result of the original fine sweep. Nothing to refine when the direct bearing was
        -- already clear, or when we are boxed in and no heading cleared: skipping the refine in
        -- the boxed-in case is exactly what keeps that (most expensive) case cheap.
        if clear_delta_deg ~= nil and clear_delta_deg ~= 0 then
            local sign     = (clear_delta_deg > 0) and 1 or -1
            local clear_mag = math.abs(clear_delta_deg)
            local prev_mag  = math.max(clear_mag - coarse_inc_deg, 0)   -- last blocked candidate this side
            for k = 1, COARSE_SWEEP_MULT do
                local test_mag = prev_mag + k * bearing_inc_deg
                if test_mag >= clear_mag then
                    break   -- reached the known-clear coarse heading; keep it
                end
                local cleared, distance_found_m, bearing_found_deg
                cleared, distance_found_m, bearing_found_deg, obstacle_distance_m =
                        probe_refine_candidate(bearing_deg, sign * test_mag, bearing_deg,
                                               distance_to_target_m, target_loc, obstacle_distance_m)
                if cleared then
                    -- a smaller deflection also clears, so prefer it (closer to the direct path)
                    best_distance_m  = distance_found_m
                    best_bearing_deg = bearing_found_deg
                    break
                end
            end
        elseif obstacle_avoiding ~= nil then
            -- Boxed in: no coarse heading cleared. A gap narrower than the coarse step can be
            -- stepped straight over, so probe outwards from the most open coarse candidate at
            -- the full resolution before accepting that we are trapped. Bounded at
            -- 2 * (COARSE_SWEEP_MULT - 1) extra probes, and it also restores full-resolution
            -- steering for the boxed-in case itself, which the coarse pass alone would leave
            -- on the coarse grid.
            local centre_deg = best_bearing_deg
            for j = 1, 2 * (COARSE_SWEEP_MULT - 1) do
                -- alternate either side of the most open heading: +1, -1, +2, -2, ... steps
                local step_n = math.floor((j + 1) / 2)
                local sign   = (j % 2 == 1) and 1 or -1
                local cleared, distance_found_m, bearing_found_deg
                cleared, distance_found_m, bearing_found_deg, obstacle_distance_m =
                        probe_refine_candidate(centre_deg, sign * step_n * bearing_inc_deg, bearing_deg,
                                               distance_to_target_m, target_loc, obstacle_distance_m)
                if cleared then
                    -- there was a gap after all; steer for it (still avoiding, so
                    -- obstacle_avoiding stays set, exactly as the full sweep would leave it)
                    best_distance_m  = distance_found_m
                    best_bearing_deg = bearing_found_deg
                    break
                end
            end
        end
        return best_distance_m, best_bearing_deg, true
    end

    -- The two obstacle-response resolvers below mirror detect_aircraft()/detect_altitude_fence()'s
    -- shape - each answers one question about obstacle_avoiding (already chosen by the sweep) and
    -- reports back via upvalues, the same style everything else in this closure already uses.
    -- They are what "choose" means in detect_impl()'s gather -> choose -> project -> log shape.

    -- Clear ALL fence/moving-obstacle avoidance state at once - an EPISODE BOUNDARY
    -- (nothing left to avoid this cycle: the moving obstacle cleared, or the sweep found
    -- no obstacle at all), not a mid-episode context switch.  resolve_fence_bearing()
    -- below deliberately does NOT call this: it has just committed a fresh fence bearing
    -- this cycle and must not discard it, only the moving-obstacle-specific smoothing
    -- state that no longer applies while on the fence path.  fence_hold_bearing_deg is
    -- deliberately NOT one of these fields either, for the same reason but stronger: it
    -- must survive being called from resolve_moving_bearing()'s "obstacle opening"
    -- branch below, so a moving obstacle outranking a still-relevant fence for a cycle
    -- or two cannot erase the fence's own memory.  Only validate_horizontal_release()
    -- clears it, and only once it has confirmed the fence is genuinely clear.
    local function reset_horizontal_avoidance()
        last_avoid_bearing_deg = nil
        reversal_target_sign   = nil
        committed_side_sign    = 0
        side_flip_pending      = false
    end

    -- Fences are fixed and containment is safety-critical: a heading slew limit or a committed
    -- side could delay/deflect the turn at a hard boundary and breach it, so this is hysteresis
    -- only (resist_fence_bearing_change), no smoothing, no CPA - the responsive bendy-ruler
    -- behaviour.
    -- Returns the (possibly resisted) bearing/distance to fly; best_distance_m is reassigned to
    -- the clearance of whichever bearing comes back - not the discarded candidate's - so
    -- DAAD.DstB reflects what is actually flown, including on the "stay the course" path.
    local function resolve_fence_bearing(target_loc, best_bearing_deg, best_distance_m)
        best_bearing_deg, best_distance_m = resist_fence_bearing_change(
            last_avoid_bearing_deg, best_bearing_deg, best_distance_m, target_loc)
        last_avoid_bearing_deg  = best_bearing_deg
        -- Independent of last_avoid_bearing_deg - see fence_hold_bearing_deg's own
        -- declaration and reset_horizontal_avoidance()'s comment for why.
        fence_hold_bearing_deg  = best_bearing_deg
        committed_side_sign     = 0
        side_flip_pending       = false
        return best_bearing_deg, best_distance_m
    end

    -- Non-fixed obstacles (aircraft, drones, birds, AIS, ...): velocity-aware smoothing.
    -- First decides whether the obstacle is actually a conflict: one that is opening range and
    -- whose predicted closest approach stays beyond well-clear is leaving, so avoidance should
    -- resume nav (obstacle_gone = true tells the caller to return nil immediately, same as the
    -- early return this branch used to make inline). Otherwise commits a side and slew-limits
    -- the heading so we track a smooth path instead of wiggling as the obstacle (and the
    -- instantaneous geometry) moves; refine_avoidance_bearing() also logs the DAAS smoothing
    -- trace each cycle.
    local function resolve_moving_bearing(bearing_deg, target_loc, best_bearing_deg, best_distance_m)
        -- resist_fence_bearing_change()'s reversal latch is fence-specific state: clear it
        -- whenever the resolver switches to a moving obstacle instead, so it cannot survive
        -- stale into a later, unrelated fence episode.
        reversal_target_sign = nil
        local motion = assess_obstacle_motion(obstacle_avoiding)
        if not motion.is_conflict then
            -- the obstacle is leaving (opening range, predicted miss beyond its keep-out
            -- radius): drop it so avoid_obstacle() does not steer or announce for it. Any
            -- avoidance already in progress reverts cleanly (avoid_obstacle(nil)). This is
            -- re-decided every cycle from current geometry (no hold) so a manoeuvring obstacle
            -- is always tracked on fresh data; near a marginal crossing that can cost a few
            -- extra (slew-limited) heading reversals, which is the safe trade.
            obstacle_avoiding       = nil
            reset_horizontal_avoidance()
            return best_bearing_deg, best_distance_m, true
        end
        best_bearing_deg, best_distance_m = refine_avoidance_bearing(
            bearing_deg, best_bearing_deg, best_distance_m, motion, obstacle_avoiding, target_loc)
        last_avoid_bearing_deg  = best_bearing_deg
        return best_bearing_deg, best_distance_m, false
    end

    -- Project a real continuation of the aircraft's CURRENT bank for VALIDATE_PROJECTION_S
    -- seconds - not location_after_course_change()'s instantaneous-bank assumption the
    -- sweep's own candidates use, and not a bearing-specific stub. Shared by
    -- validate_horizontal_release() and the DAAR diagnostics' final-bearing remeasure in
    -- detect_impl(), so both look at the same physically-honest "where is the aircraft
    -- actually about to be" point rather than two different approximations of it.
    local function project_current_trajectory(target_loc)
        local current_sign = bank_sign(current_roll_deg)
        if current_sign ~= 0 and roll_rate_dps > 0 then
            local ground_speed_ms = effective_groundspeed(airspeed_ms, ground_course_deg, wind_dir_rad, wind_speed)
            return arc_projection(current_loc, ground_course_deg, current_sign,
                    VALIDATE_PROJECTION_S, airspeed_ms, ground_speed_ms,
                    math.abs(current_roll_deg), target_loc)
        end
        -- Wings-level (or no usable roll-rate bound): continuing straight is the honest
        -- projection.
        return location_project(current_loc, ground_course_deg,
                math.max(airspeed_ms, 1.0) * VALIDATE_PROJECTION_S, target_loc)
    end

    -- One shared release-validation path for every way a horizontal avoidance can lapse
    -- this cycle - the sweep finding nothing at all, or a moving obstacle opening up -
    -- and, since round ten (00000196.BIN), also called every cycle a moving obstacle is
    -- STILL the live winner, so a fence eroding underneath a long-running moving-obstacle
    -- avoidance is not invisible until whatever cycle that avoidance happens to end on.
    -- Never called while a fence is the live winner - resolve_fence_bearing() already
    -- applies its own hysteresis in that case.
    --
    -- Always runs the check below, regardless of whether a fence previously won this
    -- cycle's single-obstacle choice: a moving obstacle being dismissed as
    -- non-conflicting only clears THAT contact, not necessarily the fence geometry
    -- underneath it. FENCE-SPECIFIC (obstacles.find_closest_fence(), not the generic
    -- find_closest_obstacle()) so a still-open moving obstacle elsewhere on the same
    -- projected path cannot mask the fence all over again. Confirmed live 2026-09-05
    -- (logs 00000187.BIN, 00000188.BIN): the sweep's own raw result was already
    -- negative (genuinely blocked) when the winning moving obstacle was dismissed as
    -- non-conflicting, but nothing checked that sign before accepting the release -
    -- fence_hold_bearing_deg was nil (the fence had never won before), so the old
    -- fence_hold_bearing_deg == nil early-return this replaced skipped the check
    -- entirely.
    --
    -- fence_hold_bearing_deg carries "a fence was recently relevant" independently of
    -- whichever obstacle actually won - see its declaration - and this is the only
    -- place that clears it. It is reused only if its OWN complete path also still
    -- clears - never blindly: the aircraft has moved on since it was set, and a stale
    -- hold can itself now cross the fence. Confirmed live 2026-09-05 (log
    -- 00000187.BIN): a retained bearing took the aircraft within 0.7 m of a fence
    -- while a fresh solution had already moved ~177 degrees away.
    --
    -- The held-bearing and escape-candidate checks below (fence_blocks_path()) project
    -- the aircraft's real BANK-AWARE path - the turn transition to
    -- location_for_candidate(), then the outgoing leg - not a straight ray from
    -- current_loc. A straight ray let a held bearing through whose actual flown path
    -- was already blocked: confirmed live 2026-09-05 (log 00000191.BIN, second of two
    -- breaches) - FPrD (the bank-aware commanded path, logged separately below) read
    -- -8.14 m a full cycle before the straight-ray check this replaced would have
    -- rejected it, and the crossing followed one cycle later. When there is no usable
    -- held bearing, the general sweep cannot be trusted to supply one either - it can
    -- be dominated or boxed in by a moving obstacle with nothing to do with this fence
    -- - so the escape search below hunts for one against the fence specifically, and
    -- that is what gets the usual hysteresis applied to it.
    --
    -- Considered and reverted (2026-09-05, same day): a "never release while
    -- fence:get_breaches() is set" recovery latch, meant to stop a fence disappearing
    -- from find_closest_fence() the instant AC_Fence itself flags a breach (the SAME
    -- deliberate skip find_threats() uses so a bendy ruler cannot be trapped forever
    -- trying to get back into an unreachable fence - see _find_fence_threats_NE()'s own
    -- comment). Live data (00000191.BIN's FIRST breach) shows that "vanish and revert
    -- to nav" is not actually harmful on its own: FncD recovered from -47 m back to
    -- positive over about 4 seconds on a smooth, continuing bearing, via ordinary
    -- navigation alone, no active re-avoidance needed. The latch, meanwhile, broke two
    -- existing tests whose whole premise is a fence that stays breached (or nearly so)
    -- for an extended stretch by design - PlaneDAABreachScopedToFenceType's
    -- permanently-distant home circle, PlaneDAAFenceDriftReversal's tight racetrack -
    -- because it cannot tell "still actively escaping a fresh breach" from "this fence
    -- is never going to un-breach and must be stood down", which is exactly what
    -- find_threats()'s original skip was built to do. The bank-aware geometry fix above
    -- already covers the one concretely reproduced defect (the -8.14 m case); the
    -- "vanish" pattern itself is left as the pre-existing, deliberate design.
    --
    -- Fourth return value `held` is for DAAR diagnostics only (see detect_impl()'s own
    -- DAAR block) - nothing in the resolution logic itself reads it.
    local function validate_horizontal_release(target_loc, candidate_bearing_deg, candidate_distance_m)
        local near_loc = project_current_trajectory(target_loc)
        local _, fence_obstacle =
                obstacles.find_closest_fence(current_loc, near_loc, detect_m, wind_speed)

        if fence_obstacle == nil then
            -- Genuinely clear of every fence on the path actually being flown.
            fence_hold_bearing_deg = nil
            return candidate_bearing_deg, candidate_distance_m, nil, false
        end

        -- Bank-aware path check for a candidate bearing: the turn transition to
        -- location_for_candidate(), then the outgoing leg - see this function's own
        -- comment for why. Declared HERE, not as another DAAcore.new() local: that
        -- function is already near Lua's 200-local-per-function ceiling (hit it three
        -- times already this session) and this has two call sites, both within this
        -- one function. obstacles.find_closest_fence() already applies its own 1m
        -- shift-toward-the-endpoint on each segment, so a segment running parallel to
        -- an exclusion edge cannot skirt along it.
        local function fence_blocks_path(bearing_test_deg)
            local adjusted_loc = location_for_candidate(bearing_test_deg, target_loc)
            local turn_distance_m, turn_obstacle =
                    obstacles.find_closest_fence(current_loc, adjusted_loc, detect_m, wind_speed)
            if turn_obstacle ~= nil then
                return turn_distance_m, turn_obstacle
            end
            local leg_loc = location_project(adjusted_loc, bearing_test_deg, detect_m, target_loc)
            return obstacles.find_closest_fence(adjusted_loc, leg_loc, detect_m, wind_speed)
        end

        if fence_hold_bearing_deg ~= nil then
            local held_distance_m, held_obstacle = fence_blocks_path(fence_hold_bearing_deg)
            if held_obstacle == nil then
                return fence_hold_bearing_deg, held_distance_m, fence_obstacle, true
            end
        end

        -- Fence-only escape search, inlined rather than a separate function (DAAcore.new()
        -- is already near Lua's 200-local-per-function ceiling - confirmed the hard way,
        -- three times now - and this has exactly one call site). No turn-lead modeling, no
        -- step-2 dead-end check like the general sweep (sweep_for_heading()/
        -- coarse_sweep()) - this only has to hand resolve_fence_bearing() A safe bearing,
        -- not the bendy-ruler's full sophistication, and it only runs here: when the held
        -- bearing above was either never armed or its own path is now also blocked, so the
        -- general sweep cannot be trusted to supply one (it can be dominated or boxed in
        -- by a moving obstacle with nothing to do with this fence). Searches alternating
        -- either side of candidate_bearing_deg, same "least deviation first" order the
        -- general sweep uses, and stops at the first fence-clear heading found.
        -- Seed the search from fence_hold_bearing_deg when there is one, even though
        -- its own path is what just failed the check above - it is still a far better
        -- anchor than candidate_bearing_deg, which after a moving-obstacle dismissal
        -- (Gon=1 in DAAR) is whatever noisy bearing THAT sweep produced, arbitrary and
        -- unstable from a fence's point of view. Seeding from it produced wide,
        -- discontinuous swings with no relation to the fence avoidance actually in
        -- progress - confirmed live 2026-09-05 (log 00000189.BIN): FnlB jumped from
        -- 53 to 289 degrees in one cycle. candidate_bearing_deg remains the only
        -- option on a genuinely first-ever encounter with this fence.
        local seed_bearing_deg  = fence_hold_bearing_deg or candidate_bearing_deg
        local fresh_bearing_deg = seed_bearing_deg
        local fresh_distance_m  = -FLT_MAX
        local coarse_inc_deg    = bearing_inc_deg * COARSE_SWEEP_MULT
        for i = 0, math.floor(360 / coarse_inc_deg) do
            local delta_deg = i * coarse_inc_deg / 2.0
            if i % 2 == 1 then
                delta_deg = -delta_deg
            end
            local test_deg = wrap_180(seed_bearing_deg + delta_deg)
            local distance_m, obstacle = fence_blocks_path(test_deg)
            if obstacle == nil then
                fresh_bearing_deg, fresh_distance_m = test_deg, FLT_MAX
                break
            end
            if distance_m > fresh_distance_m then
                fresh_bearing_deg, fresh_distance_m = test_deg, distance_m
            end
        end

        local resolved_bearing_deg, resolved_distance_m =
                resolve_fence_bearing(target_loc, fresh_bearing_deg, fresh_distance_m)
        return resolved_bearing_deg, resolved_distance_m, fence_obstacle, true
    end

    -- detect flying objects or fences when flying towards navigation_target_loc
    local function detect_impl()
        -- TODO be smarter about re-populating this
        obstacle_avoiding = nil
        aircraft_avoiding = nil

        -- we want to calculate avoidance towards the current NAVIGATION TARGET (navigation_target_loc) - coping to target_loc to avoid changing the copy/pasted code
        if navigation_target_loc == nil or current_loc == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, " AVOIDING: NO TARGET ")
            return
        end
        local target_loc = navigation_target_loc:copy()

        local bearing_deg       = math.deg(current_loc:get_bearing(target_loc))
        local best_bearing_deg  = bearing_deg
        local best_distance_m   = -FLT_MAX

        local distance_to_target_m = limit_distance(current_loc, target_loc, bearing_deg)
        local raw_ok
        best_distance_m, best_bearing_deg, raw_ok =
                sweep_for_heading(bearing_deg, distance_to_target_m, target_loc,
                                  best_distance_m, best_bearing_deg)

        -- Raw sweep answer, captured before any fence-hold/side-commit/slew adjustment -
        -- DAAR's RawB/RawD/RawOK.  detect_aircraft() below only ever touches
        -- aircraft_avoiding, never best_bearing_deg/best_distance_m, so this is genuinely
        -- what the sweep alone found. raw_ok false means the under-20m early return -
        -- RawD is then just the -FLT_MAX sentinel, not a real "boxed in" reading.
        local raw_bearing_deg, raw_distance_m = best_bearing_deg, best_distance_m

        -- we need to independently detect aircraft because even if an aircraft may not be the closest obstacle found by bendy ruler, we may still need to deal with it
        -- in other words, sometimes aircraft have higher priority than any other obstacles
        detect_aircraft()

        -- proactively check the altitude fences (vertical clamp-and-continue)
        local alt_obstacle = detect_altitude_fence()

        local obstacle_type = obstacle_avoiding ~= nil and obstacle_avoiding.type or nil
        -- obstacles.is_fence_obstacle() covers the horizontal fence types; the altitude
        -- fences are handled separately there (see its own comment) but also route
        -- through resolve_fence_bearing() below, so this ORs them in explicitly.
        local is_fence = obstacle_type ~= nil and (
            obstacles.is_fence_obstacle(obstacle_type)
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN)

        local gone = false
        if obstacle_avoiding ~= nil and is_fence then
            best_bearing_deg, best_distance_m =
                    resolve_fence_bearing(target_loc, best_bearing_deg, best_distance_m)
        elseif obstacle_avoiding ~= nil then
            best_bearing_deg, best_distance_m, gone =
                    resolve_moving_bearing(bearing_deg, target_loc, best_bearing_deg, best_distance_m)
            if gone then
                obstacle_avoiding = nil
            end
        end

        -- DAAR's Sel/Gon: the type actually selected before resolver processing (nil if
        -- the sweep found nothing at all this cycle), and whether resolve_moving_bearing()
        -- is what dismissed it - captured here, before validate_horizontal_release() can
        -- reassign obstacle_avoiding below.
        local was_release_candidate = (obstacle_avoiding == nil)
        -- Round ten (00000196.BIN, 2026-09-06): a moving obstacle can keep winning the
        -- single-obstacle choice for many seconds straight while a fence quietly erodes
        -- to a breach underneath it - the release check below used to run only at the
        -- INSTANT the moving obstacle stopped winning, so a fence that never got that
        -- instant (this one didn't: the drone was dismissed for good only after the
        -- aircraft had already crossed the boundary) was never checked at all. Run the
        -- same fence-reality check while a moving obstacle is still actively being
        -- avoided too - it is one line query, the same cost validate_horizontal_release()
        -- already pays at every ordinary release, not the full sweep.
        local was_moving_avoidance = obstacle_avoiding ~= nil and not is_fence and not gone
        local held = false
        if was_release_candidate or was_moving_avoidance then
            -- Either the sweep found nothing this cycle, a moving obstacle just opened
            -- up, or a moving obstacle is still being avoided - none of those can be
            -- trusted alone to mean the fence is not a problem: the sweep's "clear"
            -- verdict assumes the aircraft has ALREADY turned onto the chosen bearing,
            -- and a moving obstacle winning the single-obstacle choice can silently
            -- outrank a fence that is still close, for one cycle or for the whole
            -- encounter.  validate_horizontal_release() is the one place this is
            -- checked against reality before being accepted.
            local resolved_bearing_deg, resolved_distance_m, resolved_obstacle
            resolved_bearing_deg, resolved_distance_m, resolved_obstacle, held =
                    validate_horizontal_release(target_loc, best_bearing_deg, best_distance_m)
            if was_release_candidate or resolved_obstacle ~= nil then
                -- A genuine release (fence confirmed clear too - resolved_obstacle is
                -- nil), a release the fence just vetoed, OR the fence overriding an
                -- ongoing moving-obstacle avoidance because the path being flown is no
                -- longer safe: all three replace best_bearing_deg/best_distance_m and
                -- reassign obstacle_avoiding to whatever validate_horizontal_release()
                -- decided. The one case that must NOT overwrite obstacle_avoiding with
                -- nil is "still avoiding a moving obstacle and the fence check found
                -- the path clear" (was_release_candidate false, resolved_obstacle nil) -
                -- that is excluded by this same condition, leaving
                -- resolve_moving_bearing()'s own answer untouched below.
                best_bearing_deg, best_distance_m, obstacle_avoiding =
                        resolved_bearing_deg, resolved_distance_m, resolved_obstacle
            end
        end

        -- DAAR diagnostics - only when validate_horizontal_release() actually ran this
        -- cycle (was_release_candidate or was_moving_avoidance): not every idle cruise
        -- cycle (nothing to remeasure - FinalB is just the direct-to-target bearing the
        -- sweep already validated), and not while resolving a fence directly either
        -- (resolve_fence_bearing() already applied its own hysteresis to FinalB).
        --   TrjD - clearance of continuing the CURRENT bank/heading, unrelated to what is
        --          actually being commanded - "what happens if I fly on as I am".
        --   CmdD - clearance of the trajectory actually needed to REACH FinalB
        --          (location_for_candidate()'s bank-aware transition model, the same one
        --          Stage 2 reversal candidates are judged by), so FinalB is validated
        --          against the turn it really implies, not the aircraft's present bank.
        --   FPrD - fence-only clearance ALONG that same CmdD path (obstacles.
        --          find_closest_fence(), a line query, immune to being masked by
        --          traffic) - not just at its endpoint (a path can cross a fence and
        --          finish outside it, confirmed live 2026-09-05 log 00000188.BIN).
        local traj_distance_m  = best_distance_m
        local cmd_distance_m   = best_distance_m
        local fence_proj_m     = nil
        if was_release_candidate or was_moving_avoidance then
            local traj_loc = project_current_trajectory(target_loc)
            traj_distance_m = find_closest_obstacle(current_loc, traj_loc, detect_m, wind_speed)
            local cmd_loc = location_for_candidate(best_bearing_deg, target_loc)
            cmd_distance_m = find_closest_obstacle(current_loc, cmd_loc, detect_m, wind_speed)
            fence_proj_m = obstacles.find_closest_fence(current_loc, cmd_loc, detect_m, wind_speed)
        end
        -- Inlined rather than a separate local function: DAAcore.new() is already near
        -- Lua's 200-local-per-function ceiling (confirmed the hard way - a nested
        -- function here pushed it over and failed to load at all), and this has exactly
        -- one call site.
        local fence_clearance_m = obstacles.nearest_fence_clearance_m(current_loc)
        -- Field names are abbreviated - logger.write() hard-limits the label string to
        -- 57 chars total (confirmed the hard way twice now: the first full-name
        -- attempt ran to 61 and the write silently failed EVERY cycle, flooding
        -- STATUSTEXT with the error and disrupting other tests' own message timing).
        -- RawOK->ROK, FenceD->FncD, FinalB->FnlB, TrajD->TrjD, FenceProjD->FPrD,
        -- SelType->Sel, Gone->Gon.
        local status, err = pcall(logger.write, logger, "DAAR",
            'RawB,RawD,ROK,FncD,FnlB,TrjD,CmdD,FPrD,Held,ObjT,Sel,Gon',
            'ffBfffffBIIB',                  -- Formats
            'dm-mdmmm----',                  -- Units (d=degrees, m=meter)
            '------------',                  -- Multipliers (none needed - no lat/lng here)
            wrap_360(raw_bearing_deg),       -- RawB - sweep's own bearing, before adjustment
            math.max(math.min(raw_distance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            raw_ok and 1 or 0,               -- ROK - false means RawD is a sentinel, not real
            -- FncD/FPrD are SIGNED (positive clear, negative breached) since the
            -- AP_OAScripting::fence_distance() fix - LOG_CLEARANCE_MAX_M is a "no fence
            -- loaded" sentinel, not a "very safe" one, same as every other field here.
            (fence_clearance_m == nil) and LOG_CLEARANCE_MAX_M
                or math.max(math.min(fence_clearance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            wrap_360(best_bearing_deg),      -- FnlB - bearing actually being commanded
            math.max(math.min(traj_distance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            math.max(math.min(cmd_distance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            (fence_proj_m == nil) and LOG_CLEARANCE_MAX_M
                or math.max(math.min(fence_proj_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            held and 1 or 0,
            (obstacle_avoiding ~= nil and obstacle_avoiding.type) or 0,
            obstacle_type or 0,              -- Sel - type selected BEFORE resolver processing
            gone and 1 or 0)                 -- Gon - did resolve_moving_bearing() dismiss it
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log resolve:" .. tostring(err))
        end

        if obstacle_avoiding == nil then
            reset_horizontal_avoidance()
            if alt_obstacle ~= nil then
                -- no horizontal threat, but we are approaching an altitude fence: keep heading to the
                -- waypoint and let update_target_location() clamp the commanded altitude into the safe band
                obstacle_avoiding = alt_obstacle
                local alt_target_loc = navigation_target_loc:copy()
                clamp_alt_to_fence(alt_target_loc)
                return alt_target_loc
            end
            return nil -- no avoidance required
        end

        -- Where to put the commanded target along the bearing we picked - DAA_PLAN_M, which
        -- until 4.8.0-080 was DAA_LKAHD and so could not be set independently of how far the
        -- sweep probed.  Do not shorten it casually.  This location REPLACES next_WP_loc, and
        -- ArduPlane draws its past-the-waypoint finish line THROUGH next_WP_loc: a distant
        -- target puts that line out of reach, while a near one puts it alongside the aircraft,
        -- and the moment the avoidance bearing has any component back towards the previous
        -- waypoint the mission completes and moves on.  Shortening it to
        -- max(WP_LOITER_RAD, 2 x WP_RADIUS) was tried and measurably worse -
        -- PlaneDAAHungTrapFires skipped its waypoint at 106 m and finished 7 m off the fence
        -- instead of clearing it.
        local proj_distance = math.max(distance_to_target_m, plan_m)
        local new_target_loc = location_project(current_loc, best_bearing_deg, proj_distance, target_loc)
        log_detect_result(obstacle_avoiding.distance_m, best_distance_m, distance_to_target_m,
                          best_bearing_deg, new_target_loc, obstacle_avoiding.type)
        return new_target_loc
    end
    -- Multiple return values, not a report table: this is what lets mechanism and policy
    -- live in separate files, since detect_impl() still returns just the suggested target
    -- and the two contacts it settled on are handed back alongside it rather than left in
    -- shared state for the applet to read - but only actively avoiding builds this at all,
    -- so a table here would be one more allocation on the already-heavier active cycles.
    function self.detect()
        local target_loc = detect_impl()
        return target_loc, obstacle_avoiding, aircraft_avoiding
    end

    self.configure          = configure
    self.update_state       = update_state
    self.clamp_alt_to_fence = clamp_alt_to_fence
    -- the aircraft-loiter policy asks whether a contact is actually converging before it
    -- commits to a loiter, so the CPA assessment is part of the mechanism's public face
    self.assess_obstacle_motion  = assess_obstacle_motion
    self.assess_aircraft_conflict = assess_aircraft_conflict

    return self
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAcore.SCRIPT_NAME, DAAcore.SCRIPT_VERSION))

return DAAcore
