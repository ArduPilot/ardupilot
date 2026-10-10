--[[
    Altitude loiter for planedaa.lua: orbit in GUIDED to lose (or hold) altitude, usually to
    let a crewed aircraft pass, then hand the vehicle back to the mode it came from.

    This is ONE IMPLEMENTATION of a loiter policy, not the only possible one.  The applet
    talks to it through five members only:

        .active                        true while the loiter is running
        start(alt_m, frame, right, spd, mission_loc) begin; returns true if a loiter is running after the call
        stop(force)                    end it; returns false if it declined (cool-down)
        update()                       call regularly while active; notices a pilot mode change
        aircraft_seen()                refresh the cool-down timer

    Anything providing those five can be dropped in - require "daaltr2" in planedaa.lua and
    nothing else changes.  That is the point of the seam: the applet owns which policy is
    used, the module owns how it is carried out.

    Everything from the applet is pushed in rather than reached for: new() takes the
    constants and helpers, configure() the cached parameters, update_state() the per-cycle
    vehicle state.
--]]

local DAAloiter = {}

DAAloiter.SCRIPT_VERSION = "4.8.0-005"
DAAloiter.SCRIPT_NAME = "DAA loiter"
DAAloiter.SCRIPT_NAME_SHORT = "DAAloiter"

-- Load-banner severity only - NOT exported, and not what the instance logs with below.
-- This module is a tightly-coupled collaborator of planedaa.lua, not a standalone library
-- like pid.lua (whose SCRIPT_NAME/SCRIPT_VERSION pattern this follows) - MAV_SEVERITY stays
-- an injected dependency below so there is one shared severity table, not one per module
-- that could theoretically drift. MAV_SEVERITY.INFO is a fixed MAVLink wire value (6),
-- unlike that shared table, so a private literal here carries no drift risk of its own.
local BANNER_SEVERITY = 6

-- daageo is stateless (see that file) and wrap_360 needs no configured state, so it is
-- required directly rather than injected.
local wrap_360 = require("daageo").wrap_360

-- A refused reposition (fence, sanitize, mode-change blocked, ...) used to be re-attempted
-- every cycle at INFO severity forever, invisible unless someone went grepping. Escalate
-- once it has been stuck a while, then repeat at a sane cadence instead of every cycle.
local LOITER_FAIL_ESCALATE_MS = 3000
local LOITER_FAIL_REPEAT_MS   = 5000

function DAAloiter.new(deps)
    local self = { active = false }

    local PLANE_MODE                = deps.PLANE_MODE
    local ALT_FRAME                 = deps.ALT_FRAME
    local MAV_DO_REPOSITION_FLAGS   = deps.MAV_DO_REPOSITION_FLAGS
    local MAV_SEVERITY              = deps.MAV_SEVERITY
    local SCRIPT_NAME_SHORT         = DAAloiter.SCRIPT_NAME_SHORT
    local get_mode_string           = deps.get_mode_string
    local mavlink_wrappers          = deps.mavlink_wrappers
    local clamp_alt_to_fence        = deps.clamp_alt_to_fence
    -- point clearance (m, signed) to the nearest horizontal fence boundary of any category -
    -- see self.start()/self.update() for why the orbit needs this and the reposition target
    -- sanitize check alone does not.
    local nearest_fence_clearance_m = deps.nearest_fence_clearance_m

    -- pushed in by configure()
    local loiter_cool_ms, wp_loiter_rad_m, margin_fence_m
    -- pushed in by update_state()
    local current_loc, current_mode, now_ms
    -- the cool-down clock is ours alone: nothing outside this module reads it
    local aircraft_seen_now_ms = millis()
    -- consecutive-refusal tracking, reset on any success: nothing outside this module reads it
    local loiter_fail_since_ms = nil
    local loiter_fail_notify_ms = nil
    -- set by update() when it force-stops the loiter for coming too close to a fence; start()
    -- refuses to re-engage until this elapses, so DAA.avoid() falls through to ordinary
    -- bendy-ruler/fence avoidance for a while instead of immediately re-looping into the same
    -- fence (the aircraft conflict that wanted the loiter is usually still present the very
    -- next cycle).
    local fence_bailout_until_ms = nil

    local function configure(settings)
        loiter_cool_ms   = settings.loiter_cool_ms
        wp_loiter_rad_m  = settings.wp_loiter_rad_m
        margin_fence_m   = settings.margin_fence_m
    end

    -- Positional, not a table: called every cycle, and a table literal here would be one
    -- more transient allocation the run never keeps.
    local function update_state(new_current_loc, new_current_mode, new_now_ms)
        current_loc  = new_current_loc
        current_mode = new_current_mode
        now_ms       = new_now_ms
    end

    local pre_loiteralt_heading_deg = -1.0
    local previous_mode = -1
    local target_alt_m = nil
    local target_alt_frame = ALT_FRAME.GLOBAL
    -- the GUIDED destination the loiter reposition is about to overwrite, when we are started
    -- while already in GUIDED (see the comment on it below, and on the matching stop() branch)
    local saved_guided_target_loc = nil

    -- DO_REPOSITION (and mavlink_wrappers.lua's frame conversion for it) has no ABOVE_ORIGIN
    -- case and silently falls through to absolute (AMSL) - see planedaa.md's "Terrain (default
    -- altitude frame)" section for DAA_AVD_ALT_TP=2. Convert ORIGIN to GLOBAL ourselves before
    -- either call site hands the frame off, so a loiter commanded above-origin (or a saved
    -- GUIDED destination in that frame) sends the altitude it actually means. loc must already
    -- be positioned at the target lat/lng; it is set to (alt_m, alt_frame) and mutated in place
    -- by change_alt_frame.
    local function resolve_origin_alt(loc, alt_m, alt_frame)
        if alt_frame ~= ALT_FRAME.ORIGIN then
            return alt_m, alt_frame
        end
        loc:set_alt_m(alt_m, alt_frame)
        if loc:change_alt_frame(ALT_FRAME.GLOBAL) then
            local converted_alt_m = loc:get_alt_m(ALT_FRAME.GLOBAL)
            if converted_alt_m ~= nil then
                return converted_alt_m, ALT_FRAME.GLOBAL
            end
        end
        -- no origin set to convert from: nothing safe to do but send what we were given
        return alt_m, alt_frame
    end

    -- Returns true when the loiter is running once this call returns, so the caller can
    -- decide whether to enter STATE.loitering.  It used to return nil on every path,
    -- including the three that do not loiter - already active, no current_loc, and the
    -- vehicle refusing the target - and callers set STATE.loitering regardless, so the
    -- state machine could claim to be loitering while self.active was false.
    function self.start(new_alt_m, new_alt_frame, direction_right, _speed_ms, mission_target_loc)
        local direction

        if self.active then
            return true     -- already loitering: the caller's state is correct as it stands
        end

        if fence_bailout_until_ms ~= nil then
            if now_ms < fence_bailout_until_ms then
                return false    -- recently bailed out of this loiter for a fence - let
                                 -- ordinary avoidance handle it for a while rather than
                                 -- immediately re-looping into the same fence
            end
            fence_bailout_until_ms = nil
        end

        if current_loc == nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": loiteralt no current_location")
            return false
        end
        pre_loiteralt_heading_deg   = math.deg(ahrs:get_yaw_rad())
        target_alt_frame            = new_alt_frame
        target_alt_m                = new_alt_m

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

        -- The reposition command below is sanitize()-checked against fences by the core, but
        -- only at the CENTRE point - the orbit this will actually fly, radius_m around it, is
        -- invisible to that check. A fence within radius_m + margin_fence_m of the centre can
        -- still be breached mid-loiter (confirmed live, AreaXO 2026-10-06, a cell-tower
        -- exclusion circle). Check the real edge clearance here and try the other side once
        -- before giving up - update() below keeps checking for the rest of the loiter's life.
        if nearest_fence_clearance_m ~= nil then
            local clearance_m = nearest_fence_clearance_m(loiteralt_loc)
            if clearance_m ~= nil and clearance_m < (radius_m + margin_fence_m) then
                local flipped_right = not direction_right
                local flipped_loc = current_loc:copy()
                if flipped_right then
                    flipped_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg + 90), radius_m)
                else
                    flipped_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg - 90), radius_m)
                end
                local flipped_clearance_m = nearest_fence_clearance_m(flipped_loc)
                if flipped_clearance_m ~= nil and flipped_clearance_m >= (radius_m + margin_fence_m) then
                    direction        = flipped_right and "right" or "left"
                    loiteralt_loc    = flipped_loc
                else
                    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT ..
                            ": loiteralt no fence-clear side - falling through to ordinary avoidance")
                    return false
                end
            end
        end

        -- Every other commanded target goes through update_target_location(), which
        -- clamps into the safe altitude-fence band (DAA_MARGIN_ALT inside FENCE_ALT_MAX/MIN)
        -- before committing it - this is the one manoeuvre that deliberately changes
        -- altitude and it alone bypassed that, able to command below FENCE_ALT_MIN or
        -- above FENCE_ALT_MAX. Clamp the same way here, then re-read whatever frame/value
        -- the clamp left it in.
        if clamp_alt_to_fence ~= nil then
            loiteralt_loc:set_alt_m(target_alt_m, target_alt_frame)
            clamp_alt_to_fence(loiteralt_loc)
            target_alt_frame = loiteralt_loc:get_alt_frame()
            local clamped_alt_m = loiteralt_loc:get_alt_m(target_alt_frame)
            if clamped_alt_m ~= nil then
                target_alt_m = clamped_alt_m
            end
        end

        target_alt_m, target_alt_frame = resolve_origin_alt(loiteralt_loc, target_alt_m, target_alt_frame)

        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": LOITER %s to %.0f/%.0f(%.0f) alt radius %.0f m",
                direction, target_alt_m, target_alt_frame, mavlink_wrappers.alt_frame_to_mavlink(target_alt_frame), radius_m ))

        -- Ask the reposition to change mode itself.  Every rejection in Plane's
        -- handle_command_int_do_reposition() - bad location, failed sanitize(), outside the
        -- fence - returns before it touches the mode, so a refused loiter leaves the
        -- vehicle exactly where it was and there is nothing to undo.  Switching to GUIDED
        -- here first would mean owning that undo, and getting it wrong strands the aircraft
        -- in GUIDED with self.active false, which nothing recovers from.
        previous_mode = vehicle:get_mode()
        -- The mode-restore branch in stop() only undoes a MODE change (see its own comment);
        -- if we are already in GUIDED, this same DO_REPOSITION is about to overwrite GUIDED's
        -- own destination instead. Save it so stop() can put it back. mission_target_loc, not
        -- vehicle:get_target_location() - GUIDED's live target is the DAA avoidance carrot
        -- whenever avoidance is already running, and restoring that would fly to a stale
        -- avoidance waypoint instead of the real mission destination.
        if previous_mode == PLANE_MODE.GUIDED then
            saved_guided_target_loc = mission_target_loc
        else
            saved_guided_target_loc = nil
        end
        -- Plane's DO_REPOSITION handler (handle_command_int_do_reposition()) picks orbit
        -- direction from param4 (yaw), not the radius sign: zero/NaN is clockwise, any
        -- nonzero value is counter-clockwise. A hardcoded 0 here flew every loiter
        -- clockwise regardless of which side the centre was offset to - confirmed live,
        -- an announced "LOITER left" swept +618 deg clockwise about its own centre.
        local yaw = (direction == "right") and 0 or 1
        if mavlink_wrappers.set_vehicle_target_location({lat    = loiteralt_loc:lat(),
                                                        lng     = loiteralt_loc:lng(),
                                                        alt     = target_alt_m,
                                                        frame   = target_alt_frame,
                                                        radius  = radius_m,
                                                        yaw     = yaw,
                                                        bitmask = MAV_DO_REPOSITION_FLAGS.CHANGE_MODE }) then
            self.active = true
            loiter_fail_since_ms = nil
            loiter_fail_notify_ms = nil
        else
            if loiter_fail_since_ms == nil then
                loiter_fail_since_ms = now_ms
            end
            local stuck_ms = now_ms - loiter_fail_since_ms
            if stuck_ms >= LOITER_FAIL_ESCALATE_MS and
                    (loiter_fail_notify_ms == nil or (now_ms - loiter_fail_notify_ms) >= LOITER_FAIL_REPEAT_MS) then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
                        ": loiteralt set_vehicle FAILED for %.0f s - refused repeatedly", stuck_ms * 0.001))
                loiter_fail_notify_ms = now_ms
            else
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt set_vehicle FAILED" ))
            end
            previous_mode = -1
        end

        return self.active
    end

    function self.aircraft_seen()
        aircraft_seen_now_ms = now_ms
    end

    function self.stop(force_stop)
        if not force_stop then
            -- hold the loiter for DAA_LTR_COOL_S after the aircraft was last seen, so a
            -- briefly-dropped or laggy feed cannot thrash GUIDED<->AUTO
            if (now_ms - aircraft_seen_now_ms) < loiter_cool_ms then
                return false
            end
        end
        if previous_mode >= 0 and previous_mode ~= PLANE_MODE.GUIDED then
            if not vehicle:set_mode(previous_mode) then
                -- say so rather than announcing a handback that did not happen: the vehicle
                -- is still in GUIDED on the loiter target and the pilot needs to know.
                -- Keep previous_mode/self.active so the next call retries instead of
                -- losing the restore state.
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": Loiter Done but %s REFUSED - still in Guided", get_mode_string(previous_mode) ))
                return false
            end
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Loiter Done set mode: %s", get_mode_string(previous_mode) ))
            gcs:send_named_string("DAA-AVOID", "")
            gcs:send_named_float("DAA-LOITER", 0.0)
        elseif previous_mode == PLANE_MODE.GUIDED and saved_guided_target_loc ~= nil then
            -- no mode change to undo, but the loiter's own DO_REPOSITION overwrote GUIDED's
            -- destination when we started it (see self.start()) - put it back, or the vehicle
            -- is left circling at the loiter point instead of continuing to where GUIDED had
            -- actually been sent.
            local restore_alt_frame = saved_guided_target_loc:get_alt_frame()
            local restore_alt_m = saved_guided_target_loc:get_alt_m(restore_alt_frame)
            if restore_alt_m ~= nil then
                restore_alt_m, restore_alt_frame = resolve_origin_alt(saved_guided_target_loc:copy(), restore_alt_m, restore_alt_frame)
            end
            if restore_alt_m == nil then
                -- can't reconstruct the altitude (e.g. home/origin no longer set) - nothing
                -- safe to reissue, so fall through and just drop the stale destination
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": Loiter Done but GUIDED destination alt unavailable - not restored")
            elseif not mavlink_wrappers.set_vehicle_target_location({lat   = saved_guided_target_loc:lat(),
                                                                      lng   = saved_guided_target_loc:lng(),
                                                                      alt   = restore_alt_m,
                                                                      frame = restore_alt_frame}) then
                -- as above: keep previous_mode/saved_guided_target_loc so the next call retries
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": Loiter Done but GUIDED destination restore REFUSED - still circling")
                return false
            else
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Loiter Done, restored GUIDED destination")
                gcs:send_named_string("DAA-AVOID", "")
                gcs:send_named_float("DAA-LOITER", 0.0)
            end
        end
        previous_mode = -1
        saved_guided_target_loc = nil
        self.active = false
        return true
    end

    -- should be called regularly if loiteralt is active
    function self.update()
        if not self.active then
            return
        end
        if current_mode ~= PLANE_MODE.GUIDED then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Pilot changed from GUIDED to: %.0f", current_mode ))
            previous_mode = -1
            self.stop(true)
            return
        end
        -- The side check in self.start() only looked at the centre once; the vehicle keeps
        -- moving around the orbit afterward, so re-check real fence clearance from the
        -- CURRENT position every cycle for the rest of the loiter's life.
        if nearest_fence_clearance_m ~= nil and current_loc ~= nil then
            local clearance_m = nearest_fence_clearance_m(current_loc)
            if clearance_m ~= nil and clearance_m < margin_fence_m then
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
                        ": fence %.0f m - bailing out of loiter", clearance_m))
                fence_bailout_until_ms = now_ms + loiter_cool_ms
                self.stop(true)
            end
        end
    end

    self.configure     = configure
    self.update_state  = update_state

    return self
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAloiter.SCRIPT_NAME, DAAloiter.SCRIPT_VERSION))

return DAAloiter
