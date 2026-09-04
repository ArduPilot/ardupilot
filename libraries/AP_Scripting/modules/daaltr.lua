--[[
    Altitude loiter for planedaa.lua: orbit in GUIDED to lose (or hold) altitude, usually to
    let a crewed aircraft pass, then hand the vehicle back to the mode it came from.

    This is ONE IMPLEMENTATION of a loiter policy, not the only possible one.  The applet
    talks to it through five members only:

        .active                        true while the loiter is running
        start(alt_m, frame, right, spd) begin; returns true if a loiter is running after the call
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

DAAloiter.SCRIPT_VERSION = "4.8.0-002"
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

function DAAloiter.new(deps)
    local self = { active = false }

    local PLANE_MODE                = deps.PLANE_MODE
    local ALT_FRAME                 = deps.ALT_FRAME
    local MAV_DO_REPOSITION_FLAGS   = deps.MAV_DO_REPOSITION_FLAGS
    local MAV_SEVERITY              = deps.MAV_SEVERITY
    local SCRIPT_NAME_SHORT         = DAAloiter.SCRIPT_NAME_SHORT
    local get_mode_string           = deps.get_mode_string
    local mavlink_wrappers          = deps.mavlink_wrappers

    -- pushed in by configure()
    local loiter_cool_ms, wp_loiter_rad_m
    -- pushed in by update_state()
    local current_loc, current_mode, now_ms
    -- the cool-down clock is ours alone: nothing outside this module reads it
    local aircraft_seen_now_ms = millis()

    local function configure(settings)
        loiter_cool_ms   = settings.loiter_cool_ms
        wp_loiter_rad_m  = settings.wp_loiter_rad_m
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

    -- Returns true when the loiter is running once this call returns, so the caller can
    -- decide whether to enter STATE.loitering.  It used to return nil on every path,
    -- including the three that do not loiter - already active, no current_loc, and the
    -- vehicle refusing the target - and callers set STATE.loitering regardless, so the
    -- state machine could claim to be loitering while self.active was false.
    function self.start(new_alt_m, new_alt_frame, direction_right, _speed_ms)
        local direction

        if self.active then
            return true     -- already loitering: the caller's state is correct as it stands
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
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": LOITER %s to %.0f/%.0f(%.0f) alt radius %.0f m",
                direction, target_alt_m, target_alt_frame, mavlink_wrappers.alt_frame_to_mavlink(target_alt_frame), radius_m ))

        -- Ask the reposition to change mode itself.  Every rejection in Plane's
        -- handle_command_int_do_reposition() - bad location, failed sanitize(), outside the
        -- fence - returns before it touches the mode, so a refused loiter leaves the
        -- vehicle exactly where it was and there is nothing to undo.  Switching to GUIDED
        -- here first would mean owning that undo, and getting it wrong strands the aircraft
        -- in GUIDED with self.active false, which nothing recovers from.
        previous_mode = vehicle:get_mode()
        if mavlink_wrappers.set_vehicle_target_location({lat    = loiteralt_loc:lat(),
                                                        lng     = loiteralt_loc:lng(),
                                                        alt     = target_alt_m,
                                                        frame   = target_alt_frame,
                                                        radius  = radius_m,
                                                        yaw     = 0,
                                                        bitmask = MAV_DO_REPOSITION_FLAGS.CHANGE_MODE }) then
            self.active = true
        else
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt set_vehicle FAILED" ))
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
            if vehicle:set_mode(previous_mode) then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Loiter Done set mode: %s", get_mode_string(previous_mode) ))
            else
                -- say so rather than announcing a handback that did not happen: the vehicle
                -- is still in GUIDED on the loiter target and the pilot needs to know
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": Loiter Done but %s REFUSED - still in Guided", get_mode_string(previous_mode) ))
            end
            gcs:send_named_string("DAA-AVOID", "")
            gcs:send_named_float("DAA-LOITER", 0.0)
        end
        previous_mode = -1
        self.active = false
        return true
    end

    -- should be called regularly if loiteralt is active
    function self.update()
        if self.active and current_mode ~= PLANE_MODE.GUIDED then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Pilot changed from GUIDED to: %.0f", current_mode ))
            previous_mode = -1
            self.stop(true)
        end
    end

    self.configure     = configure
    self.update_state  = update_state

    return self
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAloiter.SCRIPT_NAME, DAAloiter.SCRIPT_VERSION))

return DAAloiter
