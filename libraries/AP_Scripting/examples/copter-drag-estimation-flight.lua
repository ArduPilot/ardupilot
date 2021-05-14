-- fly a Copter in specific patterns to estimate bluff body and momentum drag
-- see https://youtu.be/xVVtvVuZGQE?t=1423

-- Vehicle should be flown up, up, up into air which is likely to be
--   consistent for the duration of the test, then this script enabled

-- constants
local update_interval_ms = 10    -- script updates at 100hz
local aux_switch_function = 300  -- auxiliary switch function controlling mode.  300 is Scripting 1
local copter_guided_mode_num = 4 -- Copter's guided flight mode is mode 4
local drift_accel_max = 0.1      -- accel must be below this to be considered at max vel
local drive_accel_max = 0.1      -- accel must be below this to be considered at max vel
local min_drive_time_ms = 5000   -- minimum time to drive into wind
local min_drift_time_ms = 5000   -- minimum time to drift with wind while measuring
local state_emit_interval_ms = 1000  -- rate wat which to emit state text messages
local windspeed_min = 1 -- minimum windspeed this script is expected to be run in
local minalt = 20  -- minimum altitude (in metres) for the script to run


-- global state:
local phase = nil
local yaw_being_measured = 0
local started_return = 0

local last_state_emitted_ms = 0

local state = "unknown"
local abort_reason = nil

-- measure_drift state
local phase_initial_drift_start = nil

function reset(string)
    phase_initial_drift_start = nil
    state = string
    phase = nil
    start_pos = nil
    yaw_being_measured = 0
    started_return = 0
end

function progress(string)
    if string == nil then
       string = "nil"
    end
    gcs:send_text(0, string.format("Drag: %s: %s", tostring(millis()), string))
end

-- returns distance in metres to pos (which is a Vector3)
function distance_to(pos)
    local here = ahrs:get_relative_position_NED_home()
    local delta = pos - here
    return delta:length()
end

function accel_xy()
    -- return accel in earth-frame-xy; currently assumes z-accel is just gravity
    return ahrs:get_accel():length() - 9.81
end

function handler_initial_drift()

    if start_pos == nil then
        start_pos = ahrs:get_relative_position_NED_home()
        progress(string.format("start_pos x=%0.2f y=%0.2f", start_pos:x(), start_pos:y()))
    end

    -- instruct position controller to keep us level and at height:
    -- roll, pitch, yaw, climb_rate, use_yaw_rate, yaw_rate
    vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, 0, 0)

    -- spend at least <some> seconds in this bit
    local now = millis()
    if phase_initial_drift_start == nil then
        phase_initial_drift_start = now
    end
    local elapsed = now - phase_initial_drift_start
    if elapsed < 5000 then
        state = string.format("initial drift") -- FIXME: show seconds remaining
        return update, update_interval_ms
    end

    -- if our acceleration (presumably from wind...) is above 0.2m/s then
    --        we're not done yet
    if accel_xy() > drift_accel_max then
        state = string.format("init drift: wait accel want<%0.2fm/s got=%0.2fm/s", drift_accel_max, accel_xy())
        return update, update_interval_ms
    end

    -- ensure our velocity (presumably from wind...) is greater than some m/s:
    got_groundspeed = ahrs:groundspeed_vector():length()
    progress(string.format("groundspeed: %gm/s", got_groundspeed))
    if got_groundspeed < windspeed_min then
        state = string.format("initial drift - wind insufficient, got=%0.2fm/s want >%0.2fm/s",
                          got_groundspeed, windspeed_min)
        phase = "aborted"
        return update, update_interval_ms
    end

    wind_estimate_end_pos = ahrs:get_relative_position_NED_home()
    progress(string.format("end_pos x=%0.2f y=%0.2f", wind_estimate_end_pos:x(), wind_estimate_end_pos:y()))
    local delta = start_pos - wind_estimate_end_pos
    progress(string.format("metres travelled to measure wind: %0.2f", delta:length()))
--    progress(string.format("x=%0.2f y=%0.2f", delta:x(), delta:y()))
    wind_angle = math.deg(math.atan(delta:y(), delta:x()))
    progress(string.format("wind coming from heading %0.2f deg", wind_angle))

    phase = "measure_drag"
    yaw_being_measured = 0
    measure_drag_phase = "measure_drag_start_phase"

    return update, update_interval_ms

end -- handler_initial_drift

-- measures drag at this yaw by driving into wind until we stop
--   accelerating then drifting back to where we started

function handler_measure_drag()
    local extra = nil
    if measure_drag_phase == "measure_drag_start_phase" then
        progress(string.format("Measuring drag at yaw=%0.2f", yaw_being_measured))
        measure_drag_phase = "start_yaw"
    elseif measure_drag_phase == "start_yaw" then
        measure_drag_phase = "wait_heading"
    elseif measure_drag_phase == "wait_heading" then
        local target_roll = 0
        local target_pitch = 0
        local target_yaw = (yaw_being_measured + wind_angle) % 360
        local climb_rate = 0
        local use_yaw_rate = 1
        local yaw_rate = 20  -- degrees/second
        local current_yaw = math.deg(ahrs:get_yaw())  -- -180..180
        if current_yaw < 0 then
            current_yaw = 360 + current_yaw
        end
        local yaw_error = (180-(current_yaw - target_yaw)) % 180
        if yaw_error < 0 then
            yaw_rate = -yaw_rate
        end
        -- fixme: yaw in shorter direction
        vehicle:set_target_angle_and_climbrate(target_roll, target_pitch, target_yaw, climb_rate, use_yaw_rate, yaw_rate)
        
        extra = string.format("heading want=%0.2f got=%0.2f", target_yaw, current_yaw)
        if math.abs(yaw_error) < 2 then
            progress(string.format("got heading %0.2f; moving to start", target_yaw))
            measure_drag_phase = "move_to_start"
        end
    elseif measure_drag_phase == "move_to_start" then
        vehicle:set_target_posvel_NED(wind_estimate_end_pos, Vector3f(0, 0, 0))
        measure_drag_phase = "move_to_start_move"
    elseif measure_drag_phase == "move_to_start_move" then
        local rem = distance_to(wind_estimate_end_pos)
        extra = string.format("dist=%0.2f", rem)
        if rem < 4.0 then
            measure_drag_phase = "drive_into_wind"
            drive_into_wind_timer_start = millis()
            extra = "done"
        end
    elseif measure_drag_phase == "drive_into_wind" then
        -- FIXME: trig for this!
        local circular_angle = 30
        local target_roll = 0
        local target_pitch = 0
        if yaw_being_measured == 0 then
            target_roll = 0
            target_pitch = -circular_angle  -- FIXME
        elseif yaw_being_measured == 90 then
            target_roll = -circular_angle
            target_pitch = 0    -- FIXME
        elseif yaw_being_measured == 180 then
            target_roll = 0
            target_pitch = circular_angle    -- FIXME
        else
            target_roll = circular_angle
            target_pitch = 0
        end

        local target_yaw = (yaw_being_measured + wind_angle) % 360
        local climb_rate = 0
        local use_yaw_rate = 0
        local yaw_rate = 0  -- degrees/second
        vehicle:set_target_angle_and_climbrate(target_roll, target_pitch, target_yaw, climb_rate, use_yaw_rate, yaw_rate)
        -- check acceleration to see if we're done
        local elapsed = millis() - drive_into_wind_timer_start
        extra = string.format("accel=%0.2f want<%0.2f el=%s", accel_xy(), drive_accel_max, tostring(elapsed))
        if elapsed > min_drive_time_ms and accel_xy() < drive_accel_max then
            progress("Moving to drift_with_wind")
            measure_drag_phase = "drift_with_wind"
            drift_with_wind_timer_start = millis()
        end        
        -- TODO: check distance to see if we've gone too far
    elseif measure_drag_phase == "drift_with_wind" then
        -- command the vehicle to stay level
        local target_roll = 0
        local target_pitch = 0
        local target_yaw = 0
        local climb_rate = 0
        local use_yaw_rate = 0
        local yaw_rate = 0  -- degrees/second
        vehicle:set_target_angle_and_climbrate(target_roll,target_pitch,target_yaw,climb_rate,use_yaw_rate,yaw_rate)

        elapsed = millis() - drift_with_wind_timer_start

        extra = string.format("accel_xy=%0.2f want<%0.2f el=%s", accel_xy(), drift_accel_max, tostring(elapsed))
        -- see if we've completed the drift
        if elapsed > min_drive_time_ms and accel_xy() < drift_accel_max then
            progress("drift done")
            measure_drag_phase = "end"
        end
    elseif measure_drag_phase == "end" then
        yaw_being_measured = yaw_being_measured + 90
        if yaw_being_measured >= 360 then
            -- all measurements have been done
            phase = "return_to_start_pos"
        else
            measure_drag_phase = "measure_drag_start_phase"
        end
    else
        abort_reason = string.format("unknown measure_drag_phase (%s)", measure_drag_phase)
        phase = "aborted"
    end

    if extra == nil then
        state = measure_drag_phase
    else
        state = string.format("%s: %s", measure_drag_phase, extra)
    end

end

-- just loops idly doing nothing - user will need to take control
function handler_aborted()
    progress(abort_reason)
    return update, update_interval_ms
end

function handler_return_to_start_pos()
    if started_return == 0 then
        vehicle:set_target_posvel_NED(start_pos, Vector3f(0, 0, 0))
        started_return = 1
    end
    local rem = distance_to(start_pos)
    state = string.format("dist=%0.2f", rem)
    if rem < 1.0 then
        phase = "done"
    end
    return update, update_interval_ms
end

function handler_done()
    return update, update_interval_ms
end

-- maps from a phase to a handler
local phase_func_table = {
    initial_drift = handler_initial_drift,
    measure_drag = handler_measure_drag,
    aborted = handler_aborted,
    return_to_start_pos = handler_return_to_start_pos,
    done = handler_done
}



-- main update function; responsible for determining phase of testing procedure
function update()
    --    progress("update")
 
    local now = millis()
    if (now - last_state_emitted_ms > state_emit_interval_ms) then
        progress(state)
        last_state_emitted_ms = now
    end

    -- if no aux switch alocation then do nothing
    local aux_switch = rc:find_channel_for_option(aux_switch_function)
    if not aux_switch then
        reset("phase_want_switch_allocation")
        return update, update_interval_ms
    end

    -- make sure we're well above home (bit of an assumption...)
    local relpos_home = ahrs:get_relative_position_NED_home()
    if relpos_home == nil then
        reset("want home")
        return update, update_interval_ms
    end
    relhome_alt = -relpos_home:z()
    if relhome_alt < minalt then
        reset(string.format("phase_want_altitude >%0.2fm now=%0.2fm", minalt, relhome_alt))
        return update, update_interval_ms
    end

    -- if aux switch not asserted then do nothing
    local sw_pos = aux_switch:get_aux_switch_pos()
    if sw_pos ~= 2 then
        reset(string.format("want_switch_position (got=%s want=%u)", swp_pos, 2))
        return update, update_interval_ms
    end

    -- if not in Guided mode do nothing
    if vehicle:get_mode() ~= copter_guided_mode_num then
        reset(string.format("want_guided have %u", vehicle:get_mode()))
        return update, update_interval_ms
    end

    if phase == nil then
        phase = "initial_drift"
    end

    state = string.format("running: phase=%s", phase)

    local func = phase_func_table[phase]
    if func == nil then
        phase = string.format("error-no-function (%s)", phase)
        return update, 5000
    end

    func()

    return update, update_interval_ms
end

return update()
