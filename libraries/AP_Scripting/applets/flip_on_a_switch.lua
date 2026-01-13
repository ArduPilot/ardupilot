--[[
  flip_on_switch.lua: An applet to perform flips based on RC switch input.

  This script uses the vehicle_control module to execute a flip maneuver.
  It supports two modes via the FLIP_CHAN parameter:
  1. Simple Mode (FLIP_CHAN = 0):
     - Uses the RCx_OPTION switch (func 300).
     - When the switch is high, performs continuous flips of a fixed number of rotations.
     - When the switch is low, the maneuver stops.
  2. Advanced Mode (FLIP_CHAN > 0):
     - Reads a raw RC channel directly to bypass the 200ms debounce.
     - A flick or hold sequence (which ends with the switch low) defines the maneuver.
     - The maneuver starts automatically after a short timeout.
     - A subsequent flick cancels the active maneuver.
]]

local vehicle_control = require('vehicle_control')

-- Enum for MAV_SEVERITY levels
local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4,
    NOTICE = 5, INFO = 6, DEBUG = 7
}

-- Parameters
local PARAM_TABLE_KEY = 107
local PARAM_TABLE_PREFIX = "FLIP_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'flip_on_switch: could not add param table')

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: FLIP_ENABLE
  // @DisplayName: Enable Flip on Switch
  // @Description: Enables or disables the flip on switch functionality.
  // @User: Standard
  // @Values: 0:Disabled,1:Enabled
--]]
local FLIP_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: FLIP_AXIS
  // @DisplayName: Flip Axis
  // @Description: The axis for the flip maneuver.
  // @User: Standard
  // @Values: 1:Roll,2:Pitch
--]]
local FLIP_AXIS = bind_add_param('AXIS', 2, vehicle_control.axis.ROLL)

--[[
  // @Param: FLIP_RATE
  // @DisplayName: Flip Rate
  // @Description: The target rotation rate for the flip maneuver.
  // @User: Standard
  // @Units: deg/s
--]]
local FLIP_RATE = bind_add_param('RATE', 3, 720)

--[[
  // @Param: FLIP_THROTTLE
  // @DisplayName: Flip Throttle
  // @Description: The throttle level to use during the flip's ballistic phase (0-1). A value of -1 will cut the throttle entirely.
  // @User: Advanced
  // @Range: -1 1
--]]
local FLIP_THROTTLE = bind_add_param('THROTTLE', 4, 0.0)

--[[
  // @Param: FLIP_HOVER
  // @DisplayName: Hover Throttle
  // @Description: The vehicle's true hover throttle (0-1). This is critical for the script's physics calculations.
  // @User: Advanced
  // @Range: 0 1
--]]
local FLIP_HOVER = bind_add_param('HOVER', 5, 0.125)

--[[
  // @Param: FLIP_NUM
  // @DisplayName: Simple Mode Number of Flips
  // @Description: (Simple Mode Only) The number of flips to perform in a continuous sequence when the switch is held high.
  // @User: Standard
--]]
local FLIP_NUM = bind_add_param('NUM', 6, 1)

--[[
  // @Param: FLIP_CHAN
  // @DisplayName: Flip Control Channel
  // @Description: Selects the control mode. 0 for Simple Mode using the RCx_OPTION. >0 for Advanced Mode using the specified raw RC channel.
  // @User: Standard
  // @Range: 0 16
--]]
local FLIP_CHAN = bind_add_param('CHAN', 7, 0)

--[[
  // @Param: FLIP_FLICK_TO
  // @DisplayName: Advanced Mode Flick Timeout
  // @Description: (Advanced Mode Only) The time in seconds to differentiate a 'flick' from a 'hold'. A switch activation shorter than this is a flick.
  // @User: Advanced
  // @Units: s
--]]
local FLIP_FLICK_TO = bind_add_param('FLICK_TO', 8, 0.5)

--[[
  // @Param: FLIP_COMMIT_TO
  // @DisplayName: Advanced Mode Commit Timeout
  // @Description: (Advanced Mode Only) The timeout in seconds after the last switch input before the maneuver is committed and starts.
  // @User: Advanced
  // @Units: s
--]]
local FLIP_COMMIT_TO = bind_add_param('COMMIT_TO', 9, 0.75)

--[[
  // @Param: FLIP_CLIMB_G
  // @DisplayName: Climb G-Force
  // @Description: The desired G-force for the initial climb (e.g., 1.0 for 1g). Higher values result in a more aggressive climb.
  // @User: Standard
  // @Units: g
--]]
local FLIP_CLIMB_G = bind_add_param('CLIMB_G', 10, 1.0)

-- RC Function Constant for Simple Mode
local SCRIPTING_AUX_FUNC = 300

-- State variables
local flip_state = nil
local flip_active = false
local original_mode = nil
local abort_cooldown_end_time = 0

-- State for advanced mode switch input
local switch_logic_state = { IDLE = 0, TIMING_HIGH = 1, WAITING_FOR_COMMIT = 2 }
local current_switch_state = switch_logic_state.IDLE
local high_start_time = 0
local flick_count = 0
local last_flick_time = 0
local determined_num_flips = nil
local determined_duration_s = nil

-- State for custom debouncing
local DEBOUNCE_MS = 20
local debounced_state = 0
local last_potential_state_time = 0
local potential_state = 0
local last_debounced_state = 0

-- Precondition checks
assert(FWVersion:type() == 2, 'Script requires a Copter frame')

-- Resets all state machines to a clean state
function reset_all_states(is_abort)
    flip_active = false
    flip_state = nil
    current_switch_state = switch_logic_state.IDLE
    flick_count = 0
    determined_num_flips = nil
    determined_duration_s = nil

    if is_abort then
        abort_cooldown_end_time = millis():tofloat() + 3000 -- 3 second cooldown
        gcs:send_text(MAV_SEVERITY.WARNING, "Maneuver aborted, cooldown active.")
    end

    if original_mode then
        vehicle:set_target_rate_and_throttle(0, 0, 0, 0.5)
        if vehicle:get_mode() ~= original_mode then
            vehicle:set_mode(original_mode)
        end
        original_mode = nil
    end
end

-- Function to start the flip maneuver once parameters are determined
function start_maneuver()
    if not arming:is_armed() or not vehicle:get_likely_flying() then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Vehicle must be armed and flying")
        reset_all_states(false)
        return
    end

    -- Capture the original mode only on the first run
    if not original_mode then
        original_mode = vehicle:get_mode()
    end

    if not (original_mode == vehicle_control.mode.LOITER or original_mode == vehicle_control.mode.GUIDED) then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Must be in Loiter or Guided mode to start")
        reset_all_states(false)
        return
    end

    if vehicle:get_mode() ~= vehicle_control.mode.GUIDED then
        if not vehicle:set_mode(vehicle_control.mode.GUIDED) then
            gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to set Guided mode")
            reset_all_states(false)
            return
        end
    end

    local num_flips_arg = determined_num_flips
    local duration_s_arg = determined_duration_s
    
    if num_flips_arg then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Flip: Starting %d flips", num_flips_arg))
    elseif duration_s_arg then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Flip: Starting %.2fs flip", duration_s_arg))
    end

    flip_active = true
    flip_state = vehicle_control.maneuver.flip_start(
        FLIP_AXIS:get(), FLIP_RATE:get(), FLIP_THROTTLE:get(),
        duration_s_arg, num_flips_arg, FLIP_HOVER:get(),
        nil, FLIP_CLIMB_G:get()
    )
    if flip_state == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, "Flip: Failed to start flip")
        reset_all_states(false)
    end
end

-- Handles simple high/low switch logic
function update_simple_mode()
    local switch_pos = rc:get_aux_cached(SCRIPTING_AUX_FUNC)

    if flip_active then
        if switch_pos ~= 2 then
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Canceled by user")
            reset_all_states(false)
        else
            local status = vehicle_control.maneuver.flip_update(flip_state, reset_all_states)
            if status == vehicle_control.SUCCESS then
                start_maneuver() -- Repeat
            elseif status == vehicle_control.ABORTED then
                reset_all_states(true)
            end
        end
    elseif switch_pos == 2 then
        gcs:send_text(MAV_SEVERITY.INFO, "Flip: Starting continuous flip")
        determined_num_flips = FLIP_NUM:get()
        determined_duration_s = nil
        start_maneuver()
    end
end

-- Handles advanced logic with custom debouncing
function update_advanced_mode(channel_num)
    local pwm = rc:get_pwm(channel_num) or 1500 -- Use neutral PWM as a safe default
    local now = millis():tofloat()
    
    -- Custom debouncer
    local current_raw_state = (pwm > 1800) and 2 or 0
    if current_raw_state ~= debounced_state then
        if current_raw_state ~= potential_state then
            potential_state = current_raw_state
            last_potential_state_time = now
        elseif (now - last_potential_state_time) > DEBOUNCE_MS then
            debounced_state = potential_state
        end
    else
        last_potential_state_time = 0
    end

    local rising_edge = (debounced_state == 2 and last_debounced_state ~= 2)
    local falling_edge = (debounced_state ~= 2 and last_debounced_state == 2)

    if flip_active then
        if falling_edge and (now - high_start_time) < (FLIP_FLICK_TO:get() * 1000) then
            gcs:send_text(MAV_SEVERITY.INFO, "Flip: Canceled by subsequent flick")
            reset_all_states(false)
        else
            local status = vehicle_control.maneuver.flip_update(flip_state, reset_all_states)
            if status == vehicle_control.SUCCESS or status == vehicle_control.ABORTED then
                reset_all_states(status == vehicle_control.ABORTED)
            end
        end
    else
        -- State machine to determine maneuver
        if current_switch_state == switch_logic_state.IDLE then
            if rising_edge then
                current_switch_state = switch_logic_state.TIMING_HIGH
                flick_count = 1
                high_start_time = now
            end
        elseif current_switch_state == switch_logic_state.TIMING_HIGH then
            if falling_edge then
                local high_duration_s = (now - high_start_time) / 1000.0
                if high_duration_s >= FLIP_FLICK_TO:get() then
                    determined_duration_s = high_duration_s
                    determined_num_flips = nil
                else
                    determined_num_flips = flick_count
                    determined_duration_s = nil -- Let the library calculate duration
                end
                current_switch_state = switch_logic_state.WAITING_FOR_COMMIT
                last_flick_time = now
            end
        elseif current_switch_state == switch_logic_state.WAITING_FOR_COMMIT then
            if rising_edge then
                flick_count = flick_count + 1
                current_switch_state = switch_logic_state.TIMING_HIGH
                high_start_time = now
            elseif (now - last_flick_time) / 1000.0 > FLIP_COMMIT_TO:get() then
                start_maneuver()
            end
        end
    end
    last_debounced_state = debounced_state
end

-- Main update function
function update()
    if FLIP_ENABLE:get() == 0 then
        return update, 1000
    end

    local now = millis():tofloat()
    if now < abort_cooldown_end_time then
        return update, 200
    end

    local spring_chan = FLIP_CHAN:get()
    if spring_chan > 0 then
        update_advanced_mode(spring_chan)
    else
        update_simple_mode()
    end

    return update, 10 -- Run at 100Hz for responsiveness
end

-- Protected wrapper for error handling
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Flip Error: " .. err)
        reset_all_states(false)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "Flip on switch script loaded (Advanced)")
return protected_wrapper, 1000
