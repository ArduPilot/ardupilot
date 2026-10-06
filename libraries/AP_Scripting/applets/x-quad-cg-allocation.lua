-- x-quad-cg-allocation.lua: Adjust the control allocation matrix for offset CoG.
--
-- WARNING: This script is applicable only to X-type quadrotors and quadplanes.
--
-- How To Use
--   1. Place this script in the "scripts" directory.
--   2. Set FRAME_CLASS or Q_FRAME_CLASS to 17 to enable the dynamic scriptable mixer.
--   3. Enable Lua scripting via the SCR_ENABLE parameter.
--   4. Reboot.
--   5. Fly the vehicle.
--   6. Adjust the value of the CGA_RATIO parameter.
--
-- How It Works
--   1. The control allocation matrix is adjusted for thrust and pitch based on the ??? parameter value.

--[[
Global definitions.
--]]
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local SCRIPT_NAME = "CoG adjust script"
local LOOP_RATE_HZ = 10
local last_warning_time_ms = uint32_t() -- Time we last sent a warning message to the user.
local WARNING_DEADTIME_MS = 1000 -- How often the user should be warned.
local is_mixer_matrix_static = false
local is_mixer_matrix_dynamic = false
local last_ratio = 1

-- State machine states.
local FSM_STATE = {
    INACTIVE = 0,
    INITIALIZE = 1,
    ACTIVE = 2,
    FINISHED = 3
}
local current_state = FSM_STATE.INACTIVE
local next_state = FSM_STATE.INACTIVE


--[[
New parameter declarations
--]]
local PARAM_TABLE_KEY = 139
local PARAM_TABLE_PREFIX = "CGA_"

-- Bind a parameter to a variable.
function bind_param(name)
    return Parameter(name)
end

-- Add a parameter and bind it to a variable.
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('Could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Add param table.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), SCRIPT_NAME .. ': Could not add param table.')

--[[
  // @Param: CGA_RATIO
  // @DisplayName: CoG adjustment ratio
  // @Description: The ratio between the front and back motor outputs during steady-state hover. Positive when the CoG is in front of the motors midpoint (front motors work harder).
  // @Range: 0.5 2
  // @User: Advanced
--]]
CGA_RATIO = bind_add_param('RATIO', 1, 1)

-- Bindings to existing parameters

--[[
Potential additions:
--]]
-- Warn the user, throttling the message rate.
function warn_user(msg, severity)
    severity = severity or MAV_SEVERITY.WARNING -- Optional severity argument.
    if millis() - last_warning_time_ms > WARNING_DEADTIME_MS then
            gcs:send_text(severity, SCRIPT_NAME .. ": " .. msg)
        last_warning_time_ms = millis()
    end
end

-- Decide if the given ratio value makes sense.
function sanitize_ratio(ratio)
    if (ratio < 0.5) or (ratio > 2) then
        warn_user("CGA_RATIO value out of bounds.", MAV_SEVERITY.ERROR)
        CGA_RATIO:set(1.0)
        return 1.0 -- Return default.
    else
        return ratio
    end
end

-- Normalize the throttle factors to max 1
function normalize_throttle(factors)
    -- Find maximum value.
    local max_factor = 0
    for _, factor in ipairs(factors) do
        max_factor = math.max(max_factor, factor)
    end
    -- Adjust all values by it.
    normalized_factors = {}
    for _, factor in ipairs(factors) do
        table.insert(normalized_factors, factor / max_factor)
    end
    return normalized_factors
end

-- Calculate the thrust factors given a ratio.
function build_factors(ratio)
    local r1 = 2.0/(1+ratio)
    local r2 = 2.0*ratio/(1+ratio)
    local quad_x_factors = {r2, r1, r2, r1}
    return normalize_throttle(quad_x_factors)
end

-- Adjust the dynamic motor mixer.
function update_dynamic_mixer(ratio)

    Motors_dynamic:add_motor(0, 1)
    Motors_dynamic:add_motor(1, 3)
    Motors_dynamic:add_motor(2, 4)
    Motors_dynamic:add_motor(3, 2)

    factors = motor_factor_table()

    -- Roll stays as-is.
    factors:roll(0, -0.5)
    factors:roll(1,  0.5)
    factors:roll(2,  0.5)
    factors:roll(3, -0.5)

    -- Pitch stays as-is.
    factors:pitch(0, 0.5)
    factors:pitch(1, -0.5)
    factors:pitch(2, 0.5)
    factors:pitch(3, -0.5)

    -- Yaw stays as-is.
    factors:yaw(0,  0.5)
    factors:yaw(1,  0.5)
    factors:yaw(2, -0.5)
    factors:yaw(3, -0.5)

    -- Throttle is modulated.
    throttle_factors = build_factors(ratio)
    factors:throttle(0, throttle_factors[1])
    factors:throttle(1, throttle_factors[2])
    factors:throttle(2, throttle_factors[3])
    factors:throttle(3, throttle_factors[4])

    Motors_dynamic:load_factors(factors)

    if not Motors_dynamic:init(4) then
        warn_user("Failed to initialize motor matrix!", MAV_SEVERITY.EMERGENCY)
    else
        if ratio ~= last_ratio then
            warn_user("Set ratio to " .. tostring(ratio), MAV_SEVERITY.INFO)
            last_ratio = ratio
        end
    end
    motors:set_frame_string("Dynamic CoM adjust")

end

-- Adjust the static motor mixer.
function update_static_mixer(ratio)
    MotorsMatrix:add_motor_raw(0,-0.5, 0.5, 0.5, 2)
    MotorsMatrix:add_motor_raw(1, 0.5,-0.5, 0.5, 4)
    MotorsMatrix:add_motor_raw(2, 0.5, 0.5,-0.5, 1)
    MotorsMatrix:add_motor_raw(3,-0.5,-0.5,-0.5, 3)

    throttle_factors = build_factors(ratio)
    MotorsMatrix:set_throttle_factor(0, throttle_factors[1])
    MotorsMatrix:set_throttle_factor(1, throttle_factors[2])
    MotorsMatrix:set_throttle_factor(2, throttle_factors[3])
    MotorsMatrix:set_throttle_factor(3, throttle_factors[4])

    if not MotorsMatrix:init(4) then
        warn_user("Failed to initialize motor matrix!", MAV_SEVERITY.EMERGENCY)
    else
        if ratio ~= last_ratio then
            warn_user("Set ratio to " .. tostring(ratio), MAV_SEVERITY.INFO)
            last_ratio = ratio
        end
    end
    motors:set_frame_string("Static CoM adjust")
end

-- Decide if the UA is a Quad X quadplane.
function inspect_frame_class_fw()

    Q_ENABLE = bind_param("Q_ENABLE")
    Q_FRAME_CLASS = bind_param("Q_FRAME_CLASS")

    if FWVersion:type()==3 then
        -- Test for the validity of the parameters.
        if Q_ENABLE:get()==1 then
            if Q_FRAME_CLASS:get()==15 then
                is_mixer_matrix_static = true
            elseif Q_FRAME_CLASS:get()==17 then
                is_mixer_matrix_dynamic = true
            end
        end
    end
end

-- Decide if the UA is a Quad X multicopter.
function inspect_frame_class_mc()

    FRAME_CLASS = bind_param("FRAME_CLASS")

    if FWVersion:type()==2 then 
        if FRAME_CLASS:get()==15 then
            is_mixer_matrix_static = true
        elseif FRAME_CLASS:get()==17 then
            is_mixer_matrix_dynamic = true
        end
    end
end

--[[
Activation conditions
--]]
-- Check for script activating conditions here.
-- Check frame types.
function can_start()
    result = is_mixer_matrix_static or is_mixer_matrix_dynamic
    return result
end

--[[
State machine
--]]
function fsm_step()
    if current_state == FSM_STATE.INACTIVE then
        if can_start() then
            next_state = FSM_STATE.INITIALIZE
        else
            next_state = FSM_STATE.FINISHED
            warn_user("Could not find scriptable mixer", MAV_SEVERITY.ERROR)
        end

    elseif current_state == FSM_STATE.INITIALIZE then
        if is_mixer_matrix_static then
            local ratio = sanitize_ratio(CGA_RATIO:get())
            update_static_mixer(ratio)
            next_state = FSM_STATE.FINISHED
        else
            next_state = FSM_STATE.ACTIVE
        end

    elseif current_state == FSM_STATE.ACTIVE then
        -- Assert the parameter limits.
        local ratio = sanitize_ratio(CGA_RATIO:get())
        -- Create the control allocation matrix parameters.
        update_dynamic_mixer(ratio)
        
    else
        gcs:send_text(MAV_SEVERITY.CRITICAL, "Unexpected FSM state!")
    end

    current_state = next_state
end

-- Check once on boot if the frame type is suitable for this script.
pcall(inspect_frame_class_mc)
pcall(inspect_frame_class_fw)
gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. string.format(" loaded."))

-- Wrapper around update() to catch errors.
function protected_wrapper()
  local success, err = pcall(fsm_step)
  if not success then
    gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
    return protected_wrapper, 1000
  end
  if current_state ~= FSM_STATE.FINISHED then
    return protected_wrapper, 1000.0/LOOP_RATE_HZ
  end
end

-- Start running update loop
return protected_wrapper()
