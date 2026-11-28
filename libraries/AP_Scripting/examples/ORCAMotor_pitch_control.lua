-- This script allows testing motor functionality by pitching the flight controller up or down to move the motor back and forth
-- Make sure to set the correct position limits!

local ORCA = require("ORCAMotor")

local PORT_NUM = 0    
local INSTANCE_NUM = 1

-- Target Position Limits (in micrometers)
local MIN_POSITION_UM = 0
local MAX_POSITION_UM = 200000

-- Control Tuning
local PITCH_DEADZONE_DEG = 3     -- Ignore pitch changes smaller than this
local STEP_SIZE_UM = 3000          -- Position increment/decrement per loop step

-- Initialize Motor Instance
local motor = ORCA.new(PORT_NUM, INSTANCE_NUM)

motor:clear_motor_errors()
motor:load_params()
motor:set_auto_zero_timeout(10000)
motor:set_target_mode(ORCA.MODE.AUTO_ZERO)

function update()
    -- 1. Ensure driver loop updates continuously
    motor:update()

    -- 2. Fetch vehicle pitch angle from AHRS (returned in radians, convert to degrees)
    local pitch_deg = math.deg(ahrs:get_pitch_rad())

    -- 3. Retrieve current measured motor position from state table
    local target_position_um = motor.target_position

    -- 4. Calculate new target based on vehicle pitch angle
    --    Nose down (pitch < 0): Increase target position
    --    Nose up (pitch > 0): Decrease target position
    local new_target_um = target_position_um

    if pitch_deg < -PITCH_DEADZONE_DEG then
        new_target_um = target_position_um + STEP_SIZE_UM
    elseif pitch_deg > PITCH_DEADZONE_DEG then
        new_target_um = target_position_um - STEP_SIZE_UM
    end

    -- 5. Apply limits (Min: 0 um, Max: MAX_POSITION_UM)
    new_target_um = math.max(MIN_POSITION_UM, math.min(MAX_POSITION_UM, new_target_um))

    -- 6. Dispatch command if target position changes
    if new_target_um ~= target_position_um then
        motor:set_target_position_um(new_target_um)
    end

    return update, 50
end

return update()