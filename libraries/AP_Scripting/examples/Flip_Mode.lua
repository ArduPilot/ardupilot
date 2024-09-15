-- This script replicates the behavior of flip mode
local MODE_NUMBER = 100

-- Register flip as mode 100
assert(vehicle:register_custom_mode(MODE_NUMBER, "Flip", "FLIP"))

-- Get input channels
local THROTTLE_CHAN = math.floor(assert(param:get("RCMAP_THROTTLE")))
local pilot_throttle = assert(rc:get_channel(THROTTLE_CHAN))
local pilot_pitch = assert(rc:get_channel(assert(param:get("RCMAP_PITCH"))))
local pilot_roll = assert(rc:get_channel(assert(param:get("RCMAP_ROLL"))))

local HOVER_THROTTLE = Parameter("MOT_THST_HOVER")
local THROTTLE_MIN = Parameter("RC" .. THROTTLE_CHAN .. "_MIN")
local THROTTLE_MAX = Parameter("RC" .. THROTTLE_CHAN .. "_MAX")
local THROTTLE_DZ = Parameter("RC" .. THROTTLE_CHAN .. "_DZ")

-- Replication of the copter function
local function get_throttle_mid()
    local r_in = (THROTTLE_MIN:get() + THROTTLE_MAX:get()) * 0.5

    local radio_trim_low = THROTTLE_MIN:get() + THROTTLE_DZ:get()

    return 1000.0 * ((r_in - radio_trim_low) / (THROTTLE_MAX:get() - radio_trim_low))
end

-- Replication of the copter function
local function get_pilot_desired_throttle()

    local thr_mid = HOVER_THROTTLE:get()
    local throttle_control = (pilot_throttle:norm_input_ignore_trim() + 1.0) * (1000.0 / 2.0)

    local mid_stick = get_throttle_mid()

    local throttle_in
    if throttle_control < mid_stick then
        throttle_in = throttle_control * 0.5 / mid_stick

    else
        throttle_in = 0.5 + ((throttle_control - mid_stick) * 0.5 / (1000.0 - mid_stick))

    end

    local expo = -(thr_mid - 0.5) / 0.375
    expo = math.max(expo, -0.5)
    expo = math.min(expo,  1.0)

    return throttle_in * (1.0-expo) + expo*throttle_in*throttle_in*throttle_in
end

-- Used to check for mode changes
local last_mode_number

local MODES = {
    STABILIZE = 0,
    ALT_HOLD = 2,
}

-- Check if we should be allowed to enter flip mode
local function allow_enter(previous_mode)

    -- Only allow flip from stabilize and alt hold
    if (previous_mode ~= MODES.STABILIZE) and (previous_mode ~= MODES.ALT_HOLD) then
        return false
    end

    -- Must be armed and flying
    if (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        return false
    end

    return true
end

local FLIP_STATE = {
    START = 0,
    ROLL = 1,
    PITCH_A = 2,
    PITCH_B = 3,
    RECOVER = 4,
}
local state
local start_ms
local roll_dir
local pitch_dir
local start_attitude = {
    roll = 0,
    pitch = 0,
    yaw = 0,
}

-- Init on first call
local previous_mode
local function init()
    -- Record the previous mode, this is returned to on completion of the flip
    previous_mode = last_mode_number

    -- Return to previous mode immediately if flip cannot be performed
    if not allow_enter(previous_mode) then
        vehicle:set_mode(previous_mode)
        return
    end

    state = FLIP_STATE.START
    start_ms = millis()

    roll_dir = 0.0
    pitch_dir = 0.0

    -- choose direction based on pilot's roll and pitch sticks
    if pilot_pitch:norm_input_dz() > 0.1 then
        pitch_dir = 1.0
    elseif pilot_pitch:norm_input_dz() < -0.1 then
        pitch_dir = -1.0
    elseif pilot_roll:norm_input_dz() >= 0 then
        roll_dir = 1.0
    else
        roll_dir = -1.0
    end

    -- Record start attitude to be used in recovery stage
    start_attitude.roll = math.deg(ahrs:get_roll())
    start_attitude.pitch = math.deg(ahrs:get_pitch())
    start_attitude.yaw = math.deg(ahrs:get_yaw())

end

local FLIP_THR_INC = 0.2
local FLIP_THR_DEC = 0.24
local FLIP_ROTATION_RATE = 400

local function run()
    local NOW = millis()

    -- Disarmed, pilot input or timeout then return to previous mode
    local PILOT_INPUT = (math.abs(pilot_roll:norm_input_dz()) > 0.85) or (math.abs(pilot_pitch:norm_input_dz()) > 0.85)
    if (not arming:is_armed()) or PILOT_INPUT or ((NOW - start_ms) > 2500) then
        vehicle:set_mode(previous_mode)
        return
    end

    local roll_deg = math.deg(ahrs:get_roll())
    local pitch_deg = math.deg(ahrs:get_pitch())

    if state == FLIP_STATE.RECOVER then
        -- Target original attitude with 0 climb rate
        vehicle:set_target_angle_and_climbrate(start_attitude.roll, start_attitude.pitch, start_attitude.yaw, 0.0, false, 0.0)

        -- See if we have returned to the desired angle
        local recovery_angle = math.abs((roll_deg - start_attitude.roll) * roll_dir) + math.abs((pitch_deg - start_attitude.pitch) * pitch_dir)

        if recovery_angle < 5.0 then
            -- Complete, return to original mode
            vehicle:set_mode(previous_mode)
        end
        return
    end

    local throttle_out = get_pilot_desired_throttle()

    local flip_angle = roll_deg * roll_dir + pitch_deg * pitch_dir

    if state == FLIP_STATE.START then
        -- Increase throttle
        throttle_out = math.min(throttle_out + FLIP_THR_INC, 1.0)

        -- Check for next stage
        if flip_angle >= 45.0 then
            if roll_dir ~= 0 then
                -- Rolling flip
                state = FLIP_STATE.ROLL
            else
                -- Pitching flip
                state = FLIP_STATE.PITCH_A
            end
        end

    elseif state == FLIP_STATE.ROLL then
        -- decrease throttle
        throttle_out = math.max(throttle_out - FLIP_THR_DEC, 0.0)

        -- beyond 90deg move on to recovery
        if (flip_angle < 45.0) and (flip_angle > -90.0) then
            state = FLIP_STATE.RECOVER
        end

    elseif state == FLIP_STATE.PITCH_A then
        -- decrease throttle
        throttle_out = math.max(throttle_out - FLIP_THR_DEC, 0.0)

        -- check roll for inversion
        if (math.abs(roll_deg) > 90.0) and (flip_angle > 45.0) then
            state = FLIP_STATE.PITCH_B
        end

    elseif state == FLIP_STATE.PITCH_B then
        -- decrease throttle
        throttle_out = math.max(throttle_out - FLIP_THR_DEC, 0.0)

        -- check roll for un-inversion
        if (math.abs(roll_deg) < 90.0) and (flip_angle > -45.0) then
            state = FLIP_STATE.RECOVER
        end

    end

    -- Send rate and throttle command to vehicle
    local roll_rate = FLIP_ROTATION_RATE * roll_dir
    local pitch_rate = FLIP_ROTATION_RATE * pitch_dir

    vehicle:set_target_rate_and_throttle(roll_rate, pitch_rate, 0.0, throttle_out)

end

local function exit()
    -- Nothing to do here
end

local function update()
    local mode = vehicle:get_mode()
    if mode == MODE_NUMBER then
        if last_mode_number ~= MODE_NUMBER then
            -- Fist call after entering
            init()
        else
            -- Runtime function
            run()
        end

    elseif last_mode_number == MODE_NUMBER then
        -- Exit mode
        exit()
    end
    last_mode_number = mode

    -- Run at 100Hz
    return update, 10
end

return update()
