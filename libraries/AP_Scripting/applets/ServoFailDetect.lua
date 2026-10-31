-- A script for detecting servo failure.
-- A simple slew limited servo model is used.
-- Error is triggered if reported value deviates from the predicted value by more
-- than a given angle for more than a given time
-- A motor number can be optionally passed which will be shutdown if the servo fails
-- Warning messages are emitted on state change; a failed side is only stopped while the other servo is healthy

-- Configuration: exactly two rows, left then right. Edit this table for your aircraft.
-- Servo and motor channels are 1-based SERVO output numbers (1 to 32), not motor numbers.
-- Use 0 for automatic discovery: servo functions 75/76 (left/right), and the two
-- motors in Q_TILT_MASK in ascending motor-number order (assumed left/right).
-- Set a motor channel to nil to report that servo's failures without stopping its motor.
-- Angles are measured feedback in degrees at SERVOx_MIN and SERVOx_MAX, respectively.
-- Calibrate each servo separately; SERVOx_REVERSED does not swap these raw PWM endpoints.
-- Restart scripting or reboot after editing. See ServoFailDetect.md for setup details.
local servo_config = {
    -- Servo   Name          Motor   Angle at MIN   Angle at MAX
    {  15,     "Left Tilt",    11,        -57.5,          57.5 },
    {  16,     "Right Tilt",   13,        -57.5,          57.5 },
}

-- Read in minimum motor PWM, motors will be overridden to this value when shutdown
local MOTOR_SHUT_DOWN_PWM = Parameter("Q_M_PWM_MIN")

-- If telem has not been seen for this amount of time then the servo is not connected
local TIMEOUT_MS = uint32_t(1000)

-- Expected rate of change PWM us per second
local SLEW_RATE_US = 500

-- Allowable error as a percentage of the calibrated range
local ERROR_PERCENT = 14

-- Must have a error above the given angle threshold for more than this time for the servo to be marked stuck
local ERROR_MS = uint32_t(2000)

local PERIOD_MS = 20
local MAX_GAP_MS = 100
local OVERRIDE_MS = MAX_GAP_MS + PERIOD_MS

-- Servo telemetry used one-based output channel numbers before ArduPilot 4.7.
local servo_telem_one_indexed = (FWVersion:major() < 4) or
                                ((FWVersion:major() == 4) and (FWVersion:minor() < 7))

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

-- helper function to constrain value between min and max
local function constrain(val, min_lim, max_lim)
    val = math.min(val, max_lim)
    val = math.max(val, min_lim)
    return val
end

local function finite(value)
    return value ~= nil and value == value and math.abs(value) < math.huge
end

-- Resolve a one-based output channel, rejecting ambiguous automatic mappings.
local function output_channel(value, fn)
    assert(value >= 0 and value <= 32 and value == math.floor(value), "SRVFAIL: invalid output channel")
    if value ~= 0 then
        return value
    end
    local found
    for channel = 1, 32 do
        if param:get("SERVO" .. channel .. "_FUNCTION") == fn then
            assert(found == nil, "SRVFAIL: duplicate output function " .. fn)
            found = channel
        end
    end
    return assert(found, "SRVFAIL: missing output function " .. fn)
end

local function motor_function(motor)
    if motor <= 8 then
        return 32 + motor
    end
    return 73 + motor -- Motor9 through Motor12
end

---create a servo object to manage a single servo
---@param channel integer -- channel number of the servo (1 to 32)
---@param name string -- name to report as
---@param motorChannel integer|nil -- optional channel number of motor which is connected to servo (1 to 32)
---@param min_angle number -- angle feedback value that is expected at SERVOx_PWM_MIN
---@param max_angle number -- angle feedback value that is expected at SERVOx_PWM_MAX
---@return table -- servo object
local function servo(channel, name, motorChannel, min_angle, max_angle)
    assert((channel >= 1) and (channel <= 32), "Invalid servo channel")
    if motorChannel ~= nil then
        assert((motorChannel >= 1) and (motorChannel <= 32), "Invalid motor channel")
    end

    local state = {
        inactive = 0,
        active = 1,
        stuck = 2,
        disconnected = 3,
    }

    local self = {
        offset = 0,
        scale = 0,
        connected = false,
        state = state.inactive,
        activation_ms = nil,
        pwm = nil,
        pos = nil,
        stuck_ms = nil,
        max_error = nil,
        loss_ms = nil,
        last_telem_ms = nil,
    }

    -- Look up servo endpoints
    local prefix = "SERVO" .. channel .. "_"
    self.PWM_MIN = Parameter(prefix .. "MIN")
    self.PWM_MAX = Parameter(prefix .. "MAX")

    channel = channel - 1
    local telem_channel = channel
    if servo_telem_one_indexed then
        telem_channel = telem_channel + 1
    end

    local range_angle = max_angle - min_angle
    local range_pwm = self.PWM_MAX:get() - self.PWM_MIN:get()
    self.scale = range_angle / range_pwm
    self.offset = min_angle
    self.max_error = math.abs(range_angle) * ERROR_PERCENT * 0.01
    self.settle_ms = (range_pwm / SLEW_RATE_US) * 1000

    self.reset = function ()
        self.state = state.inactive
        self.activation_ms = nil
        self.pwm = nil
        self.pos = nil
        self.stuck_ms = nil
        self.loss_ms = nil
        self.last_telem_ms = nil
    end

    --- helper function for disconnection
    self.disconnected = function (now_ms)

        local failed = (self.state == state.stuck) or (self.state == state.disconnected)
        if self.connected then
            gcs:send_text(MAV_SEVERITY.ERROR, name .. " servo disconnected")
        end

        self.connected = false
        self.pwm = nil
        self.pos = nil
        self.stuck_ms = nil
        self.last_telem_ms = nil
        self.loss_ms = self.loss_ms or now_ms
        if not failed then
            self.state = state.inactive
        end
        if ((now_ms - self.loss_ms) > ERROR_MS) and
           (failed or ((now_ms - self.activation_ms):tofloat() >= self.settle_ms)) then
            self.state = state.disconnected
        end
    end

    --- Update servo
    ---@param active boolean -- true if the servo is expected to be moving
    self.update = function (active, dt_ms)

        -- Update active state
        if active == false then
            -- Not expecting servo to move
            self.reset()
            return

        elseif (self.state == state.inactive) and (self.activation_ms == nil) then
            -- Just become active
            self.activation_ms = millis()
            self.pwm = nil
        end

        local telem = servo_telem:get_telem(telem_channel)
        local now_ms = millis()
        if telem == nil then
            -- No telem available
            self.disconnected(now_ms)
            return
        end

        local last_ms = telem:last_update_ms()
        local timeout = (now_ms - last_ms) > TIMEOUT_MS
        local pos = telem:measured_position()
        if timeout or not finite(pos) then
            -- telem data not valid
            self.disconnected(now_ms)
            return
        end

        -- Valid data from servo
        if not self.connected and
           (self.state ~= state.stuck) and (self.state ~= state.disconnected) then
            self.activation_ms = now_ms
            self.state = state.inactive
        end
        self.connected = true
        self.loss_ms = nil

        -- Get the PWM for the channel
        local pwm = SRV_Channels:get_output_pwm_chan(channel)

        -- Apply slew limit to PWM
        if (pwm == nil) or (pwm <= 0) then
            self.reset()
            return
        elseif self.pwm == nil then
            self.pwm = pwm
        else
            local max_change = SLEW_RATE_US * dt_ms * 0.001
            self.pwm = constrain(pwm, self.pwm - max_change, self.pwm + max_change)
        end

        --- Not expecting servo to be in the correct position if inactive
        if self.state == state.inactive then
            -- Give servo some time to get to initial position after activation
            if self.activation_ms ~= nil then
                if (now_ms - self.activation_ms):tofloat() < self.settle_ms then
                    return
                end
                self.activation_ms = nil
                self.state = state.active
            end
            self.pos = nil
            return
        end

        -- Need command to calculate requested position
        if self.pwm == nil then
            self.pos = nil
            return
        end

        -- Calculate the expected angle
        self.pos = ((self.pwm - self.PWM_MIN:get()) * self.scale) + self.offset

        -- Repeated reads of one telemetry sample must not confirm a failure.
        if self.last_telem_ms ~= nil and last_ms == self.last_telem_ms then
            return
        end
        self.last_telem_ms = last_ms

        -- Check error
        local stuck = math.abs(pos - self.pos) > self.max_error

        -- no error
        if stuck == false then
            -- All good, reset stuck timer
            self.stuck_ms = nil
            self.state = state.active
            return
        end

        -- Error and already in correct state
        if self.state == state.stuck then
            return
        end

        -- Start stuck timer
        if self.stuck_ms == nil then
            self.stuck_ms = now_ms
            return
        end

        -- Check if stuck timeout has elapsed
        if (now_ms - self.stuck_ms) > ERROR_MS then
            self.state = state.stuck
            gcs:send_text(MAV_SEVERITY.ERROR, name .. " servo stuck")
        end

    end

    self.healthy = function ()
        return self.connected and (self.state == state.active) and
               (self.last_telem_ms ~= nil) and (self.stuck_ms == nil)
    end

    -- Check if motor should be shutdown, only called when armed
    self.check_failsafe = function (other)

        -- Nothing to do unless this servo has failed and the other is healthy
        local failed = (self.state == state.stuck) or (self.state == state.disconnected)
        if not failed or not other.healthy() then
            return
        end

        -- Need a motor to stop
        if motorChannel == nil then
            return
        end

        return motorChannel - 1

    end

    -- Write a log
    self.log = function ()
        local pos = 0/0 -- nan
        if self.pos ~= nil then
            pos = self.pos
        end

        logger:write("SCHK",
                     "channel,expectedPos,connected,state",
                     "BfBB",
                     "#d--",
                     "----",
                     channel,
                     pos,
                     self.connected,
                     self.state)
    end

    return self
end

local function setup()
    local tilt_mask = assert(param:get("Q_TILT_MASK"), "SRVFAIL: requires QuadPlane")
    assert(tilt_mask > 0 and tilt_mask < 4096, "SRVFAIL: invalid Q_TILT_MASK")

    local motors = {}
    for motor = 1, 12 do
        if (tilt_mask & (1 << (motor - 1))) ~= 0 then
            motors[#motors + 1] = motor_function(motor)
        end
    end
    assert(#motors == 2, "SRVFAIL: Q_TILT_MASK must select two motors")
    assert(#servo_config == 2, "SRVFAIL: configure two servos, left then right")

    local servos = {}
    local used_channels = {}
    local used_motors = {}
    local function use_channel(channel)
        assert(not used_channels[channel], "SRVFAIL: output channels must be distinct")
        used_channels[channel] = true
    end

    for i, config in ipairs(servo_config) do
        local channel, name, motor_channel, min_angle, max_angle = table.unpack(config, 1, 5)
        channel = output_channel(channel, 74 + i)
        use_channel(channel)

        if motor_channel ~= nil then
            motor_channel = output_channel(motor_channel, motors[i])
            use_channel(motor_channel)
            local fn = param:get("SERVO" .. motor_channel .. "_FUNCTION")
            assert(fn ~= nil and (fn == motors[1] or fn == motors[2]),
                   "SRVFAIL: motor output not in Q_TILT_MASK")
            assert(not used_motors[fn], "SRVFAIL: motor functions must be distinct")
            used_motors[fn] = true
        end

        assert(type(name) == "string" and #name > 0, "SRVFAIL: invalid servo name")
        assert(type(min_angle) == "number" and type(max_angle) == "number" and
               finite(min_angle) and finite(max_angle) and min_angle ~= max_angle,
               "SRVFAIL: invalid angle endpoints")
        local pwm_min = param:get("SERVO" .. channel .. "_MIN")
        local pwm_max = param:get("SERVO" .. channel .. "_MAX")
        assert(pwm_min ~= nil and pwm_max ~= nil and pwm_max > pwm_min,
               "SRVFAIL: invalid servo PWM range")

        servos[i] = servo(channel, name, motor_channel, min_angle, max_angle)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("SRVFAIL: %s servo %u motor %s",
                      name, channel, motor_channel and tostring(motor_channel) or "none"))
    end
    return servos
end

-- Servos configured in the table at the top of this script
local servos = setup()

-- Return true if motor override is allowed
-- Assume a forward transition until unassisted forward flight has been observed.
local FWtransition = true
local function allow_motor_override()

    local VTOLMode = quadplane:in_vtol_mode()
    if VTOLMode then
        -- Next time were in FW it will be a forward transition
        FWtransition = true

        -- Motor override is always allowed
        return true
    end

    -- Forward flight
    local assisted = quadplane:in_assisted_flight()
    if FWtransition and not assisted then
        -- Once were not in assisted flight then the transition is complete
        FWtransition = false
    end

    -- Allow if assisted flight is re-entered after the transition is complete
    return (not FWtransition) and assisted
end

local last_update_ms
local last_armed
local overridden_motor
local last_override_ms
local released_ms

local function release_motor_override()
    if overridden_motor == nil then
        return
    end
    -- A zero timeout still writes PWM. Preserve the current output in case the
    -- override already expired while scripting was delayed.
    local pwm = SRV_Channels:get_output_pwm_chan(overridden_motor)
    if pwm ~= nil then
        SRV_Channels:set_output_pwm_chan_timeout(overridden_motor, pwm, 0)
    end
    overridden_motor = nil
    last_override_ms = nil
    released_ms = millis()
end

local function set_motor_override(motor, now_ms)
    if overridden_motor ~= nil and overridden_motor ~= motor then
        -- Release the previous side and leave time for the main loop to apply it
        -- before stopping the other side. Never overlap two motor overrides.
        release_motor_override()
        return
    end
    if motor ~= nil and (released_ms == nil or (now_ms - released_ms):tofloat() > OVERRIDE_MS) then
        SRV_Channels:set_output_pwm_chan_timeout(motor, MOTOR_SHUT_DOWN_PWM:get(), OVERRIDE_MS)
        overridden_motor = motor
        last_override_ms = millis()
    end
end

-- Main update function
local function update()

    local now_ms = millis()
    local armed = arming:is_armed()
    local dt_ms = last_update_ms and (now_ms - last_update_ms):tofloat() or 0
    local reset = last_update_ms == nil or dt_ms > MAX_GAP_MS or armed ~= last_armed
    last_update_ms = now_ms
    last_armed = armed

    if reset then
        FWtransition = true
        for _, s in ipairs(servos) do
            s.reset()
        end
    end

    -- Check if were expecting servos to be active
    local active = SRV_Channels:get_safety_state() == false

    -- Update each of the servos
    for _, s in ipairs(servos) do
        s.update(active, dt_ms)
        s.log()
    end

    -- Run check continually so it can track state changes
    local check_failsafe = allow_motor_override()

    logger:write("SFD","allowStop,VTOLMode,Assist,Dt","BBBf",
        check_failsafe,
        quadplane:in_vtol_mode(),
        quadplane:in_assisted_flight(),
        dt_ms
    )

    local motor
    if armed == true and active and check_failsafe and not reset then
        -- Run failsafe checks. A failed servo can only stop its motor while the
        -- other servo is healthy, so both motors can never be stopped.
        for i, s in ipairs(servos) do
            motor = motor or s.check_failsafe(servos[3 - i])
        end
    end

    local action_ms = millis()
    if (action_ms - now_ms):tofloat() > MAX_GAP_MS or
       (motor ~= nil and overridden_motor == motor and last_override_ms ~= nil and
        (action_ms - last_override_ms):tofloat() >= OVERRIDE_MS) then
        -- Do not renew an override after it may have expired, even when the
        -- callback start interval and its own run time were each within limits.
        motor = nil
        last_update_ms = nil
    end
    set_motor_override(motor, action_ms)

    -- Run at 50Hz
    return update, PERIOD_MS
end

return update, PERIOD_MS
