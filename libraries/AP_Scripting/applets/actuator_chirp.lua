--[[
    run a chirp through an ESC or servo to determine frequency response
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 137
local PARAM_TABLE_PREFIX = "ACHRP_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'could not add param table')

--[[
 @Param: ACHRP_CHAN
 @DisplayName: Chirp output channel
 @Description: 1-based servo/ESC output channel to drive with the chirp. Set to 0 to disable/idle. Automatically stops when the vehicle is armed.
 @Range: 0 32
 @User: Advanced
]]
local ACHRP_CHAN      = bind_add_param('CHAN',          1, 0)

--[[
 @Param: ACHRP_PWM_MIN
 @DisplayName: Minimum PWM for chirp
 @Description: Lower PWM bound used to center and scale the chirp waveform.
 @Range: 800 2200
 @Units: pwm
 @User: Advanced
]]
local ACHRP_PWM_MIN   = bind_add_param('PWM_MIN',       2, 1300)

--[[
 @Param: ACHRP_PWM_MAX
 @DisplayName: Maximum PWM for chirp
 @Description: Upper PWM bound used to center and scale the chirp waveform.
 @Range: 800 2200
 @Units: pwm
 @User: Advanced
]]
local ACHRP_PWM_MAX   = bind_add_param('PWM_MAX',       3, 1600)

--[[
 @Param: ACHRP_F_START
 @DisplayName: Start frequency
 @Description: Chirp start frequency in Hz. A short dwell at this frequency is added automatically before the sweep.
 @Range: 0.05 200
 @Units: Hz
 @User: Advanced
]]
local ACHRP_F_START   = bind_add_param('F_START',       4, 0.5)

--[[
 @Param: ACHRP_F_STOP
 @DisplayName: Stop frequency
 @Description: Chirp stop frequency in Hz. Must be greater than or equal to start frequency.
 @Range: 0.05 500
 @Units: Hz
 @User: Advanced
]]
local ACHRP_F_STOP    = bind_add_param('F_STOP',        5, 20)

--[[
 @Param: ACHRP_F_FADE_IN
 @DisplayName: Fade-in time
 @Description: Window fade-in duration to ramp the chirp amplitude from 0 to full.
 @Range: 0 30
 @Units: s
 @User: Advanced
]]
local ACHRP_FADE_IN   = bind_add_param('F_FADE_IN',     6, 5)

--[[
 @Param: ACHRP_F_FADE_OUT
 @DisplayName: Fade-out time
 @Description: Window fade-out duration to ramp the chirp amplitude back to 0 at the end.
 @Range: 0 30
 @Units: s
 @User: Advanced
]]
local ACHRP_FADE_OUT  = bind_add_param('F_FADE_OUT',    7, 1)

--[[
 @Param: ACHRP_TIME
 @DisplayName: Total chirp duration
 @Description: Total record/sweep time in seconds, including initial dwell and fade-in/out.
 @Range: 1 600
 @Units: s
 @User: Advanced
]]
local ACHRP_TIME      = bind_add_param('TIME',          8, 60)


local ACTUATOR_TYPE_ESC = 0
local ACTUATOR_TYPE_SERVO = 1

--[[
 @Param: ACHRP_TYPE
 @DisplayName: Type of actuator
 @Description: Set type of actuator. Used to get correct type of feedback (RPM or servo position)
 @Values: 0:ESC, 1:Servo
 @User: Advanced
]]
local ACHRP_TYPE      = bind_add_param('TYPE',          9, ACTUATOR_TYPE_ESC)

-- A small utility to check approximate equality between two values
local function is_equal(a, b, epsilon)
    epsilon = epsilon or 1e-9
    return math.abs(a - b) < epsilon
end

-- reset on startup
ACHRP_CHAN:set_and_save(0)

local Chirp = {}
Chirp.__index = Chirp

-- constructor equivalent
function Chirp.new()
    local self = setmetatable({}, Chirp)
    
    -- Initialize internal variables
    self.record = 0.0
    self.magnitude = 0.0
    self.wMin = 0.0
    self.wMax = 0.0
    self.fade_in = 0.0
    self.fade_out = 0.0
    self.const_freq = 0.0
    self.B = 0.0
    self.waveform_freq_rads = 0.0
    self.window = 0.0
    self.output = 0.0
    self.complete = false
    self.running = false

    return self
end

-- initializes the chirp object
function Chirp:init(time_record, frequency_start_hz, frequency_stop_hz, time_fade_in, time_fade_out, time_const_freq)
    -- pass in variables to class
    self.record = time_record
    self.wMin = 2.0 * math.pi * frequency_start_hz
    self.wMax = 2.0 * math.pi * frequency_stop_hz
    self.fade_in = time_fade_in
    self.fade_out = time_fade_out
    self.const_freq = time_const_freq

    -- B = ln(wMax / wMin)
    self.B = math.log(self.wMax / self.wMin)

    -- Mark as incomplete
    self.complete = false
    self.running = true
end

-- determine chirp signal output at the specified time and amplitude
function Chirp:update(time, waveform_magnitude)
    self.magnitude = waveform_magnitude

    -- Window calculation: fade in/out shape
    if time <= 0.0 then
        self.window = 0.0
    elseif time <= self.fade_in then
        self.window = 0.5 - 0.5 * math.cos(math.pi * time / self.fade_in)
    elseif time <= (self.record - self.fade_out) then
        self.window = 1.0
    elseif time <= self.record then
        local tNorm = (time - (self.record - self.fade_out)) / self.fade_out
        self.window = 0.5 - 0.5 * math.cos(math.pi * tNorm + math.pi)
    else
        self.window = 0.0
    end

    -- Compute current frequency and output
    if time <= 0.0 then
        -- Before chirp start
        self.waveform_freq_rads = self.wMin
        self.output = 0.0
    elseif time <= self.const_freq then
        -- Constant frequency dwell at wMin
        self.waveform_freq_rads = self.wMin
        self.output = self.window * self.magnitude * math.sin(self.wMin * time - self.wMin * self.const_freq)
    elseif time <= self.record then
        -- Chirp portion
        if is_equal(self.wMin, self.wMax) then
            -- No sweep if frequencies are equal
            self.waveform_freq_rads = self.wMin
            self.output = self.window * self.magnitude * math.sin(self.wMin * time)
        else
            -- Exponential frequency sweep
            local expFactor = (time - self.const_freq) / (self.record - self.const_freq)
            self.waveform_freq_rads = self.wMin * math.exp(self.B * expFactor)
            -- (wMin*(record - const_freq)/B) * [exp(B * (time - const_freq)/(record - const_freq)) - 1]
            local term = (self.wMin * (self.record - self.const_freq) / self.B) 
                         * (math.exp(self.B * expFactor) - 1)
            self.output = self.window * self.magnitude * math.sin(term)
        end
    else
        -- After chirp ends
        self.waveform_freq_rads = self.wMax
        self.output = 0.0
    end

    -- Update 'complete' if we've gone beyond the record duration
    self.complete = (time > self.record)

    return self.output
end

-- accessor for the current waveform frequency
function Chirp:get_frequency_hz()
    return self.waveform_freq_rads / (2 * math.pi)
end

-- Return true if chirp is completed
function Chirp:completed()
    return self.complete
end

-- stop chirp
function Chirp:stop()
    if self.running then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Chirp Stopped"))
    end
    self.running = false
    self.complete = false
end

local t_start_us = nil

function run_chirp()
    local chan = ACHRP_CHAN:get()
    if chan <= 0 then
        Chirp:stop()
        return
    end
    if Chirp:completed() then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Chirp Finished"))
        ACHRP_CHAN:set_and_save(0)
        Chirp:stop()
        return
    end
    if not Chirp.running then
       local time_const_freq = 2.0 / ACHRP_F_START:get()
       Chirp:init(ACHRP_TIME:get(), ACHRP_F_START:get(), ACHRP_F_STOP:get(), ACHRP_FADE_IN:get(), ACHRP_FADE_OUT:get(), time_const_freq)
       t_start_us = micros()
       gcs:send_text(MAV_SEVERITY.INFO, string.format("Chirp starting"))
    end
    local now_us = micros()
    local dt_us = now_us - t_start_us
    local t = 1.0e-6 * dt_us:tofloat()
    local chirp_output = Chirp:update(t, 1)
    local pwm_min = ACHRP_PWM_MIN:get()
    local pwm_max = ACHRP_PWM_MAX:get()
    local pwm_mid = 0.5*(pwm_min+pwm_max)
    local pwm_mag = 0.5*(pwm_max - pwm_min)
    local pwm = math.floor(pwm_mid + pwm_mag * chirp_output)
    SRV_Channels:set_output_pwm_chan_timeout(chan-1, pwm, 100)
    local freq = Chirp:get_frequency_hz()
    gcs:send_named_float('FREQ',freq)
    gcs:send_named_float('PWM',pwm)

    if ACHRP_TYPE:get() == ACTUATOR_TYPE_ESC then
       local rpm = esc_telem:get_rpm(chan-1) or -1
       gcs:send_named_float('RPM',rpm)
       logger:write('ECRP', 'PWM,Freq,RPM', 'fff', pwm, freq, rpm)
    elseif ACHRP_TYPE:get() == ACTUATOR_TYPE_SERVO then
       local telem_data = servo_telem:get_telem(chan)
       local pos = -1
       if telem_data then
          pos = telem_data:measured_position() or -1
          gcs:send_named_float('POS',pos)
       end
       logger:write('SCRP', 'PWM,Freq,Pos', 'fff', pwm, freq, pos)
    end
end

function update()
    run_chirp()
    return update, 5 -- 200Hz
end

gcs:send_text(MAV_SEVERITY.INFO, "ActuatorChirp loaded")

return update()
