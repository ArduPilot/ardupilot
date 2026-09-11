--[[
   run a chirp through an ESC or servo to determine frequency response
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 137
local PARAM_TABLE_PREFIX = "ACHRP_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'could not add param table')

--[[
 @Param: ACHRP_CHAN
 @DisplayName: Chirp output channel
 @Description: 1-based servo/ESC output channel to drive with the chirp. Set to a channel number to start the chirp and to 0 to stop it. Reset to 0 on boot so a chirp never starts automatically.
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
 @Description: Time to ramp the chirp amplitude from 0 to full.
 @Range: 0 30
 @Units: s
 @User: Advanced
]]
local ACHRP_FADE_IN   = bind_add_param('F_FADE_IN',     6, 5)

--[[
 @Param: ACHRP_F_FADE_OUT
 @DisplayName: Fade-out time
 @Description: Time to ramp the chirp amplitude back to 0 at the end.
 @Range: 0 30
 @Units: s
 @User: Advanced
]]
local ACHRP_FADE_OUT  = bind_add_param('F_FADE_OUT',    7, 1)

--[[
 @Param: ACHRP_TIME
 @DisplayName: Total chirp duration
 @Description: Total sweep time in seconds, including initial dwell and fade-in/out.
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

-- ensure the chirp does not start automatically on boot
ACHRP_CHAN:set_and_save(0)

local chirp = {
   running = false,
   complete = false,
}

-- setup for a new chirp run
function chirp:init(time_record, frequency_start_hz, frequency_stop_hz, time_fade_in, time_fade_out, time_const_freq)
   self.record = time_record
   self.wMin = 2.0 * math.pi * frequency_start_hz
   self.wMax = 2.0 * math.pi * frequency_stop_hz
   self.fade_in = time_fade_in
   self.fade_out = time_fade_out
   self.const_freq = time_const_freq
   self.B = math.log(self.wMax / self.wMin)
   self.waveform_freq_rads = self.wMin
   self.complete = false
   self.running = true
end

-- chirp signal output for the given time since start, scaled by magnitude
function chirp:update(time, waveform_magnitude)
   -- window calculation, fade in/out shape
   local window
   if time <= 0.0 then
      window = 0.0
   elseif time <= self.fade_in then
      window = 0.5 - 0.5 * math.cos(math.pi * time / self.fade_in)
   elseif time <= (self.record - self.fade_out) then
      window = 1.0
   elseif time <= self.record then
      local tNorm = (time - (self.record - self.fade_out)) / self.fade_out
      window = 0.5 - 0.5 * math.cos(math.pi * tNorm + math.pi)
   else
      window = 0.0
   end

   local output
   if time <= 0.0 then
      self.waveform_freq_rads = self.wMin
      output = 0.0
   elseif time <= self.const_freq then
      -- constant frequency dwell at wMin
      self.waveform_freq_rads = self.wMin
      output = window * waveform_magnitude * math.sin(self.wMin * time - self.wMin * self.const_freq)
   elseif time <= self.record then
      if math.abs(self.wMax - self.wMin) < 1.0e-9 then
         -- no sweep if frequencies are equal
         self.waveform_freq_rads = self.wMin
         output = window * waveform_magnitude * math.sin(self.wMin * time)
      else
         -- exponential frequency sweep
         local expFactor = (time - self.const_freq) / (self.record - self.const_freq)
         self.waveform_freq_rads = self.wMin * math.exp(self.B * expFactor)
         local term = (self.wMin * (self.record - self.const_freq) / self.B) * (math.exp(self.B * expFactor) - 1)
         output = window * waveform_magnitude * math.sin(term)
      end
   else
      self.waveform_freq_rads = self.wMax
      output = 0.0
   end

   self.complete = (time > self.record)

   return output
end

-- current waveform frequency in Hz
function chirp:get_frequency_hz()
   return self.waveform_freq_rads / (2 * math.pi)
end

function chirp:stop()
   if self.running then
      gcs:send_text(MAV_SEVERITY.INFO, "Chirp stopped")
   end
   self.running = false
   self.complete = false
end

local t_start_us

local function run_chirp()
   local chan = ACHRP_CHAN:get()
   if chan <= 0 then
      chirp:stop()
      return
   end
   if chirp.complete then
      gcs:send_text(MAV_SEVERITY.INFO, "Chirp finished")
      ACHRP_CHAN:set_and_save(0)
      chirp:stop()
      return
   end
   if not chirp.running then
      local time_const_freq = 2.0 / ACHRP_F_START:get()
      chirp:init(ACHRP_TIME:get(), ACHRP_F_START:get(), ACHRP_F_STOP:get(), ACHRP_FADE_IN:get(), ACHRP_FADE_OUT:get(), time_const_freq)
      t_start_us = micros()
      gcs:send_text(MAV_SEVERITY.INFO, "Chirp starting")
   end
   local t = 1.0e-6 * (micros() - t_start_us):tofloat()
   local chirp_output = chirp:update(t, 1.0)
   local pwm_min = ACHRP_PWM_MIN:get()
   local pwm_max = ACHRP_PWM_MAX:get()
   local pwm_mid = 0.5 * (pwm_min + pwm_max)
   local pwm_mag = 0.5 * (pwm_max - pwm_min)
   local pwm = math.floor(pwm_mid + pwm_mag * chirp_output)
   SRV_Channels:set_output_pwm_chan_timeout(chan-1, pwm, 100)
   local freq = chirp:get_frequency_hz()
   gcs:send_named_float('FREQ', freq)
   gcs:send_named_float('PWM', pwm)

   if ACHRP_TYPE:get() == ACTUATOR_TYPE_ESC then
      local rpm = esc_telem:get_rpm(chan-1) or -1
      gcs:send_named_float('RPM', rpm)
      logger:write('ECRP', 'PWM,Freq,RPM', 'fff', pwm, freq, rpm)
   elseif ACHRP_TYPE:get() == ACTUATOR_TYPE_SERVO then
      local telem_data = servo_telem:get_telem(chan)
      local pos = -1
      if telem_data then
         pos = telem_data:measured_position() or -1
         gcs:send_named_float('POS', pos)
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
