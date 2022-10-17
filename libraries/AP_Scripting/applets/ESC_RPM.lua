--[[
   produce ESC RPM telemetry from PWM values
--]]

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "ERPM_"
local UPDATE_RATE_HZ = 100

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup ERPM specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 28), 'could not add param table')

local ERPM_ENABLE      = bind_add_param('ENABLE',         1, 0)
local ERPM_ESC_OFS     = bind_add_param('ESC_OFS',        2, 0)
local ERPM_FILTER_HZ   = bind_add_param('FILTER_HZ',      3, 3)

local ERPM_S1_MASK     = bind_add_param('S1_MASK',       10, 0)
local ERPM_S1_KV       = bind_add_param('S1_KV',         11, 0)
local ERPM_S1_BATT_IDX = bind_add_param('S1_BATT_IDX',   12, 0)
local ERPM_S1_PWM_MIN  = bind_add_param('S1_PWM_MIN',    13, 1000)
local ERPM_S1_PWM_MAX  = bind_add_param('S1_PWM_MAX',    14, 2000)
local ERPM_S1_REF_PWM  = bind_add_param('S1_REF_PWM',    15, 0)
local ERPM_S1_REF_AMP  = bind_add_param('S1_REF_AMP',    16, 0)
local ERPM_S1_REF_FREQ = bind_add_param('S1_REF_FREQ',   17, 0)
local ERPM_S1_REF_VOLT = bind_add_param('S1_REF_VOLT',   18, 0)

local ERPM_S2_MASK     = bind_add_param('S2_MASK',       20, 0)
local ERPM_S2_KV       = bind_add_param('S2_KV',         21, 0)
local ERPM_S2_BATT_IDX = bind_add_param('S2_BATT_IDX',   22, 0)
local ERPM_S2_PWM_MIN  = bind_add_param('S2_PWM_MIN',    23, 1000)
local ERPM_S2_PWM_MAX  = bind_add_param('S2_PWM_MAX',    24, 2000)
local ERPM_S2_REF_PWM  = bind_add_param('S2_REF_PWM',    25, 0)
local ERPM_S2_REF_AMP  = bind_add_param('S2_REF_AMP',    26, 0)
local ERPM_S2_REF_FREQ = bind_add_param('S2_REF_FREQ',   27, 0)
local ERPM_S2_REF_VOLT = bind_add_param('S2_REF_VOLT',   28, 0)


local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_EMERGENCY = 0

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end


function calc_lowpass_alpha(dt, cutoff_freq)
   if (dt <= 0.0 or cutoff_freq <= 0.0) then
      return 1.0
   end
   local rc = 1.0/(math.pi*2*cutoff_freq)
   return dt/(dt+rc)
end


--[[
   handle for one motor RPM generator
--]]
local function motor_object(_idx, _kV, _bat_idx, _pwm_min, _pwm_max, _ref_pwm, _ref_amp, _ref_freq, _ref_volt)
   local self = {}

   -- private fields as locals
   local idx = _idx
   local kV = _kV
   local bat_idx = _bat_idx
   local pwm_min = _pwm_min
   local pwm_max = _pwm_max
   local ref_pwm = _ref_pwm
   local ref_amp = _ref_amp
   local ref_freq = _ref_freq
   local ref_volt = _ref_volt
   local filt_pwm = pwm_min:get()
   local filt_volt = 0.0
   local filt_curr = 0.0
   local amp_est = 0.0
   local alpha = 1.0

   if not battery:voltage(bat_idx:get()) then
      gcs:send_text(MAV_SEVERITY_INFO, string.format("No voltage for battery index %u", bat_idx:get()))
      return nil
   end
   if not battery:current_amps(bat_idx:get()) then
      gcs:send_text(MAV_SEVERITY_INFO, string.format("No current for battery index %u", bat_idx:get()))
      return nil
   end
   if not SRV_Channels:get_output_pwm_chan(idx) then
      gcs:send_text(MAV_SEVERITY_INFO, string.format("No PWM for servo %u", idx+1))
      return nil
   end

   function self.get_bat_idx()
      return bat_idx:get()
   end

   -- convert a PWM to a 0 to 1 command value
   function self.pwm_to_command(pwm)
      local pwm_range = pwm_max:get() - pwm_min:get()
      return constrain(pwm - pwm_min:get(), 0, pwm_range) / pwm_range
   end

   -- get PWM for channel
   function self.get_pwm()
      return SRV_Channels:get_output_pwm_chan(idx)
   end

   -- estimate the current based on PWM, using the ref PWM and ref current
   -- assumes current goes as square of command
   function self.current_estimate()
      local pwm = self.get_pwm()
      if not pwm then
         return 0.0
      end
      alpha = calc_lowpass_alpha(1.0/UPDATE_RATE_HZ, ERPM_FILTER_HZ:get())
      filt_pwm = filt_pwm + (pwm - filt_pwm) * alpha
      local command = self.pwm_to_command(filt_pwm)
      local command_ref = self.pwm_to_command(ref_pwm:get())
      local amp_max = ref_amp:get() / (command_ref*command_ref)
      amp_est = (command*command) * amp_max
      return amp_est
   end

   --[[
      speedP = (pwm - pwm_min) / (pwm_max-pwm_min)
      RPM = kV * speedP * (voltage - current * R)
      RPMx = kV * speedP * voltage - current * R)
      RPM / (kV * speedP) = (voltage - current * R)
      R = (voltage - RPM / (kV * speedP)) / current
      R = (voltage - RPM / (kV * (pwm - pwm_min) / (pwm_max-pwm_min))) / current
      R = (voltage - (freq*60) / (kV * (pwm - pwm_min) / (pwm_max-pwm_min))) / current
   --]]

   function self.calc_R()
      local R = (ref_volt:get() - (ref_freq:get()*60) / (kV:get() * (ref_pwm:get() - pwm_min:get()) / (pwm_max:get()-pwm_min:get()))) / ref_amp:get()
      return R
   end

   function self.update(total_current)
      local voltage = battery:voltage(bat_idx:get())
      local total_amps = total_current[bat_idx:get()]
      local current = 0.0
      if total_amps > 0.01 then
         -- work out current for this motor using expected proportion from square law estimator
         current = battery:current_amps(bat_idx:get()) * amp_est / total_amps
      end
      if not voltage or not current then
         return
      end
      filt_volt = filt_volt + (voltage - filt_volt) * alpha
      filt_curr = filt_curr + (current - filt_curr) * alpha
      local command = self.pwm_to_command(filt_pwm)
      local R = self.calc_R()
      local rpm = kV:get() * command * constrain(filt_volt - filt_curr * R, 0, filt_volt)
      esc_telem:update_rpm(idx+ERPM_ESC_OFS:get(), math.floor(rpm+0.5), 0)
   end

   function self.configured()
      return true
   end

   return self
end

if ERPM_ENABLE:get() <= 0 then
   return
end

local num_motors = 0
local motors = {}

-- setup motor objects for a set of parameters
function setup_motors(mask, kV, bat_idx, pwm_min, pwm_max, ref_pwm, ref_amp, ref_freq, ref_volt)
   for i = 0, 31 do
      if (math.floor(mask:get()) & (1<<i) ~= 0) then
         local m = motor_object(i, kV, bat_idx, pwm_min, pwm_max, ref_pwm, ref_amp, ref_freq, ref_volt)
         if m ~= nil and m:configured() then
            motors[num_motors] = m
            num_motors = num_motors + 1
         end
      end
   end
end

setup_motors(ERPM_S1_MASK, ERPM_S1_KV, ERPM_S1_BATT_IDX, ERPM_S1_PWM_MIN, ERPM_S1_PWM_MAX, ERPM_S1_REF_PWM, ERPM_S1_REF_AMP, ERPM_S1_REF_FREQ, ERPM_S1_REF_VOLT)
setup_motors(ERPM_S2_MASK, ERPM_S2_KV, ERPM_S2_BATT_IDX, ERPM_S2_PWM_MIN, ERPM_S2_PWM_MAX, ERPM_S2_REF_PWM, ERPM_S2_REF_AMP, ERPM_S2_REF_FREQ, ERPM_S2_REF_VOLT)

gcs:send_text(MAV_SEVERITY_INFO, string.format("ERPM Setup %u motors", num_motors))

function update()
   if ERPM_ENABLE:get() <= 0 then
      return
   end

   -- get the total current estimate for each battery
   local total_current = {}
   for idx in pairs(motors) do
      local m = motors[idx]
      local bat_idx = m:get_bat_idx()
      if not total_current[bat_idx] then
         total_current[bat_idx] = 0.0
      end
      total_current[bat_idx] = total_current[bat_idx] + m:current_estimate()
   end

   for idx in pairs(motors) do
      motors[idx].update(total_current)
   end
end

-- wrapper around update()
-- if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
