-- move a servo in a sinusoidal fashion, with settable limits and frequency

local PARAM_TABLE_KEY = 135
local PARAM_TABLE_PREFIX = "STEST_"

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

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local STEST_CHAN      = bind_add_param('CHAN',          1, 0)
local STEST_PCT       = bind_add_param('PCT',           2, 100)
local STEST_PWM_MIN   = bind_add_param('PWM_MIN',       3, 1000)
local STEST_PWM_MAX   = bind_add_param('PWM_MAX',       4, 2000)
local STEST_FREQ      = bind_add_param('FREQ',          5, 1)

function update()
   local chan = STEST_CHAN:get()
   local freq = STEST_FREQ:get()
   if chan > 0 and freq > 0 then
      local t = 0.001 * millis():tofloat()
      local pi = 3.1415
      local output = math.sin(pi * t * freq * 2.0) * STEST_PCT:get() * 0.01
      local pwm_min = STEST_PWM_MIN:get()
      local pwm_max = STEST_PWM_MAX:get()
      local pwm_mid = 0.5*(pwm_min+pwm_max)
      local pwm = math.floor(pwm_mid + (pwm_max-pwm_mid) * output)
      SRV_Channels:set_output_pwm_chan_timeout(chan-1, pwm, 100)
      --gcs:send_text(0, string.format("pwm=%u", pwm))
   end
   return update, 5 -- 200Hz
end

return update()
