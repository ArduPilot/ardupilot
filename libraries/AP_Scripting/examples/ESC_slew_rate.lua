--[[
   run an ESC with a sinisoidal demand, with settable limits and frequency
--]]

local PARAM_TABLE_KEY = 136
local PARAM_TABLE_PREFIX = "ETEST_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

local ETEST_CHAN      = bind_add_param('CHAN',          1, 0)
local ETEST_PCT       = bind_add_param('PCT',           2, 100)
local ETEST_PWM_MIN   = bind_add_param('PWM_MIN',       3, 1000)
local ETEST_PWM_MAX   = bind_add_param('PWM_MAX',       4, 2000)
local ETEST_FREQ      = bind_add_param('FREQ',          5, 1)
local ETEST_WTYPE     = bind_add_param('WTYPE',         6, 0)

-- local WTYPE_SIN = 0
local WTYPE_SQUARE = 1

function update()
   local chan = ETEST_CHAN:get()
   local freq = ETEST_FREQ:get()
   if chan > 0 and freq > 0 then
      local t = 0.001 * millis():tofloat()
      local pi = 3.1415
      local out_sin = math.sin(pi * t * freq * 2.0)
      if ETEST_WTYPE:get() == WTYPE_SQUARE then
         if out_sin > 0 then
            out_sin = 1
         else
            out_sin = -1
         end
      end
      local output = out_sin * ETEST_PCT:get() * 0.01
      local pwm_min = ETEST_PWM_MIN:get()
      local pwm_max = ETEST_PWM_MAX:get()
      local pwm_mid = 0.5*(pwm_min+pwm_max)
      local pwm = math.floor(pwm_mid + (pwm_max-pwm_mid) * output)
      SRV_Channels:set_output_pwm_chan_timeout(chan-1, pwm, 100)
      logger:write('ESLW', 'PWM,Freq', 'If', pwm, freq)
      gcs:send_named_float('PWN',pwm)
      gcs:send_named_float('FREQ',freq)
   end
   return update, 5 -- 200Hz
end

return update()
