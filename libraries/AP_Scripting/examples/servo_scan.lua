-- move a servo in a sinisoidal fashion

local SERVO_FUNCTION = 94
local PERIOD = 0.5

function update() -- this is the loop which periodically runs
  local t = 0.001 * millis():tofloat()
  local pi = 3.1415
  local output = math.sin(pi * t * PERIOD * 2.0)
  local pwm = math.floor(1500 + 500 * output)
  SRV_Channels:set_output_pwm(SERVO_FUNCTION, pwm)
  return update, 20 -- reschedules the loop at 50Hz
end

return update() -- run immediately before starting to reschedule
