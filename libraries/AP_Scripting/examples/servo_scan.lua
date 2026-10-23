-- move a servo in a sinusoidal fashion

local SERVO_FUNCTION = 94
local FREQUENCY = 0.125

function update() -- this is the loop which periodically runs
  local t = 0.001 * millis():tofloat()
  local output = math.sin(math.pi * t * FREQUENCY * 2.0)
  local pwm = math.floor(1500 + 500 * output)
  SRV_Channels:set_output_pwm(SERVO_FUNCTION, pwm)
  return update, 20 -- reschedules the loop at 50Hz
end

return update() -- run immediately before starting to reschedule
