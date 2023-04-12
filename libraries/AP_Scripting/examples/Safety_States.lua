-- This Script is a example of the safety states avalalbe to lua

local last_armed
local last_motors_armed
local last_E_stop
local last_safe
function update()

  local armed = arming:is_armed()
  if armed ~= last_armed then
    gcs:send_text(0, 'Vehicle armed: ' .. tostring(armed))
  end
  last_armed = armed

  local motors_armed = motors:get_interlock()
  if motors_armed ~= last_motors_armed then
    gcs:send_text(0, 'Motors armed: ' .. tostring(motors_armed))
  end
  last_motors_armed = motors_armed

  local E_stop = SRV_Channels:get_emergency_stop()
  if E_stop ~= last_E_stop then
    gcs:send_text(0, 'E-Stop active: ' .. tostring(E_stop))
  end
  last_E_stop = E_stop

  local safe = SRV_Channels:get_safety_state()
  if safe ~= last_safe then
    gcs:send_text(0, 'Safe state: ' .. tostring(safe))
  end
  last_safe = safe

  return update, 100
end

return update()
