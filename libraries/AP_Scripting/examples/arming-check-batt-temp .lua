-- This script runs a custom arming check of the battery temperature

auth_id = arming:get_aux_auth_id()
batt_temp_max = 35

function update() -- this is the loop which periodically runs
  if auth_id then
    now = millis()
    batt_temp = battery:get_temperature(0)
    if not batt_temp then
      arming:set_aux_auth_failed(auth_id, "Could not retrieve battery temperature")
    elseif (batt_temp >= batt_temp_max) then
      arming:set_aux_auth_failed(auth_id, "Batt temp too high (" .. tostring(batt_temp) .. "C > " .. tostring(batt_temp_max) .. "C)")
    else
      arming:set_aux_auth_passed(auth_id)
    end
  end
  return update, 5000 -- reschedules the loop in 5 seconds
end

return update() -- run immediately before starting to reschedule
