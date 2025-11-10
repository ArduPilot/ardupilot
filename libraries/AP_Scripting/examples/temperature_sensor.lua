--[[
   simple example of reading a temperature sensor
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

--[[
   main update function, called at 1Hz
--]]
function update()
   local temperature_C = temperature_sensor:get_temperature(0)
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Temperature: %f", temperature_C))
   return update, 1000
end

-- start running update loop
return update, 1000

