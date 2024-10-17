--[[
 script implementing pre-arm check that internal resistance is sensible
--]]

local MAX_RESISTANCE = 0.03  -- Ohms

local auth_id = assert(arming:get_aux_auth_id())

local warning_last_sent_ms = uint32_t() -- time we last sent a warning message to the user
warning_interval_ms = 10000

function update()
   local num_batts = battery:num_instances()
   local ok = true
   for i=0,num_batts do
      local resistance = battery:get_resistance(i)
      failed = resistance > MAX_RESISTANCE
      if failed then
         msg = string.format("Batt[%u] high internal resistance %.5f Ohms", i+1, resistance)
         if millis() - warning_last_sent_ms > warning_interval_ms then
            gcs:send_text(0, msg)
            warning_last_sent_ms = millis()
         end
         arming:set_aux_auth_failed(auth_id, msg)
         ok = false
      end
   end

   if ok then
      arming:set_aux_auth_passed(auth_id)
   end
   return update, 500
end

return update()
