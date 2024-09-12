--[[
   display motor lost number from MotorsMatrix for multirotors
--]]

local last_motor_lost = -1

function update()
   local lost_index
   if not MotorsMatrix:get_thrust_boost() then
      -- when get_thrust_boost is false then we have not lost a motor
      lost_index = -1
   else
      -- otherwise get the lost motor number
      lost_index = MotorsMatrix:get_lost_motor()
   end
   if lost_index ~= last_motor_lost then
      if lost_index == -1 then
         gcs:send_text(0, string.format("Motors: recovered"))
      else
         gcs:send_text(0, string.format("Motors: lost motor %d", lost_index+1))
      end
      last_motor_lost = lost_index
   end
   return update, 100
end

return update, 100
