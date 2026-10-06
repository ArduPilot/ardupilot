-- Lua script to switch between GPS1 and GPS2 using an RC AUX switch

-- To test in SITL:
-- SIM_GPS2_ENABLE 1
-- SIM_GPS2_TYPE 1
-- GPS2_TYPE 1
-- RC8_OPTION 300
-- GPS_AUTO_SWITCH 0
-- reboot
-- param fetch GPS_PRIMARY
-- rc 8 2000
-- param fetch GPS_PRIMARY
-- rc 8 1000
-- param fetch GPS_PRIMARY


-- The RCx_OPTION should be set to "300" (script function) for the desired channel
local AUX_FUNCTION_NUM = 300

-- Parameters
local GPS_PRIMARY = Parameter("GPS_PRIMARY")
local GPS_AUTO_SWITCH = Parameter("GPS_AUTO_SWITCH") -- should be 0 to allow manual switching

-- Constants
local GPS_FIRST = 0
local GPS_SECOND = 1

function update()
   -- Read switch position (0=low, 1=mid, 2=high)
   local sw_pos = rc:get_aux_cached(AUX_FUNCTION_NUM)
   if sw_pos == nil then
      return update, 100
   end

   -- Check if GPS_AUTO_SWITCH is 0 (manual mode)
   if GPS_AUTO_SWITCH:get() ~= 0 then
      gcs:send_text(0, "Set GPS_AUTO_SWITCH to 0 for GPS switching control")
      return update, 5000
   end

   if sw_pos == 0 then
      if GPS_PRIMARY:get() ~= GPS_FIRST then
         GPS_PRIMARY:set(GPS_FIRST)
         gcs:send_text(0, "Switched to GPS1")
      end
   else
      if GPS_PRIMARY:get() ~= GPS_SECOND then
         GPS_PRIMARY:set(GPS_SECOND)
         gcs:send_text(0, "Switched to GPS2")
      end
   end

   -- check every 0.1s to allow for reasonably quick switching if one GPS is bad.
   return update, 100  
end

gcs:send_text(0, "GPS switch script loaded (GPS1 <-> GPS2)")

return update()
