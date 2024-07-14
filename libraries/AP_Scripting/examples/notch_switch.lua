--[[

   example script to switch between two notch setups by changing the
   attenuation to zero on the notch to disable. This allows for easy
   in-flight switching between two different notch setups
--]]
local INS_HNTCH_ATT = Parameter('INS_HNTCH_ATT')
local INS_HNTC2_ATT = Parameter('INS_HNTC2_ATT')

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

if not INS_HNTCH_ATT:get() or not INS_HNTC2_ATT:get() then
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("Need 2 notches configured"))
   return
end

local last_sw = -1
local AUX_FN = 300

local attenuation = INS_HNTCH_ATT:get()
if not attenuation then
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("Unable to get attenuation"))
   return
end

function update()
   local sw_current = rc:get_aux_cached(AUX_FN)
   if not sw_current then
      -- treat unset as notch1
      sw_current = 0
   end
   if sw_current ~= last_sw then
      last_sw = sw_current
      if sw_current == 0 then
         INS_HNTC2_ATT:set(0)
         INS_HNTCH_ATT:set(attenuation)
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Switched to notch1 %.2f", attenuation))
      else
         INS_HNTC2_ATT:set(attenuation)
         INS_HNTCH_ATT:set(0)
         gcs:send_text(MAV_SEVERITY.INFO, string.format("Switched to notch2 %.2f", attenuation))
      end
   end
   return update,100
end

return update()

