--[[

   example script to show interrupting a mission and then 
   reseting the crosstracking to the correct line when
   returning to the mission after the interruption
--]]

SCRIPT_NAME = "Crosstrack Restore"
SCRIPT_NAME_SHORT = "XTrack"
SCRIPT_VERSION = "4.6.0-001"

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

local last_sw = -1
local AUX_FN = 300

local previous_location

local function update()
   local sw_current = rc:get_aux_cached(AUX_FN)
   if not sw_current then
      -- treat unset as notch1
      sw_current = 0
   end
   if sw_current ~= last_sw then
      last_sw = sw_current
      if sw_current == 0 then
        vehicle:set_mode(FLIGHT_MODE.AUTO)
        if previous_location ~= nil then
            vehicle:set_crosstrack_start(previous_location)
            gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Restored previous location", SCRIPT_NAME_SHORT))
        end
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Switched to AUTO", SCRIPT_NAME_SHORT))
      else
        previous_location = vehicle:get_previous_location()
        vehicle:set_mode(FLIGHT_MODE.LOITER)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Switched to LOITER", SCRIPT_NAME_SHORT))
      end
   end
   return update,100
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )

return update()

