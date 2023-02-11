-- runcam_on_arm.lua: start and stop video recording on arm/disarm
--
-- ArduPilot can turn a RUNCAM on and off with a TX switch if you set
-- its function to 78:RunCamControl. This script uses
-- `rc:run_aux_function()' to start and stop the camera without human
-- intervention or a separate RC channel.
--
-- An interesting aspect of this script is variable rescheduling
-- time. RUNCAM serial protocol controls the camera by simulating a
-- button press both to start and stop recording--it's the same
-- button. If you send commands faster than the camera can process
-- them, it can ignore a command and interpret the next one as the
-- opposite of what you intend, e.g., you may lose a button press to
-- stop recording, and the camera keeps rolling; when you're ready to
-- start recording again, the camera interprets your button press as a
-- command to stop recording. To address this, AP has a special
-- parameter, `CAM_RC_BTN_DELAY'. I use this parameter as the
-- rescheduling delay after a button press. But between button
-- presses, I want the script to be responsive and start recording as
-- soon as the vehicle arms, so there I use a shorter delay.


-- constants
local RC_OPTION = {RunCamControl=78}
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- configuration
local DELAY_LONG = param:get("CAM_RC_BTN_DELAY")
if not DELAY_LONG then
   gcs:send_text(MAV_SEVERITY.ERROR, "CAM_RC_* parameters missing; camera autoarming unavailable")
   return
end
local DELAY_SHORT = DELAY_LONG / 3


gcs:send_text(MAV_SEVERITY.NOTICE, "Arming controls RUNCAM recording")


-- state
local prev_armed = false

function update()
   local is_armed = arming:is_armed()

   local delay = DELAY_SHORT

   if is_armed ~= prev_armed then
      -- a state transition has occurred
      if is_armed then
         gcs:send_text(MAV_SEVERITY.INFO, "RUNCAM on")
         rc:run_aux_function(RC_OPTION.RunCamControl, AuxSwitchPos.HIGH)
      else
         gcs:send_text(MAV_SEVERITY.INFO, "RUNCAM off")
         rc:run_aux_function(RC_OPTION.RunCamControl, AuxSwitchPos.LOW)
      end
      delay = DELAY_LONG
   else
      delay = DELAY_SHORT
   end

   prev_armed = is_armed

   return update, delay
end

return update()

