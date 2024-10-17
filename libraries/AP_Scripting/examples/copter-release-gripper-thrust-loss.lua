--[[
This script should only be used on multicopters
The gripper is released when thrust loss is detected
Note that thrust loss false positives are common meaning the gripper may be released unnecessarily
--]]

local RC_OPTION = {GRIPPER=19}
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local update_interval_ms = 100
local thrust_loss_counter = 0
local gripper_released = false

function update()

   -- initialise gripper released state
   if thrust_loss_counter == 0 then
      gripper_released = false
   end

   -- return immediately if not armed
   if not arming:is_armed() then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- return if no thrust loss
   if not MotorsMatrix:get_thrust_boost() then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- return if vehicle is not descending (or cannot retrieve climb rate)
   local vel_NED = ahrs:get_velocity_NED()
   if vel_NED == nil or vel_NED:z() <= 0 then
      thrust_loss_counter = 0
      return update, update_interval_ms
   end

   -- vehicle is descending and thrust loss is detected
   thrust_loss_counter = thrust_loss_counter + 1
   if thrust_loss_counter < 10 then
      return update, update_interval_ms
   end

   -- release gripper and warn user
   if not gripper_released then
      if rc:run_aux_function(RC_OPTION.GRIPPER, AuxSwitchPos.LOW) then
        gripper_released = true
        gcs:send_text(MAV_SEVERITY.WARNING, "Thrust loss, gripper released!")
      end
   end

   return update, update_interval_ms
end

return update, update_interval_ms
