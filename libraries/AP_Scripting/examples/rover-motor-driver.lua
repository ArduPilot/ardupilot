-- Rover motor driver for an Ackerman style vehicle (i.e. the frame has separate throttle and steering controls)
--
-- The following parameters should be set:
--     SERVO1_FUNCTION = 94 (Script 1) 
--     SERVO3_FUNCTION = 96 (Script 3)
-- 
-- The Frame's steering control should be connected to the autopilot's output1, throttle control to output3
--
-- CAUTION: This script only works for Rover
-- This script retrieves the high level controller outputs that have been sent to the regular motor driver
-- and then outputs them to the "Script 1" and "Script 3" outputs.  This does not add any real value beyond
-- serving as an example of how lua scripts can be used to implement a custom motor driver 

local K_SCRIPTING1 = 94 -- for steering control
local K_SCRIPTING3 = 96 -- for throttle control
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4

function update()
  if not arming:is_armed() then
    -- if not armed move steering and throttle to mid
    SRV_Channels:set_output_norm(K_SCRIPTING1, 0)
    SRV_Channels:set_output_norm(K_SCRIPTING3, 0)
  else
    -- retrieve high level steering and throttle control outputs from vehicle in -1 to +1 range
    local steering = vehicle:get_control_output(CONTROL_OUTPUT_YAW)
    local throttle = vehicle:get_control_output(CONTROL_OUTPUT_THROTTLE)
    if (steering and throttle) then
      if throttle < 0 then
          steering = -steering
      end
      SRV_Channels:set_output_norm(K_SCRIPTING1, steering)
      SRV_Channels:set_output_norm(K_SCRIPTING3, throttle)
    end
  end
  return update, 10 -- run at 100hz
end

gcs:send_text(6, "rover-motor-driver.lua is running")
return update()
