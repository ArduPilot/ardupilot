-- Example of guided mode angle control, yawing back and forth

local target_yaw_1 = 45
local target_yaw_2 = -45

local target_roll = 0 -- deg
local target_pitch = 0 -- deg

local climb_rate = 0 -- m/s
local use_yaw_rate = false
local yaw_rate = 0 -- degs/s

local flipflop = true
function update()
  if flipflop then
    vehicle:set_target_angle_and_climbrate(target_roll,target_pitch,target_yaw_1,climb_rate,use_yaw_rate,yaw_rate)
  else 
    vehicle:set_target_angle_and_climbrate(target_roll,target_pitch,target_yaw_2,climb_rate,use_yaw_rate,yaw_rate)
  end
  flipflop = not flipflop
  return update, 10000
end

return update()
