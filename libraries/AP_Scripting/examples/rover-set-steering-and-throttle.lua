-- cause a rover to drive in a figure of eight pattern by directly controlling steering and throttle
--
-- CAUTION: This script only works for Rover
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) increases throttle to 30% and turns right at 20% for 10 seconds
--    c) keep throttle at 30% and turns left at 20% for 10 seconds
--    d) switches to Hold mode

local stage = 0
local stage_counter = 0
local rover_guided_mode_num = 15
local rover_hold_mode_num = 4

-- the main update function that directly sets throttle and steering out to drive a figure of eight pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
    stage_counter = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(rover_guided_mode_num)) then     -- change to Guided mode
          stage = stage + 1
          stage_counter = 0
        end
      elseif (stage == 1) then      -- Stage1: increase throttle to 30% and turn right 20%
        if (vehicle:set_steering_and_throttle(0.2, 0.3)) then
          stage_counter = stage_counter + 1
          if (stage_counter >= 10) then
            stage = stage + 1
            stage_counter = 0
          end
        end
      elseif (stage == 2) then      -- Stage2: keep throttle at 30% and turn left 20%
        if (vehicle:set_steering_and_throttle(-0.2, 0.3)) then
          stage_counter = stage_counter + 1
          if (stage_counter >= 10) then
            stage = stage + 1
            stage_counter = 0
          end
        end
      elseif (stage == 3) then      -- Stage3: change to Hold mode
        vehicle:set_mode(rover_hold_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished, switching to Hold")
      end
    else -- RC6 has been moved low
      if stage > 3 then 
        stage = 0
        stage_counter = 0
      end
    end
  end

  return update, 1000
end

return update()
