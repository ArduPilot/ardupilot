-- performs the equivalent of a simple RTL in Guided mode using set_target_location
--
-- CAUTION: This script only works for Copter
-- this script checks for RC input > 1800 and then:
--    a) switches to Guided mode
--    b) sets the target location to be 10m above home
--    c) switches the vehicle to land once it is within a couple of meters of home

local wp_radius = 2
local target_alt_above_home = 10
local copter_guided_mode_num = 4
local copter_land_mode_num = 9
local sent_target = false

-- the main update function that performs a simplified version of RTL
function update()
  if not arming:is_armed() then -- reset state when disarmed
    sent_target = false
  else
    pwm7 = rc:get_pwm(7)
    if pwm7 and pwm7 > 1800 then                        -- check if RC input 7 has moved high
      local mode = vehicle:get_mode()                   -- get current mode
      if not sent_target then                           -- if we haven't sent the target yet
        if not (mode == copter_guided_mode_num) then    -- change to guided mode
          vehicle:set_mode(copter_guided_mode_num)
        else
          local above_home = ahrs:get_home()            -- get home location
          if above_home then
            above_home:alt(above_home:alt() + (target_alt_above_home * 100))
            sent_target = vehicle:set_target_location(above_home)   -- set target above home
          end
        end
      else

        -- change to land mode when within 2m of home
        if not (mode == copter_land_mode_num) then
          local home = ahrs:get_home()
          local curr_loc = ahrs:get_location()
          if home and curr_loc then
            local home_dist = curr_loc:get_distance(home)   -- get horizontal distance to home
            if (home_dist < wp_radius) then                 -- change to land mode if close
              vehicle:set_mode(copter_land_mode_num)
            end
          end
        end
      end
    end
  end

  return update, 1000
end

return update()
