-- command a Copter to takeoff to 10m and fly a square pattern
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 10m
--    c) flies a 20m x 20m square pattern using the velocity controller
--    d) switches to RTL mode

local takeoff_alt_above_home = 10
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0
local bottom_left_loc   -- vehicle location when starting square
local square_side_length = 20   -- length of each side of square

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC7 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then     -- change to Guided mode
          stage = stage + 1
        end
      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = stage + 1
        end
      elseif (stage == 2) then      -- Stage2: check if vehicle has reached target altitude
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_location()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())))
          if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
            stage = stage + 1
            bottom_left_loc = curr_loc          -- record location when starting square
          end
        end
      elseif (stage >= 3 and stage <= 6) then   -- fly a square using velocity controller
        local curr_loc = ahrs:get_location()
        local target_vel = Vector3f()           -- create velocity vector
        if (bottom_left_loc and curr_loc) then
          local dist_NE = bottom_left_loc:get_distance_NE(curr_loc)

          -- Stage3 : fly North at 2m/s
          if (stage == 3) then
            target_vel:x(2)
            if (dist_NE:x() >= square_side_length) then
              stage = stage + 1
            end
          end

          -- Stage4 : fly East at 2m/s
          if (stage == 4) then
            target_vel:y(2) 
            if (dist_NE:y() >= square_side_length) then
              stage = stage + 1
            end
          end

          -- Stage5 : fly South at 2m/s
          if (stage == 5) then
            target_vel:x(-2)
            if (dist_NE:x() <= 2) then
              stage = stage + 1
            end
          end

          -- Stage6 : fly West at 2m/s
          if (stage == 6) then
            target_vel:y(-2)
            if (dist_NE:y() <= 2) then
              stage = stage + 1
            end
          end

          -- send velocity request
          if (vehicle:set_target_velocity_NED(target_vel)) then   -- send target velocity to vehicle
            gcs:send_text(0, "pos:" .. tostring(math.floor(dist_NE:x())) .. "," .. tostring(math.floor(dist_NE:y())) .. " sent vel x:" .. tostring(target_vel:x()) .. " y:" .. tostring(target_vel:y()))
          else
            gcs:send_text(0, "failed to execute velocity command")
          end
        end
      elseif (stage == 7) then  -- Stage7: change to RTL mode
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished square, switching to RTL")
      end
    end
  end

  return update, 1000
end

return update()
