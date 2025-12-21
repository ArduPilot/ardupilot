-- command a Copter to takeoff to 5m and fly a vertical circle in the clockwise direction
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 5m
--    c) flies a vertical circle using the velocity controller
--    d) switches to RTL mode

local takeoff_alt_above_home = 5
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0
local circle_angle = 0
local circle_angle_increment = 1    -- increment the target angle by 1 deg every 0.1 sec (i.e. 10deg/sec)
local circle_speed = 1              -- velocity is always 1m/s
local yaw_cos = 0                   -- cosine of yaw at takeoff
local yaw_sin = 0                   -- sine of yaw at takeoff

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    stage = 0
    circle_angle = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          local yaw_rad = ahrs:get_yaw_rad()
          yaw_cos = math.cos(yaw_rad)
          yaw_sin = math.sin(yaw_rad)
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
      elseif (stage == 3 ) then   -- Stage3: fly a vertical circle

        -- calculate velocity vector
        circle_angle = circle_angle + circle_angle_increment
        if (circle_angle >= 360) then
          stage = stage + 1
        end
        local target_vel = Vector3f()
        local vel_xy = math.cos(math.rad(circle_angle)) * circle_speed
        target_vel:x(yaw_sin * vel_xy)
        target_vel:y(yaw_cos * -vel_xy)
        target_vel:z(-math.sin(math.rad(circle_angle)) * circle_speed)

        -- send velocity request
        if not (vehicle:set_target_velocity_NED(target_vel)) then
          gcs:send_text(0, "failed to execute velocity command")
        end
      elseif (stage == 4) then  -- Stage4: change to RTL mode
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished square, switching to RTL")
      end
    end
  end

  return update, 100
end

return update()
