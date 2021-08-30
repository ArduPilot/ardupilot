-- Copter descends very rapidly in a spiral pattern to a preset altitude above home
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be changed to Guided mode and then:
--    a) flies a spiral pattern using the velocity and acceleration control
--    b) slows the spiral and stops at the preset altitude
--    c) switches to RTL

-- constants
local copter_guided_mode_num = 4    -- Guided mode is 4 on copter
local copter_rtl_mode_num = 6       -- RTL is 6 on copter
local alt_above_home_min = 50       -- copter will stop at this altitude above home
local circle_radius_rate_max_ms = 1 -- radius expands at max of this many m/s
local circle_radius_accel_mss = 1   -- radius expansion speed accelerates at this many m/s/s
local circle_radius_max = 30        -- target circle's maximum radius
local speed_xy_max = 5              -- max target horizontal speed
local accel_xy = 1                  -- horizontal acceleration in m/s^2
local speed_z_max = 10              -- target descent rate is 15m/s
local accel_z = 1                   -- target vertical acceleration is 1m/s/s

-- timing and state machine variables
local stage = 0                     -- stage of descent
local last_update_ms                -- system time of last update
local dt = 0.01                     -- update rate of script (0.01 = 100hz)
local interval_ms = 1               -- update interval in ms
local last_print_ms = 0             -- pilot update timer

-- control related variables
local circle_center_pos = Vector3f()-- center of circle position as an offset from EKF origin
local circle_radius = 0             -- target circle's current radius (this is slowly expanded to circle_radius_max)
local circle_radius_rate_ms = 0     -- target circle's radius is increasing at this rate in m/s
local circle_angle_rad = 0          -- current target angle on circle (in radians)
local target_alt_D = 0              -- target altitude in m from EKF origin (Note: down is positive)
local speed_xy = 0                  -- target horizontal speed (i.e. tangential velocity or horizontal speed around the circle)
local speed_z = 0                   -- target descent rate currently
local target_yaw_deg = 0            -- target yaw in degrees (degrees is more convenient based on interface)

-- the main update function
function update()

  -- update dt
  local now_ms = millis()
  if (last_update_ms) then
      dt = (now_ms - last_update_ms):tofloat() / 1000.0
  end
  if (dt > 1) then
    dt = 0
  end
  last_update_ms = now_ms

  -- determine if progress update should be sent to user
  local update_user = false
  if (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    update_user = true
  end

  -- reset stage when disarmed or not in Guided mode
  if not arming:is_armed() or (vehicle:get_mode() ~= copter_guided_mode_num) then 
    stage = 0
    if (update_user) then
      gcs:send_text(0, "Fast Descent: waiting for Guided" .. string.format(" dt:%6.4f", dt))
    end
    return update, interval_ms
  end

  if (stage == 0) then            -- Stage0: initialise
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_position()
    if home and curr_loc then
      circle_center_pos = ahrs:get_relative_position_NED_origin()
      circle_radius_rate_ms = 0   -- reset circle radius expandion rate to zero
      circle_radius = 0           -- reset circle radius to zero
      circle_angle_rad = ahrs:get_yaw() -- reset starting angle to current heading
      target_yaw_deg = math.deg(circle_angle_rad) -- target heading will be kept at original heading
      target_alt_D = circle_center_pos:z() -- initialise target alt using current position (Note: down is positive)
      speed_xy = 0
      speed_z = 0
      stage = stage + 1           -- advance to next stage
      gcs:send_text(0, "Fast Descent: starting")
    end
  elseif (stage == 1) then        -- Stage1: descend

    -- increase circle radius
    circle_radius_rate_ms = math.min(circle_radius_rate_ms + (circle_radius_accel_mss * dt), circle_radius_rate_max_ms)   -- accelerate radius expansion
    circle_radius = math.min(circle_radius + (circle_radius_rate_ms * dt), circle_radius_max)  -- increase radius

    -- calculate horizontal and vertical speed
    if (circle_radius < circle_radius_max) then
      speed_xy = math.max(speed_xy - (accel_xy * dt), 0)  -- decelerate horizontal speed to zero
      speed_z = math.max(speed_z - (accel_z * dt), 0)     -- decelerate vertical speed to zero
    else
      speed_xy = math.min(speed_xy + (accel_xy * dt), speed_xy_max)   -- accelerate horizontal speed to max
      speed_z = math.min(speed_z + (accel_z * dt), speed_z_max)       -- accelerate to max descent rate
    end

    -- calculate angular velocity
    local ang_vel_rads = 0
    if (circle_radius >= circle_radius_max) then
      ang_vel_rads = speed_xy / circle_radius;
    end

    -- increment angular position
    circle_angle_rad = circle_angle_rad + (ang_vel_rads * dt)
    if (circle_angle_rad >= (math.pi * 2)) then
      circle_angle_rad = circle_angle_rad - (math.pi * 2)
    end

    -- calculate target position
    local cos_ang = math.cos(circle_angle_rad)
    local sin_ang = math.sin(circle_angle_rad)
    local target_pos = Vector3f()
    target_pos:x(circle_center_pos:x() + (circle_radius * cos_ang))
    target_pos:y(circle_center_pos:y() + (circle_radius * sin_ang))
    target_alt_D = target_alt_D + (speed_z * dt)
    target_pos:z(target_alt_D)

    -- calculate target velocity
    target_vel = Vector3f()
    target_vel:x(speed_xy * -sin_ang)
    target_vel:y(speed_xy * cos_ang)
    target_vel:z(speed_z)

    -- calculate target acceleration
    local centrip_accel = 0
    if (circle_radius > 0) then
      centrip_accel = speed_xy * speed_xy / circle_radius
    end
    target_accel = Vector3f()
    target_accel:x(centrip_accel * -cos_ang)
    target_accel:y(centrip_accel * -sin_ang)

    -- send targets to vehicle with original yaw target
    vehicle:set_target_posvelaccel_NED(target_pos, target_vel, target_accel, true, target_yaw_deg, false, 0, false)

    -- advance to stage 2 when below target altitude
    local rel_pos_home_NED = ahrs:get_relative_position_NED_home()
    if (rel_pos_home_NED) then 
      if (-rel_pos_home_NED:z() <= alt_above_home_min) then
        stage = stage + 1
      end
      if (update_user) then
        gcs:send_text(0, string.format("Fast Descent: alt:%d target:%d", math.floor(-rel_pos_home_NED:z()), math.floor(alt_above_home_min)))
      end
    else
      gcs:send_text(0, "Fast Descent: lost position estimate, aborting")
      stage = stage + 1
    end

  elseif (stage == 2) then  -- Stage2: change to RTL mode
    vehicle:set_mode(copter_rtl_mode_num)
    stage = stage + 1
    gcs:send_text(0, "Fast Descent: done!")
  end

  return update, interval_ms
end

return update()
