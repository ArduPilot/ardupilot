-- Copter descends very rapidly in a spiral pattern to a preset altitude above home
--
-- CAUTION: This script only works for Copter 4.2 (and higher)
-- this script waits for the vehicle to be changed to Guided mode and then:
--    a) flies a spiral pattern using the velocity and acceleration control
--    b) slows the spiral and stops at the preset altitude
--    c) switches to RTL

-- constants
local copter_guided_mode_num = 4    -- Guided mode is 4 on copter
local copter_rtl_mode_num = 6       -- RTL is 6 on copter
local circle_radius_rate_max_ms = 1 -- radius expands at max of this many m/s
local circle_radius_accel_mss = 1   -- radius expansion speed accelerates at this many m/s/s
local accel_xy = 1                  -- horizontal acceleration in m/s^2
local accel_z = 1                   -- target vertical acceleration is 1m/s/s
local speed_z_slowdown = 0.1        -- target vertical speed during final slowdown

-- timing and state machine variables
local stage = 0                     -- stage of descent
local last_update_ms                -- system time of last update
local dt = 0.01                     -- update rate of script (0.01 = 100hz)
local interval_ms = 1               -- update interval in ms
local last_print_ms = 0             -- pilot update timer
local auto_last_id = -1             -- unique id used to detect if a new NAV_SCRIPT_TIME command has started

-- control related variables
local circle_center_pos = Vector3f()-- center of circle position as an offset from EKF origin
local circle_radius = 0             -- target circle's current radius (this is slowly expanded to circle_radius_max)
local circle_radius_rate_ms = 0     -- target circle's radius is increasing at this rate in m/s
local circle_angle_rad = 0          -- current target angle on circle (in radians)
local target_alt_D = 0              -- target altitude in m from EKF origin (Note: down is positive)
local speed_xy = 0                  -- target horizontal speed (i.e. tangential velocity or horizontal speed around the circle)
local speed_z = 0                   -- target descent rate currently
local target_yaw_deg = 0            -- target yaw in degrees (degrees is more convenient based on interface)

-- create and initialise parameters
local PARAM_TABLE_KEY = 75          -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "FDST_", 6), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ACTIVATE', 0), 'could not add FDST_ACTIVATE param')     -- 0:active in Guided, 1:active in Auto's NAV_SCRIPT_TIME command
assert(param:add_param(PARAM_TABLE_KEY, 2, 'ALT_MIN', 50), 'could not add FDST_ALT_MIN param')      -- copter will stop at this altitude above home
assert(param:add_param(PARAM_TABLE_KEY, 3, 'RADIUS', 10), 'could not add FDST_RADIUS parameter')    -- target circle's maximum radius
assert(param:add_param(PARAM_TABLE_KEY, 4, 'SPEED_XY', 5), 'could not add FDST_SPEED_XY param')     -- max target horizontal speed
assert(param:add_param(PARAM_TABLE_KEY, 5, 'SPEED_DN', 10), 'could not add FDST_SPEED_DN param')    -- target descent rate
assert(param:add_param(PARAM_TABLE_KEY, 6, 'YAW_BEHAVE', 0), 'could not add FDST_YAW_BEHAVE param') -- 0:yaw does not change 1:yaw points toward center

-- bind parameters to variables
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end
local activate_type = bind_param("FDST_ACTIVATE")       -- activate type 0:Guided, 1:Auto's NAV_SCRIPT_TIME
local alt_above_home_min = bind_param("FDST_ALT_MIN")   -- copter will stop at this altitude above home
local circle_radius_max = bind_param("FDST_RADIUS")     -- target circle's maximum radius
local speed_xy_max = bind_param("FDST_SPEED_XY")        -- max target horizontal speed
local speed_z_max = bind_param("FDST_SPEED_DN")         -- target descent rate
local yaw_behave = bind_param("FDST_YAW_BEHAVE")        -- 0:yaw is static, 1:yaw points towards center of circle

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

  -- reset stage until activated in Guided or Auto mode
  if (activate_type:get() == 0) then
    -- activate_type 0: reset stage when disarmed or not in Guided mode
    if not arming:is_armed() or (vehicle:get_mode() ~= copter_guided_mode_num) then 
      stage = 0
      if (update_user and arming:is_armed()) then
        gcs:send_text(6, "Fast Descent: waiting for Guided")
      end
      return update, interval_ms
    end
  else
    -- activate_type 1: reset stage when disarmed or not in Auto executing NAV_SCRIPT_TIME command
    auto_last_id, cmd, arg1, arg2 = vehicle:nav_script_time()
    if not arming:is_armed() or not auto_last_id then 
      stage = 0
      if (update_user and arming:is_armed()) then
        gcs:send_text(6, "Fast Descent: waiting for NAV_SCRIPT_TIME")
      end
      return update, interval_ms
    end
  end

  if (stage == 0) then            -- Stage0: initialise
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_location()
    if home and curr_loc then
      circle_center_pos = ahrs:get_relative_position_NED_origin()
      circle_radius_rate_ms = 0   -- reset circle radius expandion rate to zero
      circle_radius = 0           -- reset circle radius to zero
      if yaw_behave:get() == 0 then
        -- yaw does not move so reset starting angle to current heading
        circle_angle_rad = ahrs:get_yaw()
      else
        -- yaw points towards center so start 180deg behind vehicle
        circle_angle_rad = ahrs:get_yaw() + math.pi
        if (circle_angle_rad >= (math.pi * 2)) then
          circle_angle_rad = circle_angle_rad - (math.pi * 2)
        end
      end
      target_yaw_deg = math.deg(ahrs:get_yaw()) -- target heading will be kept at original heading
      target_alt_D = circle_center_pos:z() -- initialise target alt using current position (Note: down is positive)
      speed_xy = 0
      speed_z = 0
      stage = stage + 1           -- advance to next stage
      gcs:send_text(5, "Fast Descent: starting")
    end
  elseif (stage == 1) then        -- Stage1: descend

    -- get current position
    local rel_pos_home_NED = ahrs:get_relative_position_NED_home()

    -- increase circle radius
    circle_radius_rate_ms = math.min(circle_radius_rate_ms + (circle_radius_accel_mss * dt), circle_radius_rate_max_ms)   -- accelerate radius expansion
    circle_radius = math.min(circle_radius + (circle_radius_rate_ms * dt), circle_radius_max:get())  -- increase radius

    -- calculate horizontal and vertical speed
    if (circle_radius < circle_radius_max:get()) then
      speed_xy = math.max(speed_xy - (accel_xy * dt), 0)  -- decelerate horizontal speed to zero
      speed_z = math.max(speed_z - (accel_z * dt), 0)     -- decelerate vertical speed to zero
    else
      -- determine if below slowdown point
      local slowdown = false
      local stopping_distance_z = 0.5 * speed_z_max:get() * speed_z_max:get() / accel_z
      if (rel_pos_home_NED) then
        if (-rel_pos_home_NED:z() <= alt_above_home_min:get() + stopping_distance_z) then
          slowdown = true
        end
      end

      if (slowdown) then
        -- slow down vertical and then horizontal speed
        speed_z = math.max(speed_z - (accel_z * dt), math.min(speed_z_slowdown, speed_z_max:get())) -- decelerate to 0.1m/s vertically
        if speed_z <= speed_z_slowdown then
          speed_xy = math.max(speed_xy - (accel_xy * dt), 0)
        end
      else
        -- normal speed
        speed_xy = math.min(speed_xy + (accel_xy * dt), speed_xy_max:get())   -- accelerate horizontal speed to max
        speed_z = math.min(speed_z + (accel_z * dt), speed_z_max:get())       -- accelerate to max descent rate
      end
    end

    -- calculate angular velocity
    local ang_vel_rads = 0
    if (circle_radius >= circle_radius_max:get()) then
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

    -- calculate target yaw
    if yaw_behave:get() == 1 then
      target_yaw_deg = math.deg(circle_angle_rad + math.pi)
      if target_yaw_deg  > 360 then
        target_yaw_deg = target_yaw_deg - 360
      end
    end

    -- send targets to vehicle with yaw target
    vehicle:set_target_posvelaccel_NED(target_pos, target_vel, target_accel, true, target_yaw_deg, false, 0, false)

    -- advance to stage 2 when below target altitude
    if (rel_pos_home_NED) then 
      if (-rel_pos_home_NED:z() <= alt_above_home_min:get()) then
        stage = stage + 1
      end
      if (update_user) then
        gcs:send_text(5, string.format("Fast Descent: alt:%d target:%d", math.floor(-rel_pos_home_NED:z()), math.floor(alt_above_home_min:get())))
      end
    else
      gcs:send_text(5, "Fast Descent: lost position estimate, aborting")
      stage = stage + 1
    end

  elseif (stage == 2) then  -- Stage2: done!
    stage = stage + 1
    gcs:send_text(5, "Fast Descent: done!")
    if (activate_type:get() == 0) then
      -- if activated from Guided change to RTL mode
      vehicle:set_mode(copter_rtl_mode_num)
    else
      -- if activated from Auto NAV_SCRIPT_TIME then mark command as done
      vehicle:nav_script_time_done(auto_last_id)
    end
  end

  return update, interval_ms
end

return update()
