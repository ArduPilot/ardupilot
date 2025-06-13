-- command a Copter to climb a wall at a specific distance
--
-- CAUTION: This script only works for Copter
--    a) waits for the vehicle to be switched to Guided mode (does nothing)
--    b) accepts pilot roll, pitch, throttle and yaw input
--    c) manual mode
--        - roll and pitch cause vehicle to fly horizontally in body frame at up to WP_SPEED
--        - yaw controls turn rate
--        - throttle controls climb/descent rate at up to PILOT_SPEED_UP/DOWN?  (WP_SPEED_UP, WPNAV_SPEED_DN?)
--    d) autonomous mode
--        - roll is unchanged from manual mode
--        - yaw is unchanged from manual mode
--        - pitch controls target distance to wall (limited to no less than 2m, no more than 8m)
--        - throttle input causes vehicle to switch back to manual mode
--        - climbs at 50cm/s (need parameter?) stopping at 2m intervals
--    e) ZIGZAG_SaveWP aux switch controls switching in and out of auto mode

-- constants
local update_rate_ms = 10           -- script updates at 100hz
local update_rate_dt = update_rate_ms / 1000.0 -- update rate in seconds
local copter_guided_mode_num = 4    -- Copter's guided flight mode is mode 4
local aux_switch_function = 61      -- auxiliary switch function controlling mode.  61 is ZIGZAG_SaveWP
local climb_accel_max = 0.25        -- climb rate acceleration max in m/s/s
local climb_rate_max = 2            -- climb rate max in m/s
local climb_rate_chg_max = climb_accel_max * update_rate_dt -- max change in climb rate in a single iteration
local roll_pitch_speed_max = 2      -- horizontal speed max in m/s
local roll_pitch_accel_max = 0.25      -- horizontal acceleration max in m/s/s
local roll_pitch_speed_chg_max = roll_pitch_accel_max * update_rate_dt  -- max change in roll or pitch speed in a single iteration
local rc_roll_ch = 1                -- RC input channel for roll
local rc_pitch_ch = 2               -- RC input channel for pitch
local rc_throttle_ch = 3            -- RC input channel for throttle
local rc_pwm_dz = 50                -- RC input deadzone in pwm
local wall_pitch_speed_max = roll_pitch_speed_max / 2.0 -- wall control will never overpower pilot
local wall_dist_to_speed_P = 0.5    -- P gain for converting distance to wall to forward speed
local climb_pause_sec = 3           -- pause for this many seconds during each interval
local climb_pause_counter_max = climb_pause_sec / update_rate_dt -- pause control's loop counter's initial value

-- user parmeters
local climb_dist_max = 10           -- vehicle moves to next lane at this many meters above home
local climb_stop_interval = 1       -- vehicle stops after climbing this many meters
local lane_width = 2                -- each lane is 2m apart horizontally

-- lane variables
local lane_number = 0               -- current lane number
local lane_shift_roll_speed = 1     -- vehicle moves right at this speed in m/s
local lane_shift_counter_max = lane_width / (lane_shift_roll_speed * update_rate_dt)
local lane_shift_counter = 0        -- current lane shift counter value
local lane_climb_rate = climb_rate_max / 2.0 -- autonomous climb is always 1/2 maximum

-- global variables
local aux_sw_pos_prev = 0           -- aux switch previous position
local climb_mode = 0                -- 0:manual, 1:automatic climb
local climb_rate = 0                -- current climb rate (negative is down, positive is up)
local climb_total = 0               -- total number of meters above home
local climb_interval_total = 0      -- meters climbed since last stop
local climb_pause_counter = 0       -- current pause counter value
local roll_speed = 0                -- target horizontal roll speed in m/s
local pitch_speed = 0               -- target horizontal pitch speed in m/s
local wall_dist_target = 5          -- target distance to wall

-- get the maximum speed in order to decelerate to zero within the given distance
function get_speed_max(distance, accel_max)
  return math.sqrt(2.0 * math.max(0, distance) * accel_max)
end

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()

  -- reset state when disarmed
  if not arming:is_armed() then
    climb_mode = 0
    climb_rate = 0
    climb_total = 0
    climb_pause_counter = 0
    climb_interval_total = 0
    roll_speed = 0
    pitch_speed = 0
    lane_number = 0
    lane_shift_counter = 0
    lane_climb_rate = math.abs(lane_climb_rate)
    return update, update_rate_ms
  end

  -- if not in Guided mode do nothing
  if vehicle:get_mode() ~= copter_guided_mode_num then
    return update, update_rate_ms
  end

  -- read switch input from RCx_FUNCTION
  local aux_switch = rc:find_channel_for_option(aux_switch_function)
  if aux_switch then
    local sw_pos = aux_switch:get_aux_switch_pos()
    if (sw_pos ~= aux_sw_pos_prev) then
      -- only process input if switch position changed
      aux_sw_pos_prev = sw_pos
      if (sw_pos == 0) then
        -- pilot has selected manual mode
        if (climb_mode ~= 0) then
          gcs:send_text(0, "WallClimb: pilot switch to Manual")
          climb_mode = 0
        end
      elseif (sw_pos == 2) then
        -- pilot has selected auto mode
        if (climb_mode ~= 1) then
          gcs:send_text(0, "WallClimb: pilot switch to Auto")
          climb_mode = 1
        end
      end
    end
  end

  -- read normalised roll, pitch and throttle input
  local roll_input = 0
  local rc_roll_pwm = rc:get_pwm(rc_roll_ch)
  if (rc_roll_pwm and rc_roll_pwm >= 1000 and rc_roll_pwm <= 2000 and (math.abs(rc_roll_pwm - 1500) > rc_pwm_dz)) then
    roll_input = (rc_roll_pwm - 1500) / 500.0
  end
  local pitch_input = 0
  local rc_pitch_pwm = rc:get_pwm(rc_pitch_ch)
  if (rc_pitch_pwm and rc_pitch_pwm >= 1000 and rc_pitch_pwm <= 2000 and (math.abs(rc_pitch_pwm - 1500) > rc_pwm_dz)) then
    pitch_input = (rc_pitch_pwm - 1500) / 500.0
  end
  local throttle_input = 0
  local rc_throttle_pwm = rc:get_pwm(rc_throttle_ch)
  if (rc_throttle_pwm and rc_throttle_pwm >= 1000 and rc_throttle_pwm <= 2000 and (math.abs(rc_throttle_pwm - 1500) > rc_pwm_dz)) then
    throttle_input = (rc_throttle_pwm - 1500) / 500.0
  end

  -- manual mode
  if (climb_mode == 0) then
    -- update target climb rate
    local climb_rate_target = throttle_input * climb_rate_max
    climb_rate = math.min(climb_rate_target, climb_rate + climb_rate_chg_max, climb_rate_max)
    climb_rate = math.max(climb_rate_target, climb_rate - climb_rate_chg_max, -climb_rate_max)
  end

  -- autonomous mode
  local wall_pitch_speed = 0
  local wall_roll_speed = 0
  if (climb_mode == 1) then
    -- convert rangefinder distance to pitch speed
    if rangefinder:has_data_orient(0) then
      local distance_m = rangefinder:distance_orient(0)
      wall_pitch_speed = (distance_m - wall_dist_target) * wall_dist_to_speed_P
      wall_pitch_speed = math.min(wall_pitch_speed, wall_pitch_speed_max)
      wall_pitch_speed = math.max(wall_pitch_speed, -wall_pitch_speed_max)
    end

    -- switch to manual if pilot provides throttle input
    if (math.abs(throttle_input) ~= 0) then
      gcs:send_text(0, "WallClimb: throttle input, switch to Manual")
      climb_mode = 0
    else
      -- update distance climbed
      local climb_chg = climb_rate * update_rate_dt
      climb_interval_total = climb_interval_total + climb_chg
      climb_total = climb_total + climb_chg

      -- determine if we should pause
      local dist_to_interval = climb_stop_interval - math.abs(climb_interval_total)
      if (dist_to_interval <= 0) then
        gcs:send_text(0, "WallClimb: pausing at " .. string.format("%3.1f", climb_total) .. "m")
        climb_pause_counter = climb_pause_counter_max
        climb_interval_total = 0

        -- determine if we should move to next lane
        if ((climb_total > climb_dist_max) or (climb_total <= 0)) then
          lane_number = lane_number + 1
          lane_climb_rate = -lane_climb_rate
          lane_shift_counter = lane_shift_counter_max
          gcs:send_text(0, "WallClimb: starting lane " .. tostring(lane_number))
        end
      end

      -- default target climb rate to lane climb rate
      local climb_rate_target = lane_climb_rate

      -- limit target speed so vehicle can stop before next interval
	  -- speed limit is always at least 0.1m/s so copter doesn't get stuck near interval
      local speed_max = math.max(get_speed_max(dist_to_interval, climb_accel_max), 0.1)
      climb_rate_target = math.max(climb_rate_target, -speed_max)
      climb_rate_target = math.min(climb_rate_target, speed_max)

      -- if paused set target climb rate to zero
      if (climb_pause_counter > 0) then
        climb_pause_counter = climb_pause_counter - 1
        climb_rate_target = 0
        if (climb_pause_counter == 0) then
          gcs:send_text(0, "WallClimb: restarting")
        end
      end

      -- if shifting lanes pause climb and roll right
      if (lane_shift_counter > 0) then
        lane_shift_counter = lane_shift_counter - 1
        wall_roll_speed = lane_shift_roll_speed
        climb_rate_target = 0
        -- if completing lane shift trigger another pause
        if (lane_shift_counter == 0) then
          climb_pause_counter = climb_pause_counter_max
        end
      end

      -- calculate acceleration limited climb rate
      if (climb_rate_target >= climb_rate) then
        climb_rate = math.min(climb_rate_target, climb_rate + climb_rate_chg_max, climb_rate_max)
      else
        climb_rate = math.max(climb_rate_target, climb_rate - climb_rate_chg_max, -climb_rate_max)
      end
    end
  end

  -- update target roll using both pilot and autonomous control (+ve right, -ve left)
  local roll_speed_target = roll_input * roll_pitch_speed_max + wall_roll_speed
  roll_speed = math.min(roll_speed_target, roll_speed + roll_pitch_speed_chg_max, roll_pitch_speed_max)
  roll_speed = math.max(roll_speed_target, roll_speed - roll_pitch_speed_chg_max, -roll_pitch_speed_max)

  -- update target pitch using both pilot and autonomous control (+ve forward, -ve backwards)
  local pitch_speed_target = -pitch_input * roll_pitch_speed_max + wall_pitch_speed
  pitch_speed = math.min(pitch_speed_target, pitch_speed + roll_pitch_speed_chg_max, roll_pitch_speed_max)
  pitch_speed = math.max(pitch_speed_target, pitch_speed - roll_pitch_speed_chg_max, -roll_pitch_speed_max)

  -- convert targets from body to earth frame and send to guided mode velocity controller
  local yaw_rad = ahrs:get_yaw_rad()
  yaw_cos = math.cos(yaw_rad)
  yaw_sin = math.sin(yaw_rad)
  local target_vel_ned = Vector3f()
  target_vel_ned:x(pitch_speed * yaw_cos - roll_speed * yaw_sin)
  target_vel_ned:y(pitch_speed * yaw_sin + roll_speed * yaw_cos)
  target_vel_ned:z(-climb_rate)
  if not (vehicle:set_target_velocity_NED(target_vel_ned)) then
    gcs:send_text(0, "failed to execute velocity command")
  end

  return update, update_rate_ms
end

return update()
