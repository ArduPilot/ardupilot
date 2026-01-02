--[[
  vehicle_control.lua: A library for advanced vehicle control in ArduPilot Lua scripting.

  This library provides high-level functions for executing complex flight patterns
  and aerobatic maneuvers. Functions are designed to be re-entrant and are managed
  by a state machine from a parent script's update() loop.
]]

local vehicle_control = {}

-- Define status constants for state machine management
vehicle_control.RUNNING = 0
vehicle_control.SUCCESS = 1
vehicle_control.ABORTED = 2

-- Define a constant for the special throttle-cut value to improve readability
vehicle_control.THROTTLE_CUT = -1

-- Enum for flip axis
vehicle_control.axis = {
  ROLL = 1,
  PITCH = 2,
}

-- Enum for vehicle modes
vehicle_control.mode = {
  LOITER = 5,
  GUIDED = 4,
  RTL = 6,
}

-- Enum for MAV_SEVERITY levels, as required by the playbook
vehicle_control.MAV_SEVERITY = {
  EMERGENCY = 0,
  ALERT = 1,
  CRITICAL = 2,
  ERROR = 3,
  WARNING = 4,
  NOTICE = 5,
  INFO = 6,
  DEBUG = 7,
}

--================================================================
-- Pattern Control
--================================================================
vehicle_control.pattern = {}

--[[
  Initializes a flight pattern by performing pre-flight checks and calculating geometry.
  @param radius_m The radius for the pattern's circular elements.
  @return A table containing start_location, center_1, and center_2, or nil and an error message.
]]
function vehicle_control.pattern.start(radius_m)
  -- Precondition checks
  if not arming:is_armed() or not vehicle:get_likely_flying() then
    return nil, "Vehicle must be armed and flying"
  end
  local current_mode = vehicle:get_mode()
  if not (current_mode == vehicle_control.mode.LOITER or current_mode == vehicle_control.mode.GUIDED) then
    return nil, "Vehicle must be in Loiter or Guided mode"
  end
  local current_loc = ahrs:get_location()
  if not current_loc then
    return nil, "Vehicle position not available"
  end

  -- Set vehicle to Guided mode for pattern execution
  if not vehicle:set_mode(vehicle_control.mode.GUIDED) then
    return nil, "Failed to set mode to Guided"
  end

  -- Calculate pattern geometry
  local start_location = current_loc:copy()
  local heading_rad = ahrs:get_yaw_rad()

  local center_1 = start_location:copy()
  center_1:offset_bearing(math.deg(heading_rad) + 90, radius_m)

  local center_2 = start_location:copy()
  center_2:offset_bearing(math.deg(heading_rad) - 90, radius_m)

  return {
    start_location = start_location,
    center_1 = center_1,
    center_2 = center_2,
  }
end

--[[
  Starts flying a circular arc.
  @return A state table for the fly_arc_update function.
]]
function vehicle_control.pattern.fly_arc_start(center_loc, start_bearing_deg, end_bearing_deg, radius_m, speed_ms, direction)
  vehicle:set_desired_speed(speed_ms)
  local total_angle_deg = (end_bearing_deg - start_bearing_deg)
  if direction > 0 and total_angle_deg < 0 then
    total_angle_deg = total_angle_deg + 360
  elseif direction < 0 and total_angle_deg > 0 then
    total_angle_deg = total_angle_deg - 360
  end

  local arc_length = math.abs(math.rad(total_angle_deg)) * radius_m
  local duration_s = arc_length / speed_ms

  return {
    start_time = millis():tofloat(),
    duration_s = duration_s,
    center_loc = center_loc,
    start_bearing_deg = start_bearing_deg,
    total_angle_deg = total_angle_deg,
    radius_m = radius_m,
  }
end

--[[
  Updates the vehicle's position along a circular arc.
  @param state The state table from fly_arc_start.
  @return RUNNING or SUCCESS.
]]
function vehicle_control.pattern.fly_arc_update(state)
  local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
  if elapsed_time >= state.duration_s then
    return vehicle_control.SUCCESS
  end

  local progress = elapsed_time / state.duration_s
  local current_bearing_deg = state.start_bearing_deg + (state.total_angle_deg * progress)
  local target_loc = state.center_loc:copy()
  target_loc:offset_bearing(current_bearing_deg, state.radius_m)

  vehicle:set_target_location(target_loc)
  return vehicle_control.RUNNING
end


--================================================================
-- Advanced Maneuvers
--================================================================
vehicle_control.maneuver = {}

-- Enum for flip maneuver stages
vehicle_control.maneuver.stage = {
  VERIFY_CLIMB = 1,
  MANUAL_CLIMB = 2,
  FLIPPING = 3,
  WAIT_FOR_DESCENT = 4,
  MANUAL_BRAKE = 5,
  RESTORING_WAIT = 6,
  DONE = 7,
}

--[[
  Starts a flip maneuver.
  Calculates the third parameter (rate, duration, or number of flips) based on the two provided.
  @param axis The axis of rotation (vehicle_control.axis.ROLL or vehicle_control.axis.PITCH).
  @param rate_degs (optional) The initial rotation rate in degrees/second. If nil, it will be calculated based on other parameters.
  @param throttle_level The throttle level (0-1) to be applied during the ballistic flip phase. Use vehicle_control.THROTTLE_CUT to cut throttle entirely.
  @param flip_duration_s (optional) The desired total duration of the maneuver. If nil, it's calculated from rate and number of flips.
  @param num_flips (optional) The number of flips to perform. If nil, it's calculated from rate and duration.
  @param slew_gain (optional) Legacy parameter, no longer used. Can be nil.
  @param true_hover_throttle (optional) The true hover throttle of the vehicle (0-1). If nil, it falls back to the MOT_THST_HOVER parameter.
  @param safety_params (optional) A table of safety limits:
      - max_drift (meters, default 10.0): Max allowed perpendicular drift from the flight path.
      - min_alt_margin (meters, default 5.0): Safety buffer below the starting altitude.
  @param climb_g (optional) The desired G-force for the initial climb (default 1.0). For example, 2.0 would be a 2g climb.
  @return A state table for the flip_update function, or nil and an error message if preconditions are not met.
]]
function vehicle_control.maneuver.flip_start(axis, rate_degs, throttle_level, flip_duration_s, num_flips, true_hover_throttle, safety_params, climb_g)
  -- 1. Pre-flight Checks
  if vehicle:get_mode() ~= vehicle_control.mode.GUIDED then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Flip requires Guided mode")
    return nil, "Flip requires Guided mode"
  end
  
  -- Use provided true hover throttle, or fall back to the parameter
  local hover_throttle
  if true_hover_throttle ~= nil and true_hover_throttle > 0 and true_hover_throttle < 1 then
    hover_throttle = true_hover_throttle
  else
    hover_throttle = param:get('MOT_THST_HOVER')
  end

  if not hover_throttle or hover_throttle <= 0 or hover_throttle >= 1 then
      return nil, "Valid hover throttle must be available"
  end

  -- 2. Calculate Flip Parameters
  local total_angle_deg
  local throttle_cmd_val = (throttle_level == vehicle_control.THROTTLE_CUT) and 0.0 or (throttle_level or 0.0)

  -- Determine which parameters were provided by the user
  local user_has_flips = (num_flips ~= nil and num_flips > 0)
  local user_has_duration = (flip_duration_s ~= nil and flip_duration_s > 0)
  local user_has_rate = (rate_degs ~= nil and rate_degs ~= 0)

  if not user_has_flips and not user_has_duration and not user_has_rate then
      return nil, "Provide at least one of: rate, duration, or num_flips"
  end

  -- Logic to ensure a whole number of flips
  if user_has_flips then
      total_angle_deg = 360 * num_flips
      if user_has_duration then
          rate_degs = total_angle_deg / flip_duration_s
      else
          -- Default duration if only num_flips is provided
          flip_duration_s = num_flips * 1.0 
          rate_degs = total_angle_deg / flip_duration_s
      end
  elseif user_has_duration and user_has_rate then
      local theoretical_flips = (math.abs(rate_degs) * flip_duration_s) / 360
      num_flips = math.floor(theoretical_flips + 0.5)
      if num_flips == 0 then num_flips = 1 end
      total_angle_deg = 360 * num_flips
      rate_degs = (total_angle_deg / flip_duration_s) * (rate_degs > 0 and 1 or -1)
  else
      num_flips = 1
      total_angle_deg = 360
      if user_has_duration then
          rate_degs = total_angle_deg / flip_duration_s
      end
  end

  if rate_degs == 0 then
      return nil, "Invalid flip rate calculated"
  end

  -- 3. Calculate True Flip Duration (t_flip) including acceleration
  local accel_param_name = (axis == vehicle_control.axis.ROLL) and 'ATC_ACCEL_R_MAX' or 'ATC_ACCEL_P_MAX'
  local accel_max_cdegs2 = param:get(accel_param_name)
  if not accel_max_cdegs2 or accel_max_cdegs2 <= 0 then
    return nil, "Could not get valid ATC_ACCEL_*_MAX parameter"
  end
  local accel_max_degs2 = accel_max_cdegs2 / 100.0

  local time_to_reach_rate = math.abs(rate_degs) / accel_max_degs2
  local angle_during_accel = 0.5 * accel_max_degs2 * time_to_reach_rate^2
  
  local t_flip
  if 2 * angle_during_accel >= total_angle_deg then
    -- Maneuver is purely acceleration and deceleration (bang-bang)
    t_flip = 2 * math.sqrt(total_angle_deg / accel_max_degs2)
  else
    -- Trapezoidal profile (accel, const vel, decel)
    local angle_at_const_vel = total_angle_deg - (2 * angle_during_accel)
    local time_at_const_vel = angle_at_const_vel / math.abs(rate_degs)
    t_flip = (2 * time_to_reach_rate) + time_at_const_vel
  end

  -- 4. Calculate Manual Climb Parameters using the new t_flip
  local climb_g_force = climb_g or 1.0
  local net_downward_accel_during_flip = 9.81 * (1 - (throttle_cmd_val / hover_throttle))
  local required_vz = 0.5 * net_downward_accel_during_flip * t_flip
  
  -- Calculate climb throttle and duration based on desired G-force
  local climb_accel_ms2 = climb_g_force * 9.81
  local climb_throttle = math.min((climb_g_force + 1) * hover_throttle, 0.8)
  local t_accel = required_vz / climb_accel_ms2

  -- 5. Prepare for Flip
  local initial_attitude_euler = Vector3f()
  initial_attitude_euler:x(ahrs:get_roll_rad())
  initial_attitude_euler:y(ahrs:get_pitch_rad())
  initial_attitude_euler:z(ahrs:get_yaw_rad())
  
  local initial_throttle = motors:get_throttle()
  if not initial_throttle then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Could not get initial throttle")
    return nil, "Could not get initial throttle"
  end

  local initial_location = ahrs:get_location()
  local initial_pos_ned = ahrs:get_relative_position_NED_origin()
  if not initial_location or not initial_pos_ned then
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Could not get EKF origin position")
    return nil, "Could not get EKF origin position"
  end
  local initial_state = { attitude = initial_attitude_euler, velocity = ahrs:get_velocity_NED(), location = initial_location, pos_ned = initial_pos_ned, throttle = initial_throttle }

  -- 6. Safety Parameter Setup
  local safety = safety_params or {}
  local min_alt_margin = safety.min_alt_margin or 5.0 -- meters
  local max_drift = safety.max_drift or 10.0 -- meters

  -- Check that the altitude margin is safe relative to the ground
  -- Use altitude relative to home as a proxy for AGL. Down is positive, so negate it.
  local initial_rel_alt_m = -ahrs:get_relative_position_D_home()
  if initial_rel_alt_m then
    local min_safe_hagl
    if initial_rel_alt_m > 10.0 then
        min_safe_hagl = 5.0
    else
        min_safe_hagl = initial_rel_alt_m / 2.0
    end
    
    if min_alt_margin >= (initial_rel_alt_m - min_safe_hagl) then
      local old_margin = min_alt_margin
      min_alt_margin = math.max(0, initial_rel_alt_m - min_safe_hagl)
      -- Only show the warning if the user explicitly provided an unsafe margin
      if safety.min_alt_margin ~= nil then
          local msg = string.format("Alt margin %.1fm unsafe (Rel Alt %.1fm). Clamped to %.1fm", old_margin, initial_rel_alt_m, min_alt_margin)
          gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, msg)
      end
    end
  elseif min_alt_margin > 0 then
    -- Could not get relative altitude, which is unusual. Warn the user.
    gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "No relative altitude for safety check on altitude margin.")
  end

  local initial_angle = (axis == vehicle_control.axis.ROLL) and math.deg(initial_state.attitude:x()) or math.deg(initial_state.attitude:y())

  return {
    stage = vehicle_control.maneuver.stage.VERIFY_CLIMB,
    initial_state = initial_state,
    t_accel = t_accel,
    required_vz = required_vz,
    climb_throttle = climb_throttle,
    t_flip = t_flip,
    total_angle_deg = total_angle_deg,
    rate_degs = rate_degs,
    axis = axis,
    throttle_cmd = throttle_cmd_val,
    last_angle = initial_angle,
    accumulated_angle = 0,
    num_flips = num_flips,
    hover_throttle = hover_throttle,
    accel_max_degs2 = accel_max_degs2,
    safety_max_drift = max_drift,
    safety_min_alt_margin_m = min_alt_margin,
  }
end

--[[
  Internal helper to monitor the vehicle's state during a flip and trigger an abort if safety limits are exceeded.
  @param state The state table for the maneuver.
  @param reset_fn The function to call to safely abort.
  @return true if an abort was triggered, false otherwise.
]]
local function _check_flip_safety(state, reset_fn)
  local current_pos_ned = ahrs:get_relative_position_NED_origin()
  local current_vel_ned = ahrs:get_velocity_NED()
  local current_loc = ahrs:get_location()

  -- Abort if we can't get basic state information
  if not current_pos_ned or not current_vel_ned or not current_loc then
    gcs:send_text(vehicle_control.MAV_SEVERITY.CRITICAL, "Safety Abort: Lost position/velocity.")
    if reset_fn then reset_fn(true) end
    return true
  end
  
  -- Define the safety altitude floor in cm
  local start_alt_cm = state.initial_state.location:alt()
  local abort_alt_cm = start_alt_cm - (state.safety_min_alt_margin_m * 100)
  
  -- 1. Horizontal drift check (crosstrack error)
  local initial_vel_3d = state.initial_state.velocity:copy()
  initial_vel_3d:z(0) -- Zero out Z for 3D calculation
  local drift
  if initial_vel_3d:length() < 0.1 then
      -- If stationary, use simple radial distance
      drift = (current_pos_ned - state.initial_state.pos_ned):xy():length()
  else
      -- If moving, calculate crosstrack error
      local displacement_3d = current_pos_ned - state.initial_state.pos_ned
      displacement_3d:z(0) -- Zero out Z for 3D calculation
      
      local vel_dir_3d = initial_vel_3d:copy()
      vel_dir_3d:normalize()
      
      local projected_3d = vel_dir_3d:scale(displacement_3d:dot(vel_dir_3d))
      drift = (displacement_3d - projected_3d):length()
  end

  if drift > state.safety_max_drift then
    local msg = string.format("Safety Abort: Drift %.1fm > %.1fm", drift, state.safety_max_drift)
    gcs:send_text(vehicle_control.MAV_SEVERITY.CRITICAL, msg)
    if reset_fn then reset_fn(true) end
    return true
  end

  -- 2. Altitude floor check (only active during ballistic/leveling phase)
  if state.stage >= vehicle_control.maneuver.stage.FLIPPING and state.stage <= vehicle_control.maneuver.stage.WAIT_FOR_DESCENT then
      if current_loc:alt() < abort_alt_cm then
        local msg
        if ahrs:home_is_set() then
            local home_alt_cm = ahrs:get_home():alt()
            local current_alt_ahd = (current_loc:alt() - home_alt_cm) / 100.0
            local floor_alt_ahd = (abort_alt_cm - home_alt_cm) / 100.0
            msg = string.format("Safety Abort: Alt AHD %.1fm < floor %.1fm", current_alt_ahd, floor_alt_ahd)
        else
            msg = string.format("Safety Abort: Alt %.1fm < floor %.1fm", current_loc:alt()/100.0, abort_alt_cm/100.0)
        end
        gcs:send_text(vehicle_control.MAV_SEVERITY.CRITICAL, msg)
        if reset_fn then reset_fn(true) end
        return true
      end
  end

  return false -- No safety limits breached
end

--[[
  Updates the flip maneuver state machine.
  @param state The state table from flip_start.
  @param reset_fn A function to call to safely abort the maneuver.
  @return RUNNING, SUCCESS, or ABORTED.
]]
function vehicle_control.maneuver.flip_update(state, reset_fn)
  -- Perform safety checks first on every update cycle
  if state.stage > vehicle_control.maneuver.stage.MANUAL_CLIMB and _check_flip_safety(state, reset_fn) then
    return vehicle_control.ABORTED
  end
  
  -- Get initial yaw for use in attitude commands
  local initial_yaw_deg = math.deg(state.initial_state.attitude:z())

  if state.stage == vehicle_control.maneuver.stage.VERIFY_CLIMB then
    -- Stage 1: VERIFY_CLIMB
    -- Apply climb thrust for a short duration to measure actual acceleration,
    -- then adjust the climb parameters to match reality before proceeding.
    if not state.start_time then
        state.start_time = millis():tofloat()
        state.initial_vz = -ahrs:get_velocity_NED():z()
    end
    
    -- Command a level attitude with high throttle to get a clean vertical acceleration measurement
    vehicle:set_target_angle_and_rate_and_throttle(0, 0, initial_yaw_deg, 0, 0, 0, state.climb_throttle)

    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    local verification_duration = 0.2 -- 200ms to measure acceleration

    if elapsed_time >= verification_duration then
        local final_vz = -ahrs:get_velocity_NED():z()
        local measured_accel = (final_vz - state.initial_vz) / elapsed_time

        -- Only correct if we have a sensible measured acceleration
        if measured_accel > 1.0 then -- Must have at least 1m/s/s of positive acceleration
            -- Recalculate the time to accelerate using the measured acceleration, but keep throttle high
            state.t_accel = state.required_vz / measured_accel
            
            local msg = string.format("Climb time corrected to %.2fs", state.t_accel)
            gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, msg)
            
            -- Transition to the main climb stage
            state.stage = vehicle_control.maneuver.stage.MANUAL_CLIMB
            state.start_time = nil -- Reset start time for the next stage
        else
            gcs:send_text(vehicle_control.MAV_SEVERITY.CRITICAL, "Climb verification failed, ABORTING.")
            if reset_fn then
                reset_fn(true) -- Pass true to indicate an abort
            end
            return vehicle_control.ABORTED -- Terminate the maneuver
        end
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.MANUAL_CLIMB then
    -- Stage 2: MANUAL_CLIMB
    -- Apply a strong upward thrust for a calculated duration to gain the
    -- required vertical velocity for the ballistic phase (the flip).
    if not state.start_time then
        state.start_time = millis():tofloat()
    end
    
    -- Command a level attitude with high throttle
    vehicle:set_target_angle_and_rate_and_throttle(0, 0, initial_yaw_deg, 0, 0, 0, state.climb_throttle)

    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    if elapsed_time >= state.t_accel then
      local flip_msg = string.format("Flipping %.2f times over %.2f seconds", state.num_flips, state.t_flip)
      gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, flip_msg)
      state.stage = vehicle_control.maneuver.stage.FLIPPING
      state.start_time = millis():tofloat()
      state.apex_location = ahrs:get_location() -- Record altitude at the start of the flip

      -- Start the flip with the specified throttle and rates
      local roll_rate_dps, pitch_rate_dps = 0, 0
      if state.axis == vehicle_control.axis.ROLL then
          roll_rate_dps = state.rate_degs
      else
          pitch_rate_dps = state.rate_degs
      end
      -- Command a level attitude while initiating the roll/pitch rate
      vehicle:set_target_angle_and_rate_and_throttle(0, 0, initial_yaw_deg, roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.FLIPPING then
    -- Stage 3: FLIPPING
    -- The vehicle is now in its ballistic phase. This stage commands the
    -- desired rotation rate and monitors the accumulated angle.
    
    -- Unwrap angle to track total rotation
    local current_angle = (state.axis == vehicle_control.axis.ROLL) and math.deg(ahrs:get_roll_rad()) or math.deg(ahrs:get_pitch_rad())
    local delta_angle = current_angle - state.last_angle
    if delta_angle > 180 then delta_angle = delta_angle - 360 elseif delta_angle < -180 then delta_angle = delta_angle + 360 end
    state.accumulated_angle = state.accumulated_angle + delta_angle
    state.last_angle = current_angle

    -- New logic: Start leveling when on the last flip and past the halfway point.
    local flips_completed = math.floor(math.abs(state.accumulated_angle) / 360)
    
    if flips_completed >= (state.num_flips - 1) then
        local angle_into_current_flip = math.abs(state.accumulated_angle) - (flips_completed * 360)
        if angle_into_current_flip >= 180 then
            gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Flip >180 deg, leveling.")
            state.stage = vehicle_control.maneuver.stage.WAIT_FOR_DESCENT
            return vehicle_control.RUNNING
        end
    end

    -- The original check for total angle completion is kept as a failsafe.
    if math.abs(state.accumulated_angle) >= state.total_angle_deg then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Flip rotation complete (failsafe), leveling.")
        state.stage = vehicle_control.maneuver.stage.WAIT_FOR_DESCENT
        return vehicle_control.RUNNING
    end

    -- Command the fixed target rate
    local roll_rate_dps, pitch_rate_dps = 0, 0
    if state.axis == vehicle_control.axis.ROLL then
      roll_rate_dps = state.rate_degs
    else
      pitch_rate_dps = state.rate_degs
    end
    vehicle:set_target_rate_and_throttle(roll_rate_dps, pitch_rate_dps, 0, state.throttle_cmd)
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.WAIT_FOR_DESCENT then
    -- Stage 4: WAIT_FOR_DESCENT
    -- This stage holds a level attitude and waits for the vehicle to descend
    -- back to its apex altitude before braking.
    vehicle:set_target_angle_and_rate_and_throttle(0, 0, initial_yaw_deg, 0, 0, 0, state.throttle_cmd)

    local current_loc = ahrs:get_location()
    if current_loc and state.apex_location and (current_loc:alt() <= state.apex_location:alt()) then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "At apex altitude, applying brake.")
        state.stage = vehicle_control.maneuver.stage.MANUAL_BRAKE
        state.start_time = millis():tofloat() -- Reset start_time for the brake timer
    end
    return vehicle_control.RUNNING

  elseif state.stage == vehicle_control.maneuver.stage.MANUAL_BRAKE then
    -- Stage 5: MANUAL_BRAKE
    -- This stage applies the symmetrical braking thrust. It commands a high
    -- throttle for the same duration as the initial climb. It exits early
    -- if the vehicle's vertical velocity is already restored, or as a backup,
    -- when the timer expires.
    vehicle:set_target_angle_and_rate_and_throttle(0, 0, initial_yaw_deg, 0, 0, 0, state.climb_throttle)

    -- Check if we have already recovered our initial vertical velocity
    local current_vel_ned = ahrs:get_velocity_NED()
    local brake_finished = false
    if current_vel_ned and -current_vel_ned:z() >= -state.initial_state.velocity:z() then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Brake complete (velocity met), restoring trajectory.")
        brake_finished = true
    end

    -- Check if the timer has expired (backup)
    local elapsed_time = (millis():tofloat() - state.start_time) / 1000.0
    if not brake_finished and elapsed_time >= state.t_accel then
        gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Brake complete (timer expired), restoring trajectory.")
        brake_finished = true
    end
    
    if brake_finished then
        state.restore_start_time = millis():tofloat()
        state.stage = vehicle_control.maneuver.stage.RESTORING_WAIT
        vehicle:set_target_rate_and_throttle(0, 0, 0, state.initial_state.throttle)
    end
    return vehicle_control.RUNNING
    
  elseif state.stage == vehicle_control.maneuver.stage.RESTORING_WAIT then
    -- Stage 6: RESTORING_WAIT
    -- The vehicle is now stable. This final stage commands the vehicle to
    -- return to its projected flight path and recover its original velocity.
    
    -- This calculation is only done once when entering the stage
    if not state.restore_target_pos then
        -- Calculate the total time the vehicle was "off-track" (ballistic + braking)
        local off_track_duration_s = state.t_flip + state.t_accel
        -- Project the initial position forward by the off-track duration
        local displacement = state.initial_state.velocity:copy():scale(off_track_duration_s)
        state.restore_target_pos = state.initial_state.pos_ned + displacement
    end

    -- Command the vehicle to the projected position and to resume its initial velocity and yaw
    local zero_accel = Vector3f()
    vehicle:set_target_posvelaccel_NED(state.restore_target_pos, state.initial_state.velocity, zero_accel, true, initial_yaw_deg, false, 0, false)
    
    -- Check for arrival
    local current_pos_ned = ahrs:get_relative_position_NED_origin()
    local current_vel_ned = ahrs:get_velocity_NED()

    if not current_pos_ned or not current_vel_ned then
      return vehicle_control.RUNNING
    end

    -- Check for timeout
    local elapsed_restore_time_ms = millis():tofloat() - state.restore_start_time
    if elapsed_restore_time_ms > 5000 then
        gcs:send_text(vehicle_control.MAV_SEVERITY.WARNING, "Restore timed out, completing maneuver.")
        state.stage = vehicle_control.maneuver.stage.DONE
        return vehicle_control.SUCCESS
    end

    local pos_error = (state.restore_target_pos - current_pos_ned):length()
    local vel_error = (state.initial_state.velocity - current_vel_ned):length()

    if pos_error < 1.0 and vel_error < 0.2 then
      gcs:send_text(vehicle_control.MAV_SEVERITY.INFO, "Trajectory restored.")
      state.stage = vehicle_control.maneuver.stage.DONE
      return vehicle_control.SUCCESS
    end

    return vehicle_control.RUNNING
  end

  return vehicle_control.SUCCESS
end


--================================================================
-- Utility Functions
--================================================================
vehicle_control.utils = {}

--[[
  Checks if the vehicle has arrived at a target location.
  @param target_location The destination Location object.
  @param tolerance_m The arrival radius in meters.
  @return true if arrived, false otherwise.
]]
function vehicle_control.utils.has_arrived(target_location, tolerance_m)
  local current_loc = ahrs:get_location()
  if current_loc and current_loc:get_distance(target_location) < tolerance_m then
    return true
  end
  return false
end

return vehicle_control

