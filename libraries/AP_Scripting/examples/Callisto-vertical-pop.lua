-- Copter pops up a a constant target rate and returns to starting position
--
-- CAUTION: This script only works for Copter 4.2 (and higher)
-- this script waits for channel input

-- Switch position:
--    low) abort to loiter if running
--    mid) returns to starting point or ready
--    high) increase altitude at constant rate until time out, over current or pilot input

-- Parameters:
--    Enable: Enables the feature
--    Speed: Target climb rate
--    Time: Maximum Time before return is triggered
--    Current: Maximum Current before return is triggered. Set to zero to disable
--    Vertical Offset from starting altitude

-- Procedure:
--  1. Aircraft should be positioned in Loiter with load attached and under tension ready to apply tension to release.
--  2. Switch must be in the MID position.
--  3. Switch is moved to the High position.
--  4. Aircraft changes to Guided Mode
--  5. Aircraft will climb at POP_SPEED until:
--    1. POP_TIME seconds have passed.
--    2. POP_CURRENT amps have been exceeded.
--    3. Switch is moved back to the MID position.
--    4. Process is aborted, causing switch immediately to Loiter Mode, by:
--      1. Moving Switch to the LOW position.
--      2. Pilot moves roll, pitch, yaw or throttle sticks.
--  6. Aircraft then returns to the starting altitude plus POS_OFFSET
--  7. Aircraft changes back to Loiter Mode.

-- constants
local copter_guided_mode_num = 4    -- Guided mode is 4 on copter
local copter_loiter_mode_num = 5    -- Loiter is 5 on copter
local SELECT_RC_OPTION = 300        -- rc channel option used to trigger pop. RCx_OPTION = 300 (scripting1)
local MAV_SEVERITY_INFO = 6

-- timing and state machine variables
local stage = 0                    -- stage of descent
local start_ms                     -- system time of start time
local interval_ms = 100            -- update interval in ms
local batt_instance = 0

-- control related variables
local start_pos = Vector3f()        -- center of circle position as an offset from EKF origin

local PARAM_TABLE_KEY = 75
local PARAM_TABLE_PREFIX = "POP_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- pilot inputs
local RCMAP_ROLL      = bind_param("RCMAP_ROLL")
local RCMAP_PITCH     = bind_param("RCMAP_PITCH")
local RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
local RCMAP_YAW       = bind_param("RCMAP_YAW")
local RCIN_ROLL       = rc:get_channel(RCMAP_ROLL:get())
local RCIN_PITCH      = rc:get_channel(RCMAP_PITCH:get())
local RCIN_THRTOTTLE  = rc:get_channel(RCMAP_THROTTLE:get())
local RCIN_YAW    = rc:get_channel(RCMAP_YAW:get())

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

--[[
  // @Param: _ENABLE
  // @DisplayName: enable
  // @Description: Enable system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local enable = bind_add_param('ENABLE', 1, 0) 

--[[
  // @Param: _SPEED
  // @DisplayName: Climb Speed
  // @Description: Target climb rate
  // @Range: 0.1 2.0
  // @Units: m/s
  // @User: Standard
--]]
local speed_z_max = bind_add_param('SPEED', 2, 0.25) 

--[[
  // @Param: _TIME
  // @DisplayName: Maximum Time
  // @Description: Maximum Time before return is triggered.
  // @Range: 0.1 10.0
  // @Units: s
  // @User: Standard
--]]
local time_out = bind_add_param('TIME', 3, 5) 

--[[
  // @Param: _CURRENT
  // @DisplayName: Maximum Current
  // @Description: Maximum Current before return is triggered. Set to zero to disable
  // @Range: 0 400
  // @Units: A
  // @User: Standard
--]]
local current_max = bind_add_param('CURRENT', 4, 0) 

--[[
  // @Param: _OFFSET
  // @DisplayName: Vertical Offset
  // @Description: Vertical Offset from starting altitude
  // @Range: 0 2.0
  // @Units: m
  // @User: Standard
--]]
local alt_pop = bind_add_param('OFFSET', 5, 0.25) 

gcs:send_text(MAV_SEVERITY_INFO, "hello, pop") -- send the traditional message

-- check for pilot input
function have_pilot_input()
  if (math.abs(RCIN_ROLL:norm_input_dz()) > 0 or
      math.abs(RCIN_PITCH:norm_input_dz()) > 0 or
      math.abs(RCIN_THRTOTTLE:norm_input_dz()) > 0 or
      math.abs(RCIN_YAW:norm_input_dz()) > 0) then
     return true
  end
  return false
end

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if enable:get() == 1 then
    local now_ms = millis()
    aux_pos = rc:get_aux_cached(SELECT_RC_OPTION)
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_location()

    if not arming:is_armed() then -- reset state when disarmed
      if stage ~= 0 then
        stage = 0
        -- gcs:send_text(MAV_SEVERITY_INFO, "pop: stage 0") -- state must be activated by low switch position
      end
    elseif vehicle:get_mode() ~= copter_loiter_mode_num and vehicle:get_mode() ~= copter_guided_mode_num then -- reset state when not in loiter or guided
      if stage ~= 1 then
        stage = 1
        -- gcs:send_text(MAV_SEVERITY_INFO, "pop: stage 1") -- state must be activated by low switch position in loiter
      end
    else

      if vehicle:get_mode() == copter_loiter_mode_num then
        if aux_pos and aux_pos == 0 then -- in abort position
          if stage ~= 0 then
            stage = 0
            gcs:send_text(MAV_SEVERITY_INFO, "pop: Abort Position") -- pop is in Abort position
          end
        end
        if aux_pos and aux_pos == 1 then -- ready to initiate pop
          if stage ~= 1 then
            stage = 1
            gcs:send_text(MAV_SEVERITY_INFO, "pop: Ready") -- pop is ready to initialise
          end
        end
        if not have_pilot_input() and stage == 1 then
          if aux_pos and aux_pos == 2 then
            if home and curr_loc then
              start_pos = ahrs:get_relative_position_NED_origin()
              if (vehicle:set_mode(copter_guided_mode_num)) then -- change to Guided mode
                stage = 2
                last_update_ms = now_ms
                -- gcs:send_text(MAV_SEVERITY_INFO, "pop: stage 2") -- starting to climb
                gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: Starting at Alt:%f", (start_pos:z())))
              end
            end
          end
        end

      elseif vehicle:get_mode() == copter_guided_mode_num and stage > 1 then
        -- first check for abort conditions
        if aux_pos and aux_pos == 0 then -- abort from low pop switch
          if (vehicle:set_mode(copter_loiter_mode_num)) then
            stage = 0
            gcs:send_text(MAV_SEVERITY_INFO, "pop: abort - switch") -- change to loiter and reset
          end

        elseif have_pilot_input() then
          if (vehicle:set_mode(copter_loiter_mode_num)) then
            stage = 0
            gcs:send_text(MAV_SEVERITY_INFO, "pop: abort- pilot input") -- change to loiter and reset
          end

        elseif stage == 2 then -- increasing altitude at set rate
          target_vel = Vector3f()
          target_vel:z(-speed_z_max:get())
          target_accel = Vector3f()
          vehicle:set_target_velaccel_NED(target_vel, target_accel, false, 0, true, 0, false)
          if now_ms - last_update_ms >= time_out:get() * 1000 then
            stage = 3
            gcs:send_text(MAV_SEVERITY_INFO, "pop: return - time out") -- return to starting point + offset
            --gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: stage 3: test:%f", (now_ms:tofloat() - last_update_ms:tofloat())))
          end
          local amps = battery:current_amps(batt_instance)
          if amps ~= nil and current_max:get() > 0 then
            -- gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: Current:%.01f", amps))
            if amps > current_max:get() then -- over current detected
              stage = 3
              gcs:send_text(MAV_SEVERITY_INFO, "pop: return - over current") -- return to starting point + offset
            end
          end
          if aux_pos and aux_pos == 1 then -- return from low pop switch
            stage = 3
            gcs:send_text(MAV_SEVERITY_INFO, "pop: return - switch") -- change to loiter and reset
          end

        elseif stage == 3 then -- decreasing altitude to starting altitude
          target_pos = Vector3f()
          target_pos:x(start_pos:x())
          target_pos:y(start_pos:y())
          target_pos:z(start_pos:z() - alt_pop:get())
          target_vel = Vector3f()
          target_accel = Vector3f()
          vehicle:set_target_posvelaccel_NED(target_pos, target_vel, target_accel, false, 0, true, 0, false)
          if home and curr_loc then
            current_pos = ahrs:get_relative_position_NED_origin()
            -- gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: Start:%f Offset:%f Alt:%f", start_pos:z(), alt_pop:get(), current_pos:z()))
            if target_pos:z() - 0.1 < current_pos:z() then -- less than 10cm from target altitude
              if (vehicle:set_mode(copter_loiter_mode_num)) then  -- change to Loiter mode
                stage = 0
                -- gcs:send_text(MAV_SEVERITY_INFO, "pop: stage 0") -- returned to starting point + offset
                -- gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: stage 0: Alt:%f", current_pos:z()))
                gcs:send_text(MAV_SEVERITY_INFO, string.format("pop: Finished at Alt:%f", (target_pos:z())))
              end
            end
          end
        end

      else
        gcs:send_text(MAV_SEVERITY_INFO, "pop: error")
        stage = 0 -- should not get here
      end
    end
  end

  return update, interval_ms
end

return update()