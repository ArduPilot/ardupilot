--[[

Rover QuickTune tunes the steering (aka turn rate) and speed controller gains for rovers and boats

The script is designed to be used in Circle mode and updates the following parameters

ATC_STR_RAT_P
ATC_STR_RAT_I
ATC_STR_RAT_D
ATC_STR_RAT_FF
ATC_SPEED_P
ATC_SPEED_I
ATC_SPEED_D
CRUISE_SPEED
CRUISE_THROTTLE

See the accompanying rover-quiktune.md file for instructions on how to use

--]]

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 15
local PARAM_TABLE_PREFIX = "RTUN_"
local PARAM_TABLE_SIZE = 11

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format("RTun: could not find %s parameter", name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("RTun: could not add param %s", name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, PARAM_TABLE_SIZE), "RTun: could not add param table")

--[[
  // @Param: RTUN_ENABLE
  // @DisplayName: Rover Quicktune enable
  // @Description: Enable quicktune system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local RTUN_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: RTUN_AXES
  // @DisplayName: Rover Quicktune axes
  // @Description: axes to tune
  // @Bitmask: 0:Steering,1:Speed
  // @User: Standard
--]]
local RTUN_AXES = bind_add_param('AXES', 2, 3)

--[[
  // @Param: RTUN_STR_FFRATIO
  // @DisplayName: Rover Quicktune Steering Rate FeedForward ratio
  // @Description: Ratio between measured response and FF gain. Raise this to get a higher FF gain
  // @Range: 0 1.0
  // @User: Standard
--]]
local RTUN_STR_FFRATIO = bind_add_param('STR_FFRATIO', 3, 0.9)

--[[
  // @Param: RTUN_STR_P_RATIO
  // @DisplayName: Rover Quicktune Steering FF to P ratio
  // @Description: Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
local RTUN_STR_P_RATIO = bind_add_param('STR_P_RATIO', 4, 0.5)

--[[
  // @Param: RTUN_STR_I_RATIO
  // @DisplayName: Rover Quicktune Steering FF to I ratio
  // @Description: Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
local RTUN_STR_I_RATIO = bind_add_param('STR_I_RATIO', 5, 0.5)

--[[
  // @Param: RTUN_SPD_FFRATIO
  // @DisplayName: Rover Quicktune Speed FeedForward (equivalent) ratio
  // @Description: Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value
  // @Range: 0 1.0
  // @User: Standard
--]]
local RTUN_SPD_FFRATIO = bind_add_param('SPD_FFRATIO', 6, 1.0)

--[[
  // @Param: RTUN_SPD_P_RATIO
  // @DisplayName: Rover Quicktune Speed FF to P ratio
  // @Description: Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
local RTUN_SPD_P_RATIO = bind_add_param('SPD_P_RATIO', 7, 1.0)

--[[
  // @Param: RTUN_SPD_I_RATIO
  // @DisplayName: Rover Quicktune Speed FF to I ratio
  // @Description: Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged
  // @Range: 0 2.0
  // @User: Standard
--]]
local RTUN_SPD_I_RATIO = bind_add_param('SPD_I_RATIO', 8, 1.0)

--[[
  // @Param: RTUN_AUTO_FILTER
  // @DisplayName: Rover Quicktune auto filter enable
  // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local RTUN_AUTO_FILTER = bind_add_param('AUTO_FILTER', 9, 1)

--[[
  // @Param: RTUN_AUTO_SAVE
  // @DisplayName: Rover Quicktune auto save
  // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
  // @Units: s
  // @User: Standard
--]]
local RTUN_AUTO_SAVE = bind_add_param('AUTO_SAVE', 10, 5)

--[[
  // @Param: RTUN_RC_FUNC
  // @DisplayName: Rover Quicktune RC function
  // @Description: RCn_OPTION number to use to control tuning stop/start/save
  // @Values: 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
  // @User: Standard
--]]
local RTUN_RC_FUNC = bind_add_param('RC_FUNC', 11, 300)

-- other vehicle parameters used by this script
local INS_GYRO_FILTER  = bind_param("INS_GYRO_FILTER")
local GCS_PID_MASK     = bind_param("GCS_PID_MASK")
local RCMAP_ROLL       = bind_param("RCMAP_ROLL")
local RCMAP_THROTTLE   = bind_param("RCMAP_THROTTLE")
local RCIN_ROLL  = rc:get_channel(RCMAP_ROLL:get())
local RCIN_THROTTLE = rc:get_channel(RCMAP_THROTTLE:get())

-- definitions
local UPDATE_RATE_HZ = 40           -- this script updates at 40hz
local AXIS_CHANGE_DELAY = 4.0       -- delay of 4 seconds between axis to allow vehicle to settle
local PILOT_INPUT_DELAY = 4.0       -- gains are not updated for 4 seconds after pilot releases sticks
local FLTD_MUL = 0.5                -- ATC_STR_RAT_FLTD set to 0.5 * INS_GYRO_FILTER
local FLTT_MUL = 0.5                -- ATC_STR_RAT_FLTT set to 0.5 * INS_GYRO_FILTER
local STR_RAT_FF_TURNRATE_MIN = math.rad(10)    -- steering rate feedforward min vehicle turn rate (in radians/sec)
local STR_RAT_FF_STEERING_MIN = 0.10            -- steering rate feedforward min steering output (in the range 0 to 1)
local SPEED_FF_SPEED_MIN = 0.5      -- speed feedforward minimum vehicle speed (in m/s)
local SPEED_FF_THROTTLE_MIN = 0.20  -- speed feedforward requires throttle output (in the range 0 to 1)

-- get time in seconds since boot
function get_time()
   return millis():tofloat() * 0.001
end

-- local variables
local axis_names = { "ATC_STR_RAT", "ATC_SPEED" }                       -- table of axis that may be tuned
local param_suffixes = { "FF", "P", "I", "D", "FLTT", "FLTD", "FLTE" }  -- table of parameters that may be tuned
local params_extra = {"CRUISE_SPEED", "CRUISE_THROTTLE"}                -- table of extra parameters that may be changed
local last_axis_change = get_time()     -- time (in seconds) that axis last changed
local last_pilot_input = get_time()     -- time pilot last provided RC input
local tune_done_time = nil              -- time that tuning completed (used for auto save feature)
local axes_done = {}                    -- list of axes that have been tuned
local filters_done = {}                 -- table recording if filters have been set for each axis
local gcs_pid_mask_done = {}            -- table recording if GCS_PID_MASK has been set for each axis
local gcs_pid_mask_orig                 -- GCS_PID_MASK value before tuning started

-- feed forward tuning related local variables
local ff_throttle_sum = 0               -- total throttle recorded during speed FF tuning (divided by count to calc average)
local ff_speed_sum = 0                  -- total speed recorded during speed FF tuning (divided by count to calc average)
local ff_speed_count = 0                -- number of speed and throttle samples taken during FF tuning
local ff_steering_sum = 0               -- total steering input recorded during steering rate FF tuning (divided by count to calc average)
local ff_turn_rate_sum = 0              -- total turn rate recorded during steering rate FF tuning (divided by count to calc average)
local ff_turn_rate_count = 0            -- number of steering and turn rate samples taken during FF tuning
local ff_last_warning = 0               -- time of last warning to user

-- params dictionary indexed by name, such as "ATC_STR_RAT_P"
local params = {}                       -- table of all parameters that may be tuned
local params_axis = {}                  -- table of each parameter's axis (used for logging of the appropriate srate)
local param_saved = {}                  -- table holding backup of each parameter's value from before tuning
local param_changed = {}                -- table holding whether each param's gain has been saved
local need_restore = false              -- true if any param's gain has been changed

-- initialise params, params_axis and param_changed tables
function init_params_tables()
  -- add parameters to params dictionary
  for _, axis in ipairs(axis_names) do
    for _, suffix in ipairs(param_suffixes) do
      local pname = axis .. "_" .. suffix
      params[pname] = bind_param(pname)
      params_axis[pname] = axis
      param_changed[pname] = false
    end
  end

  -- add extra parameters to param dictionary
  for _, extra_param_name in ipairs(params_extra) do
    params[extra_param_name] = bind_param(extra_param_name)
    params_axis[extra_param_name] = "ATC_SPEED"  -- axis hard-coded to always be ATC_SPEED
    param_changed[extra_param_name] = false
  end
end

-- initialise all state variables so we are ready to start another tune
function reset_axes_done()
  for _, axis in ipairs(axis_names) do
    axes_done[axis] = false
    filters_done[axis] = false
    gcs_pid_mask_done[axis] = false
  end
  tune_done_time = nil
  init_steering_ff()
  init_speed_ff()
end

-- get all current param values into param_saved dictionary
function get_all_params()
  for pname in pairs(params) do
    param_saved[pname] = params[pname]:get()
  end
end

-- restore all param values from param_saved dictionary
function restore_all_params()
  for pname in pairs(params) do
    if param_changed[pname] then
      params[pname]:set(param_saved[pname])
      param_changed[pname] = false
    end
  end
end

-- save all param values to storage
function save_all_params()
  for pname in pairs(params) do
    if param_changed[pname] then
      params[pname]:set_and_save(params[pname]:get())
      param_saved[pname] = params[pname]:get()
      param_changed[pname] = false
    end
  end
  gcs:send_text(MAV_SEVERITY.NOTICE, "RTun: tuning gains saved")
end

-- setup filter frequencies
function setup_filters(axis)
  if RTUN_AUTO_FILTER:get() > 0 then
    if axis == "ATC_STR_RAT" then
      adjust_gain(axis .. "_FLTT", INS_GYRO_FILTER:get() * FLTT_MUL)
      adjust_gain(axis .. "_FLTD", INS_GYRO_FILTER:get() * FLTD_MUL)
    end
  end
  filters_done[axis] = true
end

-- backup GCS_PID_MASK to value before tuning
function save_gcs_pid_mask()
  gcs_pid_mask_orig = GCS_PID_MASK:get()
end

-- restore GCS_PID_MASK to value before tuning started
function restore_gcs_pid_mask()
  GCS_PID_MASK:set(gcs_pid_mask_orig)
end

-- setup GCS_PID_MASK to provide real-time PID info to GCS during tuning
function setup_gcs_pid_mask(axis)
  if axis == "ATC_STR_RAT" then
    GCS_PID_MASK:set(1)
  elseif axis == "ATC_SPEED" then
    GCS_PID_MASK:set(2)
  else
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("RTun: setup_gcs_pid_mask received unhandled aixs %s", axis))
  end
  gcs_pid_mask_done[axis] = true
end

-- check for pilot input to pause tune
function have_pilot_input()
  if (math.abs(RCIN_ROLL:norm_input_dz()) > 0 or
     math.abs(RCIN_THROTTLE:norm_input_dz()) > 0) then
    return true
  end
  return false
end

-- get the axis name we are working on, or nil for all done
function get_current_axis()
  local axes = RTUN_AXES:get()
  for i = 1, #axis_names do
    local mask = (1 << (i-1))
    local axis_name = axis_names[i]
    if (mask & axes) ~= 0 and axes_done[axis_name] == false then
      return axis_names[i]
    end
  end
  return nil
end

-- get slew rate for an axis
function get_slew_rate(axis)
  local steering_srate, speed_srate = AR_AttitudeControl:get_srate()
  if axis == "ATC_STR_RAT" then
    return steering_srate
  end
  if axis == "ATC_SPEED" then
    return speed_srate
  end
  gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("RTUN: get_slew_rate unsupported axis:%s", axis))
  return 0.0
end

-- move to next axis of tune
function advance_axis(axis)
  local now_sec = get_time()
  local prev_axis = get_current_axis()
  axes_done[axis] = true
  -- check for tune completion
  if prev_axis ~= nil and get_current_axis() == nil then
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("RTun: Tuning DONE"))
    tune_done_time = now_sec
  end
  last_axis_change = now_sec
end

-- change a gain, log and update user
function adjust_gain(pname, value)
  local P = params[pname]
  local old_value = P:get()
  need_restore = true
  param_changed[pname] = true
  P:set(value)
  write_log(pname)
  gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: adjusted %s %.3f -> %.3f", pname, old_value, value))
end

-- log parameter, current gain and current slew rate
function write_log(pname)
  local param_gain = params[pname]:get()
  local pname_axis = params_axis[pname]
  local slew_rate = get_slew_rate(pname_axis)
  logger:write("RTUN","SRate,Gain,Param", "ffN", slew_rate, param_gain, pname)
end

-- initialise steering ff tuning
function init_steering_ff()
  ff_steering_sum = 0
  ff_turn_rate_sum = 0
  ff_turn_rate_count = 0
end

-- run steering turn rate controller feedforward calibration
-- ff_pname is the FF parameter being tuned
-- returns true once the tuning has completed
function update_steering_ff(ff_pname)
  -- get steering, turn rate, throttle and speed
  local steering_out, _ = vehicle:get_steering_and_throttle()
  local turn_rate_rads = ahrs:get_gyro():z()

  -- update user every 5 sec
  local now_sec = get_time()
  local update_user = false
  if (now_sec > ff_last_warning + 5) then
    update_user = true
    ff_last_warning = now_sec
  end

  -- calculate percentage complete
  local turn_rate_complete_pct = (ff_turn_rate_sum / math.pi * 2.0) * 100
  local time_complete_pct = (ff_turn_rate_count  / (10 * UPDATE_RATE_HZ)) * 100
  local complete_pct = math.min(turn_rate_complete_pct, time_complete_pct)

  -- check steering and turn rate and accumulate output and response
  local steering_ok = steering_out >= STR_RAT_FF_STEERING_MIN
  local turnrate_ok = math.abs(turn_rate_rads) > STR_RAT_FF_TURNRATE_MIN
  if (steering_ok and turnrate_ok) then
    ff_steering_sum = ff_steering_sum + steering_out
    ff_turn_rate_sum = ff_turn_rate_sum + math.abs(turn_rate_rads)
    ff_turn_rate_count = ff_turn_rate_count + 1
    if (update_user) then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: %s %.0f%% complete", ff_pname, complete_pct))
    end
  else
    if update_user then
      if not steering_ok then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("RTun: increase steering (%d%% < %d%%)", math.floor(steering_out * 100), math.floor(STR_RAT_FF_STEERING_MIN * 100)))
      elseif not turnrate_ok then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("RTun: increase turn rate (%d deg/s < %d)", math.floor(math.deg(math.abs(turn_rate_rads))), math.floor(math.deg(STR_RAT_FF_TURNRATE_MIN))))
      end
    end
  end

  -- check for completion of two rotations of turns data and 10 seconds
  if complete_pct >= 100 then
    local FF_new_gain = (ff_steering_sum / ff_turn_rate_sum) * RTUN_STR_FFRATIO:get()
    adjust_gain(ff_pname, FF_new_gain)

    -- set P gain
    if RTUN_STR_P_RATIO:get() > 0 then
      local pname = string.gsub(ff_pname, "_FF", "_P")
      adjust_gain(pname, FF_new_gain * RTUN_STR_P_RATIO:get())
    end

    -- set I gain
    if RTUN_STR_I_RATIO:get() > 0 then
      local iname = string.gsub(ff_pname, "_FF", "_I")
      adjust_gain(iname, FF_new_gain * RTUN_STR_I_RATIO:get())
    end

    return true
  end

  return false
end

-- initialise speed ff tuning
function init_speed_ff()
  ff_throttle_sum = 0
  ff_speed_sum = 0
  ff_speed_count = 0
end

-- run speed controller feedforward calibration
-- ff_pname is the FF parameter being tuned
-- returns true once the tuning has completed
function update_speed_ff(ff_pname)
  -- get steering, turn rate, throttle and speed
  local _, throttle_out = vehicle:get_steering_and_throttle()
  local velocity_ned = ahrs:get_velocity_NED()
  if velocity_ned then
    speed = ahrs:earth_to_body(velocity_ned):x()
  end

  -- update user every 5 sec
  local now_sec = get_time()
  local update_user = false
  if (now_sec > ff_last_warning + 5) then
    update_user = true
    ff_last_warning = now_sec
  end

  -- calculate percentage complete
  local complete_pct = (ff_speed_count / (10 * UPDATE_RATE_HZ)) * 100

  -- check throttle and speed
  local throttle_ok = throttle_out >= SPEED_FF_THROTTLE_MIN
  local speed_ok = speed > SPEED_FF_SPEED_MIN
  if (throttle_ok and speed_ok) then
    ff_throttle_sum = ff_throttle_sum + throttle_out
    ff_speed_sum = ff_speed_sum + speed
    ff_speed_count = ff_speed_count + 1
    if (update_user) then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: %s %.0f%% complete", ff_pname, complete_pct))
    end
  else
    if update_user then
      if not throttle_ok then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("RTun: increase throttle (%d < %d)", math.floor(throttle_out * 100), math.floor(SPEED_FF_THROTTLE_MIN * 100)))
      elseif not speed_ok then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("RTun: increase speed (%3.1f < %3.1f)", speed, SPEED_FF_SPEED_MIN))
      end
    end
  end

  -- check for 10 seconds of data
  if complete_pct >= 100 then
    local cruise_speed_new = ff_speed_sum / ff_speed_count
    local cruise_throttle_new = (ff_throttle_sum / ff_speed_count) * 100 * RTUN_SPD_FFRATIO:get()
    adjust_gain("CRUISE_SPEED", cruise_speed_new)
    adjust_gain("CRUISE_THROTTLE", cruise_throttle_new)

    -- calculate FF equivalent gain (used for setting P and I below)
    local speed_ff_equivalent = (ff_throttle_sum / ff_speed_sum) * RTUN_SPD_FFRATIO:get();

    -- set P gain
    if RTUN_SPD_P_RATIO:get() > 0 then
      local pname = string.gsub(ff_pname, "_FF", "_P")
      local P_new_gain = speed_ff_equivalent * RTUN_SPD_P_RATIO:get()
      adjust_gain(pname, P_new_gain)
    end

    -- set I gain
    if RTUN_SPD_I_RATIO:get() > 0 then
      local iname = string.gsub(ff_pname, "_FF", "_I")
      local I_new_gain = speed_ff_equivalent * RTUN_SPD_I_RATIO:get()
      adjust_gain(iname, I_new_gain)
    end

    return true
  end

  return false
end

-- initialisation
init_params_tables()
reset_axes_done()
get_all_params()
save_gcs_pid_mask()
gcs:send_text(MAV_SEVERITY.INFO, "Rover quiktune loaded")

-- main update function
local last_warning = get_time()
function update()

  -- exit immediately if not enabled
  if RTUN_ENABLE:get() <= 0 then
    return
  end

  if have_pilot_input() then
    last_pilot_input = get_time()
  end

  local sw_pos = rc:get_aux_cached(RTUN_RC_FUNC:get())
  if not sw_pos then
    return
  end

  -- get output throttle
  local _, throttle_out = vehicle:get_steering_and_throttle()

  -- check switch position (0:low is stop, 1:middle is tune, 2:high is save gains
  if sw_pos == 1 and (not arming:is_armed() or (throttle_out <= 0)) and get_time() > last_warning + 5 then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "RTun: must be armed and moving to tune")
    last_warning = get_time()
    return
  end
  if sw_pos == 0 or not arming:is_armed() then
    -- abort, revert parameters
    if need_restore then
      need_restore = false
      restore_all_params()
      restore_gcs_pid_mask()
      gcs:send_text(MAV_SEVERITY.CRITICAL, "RTun: gains reverted")
    end
    reset_axes_done()
    return
  end
  if sw_pos == 2 then
    -- save all params
    if need_restore then
      need_restore = false
      save_all_params()
      restore_gcs_pid_mask()
    end
  end

  -- if we reach here we must be tuning
  if sw_pos ~= 1 then
    return
  end

  -- return if we have just changed stages to give time for oscillations to subside
  if get_time() - last_axis_change < AXIS_CHANGE_DELAY then
    return
  end

  -- get axis currently being tuned
  axis = get_current_axis()

  -- if no axis is being tuned we must be done
  if axis == nil then
    -- check if params should be auto saved
    if tune_done_time ~= nil and RTUN_AUTO_SAVE:get() > 0 then
      if get_time() - tune_done_time > RTUN_AUTO_SAVE:get() then
         need_restore = false
         save_all_params()
         restore_gcs_pid_mask()
         tune_done_time = nil
      end
    end
    return
  end

  if not need_restore then
    -- we are just starting tuning, get current values
    get_all_params()
  end

  -- return immediately if pilot has provided input recently
  if get_time() - last_pilot_input < PILOT_INPUT_DELAY then
    return
  end

  -- check filters have been set for this axis
  if not filters_done[axis] then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("RTun: starting %s tune", axis))
    setup_filters(axis)
  end

  -- check GCS_PID_MASK has been set for this axis
  if not gcs_pid_mask_done[axis] then
    setup_gcs_pid_mask(axis)
  end

  -- get parameter currently being tuned
  local pname = axis .. "_FF"

  -- feedforward tuning
  local ff_done
  if axis == "ATC_STR_RAT" then
    ff_done = update_steering_ff(pname)
  elseif axis == "ATC_SPEED" then
    ff_done = update_speed_ff(pname)
  else
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("RTun: unsupported FF tuning %s", pname))
    ff_done = true
  end
  if ff_done then
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("RTun: %s tuning done", pname))
    advance_axis(axis)
  end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "RTun: Internal Error: " .. err)
    -- when we fault we run the update function again after 1s, slowing it
    -- down a bit so we don't flood the console with errors
    --return protected_wrapper, 1000
    return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
