--[[
 fast tuning of VTOL gains for multirotors and quadplanes

 This should be used in QLOITER mode for quadplanes and LOITER mode
 for copters, although it will work in other VTOL modes

--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil
---@diagnostic disable: missing-parameter

--[[
 - TODO:
 - disable weathervaning while tuning?
 - possibly lower P XY gains during tuning?
 - bail out on a large angle error?
--]]

local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_EMERGENCY = 0

local PARAM_TABLE_KEY = 8
local PARAM_TABLE_PREFIX = "QUIK_"

local atc_prefix = "ATC"

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

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: QUIK_ENABLE
  // @DisplayName: Quicktune enable
  // @Description: Enable quicktune system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local QUIK_ENABLE      = bind_add_param('ENABLE',         1, 0) 

--[[
  // @Param: QUIK_AXES
  // @DisplayName: Quicktune axes
  // @Description: axes to tune
  // @Bitmask: 0:Roll,1:Pitch,2:Yaw
  // @User: Standard
--]]
local QUIK_AXES        = bind_add_param('AXES',           2, 7)

--[[
  // @Param: QUIK_DOUBLE_TIME
  // @DisplayName: Quicktune doubling time
  // @Description: Time to double a tuning parameter. Raise this for a slower tune.
  // @Range: 5 20
  // @Units: s
  // @User: Standard
--]]
local QUIK_DOUBLE_TIME = bind_add_param('DOUBLE_TIME',    3, 10)

--[[
  // @Param: QUIK_GAIN_MARGIN
  // @DisplayName: Quicktune gain margin
  // @Description: Reduction in gain after oscillation detected. Raise this number to get a more conservative tune
  // @Range: 20 80
  // @Units: %
  // @User: Standard
--]]
local QUIK_GAIN_MARGIN = bind_add_param('GAIN_MARGIN',    4, 60)

--[[
  // @Param: QUIK_OSC_SMAX
  // @DisplayName: Quicktune oscillation rate threshold
  // @Description: Threshold for oscillation detection. A lower value will lead to a more conservative tune.
  // @Range: 1 10
  // @User: Standard
--]]
local QUIK_OSC_SMAX    = bind_add_param('OSC_SMAX',       5, 5)

--[[
  // @Param: QUIK_YAW_P_MAX
  // @DisplayName: Quicktune Yaw P max
  // @Description: Maximum value for yaw P gain
  // @Range: 0.1 3
  // @User: Standard
--]]
local QUIK_YAW_P_MAX   = bind_add_param('YAW_P_MAX',      6, 0.5)

--[[
  // @Param: QUIK_YAW_D_MAX
  // @DisplayName: Quicktune Yaw D max
  // @Description: Maximum value for yaw D gain
  // @Range: 0.001 1
  // @User: Standard
--]]
local QUIK_YAW_D_MAX   = bind_add_param('YAW_D_MAX',      7, 0.01)

--[[
  // @Param: QUIK_RP_PI_RATIO
  // @DisplayName: Quicktune roll/pitch PI ratio
  // @Description: Ratio between P and I gains for roll and pitch. Raise this to get a lower I gain
  // @Range: 0.5 1.0
  // @User: Standard
--]]
local QUIK_RP_PI_RATIO = bind_add_param('RP_PI_RATIO',    8, 1.0)

--[[
  // @Param: QUIK_Y_PI_RATIO
  // @DisplayName: Quicktune Yaw PI ratio
  // @Description: Ratio between P and I gains for yaw. Raise this to get a lower I gain
  // @Range: 0.5 20
  // @User: Standard
--]]
local QUIK_Y_PI_RATIO  = bind_add_param('Y_PI_RATIO',     9, 10)

--[[
  // @Param: QUIK_AUTO_FILTER
  // @DisplayName: Quicktune auto filter enable
  // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local QUIK_AUTO_FILTER = bind_add_param('AUTO_FILTER',   10, 1)

--[[
  // @Param: QUIK_AUTO_SAVE
  // @DisplayName: Quicktune auto save
  // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
  // @Units: s
  // @User: Standard
--]]
local QUIK_AUTO_SAVE   = bind_add_param('AUTO_SAVE',     11, 0)

--[[
  // @Param: QUIK_RC_FUNC
  // @DisplayName: Quicktune RC function
  // @Description: RCn_OPTION number to use to control tuning stop/start/save
  // @User: Standard
--]]
local QUIK_RC_FUNC     = bind_add_param('RC_FUNC',       12, 300)

--[[
  // @Param: QUIK_MAX_REDUCE
  // @DisplayName: Quicktune maximum gain reduction
  // @Description: This controls how much quicktune is allowed to lower gains from the original gains. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions. The default of 20% is reasonable for most vehicles. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain, which can result in very low rate gains and a dangerous angle P oscillation.
  // @Units: %
  // @Range: 0 100
  // @User: Standard
--]]
local QUIK_MAX_REDUCE  = bind_add_param('MAX_REDUCE',    13, 20)

--[[
  // @Param: QUIK_OPTIONS
  // @DisplayName: Quicktune options
  // @Description: Additional options. When the Two Position Switch option is enabled then a high switch position will start the tune, low will disable the tune. you should also set a QUIK_AUTO_SAVE time so that you will be able to save the tune.
  // @Bitmask: 0:UseTwoPositionSwitch
  // @User: Standard
--]]
local QUIK_OPTIONS     = bind_add_param('OPTIONS',       14, 0)

--[[
  // @Param: QUIK_ANGLE_MAX
  // @DisplayName: maximum angle error for tune abort
  // @Description: If while tuning the angle error goes over this limit then the tune will aborts to prevent a bad oscillation in the case of the tuning algorithm failing. If you get an error "Tuning: attitude error ABORTING" and you think it is a false positive then you can either raise this parameter or you can try increasing the QUIK_DOUBLE_TIME to do the tune more slowly. A value of zero disables this check.
  // @Units: deg
  // @User: Standard
--]]
local QUIK_ANGLE_MAX = bind_add_param('ANGLE_MAX', 15, 10)

local OPTIONS_TWO_POSITION = (1<<0)

local INS_GYRO_FILTER  = bind_param("INS_GYRO_FILTER")

local RCMAP_ROLL       = bind_param("RCMAP_ROLL")
local RCMAP_PITCH      = bind_param("RCMAP_PITCH")
local RCMAP_YAW        = bind_param("RCMAP_YAW")

local RCIN_ROLL  = rc:get_channel(RCMAP_ROLL:get())
local RCIN_PITCH = rc:get_channel(RCMAP_PITCH:get())
local RCIN_YAW   = rc:get_channel(RCMAP_YAW:get())

local UPDATE_RATE_HZ = 40
local STAGE_DELAY = 4.0
local PILOT_INPUT_DELAY = 4.0

local YAW_FLTE_MAX = 8.0
local FLTD_MUL = 0.5
local FLTT_MUL = 0.5

-- SMAX to set if none set at start
local DEFAULT_SMAX = 50.0

-- work out vehicle type
if param:get("Q_A_RAT_RLL_SMAX") then
   is_quadplane = true
   atc_prefix = "Q_A"
   gcs:send_text(MAV_SEVERITY_EMERGENCY, "Quicktune for quadplane loaded")
elseif param:get("ATC_RAT_RLL_SMAX") then
   is_quadplane = false
   gcs:send_text(MAV_SEVERITY_EMERGENCY, "Quicktune for multicopter loaded")
else
   gcs:send_text(MAV_SEVERITY_EMERGENCY, "Quicktune unknown vehicle")
   return
end

-- get time in seconds since boot
function get_time()
   return millis():tofloat() * 0.001
end

local axis_names = { "RLL", "PIT", "YAW" }
local param_suffixes = { "FF", "P", "I", "D", "SMAX", "FLTT", "FLTD", "FLTE" }
local stages = { "D", "P" }
local stage = stages[1]
local last_stage_change = get_time()
local last_gain_report = get_time()
local last_pilot_input = get_time()
local tune_done_time = nil
local slew_parm = nil
local slew_target = 0
local slew_delta = 0
local aborted = false

local axes_done = {}
local filters_done = {}


-- create params dictionary indexed by name, such as "RLL_P"
local params = {}
local param_saved = {}
local param_changed = {}
local need_restore = false

for _, axis in ipairs(axis_names) do
   for _, suffix in ipairs(param_suffixes) do
      local pname = axis .. "_" .. suffix
      params[pname] = bind_param(atc_prefix .. "_" .. "RAT_" .. pname)
      param_changed[pname] = false
   end
end

-- reset to start
function reset_axes_done()
   for _, axis in ipairs(axis_names) do
      axes_done[axis] = false
      filters_done[axis] = false
   end
   stage = stages[1]
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
end

-- setup a default SMAX if zero
function setup_SMAX()
   for _, axis in ipairs(axis_names) do
      local smax = axis .. "_SMAX"
      if params[smax]:get() <= 0 then
         adjust_gain(smax, DEFAULT_SMAX)
      end
   end
end

-- setup filter frequencies
function setup_filters(axis)
   if QUIK_AUTO_FILTER:get() <= 0 then
      filters_done[axis] = true
      return
   end
   local fltd = axis .. "_FLTD"
   local fltt = axis .. "_FLTT"
   local flte = axis .. "_FLTE"
   adjust_gain(fltt, INS_GYRO_FILTER:get() * FLTT_MUL)
   adjust_gain(fltd, INS_GYRO_FILTER:get() * FLTD_MUL)
   if axis == "YAW" then
      local FLTE = params[flte]
      if FLTE:get() < 0.0 or FLTE:get() > YAW_FLTE_MAX then
         adjust_gain(flte, YAW_FLTE_MAX)
      end
   end
   filters_done[axis] = true
end

-- check for pilot input to pause tune
function have_pilot_input()
   if (math.abs(RCIN_ROLL:norm_input_dz()) > 0 or
       math.abs(RCIN_PITCH:norm_input_dz()) > 0 or
       math.abs(RCIN_YAW:norm_input_dz()) > 0) then
      return true
   end
   return false
end

reset_axes_done()
get_all_params()

function axis_enabled(axis)
   local axes = QUIK_AXES:get()
   for i = 1, #axis_names do
      local mask = (1 << (i-1))
      local axis_name = axis_names[i]
      if axis == axis_name and (mask & axes) ~= 0 then
         return true
      end
   end
   return false
end

-- get the axis name we are working on, or nil for all done
function get_current_axis()
   local axes = QUIK_AXES:get()
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
   local roll_srate, pitch_srate, yaw_srate = AC_AttitudeControl:get_rpy_srate()
   if axis == "RLL" then
      return roll_srate
   end
   if axis == "PIT" then
      return pitch_srate
   end
   if axis == "YAW" then
      return yaw_srate
   end
   return 0.0
end

-- move to next stage of tune
function advance_stage(axis)
   if stage == "D" then
      stage = "P"
   else
      axes_done[axis] = true
      gcs:send_text(5, string.format("Tuning: %s done", axis))
      stage = "D"
   end
end

-- get param name, such as RLL_P, used as index into param dictionaries
function get_pname(axis, tstage)
   return axis .. "_" .. tstage
end

-- get axis name from parameter name
function param_axis(pname)
   return string.sub(pname, 1, 3)
end

-- get parameter suffix from parameter name
function param_suffix(pname)
   return string.sub(pname, 4)
end

-- change a gain
function adjust_gain(pname, value)
   local P = params[pname]
   need_restore = true
   param_changed[pname] = true
   P:set(value)
   if string.sub(pname, -2) == "_P" then
      -- also change I gain
      local iname = string.gsub(pname, "_P", "_I")
      local ffname = string.gsub(pname, "_P", "_FF")
      local I = params[iname]
      local FF = params[ffname]
      if FF:get() > 0 then
         -- if we have any FF on an axis then we don't couple I to P,
         -- usually we want I = FF for a one second time constant for trim
         return
      end
      param_changed[iname] = true

      -- work out ratio of P to I that we want
      local PI_ratio = QUIK_RP_PI_RATIO:get()
      if param_axis(pname) == "YAW" then
         PI_ratio = QUIK_Y_PI_RATIO:get()
      end
      if PI_ratio >= 1 then
         I:set(value/PI_ratio)
      end
   end
end

-- limit a gain change to QUIK_MAX_REDUCE
function limit_gain(pname, value)
   local saved_value = param_saved[pname]
   local max_reduction = QUIK_MAX_REDUCE:get()
   if max_reduction >= 0 and max_reduction < 100 and saved_value > 0 then
      -- check if we exceeded gain reduction
      local reduction_pct = 100.0 * (saved_value - value) / saved_value
      if reduction_pct > max_reduction then
         local new_value = saved_value * (100 - max_reduction) * 0.01
         gcs:send_text(MAV_SEVERITY_INFO, string.format("limiting %s %.3f -> %.3f", pname, value, new_value))
         value = new_value
      end
   end
   return value
end

-- change a gain, limiting to QUIK_MAX_REDUCE
function adjust_gain_limited(pname, value)
   adjust_gain(pname, limit_gain(pname, value))
end

-- return gain multipler for one loop
function get_gain_mul()
   return math.exp(math.log(2.0)/(UPDATE_RATE_HZ*QUIK_DOUBLE_TIME:get()))
end

function setup_slew_gain(pname, gain)
   slew_parm = pname
   slew_target = limit_gain(pname, gain)
   slew_steps = UPDATE_RATE_HZ/2
   slew_delta = (slew_target - params[pname]:get()) / slew_steps
end

function update_slew_gain()
   if slew_parm ~= nil then
      local P = params[slew_parm]
      local axis = param_axis(slew_parm)
      local ax_stage = string.sub(slew_parm, -1)
      adjust_gain(slew_parm, P:get()+slew_delta)
      slew_steps = slew_steps - 1
      logger.write('QUIK','SRate,Gain,Param', 'ffn', get_slew_rate(axis), P:get(), axis .. ax_stage)
      if slew_steps == 0 then
         gcs:send_text(MAV_SEVERITY_INFO, string.format("%s %.4f", slew_parm, P:get()))
         slew_parm = nil
         if get_current_axis() == nil then
            gcs:send_text(MAV_SEVERITY_NOTICE, string.format("Tuning: DONE"))
            tune_done_time = get_time()
         end
      end
   end
end

-- return gain limits on a parameter, or 0 for no limit
function gain_limit(pname)
   if param_axis(pname) == "YAW" then
      local suffix = param_suffix(pname)
      if suffix == "_P" then
         return QUIK_YAW_P_MAX:get()
      end
      if suffix == "_D" then
         return QUIK_YAW_D_MAX:get()
      end
   end
   return 0.0
end

function reached_limit(pname, gain)
   local limit = gain_limit(pname)
   if limit > 0.0 and gain >= limit then
      return true
   end
   return false
end

-- main update function
local last_warning = get_time()
function update()
   if QUIK_ENABLE:get() < 1 then
      return
   end
   if have_pilot_input() then
      last_pilot_input = get_time()
   end

   local sw_pos = rc:get_aux_cached(QUIK_RC_FUNC:get())
   if not sw_pos then
      return
   end
   local sw_pos_tune = 1
   local sw_pos_save = 2
   if (QUIK_OPTIONS:get() & OPTIONS_TWO_POSITION) ~= 0 then
      sw_pos_tune = 2
      sw_pos_save = -1
   end
   if aborted then
       if sw_pos == 0 then
           aborted = false
       else
           -- after an abort require low switch position to reset
           return
       end
   end
   if sw_pos == sw_pos_tune and (not arming:is_armed() or not vehicle:get_likely_flying()) and get_time() > last_warning + 5 then
      gcs:send_text(MAV_SEVERITY_EMERGENCY, string.format("Tuning: Must be flying to tune"))
      last_warning = get_time()
      return
   end
   if sw_pos == 0 or not arming:is_armed() or not vehicle:get_likely_flying() then
      -- abort, revert parameters
      if need_restore then
         need_restore = false
         restore_all_params()
         gcs:send_text(MAV_SEVERITY_EMERGENCY, string.format("Tuning: reverted"))
         tune_done_time = nil
         aborted = true
      end
      reset_axes_done()
      return
   end
   if sw_pos == sw_pos_save then
      -- save all params
      if need_restore then
         need_restore = false
         save_all_params()
         gcs:send_text(MAV_SEVERITY_NOTICE, string.format("Tuning: saved"))
      end
   end
   if sw_pos ~= sw_pos_tune then
      return
   end

   if QUIK_ANGLE_MAX:get() > 0 and need_restore then
       local att_error = AC_AttitudeControl:get_att_error_angle_deg()
       if att_error > QUIK_ANGLE_MAX:get() then
         need_restore = false
         restore_all_params()
         gcs:send_text(MAV_SEVERITY_CRITICAL, string.format("Tuning: attitude error %.1fdeg - ABORTING", att_error))
         tune_done_time = nil
         aborted = true
         reset_axes_done()
         return
      end
   end

   if get_time() - last_stage_change < STAGE_DELAY then
      update_slew_gain()
      return
   end

   axis = get_current_axis()
   if axis == nil then
      -- nothing left to do, check autosave time
      if tune_done_time ~= nil and QUIK_AUTO_SAVE:get() > 0 then
         if get_time() - tune_done_time > QUIK_AUTO_SAVE:get() then
            need_restore = false
            save_all_params()
            gcs:send_text(MAV_SEVERITY_NOTICE, string.format("Tuning: saved"))
            tune_done_time = nil
         end
      end
      return
   end

   if not need_restore then
      -- we are just starting tuning, get current values
      gcs:send_text(MAV_SEVERITY_NOTICE, string.format("Tuning: starting tune"))
      get_all_params()
      setup_SMAX()
   end

   if get_time() - last_pilot_input < PILOT_INPUT_DELAY then
      return
   end

   if not filters_done[axis] then
      gcs:send_text(MAV_SEVERITY_INFO, string.format("Starting %s tune", axis))
      setup_filters(axis)
   end

   local srate = get_slew_rate(axis)
   local pname = get_pname(axis, stage)
   local P = params[pname]
   local oscillating = srate > QUIK_OSC_SMAX:get()
   local limited = reached_limit(pname, P:get())
   if limited or oscillating then
      local reduction = (100.0-QUIK_GAIN_MARGIN:get())*0.01
      if not oscillating then
         reduction = 1.0
      end
      local new_gain = P:get() * reduction
      local limit = gain_limit(pname)
      if limit > 0.0 and new_gain > limit then
         new_gain = limit
      end
      local old_gain = param_saved[pname]
      if new_gain < old_gain and string.sub(pname,-2) == '_D' and param_axis(pname) ~= 'YAW' then
         -- we are lowering a D gain from the original gain. Also lower the P gain by the same amount
         -- so that we don't trigger P oscillation. We don't drop P by more than a factor of 2
         local ratio = math.max(new_gain / old_gain, 0.5)
         local P_name = string.gsub(pname, "_D", "_P")
         local old_P = params[P_name]:get()
         local new_P = old_P * ratio
         gcs:send_text(MAV_SEVERITY_INFO, string.format("adjusting %s %.3f -> %.3f", P_name, old_P, new_P))
         adjust_gain_limited(P_name, new_P)
      end
      setup_slew_gain(pname, new_gain)
      logger.write('QUIK','SRate,Gain,Param', 'ffn', srate, P:get(), axis .. stage)
      gcs:send_text(6, string.format("Tuning: %s done", pname))
      advance_stage(axis)
      last_stage_change = get_time()
   else
      local new_gain = P:get()*get_gain_mul()
      if new_gain <= 0.0001 then
         new_gain = 0.001
      end
      adjust_gain_limited(pname, new_gain)
      logger.write('QUIK','SRate,Gain,Param', 'ffn', srate, P:get(), axis .. stage)
      if get_time() - last_gain_report > 3 then
         last_gain_report = get_time()
         gcs:send_text(MAV_SEVERITY_INFO, string.format("%s %.4f sr:%.2f", pname, new_gain, srate))
      end
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY_EMERGENCY, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
