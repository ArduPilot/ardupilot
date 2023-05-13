--[[
   estimation of MOT_THST_EXPO for VTOL thrust linearisation

   should be used in copter or quadplane GUIDED mode
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 11
local PARAM_TABLE_PREFIX = "EXPO_"

local UPDATE_RATE_HZ = 200

local GRAVITY_MSS = 9.80665

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup expo specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 12), 'could not add param table')

--[[
  // @Param: EXPO_ENABLE
  // @DisplayName: Expo  enable
  // @Description: Enable expo tuning system
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EXPO_ENABLE      = bind_add_param('ENABLE',         1, 0)

--[[
  // @Param: EXPO_RC_FUNC
  // @DisplayName: Expo RC function
  // @Description: RCn_OPTION number to use to control tuning stop/start/save
  // @User: Standard
--]]
local EXPO_RC_FUNC     = bind_add_param('RC_FUNC',        2, 300)

--[[
  // @Param: EXPO_PULSE_TIME
  // @DisplayName: Expo pulse time
  // @Description: Pulse length for expo measurement
  // @Range: 0.5 4
  // @Units: s
  // @User: Standard
--]]
local EXPO_PULSE_TIME  = bind_add_param('PULSE_TIME',     3, 1.5)

--[[
  // @Param: EXPO_PULSE_THST
  // @DisplayName: Expo pulse thrust amplitude
  // @Description: Pulse thrust amplitude for expo measurement. This is a proportion of hover throttle. A value of 0.5 means the pulse will go from 0.5*hover_throttle to 1.5*hover_throttle
  // @Range: 0.1 0.6
  // @User: Standard
--]]
local EXPO_PULSE_THST  = bind_add_param('PULSE_THST',     4, 0.25)

--[[
  // @Param: EXPO_LIMIT_MIN
  // @DisplayName: Expo min value
  // @Description: Minimum value we will tune to
  // @Range: -1 1
  // @User: Standard
--]]
local EXPO_LIMIT_MIN  = bind_add_param('LIMIT_MIN',       5, 0.0)

--[[
  // @Param: EXPO_LIMIT_MAX
  // @DisplayName: Expo max value
  // @Description: Maximum value we will tune to
  // @Range: -1 1
  // @User: Standard
--]]
local EXPO_LIMIT_MAX  = bind_add_param('LIMIT_MAX',       6, 0.9)

--[[
  // @Param: EXPO_PULSE_SLEW
  // @DisplayName: Expo pulse thrust slew time
  // @Description: The time over which the thrust slews to the target thrust
  // @Range: 0.0 0.5
  // @Units: s
  // @User: Standard
--]]
local EXPO_PULSE_SLEW  = bind_add_param('PULSE_SLEW',     7, 0.05)

--[[
  // @Param: EXPO_ACC_FLTHZ
  // @DisplayName: Expo acceleration filtering
  // @Description: The cutoff frequency for accel filtering
  // @Range: 1 20
  // @Units: Hz
  // @User: Standard
--]]
local EXPO_ACC_FLTHZ  = bind_add_param('ACC_FLTHZ',     8, 3)

--[[
  // @Param: EXPO_THR_FLTHZ
  // @DisplayName: Expo throttle filtering
  // @Description: The cutoff frequency for throttle filtering
  // @Range: 0.5 10
  // @Units: Hz
  // @User: Standard
--]]
local EXPO_THR_FLTHZ  = bind_add_param('THR_FLTHZ',     9, 2)

--[[
  // @Param: EXPO_CONV_THR
  // @DisplayName: Expo convergence threshold
  // @Description: The acceleration threshold for convergence
  // @Range: 0.01 0.3
  // @Units: m/s/s
  // @User: Standard
--]]
local EXPO_CONV_THR  = bind_add_param('CONV_THR',      10, 0.15)

-- get time in seconds since boot
function get_time()
   return millis():tofloat() * 0.001
end

local is_quadplane = false

-- we must be in GUIDED mode
local MODE_COPTER_GUIDED = 4
local MODE_PLANE_GUIDED = 15

local MOT_THST_EXPO
local SPIN_MIN
local SPIN_MAX
local GUID_TIMEOUT
local expected_mode = MODE_COPTER_GUIDED

if param:get("Q_M_THST_EXPO") then
   is_quadplane = true
   expected_mode = MODE_PLANE_GUIDED
   MOT_THST_EXPO = Parameter('Q_M_THST_EXPO')
   SPIN_MIN = Parameter('Q_M_SPIN_MIN')
   SPIN_MAX = Parameter('Q_M_SPIN_MAX')
else
   MOT_THST_EXPO = Parameter('MOT_THST_EXPO')
   SPIN_MIN = Parameter('MOT_SPIN_MIN')
   SPIN_MAX = Parameter('MOT_SPIN_MAX')
end

INS_HNTCH_ENABLE = Parameter('INS_HNTCH_ENABLE')
INS_HNTCH_MODE = Parameter('INS_HNTCH_MODE')
INS_HNTCH_REF = Parameter('INS_HNTCH_REF')
INS_HNTC2_ENABLE = Parameter('INS_HNTC2_ENABLE')
INS_HNTC2_MODE = Parameter('INS_HNTC2_MODE')
INS_HNTC2_REF = Parameter('INS_HNTC2_REF')

if not is_quadplane then
   GUID_TIMEOUT = Parameter('GUID_TIMEOUT')
end

local PHASE = { NONE=0, PULSE=1, WAIT=2 }
local SWITCH = { STOP=0, RUN=1, SAVE=2 }

local test_phase = PHASE.NONE
local phase_start_time = nil
local phase_start_throttle = nil
local accel_min = nil
local accel_max = nil
local last_warning = 0
local saved_pos = nil
local saved_yaw_deg = nil
local unchanged_count = 0
local pulse_count = 0
local filtered_throttle = 0
local filtered_accel = 0
local saved_params = {}

local UNCHANGED_LIMIT = 5

-- this controls how quickly we converge
local SCALING_RATIO = 0.3

-- set vehicle thrust as 0 to 1
function set_thrust(thrust)
   vehicle:set_thrust(thrust)
end

-- get earth frame z accel, offsetting for gravity
function get_accel_ofs_z()
   return ahrs:get_accel_ef():z() + GRAVITY_MSS
end

-- constrain a value between limits
local function constrain(v, vmin, vmax)
   return math.max(math.min(v, vmax), vmin)
end

--[[
   change a parameter, remembering the old value for reversion
--]]
local function change_param(param, value)
   if saved_params[param] == nil then
      -- save old value
      saved_params[param] = param:get()
   end
   param:set(value)
end

-- save parameter changed
function save_params()
   for p, _ in pairs(saved_params) do
      p:set_and_save(p:get())
   end
   if saved_params[MOT_THST_EXPO] ~= nil then
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Expo: saved at %.2f", MOT_THST_EXPO:get()))
   end
   if saved_params[INS_HNTCH_REF] ~= nil then
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Expo: saved INS_HNTCH_REF at %.2f", INS_HNTCH_REF:get()))
   end
   if saved_params[INS_HNTC2_REF] ~= nil then
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Expo: saved INS_HNTC2_REF at %.2f", INS_HNTC2_REF:get()))
   end
   -- clear saved parameters
   saved_params = {}
end

-- get the thrust delta we will apply in a pulse
function get_thrust_delta(throttle)
   -- don't let us get over 90% throttle or below 2% throttle
   local pulse_thrust = constrain(EXPO_PULSE_THST:get(),0.1,0.6)
   return math.min(pulse_thrust*throttle, (1.0 - throttle)-0.1)
end

local function lowpass_filter(filtered, value, dt, cutoff_hz)
   local rc = 1.0/(math.pi*2*math.max(cutoff_hz, 0.01))
   local alpha = 1.0 - dt/(dt+rc)
   return (1.0 - alpha) * value + alpha * filtered
end

--[[
   get the thrust curve we use for testing expo.
   The curve is designed to give us maximum acceleration at zero
   vertical velocity, which means minimum drag. It is a double slew
   limited square wave
--]]
local function get_thrust_curve(t)
   local pulse_time = EXPO_PULSE_TIME:get()
   local pulse_slew = EXPO_PULSE_SLEW:get()
   if pulse_slew*4 > pulse_time then
      pulse_slew = pulse_time * 0.25
   end
   local pulse_hold = (pulse_time - 4*pulse_slew) * 0.5
   if t < pulse_slew then
      return (t / pulse_slew)
   end
   if t < pulse_slew + pulse_hold then
      return 1.0
   end
   if t < 3*pulse_slew + pulse_hold then
      local t1 = t - (pulse_slew + pulse_hold)
      return 1.0 - 2*t1/(pulse_slew*2)
   end
   if t < 3*pulse_slew + 2*pulse_hold then
      return -1.0
   end
   local t2 = t - (pulse_slew*3 + pulse_hold*2)
   return -1.0 + t2 / pulse_slew
end

--[[
   forward thrust curve
--]]
local function apply_thrust_curve(thrust, expo)
   expo = constrain(expo, -1.0, 1.0)
   if math.abs(expo) < 0.01 then
      -- zero expo means linear, avoid floating point exception for small values
      return thrust
   end
   local throttle_ratio = ((expo - 1.0) + math.sqrt((1.0 - expo) * (1.0 - expo) + 4.0 * expo * thrust)) / (2.0 * expo)
   return constrain(throttle_ratio, 0.0, 1.0)
end

--[[
  inverse thrust curve
--]]
local function remove_thrust_curve(throttle, expo)
   -- apply thrust curve - domain -1.0 to 1.0, range -1.0 to 1.0
   expo = constrain(expo, -1.0, 1.0)
   if math.abs(expo) < 0.01 then
      -- zero expo means linear, avoid floating point exception for small values
      return throttle
   end
   local thrust = (throttle * (2.0 * expo)) - (expo - 1.0)
   thrust = (thrust * thrust) - ((1.0 - expo) * (1.0 - expo))
   thrust = thrust / (4.0 * expo)
   return constrain(thrust, 0.0, 1.0)
end

--[[
 converts desired thrust to linearized actuator output in a range of 0~1
--]]
local function thrust_to_actuator(thrust_in, expo)
   thrust_in = constrain(thrust_in, 0.0, 1.0)
   return SPIN_MIN:get() + (SPIN_MAX:get() - SPIN_MIN:get()) * apply_thrust_curve(thrust_in, expo)
end

--[[
   converts actuator to thrust
--]]
local function actuator_to_thrust(actuator, expo)
   actuator = (actuator - SPIN_MIN:get()) /  (SPIN_MAX:get() - SPIN_MIN:get())
   return constrain(remove_thrust_curve(actuator, expo), 0.0, 1.0)
end

--[[
   adjust refernce for expo in throttle notch
--]]
local function adjust_reference(ref, old_expo, new_expo)
   local actuator = thrust_to_actuator(ref, old_expo)
   return actuator_to_thrust(actuator, new_expo)
end

--[[
   adjust throttle based notch filter references
--]]
function adjust_notch_filters(old_expo, new_expo)
   if INS_HNTCH_ENABLE:get() == 1 and INS_HNTCH_MODE:get() == 1 then
      local old_ref = INS_HNTCH_REF:get()
      change_param(INS_HNTCH_REF, adjust_reference(old_ref, old_expo, new_expo))
   end
   if INS_HNTC2_ENABLE:get() == 1 and INS_HNTC2_MODE:get() == 1 then
      local old_ref = INS_HNTC2_REF:get()
      change_param(INS_HNTC2_REF, adjust_reference(old_ref, old_expo, new_expo))
   end
end

--[[
   adjust the expo for the given thrust change and accel_min, accel_max
--]]
function adjust_expo(thrust_change)
   if pulse_count % 2 == 1 then
      -- only adjust on even pulses
      return
   end
   local expected_accel = thrust_change * GRAVITY_MSS
   --[[ if we get acceleration deviations higher than expected then
      the expo is too low. If we get deviations lower than expected
      then expo is too high
   --]]
   local accel_err = (accel_max - accel_min) * 0.5 - expected_accel
   local scale = 1 + constrain(accel_err,-1,1)*SCALING_RATIO
   local old_expo = MOT_THST_EXPO:get()
   if math.abs(old_expo) < 0.03 then
      if old_expo < 0 then
         old_expo = -0.03
      else
         old_expo = 0.03
      end
   end
   local new_expo = constrain(old_expo * scale, EXPO_LIMIT_MIN:get(), EXPO_LIMIT_MAX:get())
   gcs:send_text(MAV_SEVERITY.INFO, string.format("Expo: %.2f accel=(%.2f,%.2f) err=%.3f", new_expo, accel_min, accel_max, accel_err))
   change_param(MOT_THST_EXPO, new_expo)

   -- possibly adjust notch filters
   adjust_notch_filters(old_expo, new_expo)

   accel_min = nil
   accel_max = nil
   if math.abs(accel_err) <= EXPO_CONV_THR:get() then
      unchanged_count = (unchanged_count + 1)
      if unchanged_count >= UNCHANGED_LIMIT then
         gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Expo: finished tune %.2f", MOT_THST_EXPO:get()))
      end
   else
      unchanged_count = 0
   end
end

-- revert any expo change
function revert()
   for p, value in pairs(saved_params) do
      p:set(value)
   end
   if saved_params[MOT_THST_EXPO] ~= nil then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("Expo: reverted to %.2f", MOT_THST_EXPO:get()))
   end
   saved_params = {}
   saved_pos = nil
   saved_yaw_deg = nil
   accel_min = nil
   accel_max = nil
   unchanged_count = 0
   pulse_count = 0
end

-- main update function
function update()
   if EXPO_ENABLE:get() == 0 then
      return
   end

   if is_quadplane then
      --[[ force Q_GUIDED_MODE==2 when using the EXPO script to ensure
         users are not surprised by a fixed wing transition when
         entering guided mode from QLOITER or another Q mode
      --]]
      param:set("Q_GUIDED_MODE", 2)
   end

   local sw_pos = rc:get_aux_cached(EXPO_RC_FUNC:get())
   if not sw_pos then
      return
   end

   local now = get_time()

   if sw_pos == SWITCH.RUN and (not arming:is_armed() or not vehicle:get_likely_flying()) and get_time() > last_warning + 5 then
      if now - last_warning > 3 then
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("Expo: Must be flying to tune"))
         last_warning = get_time()
      end
      revert()
      return
   end
   if sw_pos == SWITCH.RUN and vehicle:get_mode() ~= expected_mode then
      if now - last_warning > 3 then
         gcs:send_text(MAV_SEVERITY.ERROR, string.format("Expo: Must be in GUIDED mode to tune"))
         last_warning = get_time()
      end
      revert()
      return
   end
   if not arming:is_armed() or not vehicle:get_likely_flying() then
      -- abort, revert parameters
      revert()
      return
   end

   -- we are armed and flying, update filtered throttle and accel
   filtered_throttle = lowpass_filter(filtered_throttle, motors:get_throttle(), 1.0/UPDATE_RATE_HZ, EXPO_THR_FLTHZ:get())
   filtered_accel = lowpass_filter(filtered_accel, get_accel_ofs_z(), 1.0/UPDATE_RATE_HZ, EXPO_ACC_FLTHZ:get())

   if sw_pos == SWITCH.STOP then
      -- not tuning
      revert()
      return
   end

   if sw_pos == SWITCH.SAVE then
      -- save all params
      if saved_params[MOT_THST_EXPO] ~= nil then
         save_params()
         revert()
      end
   end
   if sw_pos ~= SWITCH.RUN then
      return
   end

   if unchanged_count >= UNCHANGED_LIMIT then
      -- we've finished the tune
      return
   end

   if saved_pos == nil then
      -- we are just starting tuning, get current values
      gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Expo: starting tune with expo=%.2f", MOT_THST_EXPO:get()))
      saved_pos = ahrs:get_relative_position_NED_origin()
      saved_yaw_deg = math.deg(ahrs:get_yaw())
      phase_start_time = now

      -- initialise filtered throttle and accel
      filtered_throttle = motors:get_throttle()
      filtered_accel = get_accel_ofs_z()
   end

   local pos_err = (ahrs:get_relative_position_NED_origin() - saved_pos):length()
   local vel_err = ahrs:get_velocity_NED():length()
   local accel = filtered_accel

   logger.write('VEXP','t,Accel,AccelF,PErr,VErr,Phase,PC,Thr,ThrF,Expo', 'fffffBIfff',
                now-phase_start_time,
                get_accel_ofs_z(), accel, pos_err, vel_err, test_phase, pulse_count,
                motors:get_throttle(), filtered_throttle,
                MOT_THST_EXPO:get())

   local quiescent = (vel_err < 0.2 and pos_err < 0.2 and math.abs(filtered_accel) < 0.1)

   if test_phase == PHASE.NONE and quiescent then
      phase_start_time = now
      phase_start_throttle = filtered_throttle
      if not is_quadplane then
         GUID_TIMEOUT:set(0.1)
      end
      set_thrust(phase_start_throttle)
      test_phase = PHASE.PULSE
   end

   if test_phase == PHASE.PULSE then
      if accel_max == nil or accel > accel_max then
         accel_max = accel
      end
      if accel_min == nil or accel < accel_min then
         accel_min = accel
      end
      local thrust_delta = get_thrust_delta(phase_start_throttle)
      local t = now - phase_start_time
      local direction = (pulse_count % 2) * 2 - 1
      local thrust_curve = get_thrust_curve(t)
      set_thrust(phase_start_throttle + thrust_delta * direction * thrust_curve)
   end
   
   if test_phase == PHASE.PULSE and now - phase_start_time >= EXPO_PULSE_TIME:get() then
      phase_start_time = now
      set_thrust(phase_start_throttle)
      test_phase = PHASE.WAIT
      pulse_count = pulse_count + 1
      adjust_expo(get_thrust_delta(phase_start_throttle)/phase_start_throttle)
      if not is_quadplane then
         vehicle:set_target_pos_NED(saved_pos, true, saved_yaw_deg, false, 0, false, false)
      end
   end

   if test_phase == PHASE.WAIT then
      if not is_quadplane then
         vehicle:set_target_pos_NED(saved_pos, true, saved_yaw_deg, false, 0, false, false)
      end
      if quiescent then
         test_phase = PHASE.NONE
      end
   end
   
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded VTOL-expo tuning")

-- wrapper around update(). This calls update() at 100Hz,
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY.EMERGENCY, "Internal Error: " .. err)
     revert()
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
