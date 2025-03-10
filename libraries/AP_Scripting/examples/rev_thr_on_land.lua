--[[
    This script automatically engages reverse thrust on planes in FBWA mode on touchdown.
    Prerequisities:
    - ArduPlane 4.1 or later
    - A Fixed Wing platform capable of CTOL operation
    - ESCs capable of reversing motor direction
    - A Separate RC channel for triggering throttle reverse, assigned by RCx_FUNCTION=64
    - A good rangefinder (downward facing) capable of reliably reading [0.1, 5]m of range
    - A separate RC channel that will serve as an "arming" switch for reverse throttle operation. 
      This script uses RC_ARM_CHANNEL_ for this purpose (i.e. reverse throttle is armed whenever this switch PWM >1500, 
      set by RC_ARM_CHANNEL_HI_PWM_)
    Params to adjust:
    - THRESH_LAND_HEIGHT_ (m) - this is the height read by rangefinder which corresponds to a plane's height above ground
      on touchdown. Set this a couple of cms higher than the height reading by your rangefider when the aircraft is on the ground
      but well within the height that the rangefinder can reliably read.
    - LAND_DET_TIME_ (s) - time during which rangefinder height estimate is consistently below THRESH_LAND_HEIGHT_
    - MIN_GROUND_SPEED_ (m/s) - minimum ground speed at which the autopilot will disengage reverse thrust and switch to
      idle forward thrust
    - REV_THRUST_HIGH_PWM_, REV_THRUST_IDLE_PWM_ - max and min pwm values for RC_CHANNEL_NUM_ when in reverse throttle mode
    - REV_THRUST_CHG_PWM_PS_(pwm) - this is the motors' linear acceleration time in seconds, from REV_THRUST_IDLE_PWM_ to REV_THRUST_HIGH_PWM_
    - MAXTIME_REV_THRUST_APPLIED_MS_ (ms) - maximum time, in seconds, reverse throttle is allowed to apply
    - TIME_THROT_WAIT_MS_ (ms) - time the throttle is suppressed, sitting at forward idle
    - REV_THRUST_SPOOL_DPWM_ (pwm) - initial increase in pwm to apply when in reverse throttle mode
]]--


----------***---Parameter Binding (Taken from VTOL-quicktune.lua)**** 

local PARAM_TABLE_KEY = 9
local PARAM_TABLE_PREFIX = "REV_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end
 
-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- *****---PARAMS TO CHANGE---*****

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 12), 'could not add param table')

--REV arming parameters
local REV_ENABLE_                    = bind_add_param('ENABLE',                   1, 0)
local RC_ARM_CHANNEL_                = bind_add_param('ARM_CHANNEL',              2, 6)
local ARM_CHANNEL_HI_PWM_            = bind_add_param('ARM_HI_PWM',               3, 1500)

--REV activation and spool up parameters
local LAND_DET_TIME_                 = bind_add_param('LAND_TIME',                4, 1)
local THRESH_LAND_HEIGHT_            = bind_add_param('LAND_HEIGHT',              5, 0.12)
local TIME_THROT_WAIT_MS_            = bind_add_param('WAIT_MS',                  6, 1000)
local REV_THRUST_SPOOL_DPWM_         = bind_add_param('SPOOL_DPWM',               7, 60)
local REV_THRUST_CHG_PWM_PS_         = bind_add_param('CHGPWM_PS',                8, 150)

--REV strength and duration setup
local REV_THRUST_IDLE_PWM_           = bind_add_param('THR_IDLE_PWM',             9, 990)
local REV_THRUST_HIGH_PWM_           =  bind_add_param('HI_PWM',                 10, 1600)
local MAXTIME_REV_THRUST_APPLIED_MS_ = bind_add_param('MAXTIME_MS',              11, 10000)
local MIN_GROUND_SPEED_              = bind_add_param('GNDSPEED_MIN',            12, 2.5)

-- *****---INTERNAL VARIABLES AND BOOL FLAGS --- DO NOT CHANGE ANYTHING HERE UNLESS MODIFYING THE CODE---*****
local AP_VERSION_NUM_ = 10 * tonumber(FWVersion:major()) + tonumber(FWVersion:minor())
local RFND_ROT_DN_ = 25
local MAX_DIST_ = rangefinder:max_distance_cm_orient(RFND_ROT_DN_) * 0.01
local MODE_FBWA = 5
local COUNT_IN_RANGE_ = 0
local RCMAP_THROTTLE_  = Parameter("RCMAP_THROTTLE")
local THROT_CHANNEL_ = assert(rc:get_channel(RCMAP_THROTTLE_:get()))
local REV_THRUST_AUX_SW_ = assert(rc:find_channel_for_option(64))
local REV_THRUST_SW_LOW_PWM_ = 1000
local REV_THRUST_SW_HIGH_PWM_ = 2000
local TIME_SEC_ = 0
local CALLBACK_TIME_ = 200
local IS_THR_SUP_ = true
local IS_IN_REV_THRUST_ = false
local REV_THRUST_SW_ = false

local REV_THRUST_CUR_PWM_ = REV_THRUST_IDLE_PWM_:get() + REV_THRUST_SPOOL_DPWM_:get()
local TIM_AFTER_TOUCHDN_ = LAND_DET_TIME_:get() * 1000 * 0.01

function UPDATE()
  if (REV_ENABLE_:get() < 1) then
    return
  end
  if (vehicle:get_mode() == MODE_FBWA and (arming:is_armed())
      and (rc:get_pwm(RC_ARM_CHANNEL_:get()) > ARM_CHANNEL_HI_PWM_:get())
      and (THROT_CHANNEL_:norm_input_dz() == 0)) then
    if (RFND_READING() < THRESH_LAND_HEIGHT_:get()) then
      if (COUNT_IN_RANGE_ > TIM_AFTER_TOUCHDN_) then
        TIME_SEC_ = millis():tofloat()
        return ENGAGE_REV_THRUST()
      end
      COUNT_IN_RANGE_ = COUNT_IN_RANGE_ + 1
    else
      COUNT_IN_RANGE_ = 0
    end
  end
  return UPDATE, 100 -- check again in 10Hz
end

function RFND_READING()
  if rangefinder:has_data_orient(RFND_ROT_DN_) then
    return rangefinder:distance_cm_orient(RFND_ROT_DN_) * 0.01
  end
  return MAX_DIST_
end

function DISENGAGE_REV_THRUST()
  THROT_CHANNEL_:set_override(REV_THRUST_IDLE_PWM_:get())
    rc:run_aux_function(64, 0)
  gcs:send_text(4, "Reverse Thrust Disengaged")
  IS_IN_REV_THRUST_ = false
end

function ENGAGE_REV_THRUST()
  local groundspeed = ahrs:groundspeed_vector():length()

  if (IS_THR_SUP_ and (vehicle:get_mode() == MODE_FBWA)) then
    if (millis():tofloat() - TIME_SEC_ < TIME_THROT_WAIT_MS_:get()) then
      THROT_CHANNEL_:set_override(REV_THRUST_IDLE_PWM_:get()) -- set idle throttle and prepare for reverse, letting the motors stop
    else
      REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
      if (REV_THRUST_AUX_SW_:get_aux_switch_pos() == 2) then
        IS_THR_SUP_ = false
        REV_THRUST_SW_ = true --set flag for reverse thrust active
      end
    end
  end

  if ((rc:get_aux_cached(64) == 2)
    and (IS_THR_SUP_ == false)
    and (REV_THRUST_SW_ == true)) then
    TIME_SEC_ = millis():tofloat()
    REV_THRUST_SW_ = false
    IS_IN_REV_THRUST_ = true
    REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
    gcs:send_text(4, "Reverse Thrust Engaged")
  end

  if (IS_IN_REV_THRUST_) then
    if (groundspeed > MIN_GROUND_SPEED_:get()) then
      if (REV_THRUST_AUX_SW_:get_aux_switch_pos() == 2) then
        REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
        THROT_CHANNEL_:set_override(math.min(REV_THRUST_CUR_PWM_, REV_THRUST_HIGH_PWM_:get())) -- set rev thrust
        REV_THRUST_CUR_PWM_ = REV_THRUST_CUR_PWM_ + math.floor(REV_THRUST_CHG_PWM_PS_:get() * CALLBACK_TIME_ / 1000)
        if (millis():tofloat() - TIME_SEC_ > MAXTIME_REV_THRUST_APPLIED_MS_:get()) then
          DISENGAGE_REV_THRUST()
          return --stop the script
        end
      else
        DISENGAGE_REV_THRUST()
        return --stop the script
      end
    else
      DISENGAGE_REV_THRUST()
      return --stop the script
    end
  end
  return ENGAGE_REV_THRUST, CALLBACK_TIME_
end

-- checking Vehicle Type and FW Version
if (not(FWVersion:type() == 3)) then
  gcs:send_text(4, "Rev_Throttle Script: must be running Plane FW")
  return
end

if (AP_VERSION_NUM_ < 41) then
  gcs:send_text(4, "Rev_Throttle Script: must be running Plane4.1 or higher")
  return
end

gcs:send_text(4, "Auto Reverse Thrust Script Loaded")

return UPDATE, 10000 -- delay by 10 sec
