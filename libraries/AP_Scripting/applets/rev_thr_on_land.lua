--[[
    This script automatically engages reverse thrust on planes in FBWA mode on touchdown.
    Prerequisities:
    - ArduPlane 4.1 or later
    - A Fixed Wing platform capable of CTOL operations
    - ESCs capable of reversing motor direction
    - A Separate RC channel for triggering throttle reverse, assigned by RCx_FUNCTION=64
    - A good rangefinder (downward facing) capable of reliably reading anything between [0.1, 5]m
    - A separate RC channel that will serve as an "arming" switch for reverse throttle operation. 
      This script uses RC_ARM_CHANNEL_ for this purpose (i.e. reverse throttle is armed whenever this switch PWM >1500, 
      set by RC_ARM_CHANNEL_HI_PWM_)
    Params to adjust for a particular fw frame:
    - THRESH_LAND_HEIGHT_ (m) - this is the height read by rangefinder which corresponds to a plane's height above ground
      on touchdown. Set this a couple of cms higher than the height reading by your rangefider when the aircraft is on the ground
      but well within the height that the rangefinder can reliably read.
    - LAND_DET_TIME_ (s) - time during which rangefinder height estimate is consistently below THRESH_LAND_HEIGHT_
    - MIN_GROUND_SPEED_ (m/s) - minimum ground speed at which the autopilot will disengage reverse thrust and switch to
      idle forward thrust (FWD_THRUST_IDLE_PWM_)
    - RC_REV_THROTTLE_CHANNEL_NUM_ - is the RC channel that tells autopilot to reverse motor directions (RCx_FUNCTION=64)
    - REV_THRUST_SW_HIGH_PWM_ - this sets high position that the RCx_FUNCTION=64 switch should have for reverse throttle operation
    - REV_THRUST_SW_LOW_PWM_ - this sets low position that the RCx_FUNCTION=64 switch should have for forward throttle operation
    - REV_THRUST_HIGH_PWM_, REV_THRUST_IDLE_PWM_ - max and min pwm values for RC_CHANNEL_NUM_ when in reverse throttle mode
    - FWD_THRUST_IDLE_PWM_ - min value of the motors when in forward throttle mode, set this to REV_THRUST_IDLE_PWM_ for smooth operation
    - REV_THRUST_CHG_PWM_PS_(pwm) - this is the motors' linear acceleration time in seconds, from REV_THRUST_IDLE_PWM_ to REV_THRUST_HIGH_PWM_
    - MAXTIME_REV_THRUST_APPLIED_MS_ (ms) - maximum time, in seconds, reverse throttle is allowed to apply
    - TIME_THROT_WAIT_MS_ (ms) - time the throttle is suppressed, sitting at forward idle (used to ensure props are not spinning at a rate higher 
      than FWD_THRUST_IDLE_PWM_ shortly before reverse throttle is applied)
    - REV_THRUST_SPOOL_DPWM_ (pwm) - initial increase in pwm to apply when in reverse throttle mode
]]--


-- *****---PARAMS TO CHANGE---*****
--REV arming parameters
local RC_ARM_IN_CHANNEL_ = 6
local ARM_CHANNEL_HI_PWM_ = 1500

--REV switch setup
local RC_REV_THROTTLE_CHANNEL_NUM_ = 11
local REV_THRUST_SW_LOW_PWM_ = 1000
local REV_THRUST_SW_HIGH_PWM_ = 2000

--REV activation and spool up parameters
local LAND_DET_TIME_ = 1
local THRESH_LAND_HEIGHT_ = 0.12
local TIME_THROT_WAIT_MS_ = 1000
local REV_THRUST_SPOOL_DPWM_ = 60
local REV_THRUST_CHG_PWM_PS_ = 150

--REV strength and duration setup
local FWD_THRUST_IDLE_PWM_ = 990
local REV_THRUST_IDLE_PWM_ = 990
local REV_THRUST_HIGH_PWM_ = 1600
local MAXTIME_REV_THRUST_APPLIED_MS_ = 10000
local MIN_GROUND_SPEED_ = -1

-- *****---INTERNAL VARIABLES AND BOOL FLAGS --- DO NOT CHANGE ANYTHING HERE UNLESS MODIFYING THE CODE---*****
local RFND_ROT_DN_ = 25
local MAX_DIST_ = rangefinder:max_distance_cm_orient(RFND_ROT_DN_) * 0.01
local MODE_FBWA = 5
local COUNT_IN_RANGE_ = 0
local THROT_CHANNEL_ = rc:get_channel(RCMAP_THROTTLE:get())
local REV_THRUST_AUX_SW_ = rc:get_channel(RC_REV_THROTTLE_CHANNEL_NUM_)
local TIME_SEC_ = 0
local CALLBACK_TIME_ = 200
local IS_THR_SUP_ = true
local IS_IN_REV_THRUST_ = false
local REV_THRUST_SW_ = false

local REV_THRUST_CUR_PWM_ = FWD_THRUST_IDLE_PWM_ + REV_THRUST_SPOOL_DPWM_
local TIM_AFTER_TOUCHDN_ = LAND_DET_TIME_ * 1000 * 0.01

function UPDATE()
  if (vehicle:get_mode() == MODE_FBWA and (arming:is_armed())
      and (rc:get_pwm(RC_ARM_IN_CHANNEL_) > ARM_CHANNEL_HI_PWM_)) then
    if (RFND_READING() < THRESH_LAND_HEIGHT_) then
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
  THROT_CHANNEL_:set_override(REV_THRUST_IDLE_PWM_)
  REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_LOW_PWM_)
  gcs:send_text(4, "Reverse Thrust Disengaged")
  IS_IN_REV_THRUST_ = false
end

function ENGAGE_REV_THRUST()
  local groundspeed = ahrs:groundspeed_vector():length()

  if (IS_THR_SUP_ and (vehicle:get_mode() == MODE_FBWA)) then
    if (millis():tofloat() - TIME_SEC_ < TIME_THROT_WAIT_MS_) then
      THROT_CHANNEL_:set_override(FWD_THRUST_IDLE_PWM_) -- set idle throttle and prepare for reverse, letting the motors stop
    else
      REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
      if (rc:get_pwm(RC_REV_THROTTLE_CHANNEL_NUM_) == REV_THRUST_SW_HIGH_PWM_) then
        IS_THR_SUP_ = false
        REV_THRUST_SW_ = true --set flag for reverse thrust active
      end
    end
  end

  if ((rc:get_pwm(RC_REV_THROTTLE_CHANNEL_NUM_) == REV_THRUST_SW_HIGH_PWM_)
    and (IS_THR_SUP_ == false)
    and (REV_THRUST_SW_ == true)) then
    TIME_SEC_ = millis():tofloat()
    REV_THRUST_SW_ = false
    IS_IN_REV_THRUST_ = true
    REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
    gcs:send_text(4, "Reverse Thrust Engaged")
  end

  if (IS_IN_REV_THRUST_) then
    if (groundspeed > MIN_GROUND_SPEED_) then
      if (rc:get_pwm(RC_REV_THROTTLE_CHANNEL_NUM_) == REV_THRUST_SW_HIGH_PWM_) then
        REV_THRUST_AUX_SW_:set_override(REV_THRUST_SW_HIGH_PWM_)
        THROT_CHANNEL_:set_override(math.min(REV_THRUST_CUR_PWM_, REV_THRUST_HIGH_PWM_)) -- set rev thrust
        REV_THRUST_CUR_PWM_ = REV_THRUST_CUR_PWM_ + math.floor(REV_THRUST_CHG_PWM_PS_ * CALLBACK_TIME / 1000)
        if (millis():tofloat() - TIME_SEC_ > MAXTIME_REV_THRUST_APPLIED_MS_) then
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

gcs:send_text(4, "Auto Reverse Thrust Script Loaded")

return UPDATE, 10000 -- delay by 10 sec