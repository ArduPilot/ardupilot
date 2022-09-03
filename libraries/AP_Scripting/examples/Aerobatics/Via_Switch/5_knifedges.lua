-- knife edge on switch

TRICK_NUMBER = 5 -- selector number recognized to execute trick, change as desired
-- knife-edge angle set by AERO_TRICK_ANG, active for as long as trick id = 5
-------------------------------------------------------------------
-- do not change anything below unless noted

local running = false
local not_bound = true
local initial_yaw_deg = 0
local initial_height = 0
local repeat_count

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end


function wrap_360(angle)  --function returns positive angle modulo360, -710 in returns 10, -10 returns +350
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function wrap_180(angle) 
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
   local _iMax = iMax
   local _last_t = nil
   local _I = 0
   local _P = 0
   local _total = 0
   local _counter = 0
   local _target = 0
   local _current = 0

   -- update the controller.
   function self.update(target, current)
      local now = millis():tofloat() * 0.001
      if not _last_t then
         _last_t = now
      end
      local dt = now - _last_t
      _last_t = now
      local err = target - current
      _counter = _counter + 1

      local P = _kP * err
      _I = _I + _kI * err * dt
      if _iMax then
         _I = constrain(_I, -_iMax, iMax)
      end
      local I = _I
      local ret = P + I

      _target = target
      _current = current
      _P = P
      _total = ret
      return ret
   end

   -- reset integrator to an initial value
   function self.reset(integrator)
      _I = integrator
   end

   function self.set_I(I)
      _kI = I
   end

   function self.set_P(P)
      _kP = P
   end
   
   function self.set_Imax(Imax)
      _iMax = Imax
   end
   
   -- log the controller internals
   function self.log(name, add_total)
      -- allow for an external addition to total
      logger:write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
   end
   -- return the instance
   return self
end

local function height_controller(kP_param,kI_param,KnifeEdge_param,Imax)
   local self = {}
   local kP = kP_param
   local kI = kI_param
   local KnifeEdge = KnifeEdge_param
   local PI = PI_controller(kP:get(), kI:get(), Imax)

   function self.update(target)
      local target_pitch = PI.update(initial_height, ahrs:get_position():alt()*0.01)
      local roll_rad = ahrs:get_roll()
      local ke_add = math.abs(math.sin(roll_rad)) * KnifeEdge:get()
      target_pitch = target_pitch + ke_add
      PI.log("HPI", ke_add)
      return constrain(target_pitch,-45,45)
   end

   function self.reset()
      PI.reset(math.max(math.deg(ahrs:get_pitch()), 3.0))
      PI.set_P(kP:get())
      PI.set_I(kI:get())
   end

   return self
end

-- a controller to target a zero pitch angle and zero heading change, used in a roll
-- output is a body frame pitch rate, with convergence over time tconst in seconds
function pitch_controller(target_pitch_deg, target_yaw_deg, tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local yaw_deg = math.deg(ahrs:get_yaw())

   -- get earth frame pitch and yaw rates
   local ef_pitch_rate = (target_pitch_deg - pitch_deg) / tconst
   local ef_yaw_rate = wrap_180(target_yaw_deg - yaw_deg) / tconst

   local bf_pitch_rate = math.sin(math.rad(roll_deg)) * ef_yaw_rate + math.cos(math.rad(roll_deg)) * ef_pitch_rate
   local bf_yaw_rate   = math.cos(math.rad(roll_deg)) * ef_yaw_rate - math.sin(math.rad(roll_deg)) * ef_pitch_rate
   return bf_pitch_rate, bf_yaw_rate
end

-- a controller for throttle to account for pitch
function throttle_controller()
   local pitch_rad = ahrs:get_pitch()
   local thr_ff = THR_PIT_FF:get()
   local throttle = TRIM_THROTTLE:get() + math.sin(pitch_rad) * thr_ff
   return constrain(throttle, TRIM_THROTTLE:get(), 100.0)
end

function bind_param(name)
   local p = Parameter()
     if not p:init(name) then
     not_bound = true
     end
   return p
end
----------------------------------------------------------------------------------------------
--every trick needs an init, change as needed...to bind the AERO params it uses(which will depend on the trick), and setup PID controllers used by script

function init()
  not_bound = false
  HGT_P = bind_param("AERO_HGT_P") -- height P gain, required for height controller
  HGT_I = bind_param("AERO_HGT_I") -- height I gain, required for height controller
  HGT_KE_BIAS = bind_param("AERO_HGT_KE_BIAS") -- height knifeedge addition for pitch
  THR_PIT_FF = bind_param("AERO_THR_PIT_FF") -- throttle FF from pitch
  TRIM_THROTTLE = bind_param("TRIM_THROTTLE") --usually required for any trick
  TRIM_ARSPD_CM = bind_param("TRIM_ARSPD_CM") --usually required for any trick
  RLL2SRV_TCONST = bind_param("RLL2SRV_TCONST") --usually required for any trick
  PITCH_TCONST = bind_param("PTCH2SRV_TCONST")  --usually required for any trick
  KE_ANG = bind_param("AERO_TRICK_ANG") --this particulat trick needs a target angle for the knife-edge its doing
  TRICK_ID = bind_param("AERO_TRICK_ID") --- required for any trick
  RPT_COUNT = bind_param("AERO_RPT_COUNT") -- if trick can repeat
if not_bound then
   gcs:send_text(0,string.format("Not bound yet"))
   return init, 100
else 
   gcs:send_text(0,string.format("Params bound,Trick %.0f loaded", TRICK_NUMBER))
   height_PI = height_controller(HGT_P, HGT_I, HGT_KE_BIAS, 20.0) -- this trick needs this height PID controller setup to hold height during the trick
   return update, 1000
end
end
-----------------------------------------------------------------------------------------------
--every trick will have its own do_trick function to perform the trick...this will change totally for each different trick

function do_trick(arg1)
  -- arg1 is angle +/-180
        local roll_deg = math.deg(ahrs:get_roll())
        local roll_angle_error = (arg1 - roll_deg)
        if math.abs(roll_angle_error) > 180 then
         if roll_angle_error > 0 then
           roll_angle_error = roll_angle_error - 360
         else 
           roll_angle_error= roll_angle_error +360
         end 
        end
        local roll_rate = roll_angle_error/(RLL2SRV_TCONST:get())
        local target_pitch = height_PI.update(initial_height)
        local pitch_rate, yaw_rate = pitch_controller(target_pitch, initial_yaw_deg, PITCH_TCONST:get())
        local throttle = throttle_controller()
        vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
        return
end
-------------------------------------------------------------------------------------------
--trick should noramlly only have to change the notification text in this routine,initialize trick specific variables, and any parameters needing to be passed to do_trick(..)

function update()
    if (TRICK_ID:get() == TRICK_NUMBER) then
      local current_mode = vehicle:get_mode()
      if arming:is_armed() and running == false then
         if not vehicle:nav_scripting_enable(current_mode) then
            return update, 50
         end
         running = true
         initial_height = ahrs:get_position():alt()*0.01
         initial_yaw_deg = math.deg(ahrs:get_yaw())
         height_PI.reset()
         repeat_count = RPT_COUNT:get() --not used in this trick, knife-edge on as long as mode does not change or sw is not lowered
         -----------------------------------------------trick specific
         gcs:send_text(0, string.format("%d Knife edge", KE_ANG:get()))  --change announcement as appropriate for trick
         ----------------------------------------------------------
      elseif running == true then
         do_trick(KE_ANG:get()) -- change arguments as appropriate for trick
      end
   else 
      running = false
   end
   return update, 50
end

return init,1000

