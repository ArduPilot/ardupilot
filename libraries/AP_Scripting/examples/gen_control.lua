--[[
   control throttle on a generator to achieve a target battery voltage

   This is meant to replace throttle governors on some hybrid drone
   generators. It monitors battery voltage and controls the throttle
   of the generator to maintain a target voltage using a PI controller
--]]
-- luacheck: only 0


UPDATE_RATE_HZ = 10

-- setup a parameter block
local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'GENCTL_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

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

GENCTL_ENABLE = bind_add_param('ENABLE', 1, 0)
if GENCTL_ENABLE:get() <= 0 then
   return
end

-- MIN, MAX and IDLE PWM for throttle output
GENCTL_PWM_MIN = bind_add_param('PWM_MIN', 2, 1000)
GENCTL_PWM_MAX = bind_add_param('PWM_MAX', 3, 2000)
GENCTL_PWM_IDLE = bind_add_param('PWM_IDLE', 4, 1200)

-- P and I gains for controller
GENCTL_PID_P = bind_add_param('PID_P', 5, 0.1)
GENCTL_PID_I = bind_add_param('PID_I', 6, 0.1)

-- maximum I contribution
GENCTL_PID_IMAX = bind_add_param('PID_IMAX', 7, 1.0)

-- RCn_OPTION value for 3 position switch
GENCTL_RC_FUNC  = bind_add_param('RC_FUNC',  8, 300)

-- output servo channel that we will control
GENCTL_THR_CHAN = bind_add_param('THR_CHAN', 9, 0)

-- battery index to monitor, 0 is first battery
GENCTL_BAT_IDX  = bind_add_param('BAT_IDX', 10, 0)

-- target voltage
GENCTL_VOLT_TARG = bind_add_param('VOLT_TARG', 11, 0)

-- maximum slew rate in percent/second for throttle change
GENCTL_SLEW_RATE = bind_add_param('SLEW_RATE', 12, 100)

local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_EMERGENCY = 0

local switch = nil
local thr_pwm = GENCTL_PWM_MIN:get()
local last_switch_pos = nil

-- constrain a value between limits
local function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax,min,max)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kP = kP
   local _kI = kI
   local _iMax = iMax
   local _min = min
   local _max = max
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

      local P = _kP:get() * err
      if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
         _I = _I + _kI:get() * err * dt
      end
      if _iMax:get() > 0 then
         _I = constrain(_I, -_iMax:get(), iMax:get())
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

   -- log the controller internals
   function self.log(name)
      -- allow for an external addition to total
      logger.write(name,'Targ,Curr,P,I,Total','fffff',_target,_current,_P,_I,_total)
   end

   -- return the instance
   return self
end

local thr_PI = PI_controller(GENCTL_PID_P, GENCTL_PID_I, GENCTL_PID_IMAX, 0, 1)
local last_pwm = GENCTL_PWM_MIN:get()

function update()
   if switch == nil then
      switch = rc:find_channel_for_option(math.floor(GENCTL_RC_FUNC:get()))
   end
   if switch == nil or GENCTL_ENABLE:get() < 1 then
      -- nothing to do
      return
   end

   local sw_pos = switch:get_aux_switch_pos()
   if sw_pos ~= last_switch_pos then
      if sw_pos == 0 then
         gcs:send_text(MAV_SEVERITY_INFO,"GenCtl: off")
      elseif sw_pos == 1 then
         gcs:send_text(MAV_SEVERITY_INFO,"GenCtl: idle")
      else
         gcs:send_text(MAV_SEVERITY_INFO,"GenCtl: run")
      end
      last_switch_pos = sw_pos
   end

   if sw_pos == 0 then
      -- force low throttle
      thr_pwm = GENCTL_PWM_MIN:get()
      thr_PI.reset(0)
   elseif sw_pos == 1 then
      -- force idle
      thr_pwm = GENCTL_PWM_IDLE:get()
      thr_PI.reset(0)
   else
      local bat_volt = battery:voltage_resting_estimate(GENCTL_BAT_IDX:get())
      local thr_out = thr_PI.update(GENCTL_VOLT_TARG:get(), bat_volt)
      thr_out = constrain(thr_out, 0, 1)
      thr_pwm = GENCTL_PWM_IDLE:get() + thr_out * (GENCTL_PWM_MAX:get() - GENCTL_PWM_IDLE:get())
      thr_PI.log("GENC")
   end
   local max_change = GENCTL_SLEW_RATE:get() * (GENCTL_PWM_MAX:get() - GENCTL_PWM_MIN:get()) * 0.01 / UPDATE_RATE_HZ
   thr_pwm = constrain(thr_pwm, last_pwm - max_change, last_pwm + max_change)
   last_pwm = thr_pwm
   if GENCTL_THR_CHAN:get() > 0 then
      SRV_Channels:set_output_pwm_chan(GENCTL_THR_CHAN:get()-1, math.floor(thr_pwm))
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
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

gcs:send_text(MAV_SEVERITY_INFO,"Loaded gen_control.lua")

-- start running update loop
return protected_wrapper()
