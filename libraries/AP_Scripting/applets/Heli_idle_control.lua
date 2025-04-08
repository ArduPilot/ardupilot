-- idle_control.lua: a closed loop control throttle control while on ground idle (trad-heli)
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: need-check-nil

local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'IDLE_'
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- parameters for idle control

IDLE_GAIN_I = bind_add_param('GAIN_I', 1, 0.05)
IDLE_GAIN_P = bind_add_param('GAIN_P', 2, 0.25)
IDLE_GAIN_MAX = bind_add_param('GAIN_MAX', 3, 1)
IDLE_MAX = bind_add_param('MAX', 4, 17)
IDLE_RANGE = bind_add_param('RANGE', 5, 300)
IDLE_SETPOINT = bind_add_param('SETPOINT', 6, 600)
IDLE_RPM_ENABLE = bind_add_param('RPM_ENABLE', 7, 0)

-- internal variables

local thr_out = nil
local idle_control_active = false
local idle_control_active_last = false
local last_scaled_output = nil
local idle_default = nil
local ramp_up_complete = false
local idle_adjusted = false
local last_idc_time = nil
local time_now = nil
local pv = nil
local thr_ctl = nil
local thr_out_last = nil
local pot_input = rc:find_channel_for_option(301)
local switch_rsc = rc:find_channel_for_option(32)
local rsc_output = SRV_Channels:find_channel(31)
local SERVO_MAX = Parameter('SERVO' .. (rsc_output+1) .. '_MAX')
local SERVO_MIN = Parameter('SERVO' .. (rsc_output+1) .. '_MIN')
local SERVO_REV = Parameter('SERVO' .. (rsc_output+1) .. '_REVERSED')
local servo_range = SERVO_MAX:get() - SERVO_MIN:get()
local H_RSC_IDLE = Parameter('H_RSC_IDLE')
local H_RSC_RUNUP_TIME = Parameter('H_RSC_RUNUP_TIME')

-- map function

function map(x, in_min, in_max, out_min, out_max)
   return out_min + (x - in_min)*(out_max - out_min)/(in_max - in_min)
end

-- constrain function

local function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

-- PI controller function
local function PI_controller(kP,kI,iMax,min,max)

   local self = {}

   local _kP = kP
   local _kI = kI
   local _iMax = iMax
   local _min = min
   local _max = max
   local _last_t = nil
   local _I = 0
   local _total = 0
   local _counter = 0

   function self.update(target, current)
      local now = millis()
      if not _last_t then
         _last_t = now
      end
      local dt = (now - _last_t):tofloat()*0.001
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
      local ret = P + _I
      _total = ret
      return ret
   end

   function self.reset(integrator)
      _I = integrator
   end

   return self
end

local thr_PI = PI_controller(IDLE_GAIN_P, IDLE_GAIN_I, IDLE_GAIN_MAX, 0, 1)

-- main update function

function update()  

   local armed = arming:is_armed()

  -- aux potentiometer for manual adjusting of idle

   local pot_pos = pot_input:norm_input()
   local thr_man = map(pot_pos,-1,1,0,1)

   if armed == false then
      idle_default = H_RSC_IDLE:get()
      idle_control_active = false
      ramp_up_complete = false
      idle_adjusted = false
      thr_PI.reset(0)
   else 
      if switch_rsc:get_aux_switch_pos() == 0 and vehicle:get_likely_flying() == false then
         if H_RSC_IDLE:get()~= idle_default then
            H_RSC_IDLE:set(idle_default)
            gcs:send_text(5, "H_RSC_IDLE set to:".. tostring(H_RSC_IDLE:get()))
         end
         if IDLE_RPM_ENABLE:get() == 0 then
            ramp_up_complete = false
            idle_adjusted = false
            thr_out = H_RSC_IDLE:get() + thr_man*(IDLE_MAX:get() - H_RSC_IDLE:get())
         else
            if thr_man == 0 then
              idle_control_active = false
              if idle_control_active_last ~= idle_control_active then
                 gcs:send_text(5, "idle control: OFF")
              end
               thr_out = H_RSC_IDLE:get()
               thr_ctl = 0
               thr_PI.reset(0)
            else
               local rpm_current = RPM:get_rpm((IDLE_RPM_ENABLE:get())-1)
               ramp_up_complete = false
               idle_adjusted = false
               if rpm_current < (IDLE_SETPOINT:get() - IDLE_RANGE:get()) then
                  thr_out = H_RSC_IDLE:get() + thr_man*(IDLE_MAX:get() - H_RSC_IDLE:get())
                  thr_out_last = thr_out
               elseif rpm_current > (IDLE_SETPOINT:get() + IDLE_RANGE:get()) then 
                  thr_out = H_RSC_IDLE:get()
                  thr_out_last = thr_out
               else
                  -- throttle output set from the PI controller
                  pv = rpm_current/(IDLE_SETPOINT:get())
                  thr_ctl = thr_PI.update(1, pv)
                  thr_ctl = constrain(thr_ctl,0,1)
                  thr_out = H_RSC_IDLE:get() + thr_ctl*(IDLE_MAX:get() - H_RSC_IDLE:get())
                  if thr_out_last == nil then
                     thr_out_last = 0
                  end
                  thr_out = constrain(thr_out, thr_out_last-0.05, thr_out_last+0.05)
                  thr_out_last = thr_out
                  idle_control_active = true
               end
               if idle_control_active_last ~= idle_control_active then
                  gcs:send_text(5, "idle control: ON")
               end
             end
         end
         last_idc_time = millis()
         last_scaled_output = thr_out/100
         if SERVO_REV:get() == 0 then
            SRV_Channels:set_output_pwm_chan_timeout(rsc_output, math.floor((last_scaled_output*servo_range)+SERVO_MIN:get()), 150)
         else
            SRV_Channels:set_output_pwm_chan_timeout(rsc_output, math.floor(SERVO_MAX:get()-(last_scaled_output*servo_range)), 150)
         end
      else
         -- motor interlock disabled, armed state, flight
         idle_control_active = false
         if idle_control_active_last ~= idle_control_active then
            gcs:send_text(5, "idle control: deactivated")
         end
         if ramp_up_complete ~= true then
            time_now = millis()
            if ((time_now-last_idc_time):tofloat()*0.001) < H_RSC_RUNUP_TIME:get() then
               if idle_adjusted ~= true then
                  H_RSC_IDLE:set(last_scaled_output*100)
                  idle_adjusted = true
                  gcs:send_text(5, "H_RSC_IDLE updated for ramp up:".. tostring(H_RSC_IDLE:get()))
               end
            else
               if H_RSC_IDLE:get()~= idle_default then
                  H_RSC_IDLE:set(idle_default)
                  gcs:send_text(5, "H_RSC_IDLE default restored:".. tostring(H_RSC_IDLE:get()))
               end
               ramp_up_complete = true
            end
         end
      end
  -- update notify variable
  idle_control_active_last = idle_control_active
   end

   return update, 100 -- 10Hz rate
end

gcs:send_text(5, "idle_control_running")

return update()
