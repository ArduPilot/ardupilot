--[[ perform simple aerobatic manoeuvres in AUTO mode
cmd = 1: axial rolls, arg1 = roll rate dps, arg2 = number of rolls
cmd = 2: loops or 180deg return, arg1 = pitch rate dps, arg2 = number of loops, if zero do a 1/2 cuban8-like return
cmd = 3: rolling circle, arg1 = radius, arg2 = number of rolls
cmd = 4: knife edge at any angle, arg1 = roll angle to hold, arg2 = duration
cmd = 5: pause, holding heading and alt to allow stabilization after a move, arg1 = duration in seconds
]]--

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 70
local PARAM_TABLE_PREFIX = 'AEROM_'
assert(param:add_table(PARAM_TABLE_KEY, "AEROM_", 30), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

HGT_P = bind_add_param('HGT_P', 1, 1)
HGT_I = bind_add_param('HGT_I', 2, 2)
HGT_KE_BIAS = bind_add_param('HGT_KE_ADD', 3, 20)
THR_PIT_FF = bind_add_param('THR_PIT_FF', 4, 80)
SPD_P = bind_add_param('SPD_P', 5, 5)
SPD_I = bind_add_param('SPD_I', 6, 25)
ERR_CORR_TC = bind_add_param('ERR_COR_TC', 7, 3)
ROLL_CORR_TC = bind_add_param('ROL_COR_TC', 8, 0.25)
AUTO_MIS = bind_add_param('AUTO_MIS', 9, 0)
AUTO_RAD = bind_add_param('AUTO_RAD', 10, 40)
TIME_CORR_P = bind_add_param('TIME_COR_P', 11, 1.0)
ERR_CORR_P = bind_add_param('ERR_COR_P', 12, 2.0)
ERR_CORR_D = bind_add_param('ERR_COR_D', 13, 2.8)

--local VEL_TC = bind_add_param('VEL_TC', 8, 3)

function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local NAV_TAKEOFF = 22
local NAV_WAYPOINT = 16
local NAV_SCRIPT_TIME = 42702

local LOOP_RATE = 20
DO_JUMP = 177
k_throttle = 70

local TRIM_THROTTLE = Parameter("TRIM_THROTTLE")
local TRIM_ARSPD_CM = Parameter("TRIM_ARSPD_CM")
local RLL2SRV_TCONST = Parameter("RLL2SRV_TCONST")
local PITCH_TCONST = Parameter("PTCH2SRV_TCONST")

local last_roll_err = 0.0
local last_id = 0
local initial_yaw_deg = 0
local wp_yaw_deg = 0
local initial_height = 0
local repeat_count = 0
local running = false
local roll_stage = 0
local MIN_SPEED = 0.1
local LOOKAHEAD = 1

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

-- roll angle error 180 wrap to cope with errors while in inverted segments
function roll_angle_error_wrap(roll_angle_error)
   if math.abs(roll_angle_error) > 180 then
    if roll_angle_error > 0 then
       roll_angle_error = roll_angle_error - 360
    else 
       roll_angle_error= roll_angle_error +360
    end 
   end
   return roll_angle_error
end
    
--roll controller to keep wings level in earth frame. if arg is 0 then level is at only 0 deg, otherwise its at 180/-180 roll also for loops
function earth_frame_wings_level(arg)
   local roll_deg = math.deg(ahrs:get_roll())
   local roll_angle_error = 0.0
   if (roll_deg > 90 or roll_deg < -90) and arg ~= 0 then
    roll_angle_error = 180 - roll_deg
   else
    roll_angle_error = - roll_deg
   end
   return roll_angle_error_wrap(roll_angle_error)/(RLL2SRV_TCONST:get())
end


function wrap_360(angle)
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

function wrap_pi(angle)
   local angle_deg = math.deg(angle)
   local angle_wrapped = wrap_180(angle_deg)
   return math.rad(angle_wrapped)
end

function wrap_2pi(angle)
   local angle_deg = math.deg(angle)
   local angle_wrapped = wrap_360(angle_deg)
   return math.rad(angle_wrapped)
end

function euler_rad_ef_to_bf(roll, pitch, yaw, ef_roll_rate, ef_pitch_rate, ef_yaw_rate)
   local sr = math.sin(roll)
   local cr = math.cos(roll)
   local sp = math.sin(pitch)
   local cp = math.cos(pitch)
   local sy = math.sin(yaw)
   local cy = math.cos(yaw)

   local bf_roll_rate = ef_roll_rate + -sp*ef_yaw_rate
   local bf_pitch_rate = cr*ef_pitch_rate + sr*cp*ef_yaw_rate
   local bf_yaw_rate = -sr*ef_pitch_rate + cr*cp*ef_yaw_rate

   return makeVector3f(bf_roll_rate, bf_pitch_rate, bf_yaw_rate)

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
      logger.write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
   end
   -- return the instance
   return self
end

local function speed_controller(kP_param,kI_param, kFF_roll_param, kFF_pitch_param, Imax)
   local self = {}
   local kFF_roll = kFF_roll_param
   local kFF_pitch = kFF_pitch_param
   local PI = PI_controller(kP_param:get(), kI_param:get(), Imax)

   function self.update(target)
      local current_speed = ahrs:get_velocity_NED():length()
      local throttle = PI.update(target, current_speed)
      throttle = throttle + math.sin(ahrs:get_pitch())*kFF_pitch:get()
      throttle = throttle + math.abs(math.sin(ahrs:get_roll()))*kFF_roll:get()
      return throttle
   end

   function self.reset()
      PI.reset(0)
      local temp_throttle = self.update(ahrs:get_velocity_NED():length())
      local current_throttle = SRV_Channels:get_output_scaled(k_throttle)
      PI.reset(current_throttle-temp_throttle)
   end

   return self
end

local function height_controller(kP_param,kI_param,KnifeEdge_param,Imax)
   local self = {}
   local kP = kP_param
   local kI = kI_param
   local KnifeEdge = KnifeEdge_param
   local PI = PI_controller(kP:get(), kI:get(), Imax)

   function self.update(target)
      local target_pitch = PI.update(target, ahrs:get_position():alt()*0.01)
      local roll_rad = ahrs:get_roll()
      local ke_add = math.abs(math.sin(roll_rad)) * KnifeEdge:get()
      target_pitch = target_pitch + ke_add
      PI.log("HPI", ke_add)
      return target_pitch
   end

   function self.reset()
      PI.reset(math.max(math.deg(ahrs:get_pitch()), 3.0))
      PI.set_P(kP:get())
      PI.set_I(kI:get())
   end

   return self
end

local height_PI = height_controller(HGT_P, HGT_I, HGT_KE_BIAS, 20.0)
local speed_PI = speed_controller(SPD_P, SPD_I, HGT_KE_BIAS, THR_PIT_FF, 100.0)


function euler_rate_ef_to_bf(rrate, prate, yrate, roll, pitch, yaw)

   local sr = math.sin(roll)
   local cr = math.cos(roll)
   local sp = math.sin(pitch)
   local cp = math.cos(pitch)
   local sy = math.sin(yaw)
   local cy = math.cos(yaw)
   
   local bf_roll_rate = rrate -sp*yrate
   local bf_pitch_rate = cr*prate + sr*cp*yrate
   local bf_yaw_rate = -sr*prate + cr*cp*yrate

   return makeVector3f(bf_roll_rate, bf_pitch_rate, bf_yaw_rate)
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
   return constrain(throttle, 0, 100.0)
end

-- recover entry altitude
function recover_alt()
       local target_pitch = height_PI.update(initial_height)
       local pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
       return target_pitch, pitch_rate, yaw_rate
end

function get_wp_location(i)
   local m = mission:get_item(i)
   local loc = Location()
   loc:lat(m:x())
   loc:lng(m:y())
   loc:relative_alt(true)
   loc:terrain_alt(false)
   loc:origin_alt(false)
   loc:alt(math.floor(m:z()*100))
   return loc
end

function resolve_jump(i)
   local m = mission:get_item(i)
   while m:command() == DO_JUMP do
      i = math.floor(m:param1())
      m = mission:get_item(i)
   end
   return i
end

--------Trajectory definitions---------------------
function path_circle(t, radius, arg2, arg3, arg4)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 1.0-math.cos(t), 0)
   return vec:scale(radius), 0.0
end

function knife_edge_circle(t, radius, arg2, arg3, arg4)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 1.0-math.cos(t), 0)
   return vec:scale(radius), math.pi/2
end

function climbing_circle(t, radius, height, arg3, arg4)
   local angle = t*math.pi*2
   local vec = makeVector3f(radius*math.sin(angle), radius*(1.0-math.cos(angle)), -math.sin(0.5*angle)*height)
   return vec, 0.0
end

function figure_eight(t, r, bank_angle, arg3, arg4)
   local T = 3.0*math.pi*r + r*math.sqrt(2) + 2*r

   local rsqr2 = r*math.sqrt(2)

   local pos
   local roll
   if (t < rsqr2/T) then
      pos = makeVector3f(T*t, 0.0, 0.0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0)/T) then
      pos = makeVector3f(r*math.cos(T*t/r - math.sqrt(2) - math.pi/2)+rsqr2, r + r*math.sin(T*t/r - math.sqrt(2) - math.pi/2), 0)
      roll = math.rad(bank_angle)
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r/4)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + 3*r/4)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0)/T) then
      pos = makeVector3f(r*math.cos(-T*t/r +5.0*math.pi/4.0 + math.sqrt(2) + 1 - math.pi/4) - r*math.sqrt(2.0), r + r*math.sin(-T*t/r +5.0*math.pi/4.0 + math.sqrt(2) + 1 - math.pi/4), 0)
      roll = -math.rad(bank_angle)
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + r/4.0)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)), r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)), 0)
      roll = 0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + 3*r/4.0)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)), r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)), 0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + r)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)),  r  +r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)), 0)
      roll = 0.0
   else
      pos = makeVector3f(r*math.cos(T*t/r - (6*math.pi/4.0 + math.sqrt(2) +2 ))+rsqr2, r + r*math.sin(T*t/r - (6*math.pi/4.0 + math.sqrt(2) +2 )), 0)
      roll = math.rad(bank_angle)
   end
   return pos, roll

end

function loop(t, radius, bank_angle, arg3, arg4)
   local num_loops = math.abs(arg3)
   if(arg3 <= 0.0) then
      num_loops = 1
   end

   t = num_loops*t*math.pi*2
   local vec = makeVector3f(math.sin(t), 0.0, -1.0 + math.cos(t))
   return vec:scale(radius), math.rad(bank_angle)
end

function straight_roll(t, length, num_rolls, arg3, arg4)

   local vec = makeVector3f(t*length, 0.0, 0.0)
   return vec, t*num_rolls*2*math.pi
end

function straight_flight(t, length, bank_angle, arg3, arg4)
   local pos = makeVector3f(t*length, 0, 0)
   local roll = math.rad(bank_angle)
   return pos, roll
end

function rolling_circle(t, radius, num_rolls, arg3, arg4)
   --t = t*math.pi*2
   local vec = Vector3f()
   if radius < 0.0 then
      vec = makeVector3f(math.sin(2*math.pi*t), -1.0+math.cos(2*math.pi*t), 0)
   else
      vec = makeVector3f(math.sin(2*math.pi*t), 1.0-math.cos(2*math.pi*t), 0)
   end
   return vec:scale(math.abs(radius)), t*num_rolls*2*math.pi
end

function banked_circle(t, radius, bank_angle, arg3, arg4)
   --t = t*math.pi*2
   local vec = Vector3f()
   if radius < 0.0 then
      vec = makeVector3f(math.sin(2*math.pi*t), -1.0+math.cos(2*math.pi*t), 0)
   else
      vec = makeVector3f(math.sin(2*math.pi*t), 1.0-math.cos(2*math.pi*t), 0)
   end
   return vec:scale(math.abs(radius)), math.deg(bank_angle)
end

function half_cuban_eight(t, r, unused, arg3, arg4)
   local T = 3.0*math.pi*r/2.0 + 2*r*math.sqrt(2) + r

   local trsqr2 = 2*r*math.sqrt(2)

   local pos
   local roll
   if (t < trsqr2/T) then
      pos = makeVector3f(T*t, 0.0, 0.0)
      roll = 0.0
   elseif (t < (trsqr2 + 5.0*math.pi*r/4.0)/T) then
      pos = makeVector3f(r*math.cos(T*t/r - 2*math.sqrt(2) - math.pi/2)+trsqr2, 0, -r - r*math.sin(T*t/r - 2*math.sqrt(2) - math.pi/2))
      roll = 0.0
   elseif (t < (trsqr2 + 5.0*math.pi*r/4.0 + r/4)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = 0.0
   elseif (t < (trsqr2 + 5.0*math.pi*r/4.0 + 3*r/4)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = (t - (trsqr2 + 5.0*math.pi*r/4.0 + r/4)/T)*2*math.pi*T/(r)
   elseif (t < (trsqr2 + 5.0*math.pi*r/4.0 + r)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = math.pi
   else
      pos = makeVector3f(r*math.cos(-T*t/r +5.0*math.pi/4.0 + 2*math.sqrt(2) + 1 - math.pi/4), 0, -r  -r*math.sin(-T*t/r +5.0*math.pi/4.0 + 2*math.sqrt(2) + 1 - math.pi/4))
      roll = math.pi
      --roll = 0
   end

   return pos, roll

end

function cuban_eight(t, r, unused, arg3, arg4)
   local T = 3.0*math.pi*r + r*math.sqrt(2) + 2*r

   local rsqr2 = r*math.sqrt(2)

   local pos
   local roll
   if (t < rsqr2/T) then
      pos = makeVector3f(T*t, 0.0, 0.0)
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0)/T) then
      pos = makeVector3f(r*math.cos(T*t/r - math.sqrt(2) - math.pi/2)+rsqr2, 0, -r - r*math.sin(T*t/r - math.sqrt(2) - math.pi/2))
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r/4)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)))
      roll = 0.0
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + 3*r/4)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)))
      roll = (t - (rsqr2 + 5.0*math.pi*r/4.0 + r/4)/T)*2*math.pi*T/(r)
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r)/T) then
      pos = makeVector3f(r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2))*(-r*math.sqrt(2)))
      roll = math.pi
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0)/T) then
      pos = makeVector3f(r*math.cos(-T*t/r +5.0*math.pi/4.0 + math.sqrt(2) + 1 - math.pi/4) - r*math.sqrt(2.0), 0, -r - r*math.sin(-T*t/r +5.0*math.pi/4.0 + math.sqrt(2) + 1 - math.pi/4))
      roll = math.pi
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + r/4.0)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)))
      roll = math.pi
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + 3*r/4.0)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)))
      roll = math.pi +(t - (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + r/4.0)/T)*2 *math.pi*T/(r)
   elseif (t < (rsqr2 + 5.0*math.pi*r/4.0 + r + 3*math.pi*r/2.0 + r)/T) then
      pos = makeVector3f(-r/math.sqrt(2) + (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2)*(r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*t/r - 5*math.pi/4 - math.sqrt(2) - 1 - 3*math.pi/2.0 )*(-r*math.sqrt(2)))
      roll = 0.0
   else
      pos = makeVector3f(r*math.cos(T*t/r - (6*math.pi/4.0 + math.sqrt(2) +2 ))+rsqr2, 0, -r - r*math.sin(T*t/r - (6*math.pi/4.0 + math.sqrt(2) +2 )))
      roll = 0.0
   end
   return pos, roll

end

function half_reverse_cuban_eight(t, r, arg2, arg3, arg4)
   local T = 3.0*math.pi*r/2.0 + 2*r*math.sqrt(2) + r

   local trsqr2 = 2*r*math.sqrt(2)

   local pos
   local roll


   if(t < (math.pi*r/4)/T) then
      pos = makeVector3f(r*math.cos(-T*(1-t)/r +5.0*math.pi/4.0 + 2*math.sqrt(2) + 1 - math.pi/4), 0, -r  -r*math.sin(-T*(1-t)/r +5.0*math.pi/4.0 + 2*math.sqrt(2) + 1 - math.pi/4))
      
      roll = 0
   elseif (t < (math.pi*r/4 + r/4)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = 0
   elseif (t < (math.pi*r/4 + 3*r/4)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = (t - (math.pi*r/4 + r/4)/T)*2*math.pi*T/(r)
   elseif (t < (math.pi*r/4 + r)/T) then
      pos = makeVector3f(3*r/math.sqrt(2) + (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)), 0, -r  -r/math.sqrt(2) - (T*(1-t)/r - 5*math.pi/4 - 2*math.sqrt(2))*(-r*math.sqrt(2)))
      roll = math.pi
   elseif (t < (3*math.pi*r/2 + r) /T) then
      pos = makeVector3f(r*math.cos(T*(1-t)/r - 2*math.sqrt(2) - math.pi/2)+trsqr2, 0, -r - r*math.sin(T*(1-t)/r - 2*math.sqrt(2) - math.pi/2))
      roll = math.pi
   else
      pos = makeVector3f(T*(1-t), 0.0, 0.0)
      roll = math.pi
   end

   return pos, roll

end

function humpty_bump(t, r, h, arg3, arg4)
   assert(h >= 2*r)
   local T = 2*(math.pi*r + h - r)
   local l = h - 2*r 
   local pos
   local roll
   if (t < (math.pi*r/2)/T) then
      pos = makeVector3f(r*math.cos(T*t/r - math.pi/2), 0, -r  -r*math.sin(T*t/r - math.pi/2))
      roll = 0
   elseif (t < (math.pi*r/2 + l/4)/T) then
      pos = makeVector3f(r, 0, -r -(T*t - r*math.pi/2))
      roll = 0
   elseif (t < (math.pi*r/2 + 3*l/4)/T) then
      pos = makeVector3f(r, 0, -r -(T*t - r*math.pi/2))
      roll = (t - (math.pi*r/2 + l/4)/T)*2*math.pi*T/l
   elseif (t < (math.pi*r/2 + l)/T) then
      pos = makeVector3f(r, 0, -r -(T*t - r*math.pi/2))
      roll = math.pi
   elseif (t < (3*math.pi*r/2 + l)/T) then
      pos = makeVector3f(2*r + r*math.cos(T*t/r - 3*math.pi/2 - l/r), 0, -r-l  +r*math.sin(T*t/r - 3*math.pi/2 - l/r))
      roll = math.pi
   elseif (t < (3*math.pi*r/2 + 2*l)/T) then
      pos = makeVector3f(3*r,0, -r -l + (T*t - 3*r*math.pi/2.0 -l))
      roll = math.pi
   elseif (t < (2*math.pi*r + 2*l)/T) then
      pos = makeVector3f(2*r + r*math.cos(T*t/r - 3*math.pi/2 -2*l/r),0, -r + r*math.sin(T*t/r - 3*math.pi/2 -2*l/r))
      roll = math.pi
   else
      pos = makeVector3f(2*r -(T*t - 2*r*math.pi - 2*l), 0, 0)
      roll = math.pi
   end
   return pos, roll
end

function scale_figure_eight(t, r, bank_angle, arg3, arg4)
   local T = 4*math.pi + 2
   local pos
   local roll
   if (t < (math.pi/2)/T) then
      pos = makeVector3f(r*math.cos(T*t - math.pi/2), r  +r*math.sin(T*t - math.pi/2), 0)
      roll = math.rad(bank_angle)
   elseif (t < (5*math.pi/2)/T) then
      pos = makeVector3f(2*r + r*math.cos(T*t + math.pi/2), r -r*math.sin(T*t + math.pi/2), 0)
      roll = -math.rad(bank_angle)
   elseif (t < (4*math.pi)/T) then
      pos = makeVector3f(r*math.cos(T*t - math.pi/2), r + r*math.sin(T*t - math.pi/2), 0)
      roll = math.rad(bank_angle)
   else
      pos = makeVector3f(r*(T*t - 4*math.pi), 0, 0)
      roll = 0
   end
   return pos, roll
end

function test_height_control(t, length, arg2, arg3, arg4)
   if t < 0.25 then
      return makeVector3f(t*length, 0.0, 0.0), 0.0
   elseif t < 0.5 then
      return makeVector3f(t*length, 0.0, -10.0), 0.0
   elseif t < 0.75 then
      return makeVector3f(t*length, 0.0, -20.0), 0.0
   else
      return makeVector3f(t*length, 0.0, -30.0), 0.0
   end
end

function test_lane_change(t, length, arg2, arg3, arg4)
   if t < 0.25 then
      return makeVector3f(t*length, 0.0, 0.0), 0.0
   elseif t < 0.5 then
      return makeVector3f(t*length, 10.0, 0.0), 0.0
   elseif t < 0.75 then
      return makeVector3f(t*length, 20.0, 0.0), 0.0
   else
      return makeVector3f(t*length, 30.0, 0.0), 0.0
   end
end

function path_straight_roll(t, length, num_rolls, arg3, arg4)

   local vec = makeVector3f(t*length, 0.0, 0.0)
   return vec, t*num_rolls*2*math.pi
end


--todo: change y coordinate to z for vertical box
--function aerobatic_box(t, l, w, r):
function horizontal_rectangle(t, arg1, arg2, arg3, arg4)
   
   local r = math.abs(arg3)
   if(arg3 <= 0.0) then
      gcs:send_text(0, string.format("Invalid radius value of : %f, using default.", arg3))
      r = math.min(arg1, arg2)/3.0
   end
   local bank_angle = math.abs(arg4)
   local l = arg1 - 2*r
   local w = arg2 - 2*r
   local perim = 2*l + 2*w + 2*math.pi*r
   local pos
   if (t < 0.5*l/(perim)) then
      pos = makeVector3f(perim*t, 0.0, 0.0)
   elseif (t < (0.5*l + 0.5*math.pi*r)/perim) then
      pos = makeVector3f(0.5*l + r*math.sin((perim*t - 0.5*l)/r), r*(1 - math.cos((perim*t - 0.5*l)/r)), 0.0)
   elseif (t < (0.5*l + 0.5*math.pi*r + w)/perim) then
      pos = makeVector3f(0.5*l + r, r + (perim*t - (0.5*l + 0.5*math.pi*r)), 0.0)
   elseif(t < (0.5*l + math.pi*r + w)/perim) then
      pos = makeVector3f(0.5*l + r + r*(-1 + math.cos((perim*t - (0.5*l + 0.5*math.pi*r + w))/r)), r + w + r*(math.sin((perim*t - (0.5*l + 0.5*math.pi*r + w))/r)), 0.0)
   elseif(t < (1.5*l + math.pi*r + w)/perim) then
      pos = makeVector3f(0.5*l - (perim*t - (0.5*l + math.pi*r + w)), 2*r + w, 0.0)
   elseif(t < (1.5*l + 1.5*math.pi*r + w)/perim) then
      pos = makeVector3f(-0.5*l + r*(-math.sin((perim*t - (1.5*l + math.pi*r + w))/r)), 2*r + w + r*(-1 + math.cos((perim*t - (1.5*l + math.pi*r + w))/r)), 0.0)
   elseif(t < (1.5*l + 1.5*math.pi*r + 2*w)/perim) then
      pos = makeVector3f(-0.5*l -r, w + r - (perim*t - (1.5*l + 1.5*math.pi*r + w)), 0.0)
   elseif(t < (1.5*l + 2*math.pi*r + 2*w)/perim) then
      pos = makeVector3f(-0.5*l -r + r*(1 - math.cos((perim*t - (1.5*l + 1.5*math.pi*r + 2*w))/r)), r + r*(-math.sin((perim*t - (1.5*l + 1.5*math.pi*r + 2*w))/r)), 0.0)
   else
      pos = makeVector3f(-0.5*l + perim*t - (1.5*l + 2*math.pi*r + 2*w), 0.0, 0.0)
   end

   return pos, math.rad(bank_angle)
end 

function vertical_aerobatic_box(t, arg1, arg2, arg3, arg4)
   
   --gcs:send_text(0, string.format("t val: %f", t))
   local q = Quaternion()
   q:from_euler(-math.rad(90), 0, 0)
   local point, angle = horizontal_rectangle(t, arg1, arg2, arg3, arg4)
   q:earth_to_body(point)
   return point, angle
end 
---------------------------------------------------

function target_groundspeed()
   return ahrs:get_EAS2TAS()*TRIM_ARSPD_CM:get()*0.01
end

--Estimate the length of the path in metres
function path_length(path_f, arg1, arg2, arg3, arg4)
   local dt = 0.01
   local total = 0.0
   for i = 0, math.floor(1.0/dt) do
      local t = i*dt
      local t2 = t + dt
      local v1 = path_f(t, arg1, arg2, arg3, arg4)
      local v2 = path_f(t2, arg1, arg2, arg3, arg4)

      local dv = v2-v1
      total = total + dv:length()
   end
   return total
end


--args: 
--  path_f: path function returning position 
--  t: normalised [0, 1] time
--  arg1, arg2: arguments for path function
--  orientation: maneuver frame orientation
--returns: requested position in maneuver frame
function rotate_path(path_f, t, arg1, arg2, arg3, arg4, orientation, offset)
   point, angle = path_f(t, arg1, arg2, arg3, arg4)
   orientation:earth_to_body(point)
   --TODO: rotate angle?
   return point+offset, angle
end

--args:
--  dt: sample time
--  cutoff_freq: cutoff frequency for low pass filter, in Hz
--returns: alpha value required to implement LP filter
function calc_lowpass_alpha_dt(dt, cutoff_freq)
   if dt <= 0.0 or cutoff_freq <= 0.0 then
      return 1.0
   end
   
   local rc = 1.0/(2.0*3.14159265*cutoff_freq)
   local drc = dt/(dt+rc)
   if drc < 0.0 then
      return 0.0
   end
   if drc > 1.0 then
      return 1.0
   end
   return drc
end

--Wrapper to construct a Vector3f{x, y, z} from (x, y, z)
function makeVector3f(x, y, z)
   local vec = Vector3f()
   vec:x(x)
   vec:y(y)
   vec:z(z)
   return vec
end

--Given vec1, vec2, returns an (rotation axis, angle) tuple that rotates vec1 to be parallel to vec2
--If vec1 and vec2 are already parallel, returns a zero vector and zero angle
--Note that the rotation will not be unique.
function vectors_to_rotation(vector1, vector2)
   axis = vector1:cross(vector2)
   if axis:length() < 0.00001 then
      local vec = Vector3f()
      vec:x(1)
      return vec, 0
   end
   axis:normalize()
   angle = vector1:angle(vector2)
   return axis, angle
end

--returns Quaternion
function vectors_to_rotation_w_roll(vector1, vector2, roll)
   axis, angle = vectors_to_rotation(vector1, vector2)
   local vector_rotation = Quaternion()
   vector_rotation:from_axis_angle(axis, angle)

   local roll_rotation = Quaternion()
   roll_rotation:from_euler(roll, 0, 0)
   
   local total_rot = vector_rotation*roll_rotation
   return to_axis_and_angle(total_rot)
end

--Given vec1, vec2, returns an angular velocity  tuple that rotates vec1 to be parallel to vec2
--If vec1 and vec2 are already parallel, returns a zero vector and zero angle 
function vectors_to_angular_rate(vector1, vector2, time_constant)
   axis, angle = vectors_to_rotation(vector1, vector2)
   angular_velocity = angle/time_constant
   return axis:scale(angular_velocity)
end

function vectors_to_angular_rate_w_roll(vector1, vector2, time_constant, roll)
   axis, angle = vectors_to_rotation_w_roll(vector1, vector2, roll)
   angular_velocity = angle/time_constant
   return axis:scale(angular_velocity)
end

function to_axis_and_angle(quat)

   local axis_angle = Vector3f()
   quat:to_axis_angle(axis_angle)
   angle = axis_angle:length() 
   if(angle < 0.00001) then
      return makeVector3f(1.0, 0.0, 0.0), 0.0
   end
   return axis_angle:scale(1.0/angle), angle
end

function test_axis_and_angle()
   local quat = Quaternion()
   quat:q1(1.0)
   local axis, angle = to_axis_and_angle(quat)
   gcs:send_text(0, string.format("axis angle test: %f %f %f %f", axis:x(), axis:y(), axis:z(), angle))

   local quat2 = Quaternion()
   quat2:q1(math.cos(math.pi/4))
   quat2:q2(0)
   quat2:q3(0)
   quat2:q4(math.sin(math.pi/4))
   local axis2, angle2 = to_axis_and_angle(quat2)
   gcs:send_text(0, string.format("axis angle test2: %f %f %f %f", axis2:x(), axis2:y(), axis2:z(), angle2))

   local quat3 = Quaternion()
   quat3:q1(math.cos(math.pi/2))
   quat3:q2(0)
   quat3:q3(math.sin(math.pi/2))
   quat3:q4(0)
   local axis3, angle3 = to_axis_and_angle(quat3)
   gcs:send_text(0, string.format("axis angle test3: %f %f %f %f", axis3:x(), axis3:y(), axis3:z(), angle3))
end

--Just used this to test the above function, can probably delete now.
function test_angular_rate()
   local vector1 = makeVector3f(1.0, 0.0, 0.0)
   local vector2 = makeVector3f(1.0, 1.0, 0.0)
   local angular_rate = vectors_to_angular_rate(vector1, vector2, 1.0)
   gcs:send_text(0, string.format("angular rate: %.1f %.1f %.1f", math.deg(angular_rate:x()), math.deg(angular_rate:y()), math.deg(angular_rate:z())))
end

--projects x onto the othogonal subspace of span(unit_v)
function ortho_proj(x, unit_v)
   local temp_x = unit_v:cross(x)
   return unit_v:cross(temp_x)
end
--test_angular_rate()
--test_axis_and_angle()
-- function maneuver_to_body(vec)
--    path_var.initial_maneuver_to_earth:earth_to_body(vec)
--    vec = ahrs:earth_to_body(vec)
   
--    return vec
-- end

--returns body frame angular rate as Vec3f
-- function path_proportional_error_correction(current_pos_ef, target_pos_ef, forward_velocity, target_velocity_ef)
   
--    if forward_velocity <= MIN_SPEED then
--       return makeVector3f(0.0, 0.0, 0.0)
--    end
   
--    --time over which to correct position error
--    local time_const_pos_to_vel = POS_TC:get()
--    --time over which to achieve desired velocity
--    local time_const_vel_to_acc = VEL_TC:get()
--    local pos_err_ef = target_pos_ef - current_pos_ef

--    local correction_vel_ef = pos_err_ef:scale(1.0/time_const_pos_to_vel)
--    correction_vel_ef = correction_vel_ef:scale(forward_velocity)

--    local curr_vel_ef = ahrs:get_velocity_NED()

--    local vel_error_ef = correction_vel_ef - curr_vel_ef

--    local acc_err_bf = ahrs:earth_to_body(vel_error_ef):scale(1.0/time_const_vel_to_acc)

--    local ang_vel = makeVector3f(0, -acc_err_bf:z()/forward_velocity, acc_err_bf:y()/forward_velocity)

--    return ang_vel
-- end

-- log a pose from position and quaternion attitude
function log_pose(logname, pos, quat)
   logger.write(logname, 'px,py,pz,q1,q2,q3,q4,r,p,y','ffffffffff',
                pos:x(),
                pos:y(),
                pos:z(),
                quat:q1(),
                quat:q2(),
                quat:q3(),
                quat:q4(),
                math.deg(quat:get_euler_roll()),
                math.deg(quat:get_euler_pitch()),
                math.deg(quat:get_euler_yaw()))
end

local path_var = {}
path_var.count = 0
path_var.initial_ori = Quaternion()
path_var.initial_maneuver_to_earth = Quaternion()

function do_path(path, initial_yaw_deg, arg1, arg2, arg3, arg4)

   local now = millis():tofloat() * 0.001

   path_var.count = path_var.count + 1
   local target_dt = 1.0/LOOP_RATE

   if not running then
      running = true
      path_var.length = path_length(path, arg1, arg2, arg3, arg4)

      path_var.total_rate_rads_ef = makeVector3f(0.0, 0.0, 0.0)
      local speed = target_groundspeed()

      --assuming constant velocity
      path_var.total_time = path_var.length/speed
      path_var.last_pos, last_angle = path(0.0, arg1, arg2, arg3, arg4) --position at t0

      --deliberately only want yaw component, because the maneuver should be performed relative to the earth, not relative to the initial orientation
      path_var.initial_ori:from_euler(0, 0, math.rad(initial_yaw_deg))

      path_var.initial_maneuver_to_earth:from_euler(0, 0, -math.rad(initial_yaw_deg))
      
      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()


      local corrected_position_t0_ef, angle_t0 = rotate_path(path, LOOKAHEAD*target_dt/path_var.total_time,
                                                             arg1, arg2, arg3, arg4,
                                                             path_var.initial_ori, path_var.initial_ef_pos)
      local corrected_position_t1_ef, angle_t1 = rotate_path(path, 2*LOOKAHEAD*target_dt/path_var.total_time,
                                                             arg1, arg2, arg3, arg4,
                                                             path_var.initial_ori, path_var.initial_ef_pos)

      path_var.start_pos = ahrs:get_position()
      path_var.path_int = path_var.start_pos:copy()

      height_PI.reset()

      speed_PI.reset()


      path_var.accumulated_orientation_rel_ef = path_var.initial_ori

      path_var.time_correction = 0.0 

      path_var.filtered_angular_velocity = Vector3f()

      path_var.start_time = now + target_dt
      path_var.last_time = now
      path_var.average_dt = target_dt
      path_var.scaled_dt = target_dt

      path_var.path_t = 0
      path_var.target_speed = speed
      return true
   end
   

   --TODO: dt taken from actual loop rate or just desired loop rate?
   --local dt = now - path_var.last_time
   --local dt = target_dt
   local vel_length = ahrs:get_velocity_NED():length()

   local actual_dt = now - path_var.last_time

   --path_var.average_dt = 0.98*path_var.average_dt + 0.02*actual_dt

   --local scaled_dt = path_var.average_dt--*vel_length/path_var.target_speed
   --path_var.scaled_dt = scaled_dt

   local local_n_dt = actual_dt/path_var.total_time

   path_var.last_time = now
   --path_var.path_t = path_var.path_t + scaled_dt/path_var.total_time
   
   --TODO: Fix this exit condition
   --local t = path_var.path_t

   if path_var.path_t > 1.0 then --done
      return false
   end

   --[[
      calculate positions and angles at previous, current and next time steps
   --]]

   next_target_pos_ef = next_target_pos_ef
   local p0, r0 = rotate_path(path, path_var.path_t + 0*local_n_dt,
                              arg1, arg2, arg3, arg4,
                              path_var.initial_ori, path_var.initial_ef_pos)
   local p1, r1 = rotate_path(path, path_var.path_t + 1*local_n_dt,
                              arg1, arg2, arg3, arg4,
                              path_var.initial_ori, path_var.initial_ef_pos)
   local p2, r2 = rotate_path(path, path_var.path_t + 2*local_n_dt,
                              arg1, arg2, arg3, arg4,
                              path_var.initial_ori, path_var.initial_ef_pos)

   local current_measured_pos_ef = ahrs:get_relative_position_NED_origin()

   --[[
      get tangents to the path
   --]]
   local tangent1_ef = p1 - p0
   local tangent2_ef = p2 - p1
   local tv_unit = tangent2_ef:copy()
   tv_unit:normalize()

   --[[
      use actual vehicle velocity to calculate how far along the
      path we have progressed
   --]]
   local v = ahrs:get_velocity_NED()
   local path_dist = v:dot(tv_unit)*actual_dt
   if path_dist < 0 then
      gcs:send_text(0, string.format("aborting"))
      return false
   end
   local path_t_delta = constrain(path_dist/path_var.length, 0.2*local_n_dt, 4*local_n_dt)
   path_var.path_t = path_var.path_t + path_t_delta

   --[[
      recalculate the current path position and angle based on actual delta time
   --]]
   p2, r2 = rotate_path(path, path_var.path_t + path_t_delta,
      arg1, arg2, arg3, arg4,
      path_var.initial_ori, path_var.initial_ef_pos)

   -- tangents needs to be recalculated
   tangent1_ef = p1 - p0
   tangent2_ef = p2 - p1
   tv_unit = tangent2_ef:copy()
   tv_unit:normalize()

   -- error in position versus current point on the path
   local pos_error_ef = current_measured_pos_ef - p1

   --[[
      calculate a time correction. We first get the projection of
      the position error onto the track. This tells us how far we
      are ahead or behind on the track
   --]]
   local path_dist_err_m = tv_unit:dot(pos_error_ef)

   -- normalize against the total path length
   local path_err_t = path_dist_err_m / path_var.length

   -- don't allow the path to go backwards in time, or faster than twice the actual rate
   path_err_t = constrain(path_err_t, -0.9*path_t_delta, 2*path_t_delta)

   -- correct time to bring us back into sync
   path_var.path_t = path_var.path_t + TIME_CORR_P:get() * path_err_t

   
   --[[
      calculation of error correction, calculating acceleration
      needed to bring us back on the path, and body rates in pitch and
      yaw to achieve those accelerations
   --]]
   
   -- component of pos_err perpendicular to the current path tangent
   local B = ortho_proj(pos_error_ef, tv_unit)

   -- derivative of pos_err perpendicular to the current path tangent, assuming tangent is constant
   local B_dot = ortho_proj(v, tv_unit)

   -- gains for error correction.
   local acc_err_ef = B:scale(ERR_CORR_P:get()) + B_dot:scale(ERR_CORR_D:get())

   local acc_err_bf = ahrs:earth_to_body(acc_err_ef)

   local TAS = constrain(ahrs:get_EAS2TAS()*ahrs:airspeed_estimate(), 3, 100)
   local corr_rate_bf_y_rads = -acc_err_bf:z()/TAS
   local corr_rate_bf_z_rads = acc_err_bf:y()/TAS

   local cor_ang_vel_bf_rads = makeVector3f(0.0, corr_rate_bf_y_rads, corr_rate_bf_z_rads)
   local cor_ang_vel_bf_dps = cor_ang_vel_bf_rads:scale(math.deg(1))


   --[[
      work out body frame path rate, this is based on two adjacent tangents on the path
   --]]
   local path_rate_ef_rads = vectors_to_angular_rate(tangent1_ef, tangent2_ef, actual_dt)
   local path_rate_ef_dps = path_rate_ef_rads:scale(math.deg(1))
   local path_rate_bf_dps = ahrs:earth_to_body(path_rate_ef_dps)

   -- set the path roll rate
   path_rate_bf_dps:x(math.deg(wrap_pi(r1 - r0)/actual_dt))


   --[[
      calculate body frame roll rate to achieved the desired roll
      angle relative to the maneuver path
   --]]
   local zero_roll_angle_delta = Quaternion()
   zero_roll_angle_delta:from_angular_velocity(path_rate_ef_rads, actual_dt)

   path_var.accumulated_orientation_rel_ef = zero_roll_angle_delta*path_var.accumulated_orientation_rel_ef
   path_var.accumulated_orientation_rel_ef:normalize()
   
   local mf_axis = makeVector3f(1, 0, 0)
   path_var.accumulated_orientation_rel_ef:earth_to_body(mf_axis)

   local orientation_rel_mf_with_roll_angle = Quaternion()
   orientation_rel_mf_with_roll_angle:from_axis_angle(mf_axis, r1)
   orientation_rel_ef_with_roll_angle = orientation_rel_mf_with_roll_angle*path_var.accumulated_orientation_rel_ef

   --[[
      calculate the error correction for the roll versus the desired roll
   --]]
   local roll_error = orientation_rel_ef_with_roll_angle*ahrs:get_quaternion():inverse()
   roll_error:normalize()
   local err_axis_ef, err_angle_rad = to_axis_and_angle(roll_error)
   local time_const_roll = ROLL_CORR_TC:get()
   local err_angle_rate_ef_rads = err_axis_ef:scale(err_angle_rad/time_const_roll)
   local err_angle_rate_bf_dps = ahrs:earth_to_body(err_angle_rate_ef_rads):scale(math.deg(1))
   -- zero any non-roll components
   err_angle_rate_bf_dps:y(0)
   err_angle_rate_bf_dps:z(0)

   --[[
      total angular rate is sum of path rate, correction rate and roll correction rate
   --]]
   local tot_ang_vel_bf_dps = path_rate_bf_dps + cor_ang_vel_bf_dps + err_angle_rate_bf_dps


   --[[
      log POSM is pose-measured, POST is pose-track, POSB is pose-track without the roll
   --]]
   log_pose('POSM', current_measured_pos_ef, ahrs:get_quaternion())
   log_pose('POST', p1, orientation_rel_ef_with_roll_angle)

   logger.write('AETM', 'T,Terr','ff',
                path_var.path_t,
                path_err_t)

   logger.write('AERT','Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz', 'ffffffffff',
                cor_ang_vel_bf_dps:x(), cor_ang_vel_bf_dps:y(), cor_ang_vel_bf_dps:z(),
                path_rate_bf_dps:x(), path_rate_bf_dps:y(), path_rate_bf_dps:z(),
                err_angle_rate_bf_dps:x(),
                tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z())

   --log_pose('POSB', p1, path_var.accumulated_orientation_rel_ef)

   --[[
      run the throttle based speed controller
   --]]
   local target_speed = target_groundspeed()--TRIM_ARSPD_CM:get()*0.01
   throttle = speed_PI.update(target_speed)
   throttle = constrain(throttle, 0, 100.0)


   vehicle:set_target_throttle_rate_rpy(throttle, tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z())
   
   return true
end

command_table = {}
command_table[1]={figure_eight, "Figure Eight"}
command_table[2]={loop, "Loop"}
command_table[3]={horizontal_rectangle, "Horizontal Rectangle"}
command_table[4]={climbing_circle, "Climbing Circle"}
command_table[5]={vertical_aerobatic_box, "Vertical Box"}
command_table[6]={banked_circle, "Banked Circle"}
command_table[7]={straight_roll, "Axial Roll"}
command_table[8]={rolling_circle, "Rolling Circle"}
command_table[9]={half_cuban_eight, "Half Cuban Eight"}
command_table[10]={half_reverse_cuban_eight, "Half Reverse Cuban Eight"}
command_table[11]={cuban_eight, "Cuban Eight"}
command_table[12]={humpty_bump, "Humpty Bump"}
command_table[13]={straight_flight, "Straight Flight"}
command_table[14]={scale_figure_eight, "Scale Figure Eight"}

-- get a location structure from a waypoint number
function get_location(i)
   local m = mission:get_item(i)
   local loc = Location()
   loc:lat(m:x())
   loc:lng(m:y())
   loc:relative_alt(true)
   loc:terrain_alt(false)
   loc:origin_alt(false)
   loc:alt(math.floor(m:z()*100))
   return loc
end

-- set wp location
function wp_setloc(wp, loc)
   wp:x(loc:lat())
   wp:y(loc:lng())
   wp:z(loc:alt()*0.01)
end

-- add a waypoint to the end of the mission
function wp_add(loc,ctype,param1,param2)
   local wp = mavlink_mission_item_int_t()
   wp_setloc(wp,loc)
   wp:command(ctype)
   local seq = mission:num_commands()
   wp:seq(seq)
   wp:param1(param1)
   wp:param2(param2)
   wp:frame(3) -- global position, relative alt
   mission:set_item(seq, wp)
end

-- add a NAV_SCRIPT_TIME waypoint to the end of the mission
function wp_add_nav_script(cmdid,arg1,arg2,arg3,arg4)
   local wp = mavlink_mission_item_int_t()
   wp:command(NAV_SCRIPT_TIME)
   local seq = mission:num_commands()
   wp:seq(seq)
   wp:param1(cmdid)
   wp:param2(0) -- timeout
   wp:param3(arg1)
   wp:param4(arg2)
   wp:x(arg3)
   wp:y(arg4)
   mission:set_item(seq, wp)
end

--[[
   create auto mission 1
--]]
function create_auto_mission1()
   local N = mission:num_commands()
   if N ~= 4 then
      gcs:send_text(0,string.format("Auto mission needs takeoff and 2 WPs (got %u)", N))
      return
   end
   local takeoff_m = mission:get_item(1)
   if takeoff_m:command() ~= NAV_TAKEOFF then
      gcs:send_text(0,string.format("First WP needs to be takeoff"))
      return
   end
   local wp1 = get_location(2)
   local wp2 = get_location(3)

   local wp_dist = wp1:get_distance(wp2)
   local wp_bearing = math.deg(wp1:get_bearing(wp2))
   local radius = AUTO_RAD:get()

   gcs:send_text(0, string.format("WP Distance %.0fm bearing %.1fdeg", wp_dist, wp_bearing))

   -- find mid-point, 25% and 75% points
   local wp_mid = wp1:copy()
   wp_mid:offset_bearing(wp_bearing, wp_dist*0.5)

   local wp_25pct = wp1:copy()
   wp_25pct:offset_bearing(wp_bearing, wp_dist*0.25)

   local wp_75pct = wp1:copy()
   wp_75pct:offset_bearing(wp_bearing, wp_dist*0.75)
   
   gcs:send_text(0,"Adding half cuban eight")
   wp_add_nav_script(9, radius, 0, 0, 0)

   gcs:send_text(0,"Adding loop")
   wp_add(wp_mid, NAV_WAYPOINT, 0, 1)
   wp_add_nav_script(2, radius, 0, 0, 0)

   gcs:send_text(0,"Adding half reverse cuban eight")
   wp_add(wp1, NAV_WAYPOINT, 0, 0)
   wp_add_nav_script(10, radius, 0, 0, 0)

   gcs:send_text(0,"Adding axial roll")
   wp_add(wp_25pct, NAV_WAYPOINT, 0, 1)
   wp_add_nav_script(7, wp_dist*0.5, 1, 0, 0)
   
   gcs:send_text(0,"Adding humpty bump")
   wp_add(wp2, NAV_WAYPOINT, 0, 1)
   wp_add_nav_script(12, radius*0.25, 0.5*radius, 0, 0)

   gcs:send_text(0,"Adding cuban eight")
   wp_add(wp_mid, NAV_WAYPOINT, 0, 0)
   wp_add_nav_script(11, radius, 0, 0, 0)

   wp_add(wp1, NAV_WAYPOINT, 0, 0)

end

function create_auto_mission()
   if AUTO_MIS:get() == 1 then
      create_auto_mission1()
   else
      gcs:send_text(0, string.format("Unknown auto mission", AUTO_MIS:get()))
   end
end

function update()

   -- check if we should create a mission
   if AUTO_MIS:get() > 0 then
      create_auto_mission()
      AUTO_MIS:set_and_save(0)
   end

   id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
   if id then
      if id ~= last_id then
         -- we've started a new command
         running = false
         last_id = id
         repeat_count = 0
         initial_yaw_deg = math.deg(ahrs:get_yaw())
         gcs:send_text(0, string.format("Starting %s!", command_table[cmd][2] ))

         initial_height = ahrs:get_position():alt()*0.01
         -- work out yaw between previous WP and next WP
         local cnum = mission:get_current_nav_index()
         -- find previous nav waypoint
         local loc_prev = get_wp_location(cnum-1)
         local loc_next = get_wp_location(cnum+1)
         local i= cnum-1
         while get_wp_location(i):lat() == 0 and get_wp_location(i):lng() == 0 do
             i = i-1
             loc_prev = get_wp_location(i)   
         end
         -- find next nav waypoint
         i = cnum+1
         while get_wp_location(i):lat() == 0 and get_wp_location(i):lng() == 0 do
             i = i+1
             loc_next = get_wp_location(resolve_jump(i))
         end
         wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
         if math.abs(wrap_180(initial_yaw_deg - wp_yaw_deg)) > 90 then
            gcs:send_text(0, string.format("Doing turnaround!"))
            wp_yaw_deg = wrap_180(wp_yaw_deg + 180)
         end
         initial_yaw_deg = wp_yaw_deg
      end
      local done = not do_path(command_table[cmd][1], initial_yaw_deg, arg1, arg2, arg3, arg4)
      if done then
         vehicle:nav_script_time_done(last_id)
         gcs:send_text(0, string.format("Finishing %s!", command_table[cmd][2] ))
         running = false
      end
   else
      running = false
   end
   return update, 1000.0/LOOP_RATE
end

return update()
