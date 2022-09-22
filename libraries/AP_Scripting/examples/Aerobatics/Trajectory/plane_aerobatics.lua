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

-- this control script uses AERO_TRICK_ID to report the selected trick number from the scripting_rc_selection rc channel
assert(param:add_param(PARAM_TABLE_KEY, 1,  'HGT_P', 1), 'could not add param4') -- height P gain
assert(param:add_param(PARAM_TABLE_KEY, 2,  'HGT_I', 2), 'could not add param5') -- height I gain
assert(param:add_param(PARAM_TABLE_KEY, 3,  'HGT_KE_ADD', 20), 'could not add param6') --height knife-edge addition for pitch
assert(param:add_param(PARAM_TABLE_KEY, 4,  'THR_PIT_FF', 80), 'could not add param67') --throttle FF from pitch
assert(param:add_param(PARAM_TABLE_KEY, 5,  'SPD_P', 5), 'could not add param8') -- speed P gain
assert(param:add_param(PARAM_TABLE_KEY, 6,  'SPD_I', 25), 'could not add param9')  -- speed I gain

function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

local ERR_CORR_TC = bind_add_param('ERR_COR_TC', 7, 3)
local ROLL_CORR_TC = bind_add_param('ROL_COR_TC', 8, 1)

--local VEL_TC = bind_add_param('VEL_TC', 8, 3)




local LOOP_RATE = 20
DO_JUMP = 177
k_throttle = 70

local HGT_P = bind_param("AEROM_HGT_P") -- height P gain
local HGT_I = bind_param("AEROM_HGT_I") -- height I gain
local HGT_KE_BIAS = bind_param("AEROM_HGT_KE_ADD") -- height knifeedge addition for pitch
local THR_PIT_FF = bind_param("AEROM_THR_PIT_FF") -- throttle FF from pitch
local SPD_P = bind_param("AEROM_SPD_P") -- speed P gain
local SPD_I = bind_param("AEROM_SPD_I") -- speed I gain
local TRIM_THROTTLE = bind_param("TRIM_THROTTLE")
local TRIM_ARSPD_CM = bind_param("TRIM_ARSPD_CM")
local RLL2SRV_TCONST = bind_param("RLL2SRV_TCONST")
local PITCH_TCONST = bind_param("PTCH2SRV_TCONST")

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

local arg3 = 0
local arg4 = 0

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
   loc:relative_alt(false)
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
function path_circle(t, radius, unused)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 1.0-math.cos(t), 0)
   return vec:scale(radius), 0.0
end

function knife_edge_circle(t, radius, unused)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 1.0-math.cos(t), 0)
   return vec:scale(radius), math.pi/2
end

function path_climbing_circle(t, radius, height)
   local angle = t*math.pi*2
   local vec = makeVector3f(radius*math.sin(angle), radius*(1.0-math.cos(angle)), -math.sin(0.5*angle)*height)
   return vec, 0.0
end

--TODO: fix this to have initial tangent 0
function path_figure_eight(t, radius)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), math.sin(t)*math.cos(t), 0)
   return vec:scale(radius), 0.0
end

function path_vertical_circle(t, radius, unused)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 0.0, -1.0 + math.cos(t))
   return vec:scale(radius), 0.0
end

function path_straight_roll(t, length, num_rolls)

   local vec = makeVector3f(t*length, 0.0, 0.0)
   return vec, t*num_rolls*2*math.pi
end

function path_rolling_circle(t, radius, num_rolls)
   --t = t*math.pi*2
   local vec = Vector3f()
   if radius < 0.0 then
      vec = makeVector3f(math.sin(2*math.pi*t), -1.0+math.cos(2*math.pi*t), 0)
   else
      vec = makeVector3f(math.sin(2*math.pi*t), 1.0-math.cos(2*math.pi*t), 0)
   end
   return vec:scale(math.abs(radius)), t*num_rolls*2*math.pi
end

function path_banked_circle(t, radius, bank_angle)
   --t = t*math.pi*2
   local vec = Vector3f()
   if radius < 0.0 then
      vec = makeVector3f(math.sin(2*math.pi*t), -1.0+math.cos(2*math.pi*t), 0)
   else
      vec = makeVector3f(math.sin(2*math.pi*t), 1.0-math.cos(2*math.pi*t), 0)
   end
   return vec:scale(math.abs(radius)), math.deg(bank_angle)
end

function test_height_control(t, length, unused)
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

function test_lane_change(t, length, unused)
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

function path_straight_roll(t, length, num_rolls)

   local vec = makeVector3f(t*length, 0.0, 0.0)
   return vec, t*num_rolls*2*math.pi
end


--todo: change y coordinate to z for vertical box
--function aerobatic_box(t, l, w, r):
function horizontal_aerobatic_box(t, arg1, arg2)
   
   local r = math.abs(arg3)
   if(arg3 <= 0.0) then
      gcs:send_text(0, string.format("Invalid radius value of : %f, using default.", arg3))
      r = math.min(arg1, arg2)/3.0
   end
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

   return pos, 0.0
end 

function vertical_aerobatic_box(t, arg1, arg2)
   
   --gcs:send_text(0, string.format("t val: %f", t))
   local q = Quaternion()
   q:from_euler(-math.rad(90), 0, 0)
   local point, angle = horizontal_aerobatic_box(t, arg1, arg2)
   q:earth_to_body(point)
   return point, angle
end 
---------------------------------------------------

function target_groundspeed()
   return ahrs:get_EAS2TAS()*TRIM_ARSPD_CM:get()*0.01
end

--Estimate the length of the path in metres
function path_length(path_f, arg1, arg2)
   local dt = 0.01
   local total = 0.0
   for i = 0, math.floor(1.0/dt) do
      local t = i*dt
      local t2 = t + dt
      local v1 = path_f(t, arg1, arg2)
      local v2 = path_f(t2, arg1, arg2)

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
function rotate_path(path_f, t, arg1, arg2, orientation, offset)
   point, angle = path_f(t, arg1, arg2)
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

local path_var = {}
path_var.count = 0
path_var.positions_ef = {}
path_var.roll_angles_bf = {}
path_var.initial_ori = Quaternion()
path_var.initial_maneuver_to_earth = Quaternion()

function do_path(path, initial_yaw_deg, arg1, arg2)

   local now = millis():tofloat() * 0.001

   path_var.count = path_var.count + 1
   local target_dt = 1.0/LOOP_RATE

   if not running then
      running = true
      path_var.length = path_length(path, arg1, arg2)

      path_var.total_rate_rads_ef = makeVector3f(0.0, 0.0, 0.0)
      local speed = target_groundspeed()

      --assuming constant velocity
      path_var.total_time = path_var.length/speed
      path_var.last_pos, last_angle = path(0.0, arg1, arg2) --position at t0

      --deliberately only want yaw component, because the maneuver should be performed relative to the earth, not relative to the initial orientation
      path_var.initial_ori:from_euler(0, 0, math.rad(initial_yaw_deg))

      path_var.initial_maneuver_to_earth:from_euler(0, 0, -math.rad(initial_yaw_deg))
      
      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()


      local corrected_position_t0_ef, angle_t0 = rotate_path(path, LOOKAHEAD*target_dt/path_var.total_time, arg1, arg2, path_var.initial_ori, path_var.initial_ef_pos)
      local corrected_position_t1_ef, angle_t1 = rotate_path(path, 2*LOOKAHEAD*target_dt/path_var.total_time, arg1, arg2, path_var.initial_ori, path_var.initial_ef_pos)

      path_var.start_pos = ahrs:get_position()
      path_var.path_int = path_var.start_pos:copy()

      height_PI.reset()

      speed_PI.reset()

      --path_var.positions[-1] is not used in initial runthrough
      path_var.positions_ef[0] = corrected_position_t0_ef
      path_var.positions_ef[1] = corrected_position_t1_ef

      path_var.roll_angles_bf[0] = angle_t0
      path_var.roll_angles_bf[1] = angle_t1

      path_var.accumulated_orientation_rel_ef = path_var.initial_ori

      path_var.time_correction = 0.0 

      path_var.filtered_angular_velocity = Vector3f()

      path_var.start_time = now + target_dt
      path_var.last_time = now
      path_var.average_dt = target_dt
      path_var.scaled_dt = target_dt

      path_var.path_t = 0
      path_var.target_speed = speed

   end
   

   --TODO: dt taken from actual loop rate or just desired loop rate?
   --local dt = now - path_var.last_time
   --local dt = target_dt
   local vel_length = ahrs:get_velocity_NED():length()

   local actual_dt = now - path_var.last_time

   path_var.average_dt = 0.98*path_var.average_dt + 0.02*actual_dt

   local scaled_dt = path_var.average_dt--*vel_length/path_var.target_speed
   path_var.scaled_dt = scaled_dt

   path_var.last_time = now
   path_var.path_t = path_var.path_t + scaled_dt/path_var.total_time
   
   --TODO: Fix this exit condition
   local t = path_var.path_t

   if t > 1.0 then --done
      return false
   end

   --where we aim to be on the path at this timestamp
   --rotate from maneuver frame to 'local' EF
   local next_target_pos_ef, next_target_angle = rotate_path(path, path_var.path_t + LOOKAHEAD*path_var.average_dt/path_var.total_time, arg1, arg2, path_var.initial_ori, path_var.initial_ef_pos)

   next_target_pos_ef = next_target_pos_ef
   logger.write("TML", 't', 'f', path_var.path_t + LOOKAHEAD*path_var.average_dt/path_var.total_time)

   path_var.positions_ef[-1] = path_var.positions_ef[0]:copy()
   path_var.positions_ef[0] = path_var.positions_ef[1]:copy()
   path_var.positions_ef[1] = next_target_pos_ef:copy()

   --roll angle relative to maneuver position without rolling
   path_var.roll_angles_bf[-1] = path_var.roll_angles_bf[0]
   path_var.roll_angles_bf[0] = path_var.roll_angles_bf[1]
   path_var.roll_angles_bf[1] = next_target_angle

   local current_measured_pos_ef = ahrs:get_relative_position_NED_origin()

   -- local path_error = {}
   -- path_error[-1] = (current_measured_pos - path_var.positions[-1]):length()
   -- path_error[0] = (current_measured_pos - path_var.positions[0]):length() 
   -- path_error[1] = (current_measured_pos - path_var.positions[1]):length()

   -----------------------------------------------------------------------------------------------------------------------------
   --TODO: Get the "time correction" logic working
   -- local smallest_error_index = -1
   -- for i = 0,1,1
   -- do
   --    if(path_error[i] < path_error[smallest_error_index]) then
   --       smallest_error_index = i
   --    end
   -- end

   -- if(smallest_error_index == 1) then
   --   path_var.positions[-1] = path_var.positions[0]
   --   path_var.positions[0] = path_var.positions[1]
   --   path_var.positions[1] = rotate_path(path, t + 2*dt, arg1, arg2, path_var.initial_ori)
   -- end

   -- if(smallest_error_index == -1) then
   --   path_var.positions[1] = path_var.positions[0] 
   --   path_var.positions[0] = path_var.positions[-1]
   --   path_var.positions[-1] = rotate_path(path, t - 2*dt, arg1, arg2, path_var.initial_ori)
   -- end

   -- path_var.time_correction = path_var.time_correction + smallest_error_index*target_dt
   ------------------------------------------------------------------------------------------------------------------------------

   local position_error_ef = path_var.positions_ef[0]- current_measured_pos_ef
   local path_loc = path_var.start_pos:copy()
   path_loc:offset(path_var.positions_ef[0]:x() - path_var.initial_ef_pos:x(), path_var.positions_ef[0]:y() - path_var.initial_ef_pos:y())
   path_loc:alt(path_loc:alt() - math.floor(path_var.positions_ef[0]:z()*100))

   --logger.write("POSM",'x,y,z','fff',current_measured_pos_ef:x(),current_measured_pos_ef:y(),current_measured_pos_ef:z())
   --logger.write("POSE",'x,y,z','fff',path_var.positions_ef[0]:x(),path_var.positions_ef[0]:y(),path_var.positions_ef[0]:z())
   logger.write("PERR",'x,y,z,tc,Lat,Lng,Alt','ffffLLf',position_error_ef:x(),position_error_ef:y(),position_error_ef:z(), path_var.time_correction, path_loc:lat(), path_loc:lng(), path_loc:alt()*0.01)
   --velocity required to travel along trajectory
   local trajectory_velocity_ef = (path_var.positions_ef[1] - path_var.positions_ef[-1]):scale(0.5/path_var.scaled_dt) --derivative from -dt to dt for more accuracy
   local tangent1_ef = (path_var.positions_ef[0] - path_var.positions_ef[-1])
   local tangent2_ef = (path_var.positions_ef[1] - path_var.positions_ef[0])
   local path_rate_rads_ef = vectors_to_angular_rate(tangent1_ef, tangent2_ef, path_var.scaled_dt)

   local zero_roll_angle_delta = Quaternion()
   zero_roll_angle_delta:from_angular_velocity(path_rate_rads_ef, path_var.scaled_dt)

   path_var.accumulated_orientation_rel_ef = zero_roll_angle_delta*path_var.accumulated_orientation_rel_ef
   path_var.accumulated_orientation_rel_ef:normalize()

   --velocity to correct error
   local err_corr_tc = ERR_CORR_TC:get() --tested with 3.0 seconds
   local err_velocity_ef = (path_var.positions_ef[0] - current_measured_pos_ef):scale(1.0/err_corr_tc)

   local total_velocity_ef = trajectory_velocity_ef + err_velocity_ef
   local curr_vel_ef = ahrs:get_velocity_NED()

   local total_ang_vel_ef = vectors_to_angular_rate(curr_vel_ef, total_velocity_ef, 1.0)
   local total_ang_vel_bf = ahrs:earth_to_body(total_ang_vel_ef)

   local mf_axis = makeVector3f(1, 0, 0)
   path_var.accumulated_orientation_rel_ef:earth_to_body(mf_axis)

   local orientation_rel_mf_with_roll_angle = Quaternion()
   orientation_rel_mf_with_roll_angle:from_axis_angle(mf_axis, path_var.roll_angles_bf[0])
   orientation_rel_ef_with_roll_angle = orientation_rel_mf_with_roll_angle*path_var.accumulated_orientation_rel_ef

   --logger.write("ACCO",'r,p,y', 'fff', orientation_rel_ef_with_roll_angle:get_euler_roll(), orientation_rel_ef_with_roll_angle:get_euler_pitch(), orientation_rel_ef_with_roll_angle:get_euler_yaw())
   --logger.write("ACCQ",'q1,q2,q3,q4', 'ffff', orientation_rel_ef_with_roll_angle:q1(),orientation_rel_ef_with_roll_angle:q2(),orientation_rel_ef_with_roll_angle:q3(),orientation_rel_ef_with_roll_angle:q4() )
   logger.write("IORI",'r,p,y','fff',ahrs:get_roll(), ahrs:get_pitch(), ahrs:get_yaw())
   local roll_error = orientation_rel_ef_with_roll_angle*ahrs:get_quaternion():inverse()
   roll_error:normalize()
   local err_axis_ef, err_angle = to_axis_and_angle(roll_error)
   local time_const_roll = ROLL_CORR_TC:get()
   local err_angle_rate_ef = err_axis_ef:scale(err_angle/time_const_roll)

   local err_angle_rate_bf = ahrs:earth_to_body(err_angle_rate_ef)

   local angular_velocity_bf = total_ang_vel_bf
   angular_velocity_bf:x(err_angle_rate_bf:x())
   angular_velocity_bf = angular_velocity_bf:scale(math.deg(1))

   --logger.write("CAV",'x,y,z','fff',angular_velocity_bf:x(),angular_velocity_bf:y(),angular_velocity_bf:z())

   local target_speed = target_groundspeed()--TRIM_ARSPD_CM:get()*0.01
   throttle = speed_PI.update(target_speed)
   throttle = constrain(throttle, 0, 100.0)
   vehicle:set_target_throttle_rate_rpy(throttle, angular_velocity_bf:x(), angular_velocity_bf:y(), angular_velocity_bf:z())
   return true
end

command_table = {}
command_table[1]={path_figure_eight, "Figure Eight"}
command_table[2]={path_vertical_circle, "Loop"}
command_table[3]={horizontal_aerobatic_box, "Horizontal Rectangle"}
command_table[4]={path_climbing_circle, "Climbing Circle"}
command_table[5]={vertical_aerobatic_box, "Vertical Box"}
command_table[6]={path_banked_circle, "Banked Circle"}
command_table[7]={path_straight_roll, "Axial Roll"}
command_table[8]={path_rolling_circle, "Rolling Circle"}
command_table[100]={save_parameters,"Reading parameters"}

function save_parameters(arg1, arg2)
   arg3 = arg1
   arg4 = arg2
end

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
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
         initial_yaw_deg = wp_yaw_deg
      end
      local done = false
      if(cmd == 100) then
         --parameter node
         save_parameters(arg1, arg2)
         done = true
      else 
         done = not do_path(command_table[cmd][1], initial_yaw_deg, arg1, arg2)
      end
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
