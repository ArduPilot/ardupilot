--[[
   trajectory tracking aerobatic control
   See README.md for usage
   Written by Matthew Hampsey, Andy Palmer and Andrew Tridgell, with controller
   assistance from Paul Riseborough, testing by Henry Wurzburg
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

AEROM_ANG_ACCEL = bind_add_param('ANG_ACCEL', 1, 6000)
AEROM_ANG_TC = bind_add_param('ANG_TC', 2, 0.1)
AEROM_KE_ANG = bind_add_param('KE_ANG', 3, 0)
THR_PIT_FF = bind_add_param('THR_PIT_FF', 4, 80)
SPD_P = bind_add_param('SPD_P', 5, 5)
SPD_I = bind_add_param('SPD_I', 6, 25)
ERR_CORR_TC = bind_add_param('ERR_COR_TC', 7, 3)
ROLL_CORR_TC = bind_add_param('ROL_COR_TC', 8, 0.25)
-- removed 9 and 10
TIME_CORR_P = bind_add_param('TIME_COR_P', 11, 1.0)
ERR_CORR_P = bind_add_param('ERR_COR_P', 12, 2.0)
ERR_CORR_D = bind_add_param('ERR_COR_D', 13, 2.8)
AEROM_ENTRY_RATE = bind_add_param('ENTRY_RATE', 14, 60)
AEROM_THR_LKAHD = bind_add_param('THR_LKAHD', 15, 1)
AEROM_DEBUG = bind_add_param('DEBUG', 16, 0)
AEROM_THR_MIN = bind_add_param('THR_MIN', 17, 0)
AEROM_THR_BOOST = bind_add_param('THR_BOOST', 18, 50)
AEROM_YAW_ACCEL = bind_add_param('YAW_ACCEL', 19, 1500)
AEROM_LKAHD = bind_add_param('LKAHD', 20, 0.5)
AEROM_PATH_SCALE = bind_add_param('PATH_SCALE', 21, 1.0)

-- cope with old param values
if AEROM_ANG_ACCEL:get() < 100 and AEROM_ANG_ACCEL:get() > 0 then
   AEROM_ANG_ACCEL:set_and_save(3000)
end
if AEROM_ANG_TC:get() > 1.0 then
   AEROM_ANG_TC:set_and_save(0.2)
end

ACRO_ROLL_RATE = Parameter("ACRO_ROLL_RATE")
ARSPD_FBW_MIN = Parameter("ARSPD_FBW_MIN")
SCALING_SPEED = Parameter("SCALING_SPEED")

local GRAVITY_MSS = 9.80665

--[[
   Aerobatic tricks on a switch support - allows for tricks to be initiated outside AUTO mode
--]]
-- 2nd param table for tricks on a switch
local PARAM_TABLE_KEY2 = 71
local PARAM_TABLE_PREFIX2 = "TRIK"
assert(param:add_table(PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2, 63), 'could not add param table2')

-- add a parameter and bind it to a variable in table2
function bind_add_param2(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY2, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX2 .. name)
end

local TRIK_ENABLE = bind_add_param2("_ENABLE", 1, 0)
local TRICKS = nil
local TRIK_SEL_FN = nil
local TRIK_ACT_FN = nil
local TRIK_COUNT  = nil

local function TrickDef(id, arg1, arg2, arg3, arg4)
   local self = {}
   self.id = id
   self.args = {arg1, arg2, arg3, arg4}
   return self
end

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

function sq(x)
   return x*x
end

if TRIK_ENABLE:get() > 0 then
   TRIK_SEL_FN = bind_add_param2("_SEL_FN", 2, 301)
   TRIK_ACT_FN = bind_add_param2("_ACT_FN", 3, 300)
   TRIK_COUNT  = bind_add_param2("_COUNT",  4, 3)
   TRICKS = {}

   -- setup parameters for tricks
   local count = math.floor(constrain(TRIK_COUNT:get(),1,11))
   for i = 1, count do
      local k = 5*i
      local prefix = string.format("%u", i)
      TRICKS[i] = TrickDef(bind_add_param2(prefix .. "_ID",   k+0, i),
                           bind_add_param2(prefix .. "_ARG1", k+1, 30),
                           bind_add_param2(prefix .. "_ARG2", k+2, 0),
                           bind_add_param2(prefix .. "_ARG3", k+3, 0),
                           bind_add_param2(prefix .. "_ARG4", k+4, 0))
   end
   gcs:send_text(0, string.format("Enabled %u aerobatic tricks", TRIK_COUNT:get()))
end

local NAV_TAKEOFF = 22
local NAV_WAYPOINT = 16
local NAV_SCRIPT_TIME = 42702

local MODE_AUTO = 10

local LOOP_RATE = 20
local DO_JUMP = 177
local k_throttle = 70
local NAME_FLOAT_RATE = 2

local TRIM_ARSPD_CM = Parameter("TRIM_ARSPD_CM")

local last_id = 0
local current_task = nil
local last_named_float_t = 0

local path_var = {}

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


--[[
   calculate an alpha for a first order low pass filter
--]]
function calc_lowpass_alpha(dt, time_constant)
   local rc = time_constant/(math.pi*2)
   return dt/(dt+rc)
end


-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax,min,max)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
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

      local P = _kP * err
      if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
         _I = _I + _kI * err * dt
      end
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

local function speed_controller(kP_param,kI_param, kFF_pitch_param, Imax, min, max)
   local self = {}
   local kFF_pitch = kFF_pitch_param
   local PI = PI_controller(kP_param:get(), kI_param:get(), Imax, min, max)

   function self.update(target, anticipated_pitch_rad)
      local current_speed = ahrs:get_velocity_NED():length()
      local throttle = PI.update(target, current_speed)
      local FF = math.sin(anticipated_pitch_rad)*kFF_pitch:get()
      PI.log("AESP", FF)
      return throttle + FF
   end

   function self.reset()
      PI.reset(0)
      local temp_throttle = self.update(ahrs:get_velocity_NED():length(), 0)
      local current_throttle = SRV_Channels:get_output_scaled(k_throttle)
      PI.reset(current_throttle-temp_throttle)
   end

   return self
end

local speed_PI = speed_controller(SPD_P, SPD_I, THR_PIT_FF, 100.0, 0.0, 100.0)

function sgn(x)
   local eps = 0.000001
   if (x > eps) then
      return 1.0
   elseif x < eps then
      return -1.0
   else
      return 0.0
   end
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

--[[
   Wrapper to construct a Vector3f{x, y, z} from (x, y, z)
--]]
function makeVector3f(x, y, z)
   local vec = Vector3f()
   vec:x(x)
   vec:y(y)
   vec:z(z)
   return vec
end

--[[
   get quaternion rotation between vector1 and vector2
   with thanks to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
--]]
function vectors_to_quat_rotation(vector1, vector2)
   local v1 = vector1:copy()
   local v2 = vector2:copy()
   v1:normalize()
   v2:normalize()
   local dot = v1:dot(v2)
   local a = v1:cross(v2)
   local w = 1.0 + dot
   local q = Quaternion()
   q:q1(w)
   q:q2(a:x())
   q:q3(a:y())
   q:q4(a:z())
   q:normalize()
   return q
end

--[[
   get path rate from two tangents and delta time
--]]
function tangents_to_rate(t1, t2, dt)
   local q_delta = vectors_to_quat_rotation(t1, t2)
   local rate_rads = Vector3f()
   q_delta:to_axis_angle(rate_rads)
   rate_rads = rate_rads:scale(1.0/dt)
   return rate_rads
end

--[[
   trajectory building blocks. We have two types of building blocks,
   roll blocks and path blocks. These are combined to give composite paths
--]]

--[[
  all roll components inherit from RollComponent
--]]
function RollComponent()
   local self = {}
   self.name = nil
   function self.get_roll(t)
      return 0
   end
   return self
end

--[[
  roll component that goes through a fixed total angle at a fixed roll rate
--]]
function roll_angle(_angle)
   local self = RollComponent()
   self.name = "roll_angle"
   local angle = _angle
   function self.get_roll(t)
      return angle * t
   end
   return self
end

--[[
   roll component that banks to _angle over AEROM_ENTRY_RATE
   degrees/s, then holds that angle, then banks back to zero at
   AEROM_ENTRY_RATE degrees/s
--]]
function roll_angle_entry_exit(_angle)
   local self = RollComponent()
   self.name = "roll_angle_entry_exit"
   local angle = _angle
   local entry_s = math.abs(angle) / AEROM_ENTRY_RATE:get()
   
   function self.get_roll(t, time_s)
      local entry_t = entry_s / time_s
      if entry_t > 0.5 then
         entry_t = 0.5
      end
      if t <= 0 then
         return 0
      end
      if t < entry_t then
         return angle * t / entry_t
      end
      if t < 1.0 - entry_t then
         return angle
      end
      if angle == 0 or t >= 1.0 then
         return 0
      end
      return (1.0 - ((t - (1.0 - entry_t)) / entry_t)) * angle
   end
   return self
end

--[[
   roll component that banks to _angle over AEROM_ENTRY_RATE
   degrees/s, then holds that angle
--]]
function roll_angle_entry(_angle)
   local self = RollComponent()
   self.name = "roll_angle_entry"
   local angle = _angle
   local entry_s = math.abs(angle) / AEROM_ENTRY_RATE:get()
   
   function self.get_roll(t, time_s)
      local entry_t = entry_s / time_s
      if entry_t > 0.5 then
         entry_t = 0.5
      end
      if t < entry_t then
         return angle * t / entry_t
      end
      return angle
   end
   return self
end

--[[
   roll component that holds angle until the end of the subpath, then
   rolls back to 0 at the AEROM_ENTRY_RATE
--]]
function roll_angle_exit(_angle)
   local self = {}
   self.name = "roll_angle_exit"
   local angle = _angle
   local entry_s = math.abs(angle) / AEROM_ENTRY_RATE:get()
   
   function self.get_roll(t, time_s)
      local entry_t = entry_s / time_s
      if t < 1.0 - entry_t then
         return 0
      end
      if angle == 0 then
         return 0
      end
      return ((t - (1.0 - entry_t)) / entry_t) * angle
   end
   return self
end

--[[
  all path components inherit from PathComponent
--]]
function PathComponent()
   local self = {}
   self.name = nil
   function self.get_pos(t)
      return makeVector3f(0, 0, 0)
   end
   function self.get_length()
      return 0
   end
   function self.get_final_orientation()
      return Quaternion()
   end
   function self.get_roll_correction(t)
      return 0
   end
   function self.get_throttle_boost(t)
      return self.thr_boost or false
   end
   return self
end

--[[
   rotate a vector by a quaternion
--]]
function quat_earth_to_body(quat, v)
   local v = v:copy()
   quat:earth_to_body(v)
   return v
end

--[[
   rotate a vector by a inverse quaternion
--]]
function quat_body_to_earth(quat, v)
   local v = v:copy()
   quat:inverse():earth_to_body(v)
   return v
end

--[[
   copy a quaternion
--]]
function quat_copy(q)
   return q:inverse():inverse()
end

--[[
   path component that does a straight horizontal line
--]]
function path_straight(_distance)
   local self = PathComponent()
   self.name = "path_straight"
   local distance = _distance
   function self.get_pos(t)
      return makeVector3f(distance*t, 0, 0)
   end
   function self.get_length()
      return distance
   end
   return self
end

--[[
   path component that does a vertical arc over a given angle
--]]
function path_vertical_arc(_radius, _angle)
   local self = PathComponent()
   self.name = "path_vertical_arc"
   local radius = _radius
   local angle = _angle
   function self.get_pos(t)
      local t2ang = wrap_2pi(t * math.rad(angle))
      return makeVector3f(math.abs(radius)*math.sin(t2ang), 0, -radius*(1.0 - math.cos(t2ang)))
   end
   function self.get_length()
      return math.abs(radius) * 2 * math.pi * math.abs(angle) / 360.0
   end
   function self.get_final_orientation()
      local q = Quaternion()
      q:from_axis_angle(makeVector3f(0,1,0), sgn(radius)*math.rad(wrap_180(angle)))
      q:normalize()
      return q
   end
   return self
end

--[[
 integrate a function, assuming fn takes a time t from 0 to 1
--]]
function integrate_length(fn)
   local total = 0.0
   local p = fn(0)
   for i = 1, 100 do
      local t = i*0.01
      local p2 = fn(t)
      local dv = p2 - p
      total = total + dv:length()
      p = p2
   end
   return total
end

--[[
   path component that does a horizontal arc over a given angle
--]]
function path_horizontal_arc(_radius, _angle, _height_gain)
   local self = PathComponent()
   self.name = "path_horizontal_arc"
   local radius = _radius
   local angle = _angle
   local height_gain = _height_gain
   if height_gain == nil then
      height_gain = 0
   end
   function self.get_pos(t)
      local t2ang = t * math.rad(angle)
      return makeVector3f(math.abs(radius)*math.sin(t2ang), radius*(1.0 - math.cos(t2ang)), -height_gain*t)
   end
   function self.get_length()
      local circumference = 2 * math.pi * math.abs(radius)
      local full_circle_height_gain = height_gain * 360.0 / math.abs(angle)
      local helix_length = math.sqrt(full_circle_height_gain*full_circle_height_gain + circumference*circumference)
      return helix_length * math.abs(angle) / 360.0
   end
   function self.get_final_orientation()
      local q = Quaternion()
      q:from_axis_angle(makeVector3f(0,0,1), sgn(radius)*math.rad(angle))
      return q
   end

   --[[
      roll correction for the rotation caused by height gain
   --]]
   function self.get_roll_correction(t)
      if height_gain == 0 then
         return 0
      end
      local gamma=math.atan(height_gain*(angle/360)/(2*math.pi*radius))
      return -t*360*math.sin(gamma)
   end
   return self
end

--[[
   path component that does a cylinder for a barrel roll
--]]
function path_cylinder(_radius, _length, _num_spirals)
   local self = PathComponent()
   self.name = "path_cylinder"
   local radius = _radius
   local length = _length
   local num_spirals = _num_spirals
   local gamma = math.atan((length/num_spirals)/(2*math.pi*radius))
   local qrot = Quaternion()
   qrot:from_axis_angle(makeVector3f(0,0,1), (0.5*math.pi)-gamma)

   function self.get_pos(t)
      local t2ang = t * num_spirals * math.pi * 2
      local v = makeVector3f(length*t, math.abs(radius)*math.sin(t2ang+math.pi), -radius*(1.0 - math.cos(t2ang)))
      return quat_earth_to_body(qrot, v)
   end

   function self.get_length()
      local circumference = 2 * math.pi * math.abs(radius)
      local length_per_spiral = length / num_spirals
      local helix_length = math.sqrt(length_per_spiral*length_per_spiral + circumference*circumference)
      return helix_length * num_spirals
   end

   --[[
      roll correction for the rotation caused by the path
   --]]
   function self.get_roll_correction(t)
      return t*360*math.sin(gamma)*num_spirals
   end

   return self
end

--[[
   a Path has the methods of both RollComponent and
   PathComponent allowing for a complete description of a subpath
--]]
function Path(_path_component, _roll_component)
   local self = {}
   self.name = string.format("%s|%s", _path_component.name, _roll_component.name)
   local path_component = _path_component
   local roll_component = _roll_component
   function self.get_roll(t, time_s)
      return wrap_180(roll_component.get_roll(t, time_s))
   end
   function self.get_roll_correction(t)
      return path_component.get_roll_correction(t)
   end
   function self.get_pos(t)
      return path_component.get_pos(t)
   end
   function self.get_speed(t)
      return nil
   end
   function self.get_length()
      return path_component.get_length()
   end
   function self.get_final_orientation()
      return path_component.get_final_orientation()
   end
   function self.get_throttle_boost(t)
      return self.thr_boost or false
   end
   return self
end


--[[
   componse multiple sub-paths together to create a full trajectory
--]]
function path_composer(_name, _subpaths)
   local self = {}
   self.name = _name
   local subpaths = _subpaths
   local lengths = {}
   local proportions = {}
   local start_time = {}
   local end_time = {}
   local start_orientation = {}
   local start_pos = {}
   local start_angle = {}
   local start_roll_correction = {}
   local end_speed = {}
   local start_speed = {}
   local total_length = 0
   local num_sub_paths = #subpaths
   local last_subpath_t = { -1, 0, 0 }

   local orientation = Quaternion()
   local pos = makeVector3f(0,0,0)
   local angle = 0
   local roll_correction = 0
   local speed = target_groundspeed()
   local highest_i = 0
   local cache_i = -1
   local cache_sp = nil
   local message = nil

   -- return the subpath with index i. Used to cope with two ways of calling path_composer
   function self.subpath(i)
      if i == cache_i then
         return cache_sp
      end
      cache_i = i
      local sp = subpaths[i]
      if sp.name then
         -- we are being called with a list of Path objects
         cache_sp = sp
         message = nil
      else
         -- we are being called with a list function/argument tuples
         local args = subpaths[i][2]
         cache_sp = subpaths[i][1](args[1], args[2], args[3], args[4], start_pos[i], start_orientation[i])
         message = subpaths[i].message
      end
      return cache_sp
   end
   
   for i = 1, num_sub_paths do
      -- accumulate orientation, position and angle
      start_orientation[i] = quat_copy(orientation)
      start_pos[i] = pos:copy()
      start_angle[i] = angle
      start_roll_correction[i] = roll_correction

      local sp = self.subpath(i)
      lengths[i] = sp.get_length()
      total_length = total_length + lengths[i]

      local spos = quat_earth_to_body(orientation, sp.get_pos(1.0))

      pos = pos + spos
      orientation = sp.get_final_orientation() * orientation
      orientation:normalize()

      angle = angle + sp.get_roll(1.0, lengths[i]/speed)
      roll_correction = roll_correction + sp.get_roll_correction(1.0)

      start_speed[i] = speed
      end_speed[i] = sp.get_speed(1.0)
      if end_speed[i] == nil then
         end_speed[i] = target_groundspeed()
      end
      speed = end_speed[i]

      if sp.roll_ref ~= nil then
         local q = Quaternion()
         q:from_axis_angle(makeVector3f(1,0,0), math.rad(sp.roll_ref))
         orientation = orientation * q
         orientation:normalize()
      end
   end

   -- get our final orientation, including roll
   local final_orientation = quat_copy(orientation)
   local q = Quaternion()
   q:from_axis_angle(makeVector3f(1,0,0), math.rad(wrap_180(angle)))
   final_orientation = q * final_orientation

   -- work out the proportion of the total time we will spend in each sub path
   local total_time = 0
   for i = 1, num_sub_paths do
      proportions[i] = lengths[i] / total_length
      start_time[i] = total_time
      end_time[i] = total_time + proportions[i]
      total_time = total_time + proportions[i]
   end

   function self.get_subpath_t(t)
      if last_subpath_t[1] == t then
         -- use cached value
         return last_subpath_t[2], last_subpath_t[3]
      end
      local i = 1
      while t >= end_time[i] and i < num_sub_paths do
         i = i + 1
      end
      local subpath_t = (t - start_time[i]) / proportions[i]
      last_subpath_t = { t, subpath_t, i }
      local sp = self.subpath(i)
      if i > highest_i and t < 1.0 and t > 0 then
         highest_i = i
         if message ~= nil then
            gcs:send_text(0, message)
         end
         if AEROM_DEBUG:get() > 0 then
            gcs:send_text(0, string.format("starting %s[%d] %s", self.name, i, sp.name))
         end
      end
      return subpath_t, i
   end

   -- return position at time t
   function self.get_pos(t)
      local subpath_t, i = self.get_subpath_t(t)
      local sp = self.subpath(i)
      return quat_earth_to_body(start_orientation[i], sp.get_pos(subpath_t)) + start_pos[i]
   end

   -- return angle for the composed path at time t
   function self.get_roll(t, time_s)
      local subpath_t, i = self.get_subpath_t(t)
      local speed = self.get_speed(t)
      if speed == nil then
         speed = target_groundspeed()
      end
      local sp = self.subpath(i)
      angle = sp.get_roll(subpath_t, lengths[i]/speed)
      return angle + start_angle[i]
   end

   function self.get_roll_correction(t)
      local subpath_t, i = self.get_subpath_t(t)
      local sp = self.subpath(i)
      return sp.get_roll_correction(subpath_t) + start_roll_correction[i]
   end
   
   -- return speed for the composed path at time t
   function self.get_speed(t)
      local subpath_t, i = self.get_subpath_t(t)
      return start_speed[i] + subpath_t * (end_speed[i] - start_speed[i])
   end

   function self.get_length()
      return total_length
   end

   function self.get_final_orientation()
      return final_orientation
   end

   function self.get_throttle_boost(t)
      local subpath_t, i = self.get_subpath_t(t)
      local sp = self.subpath(i)
      return sp.get_throttle_boost(t)
   end

   return self
end


--[[
   make a list of Path() objects from a list of PathComponent, RollComponent pairs
--]]
function make_paths(name, paths)
   local p = {}
   for i = 1, #paths do
      if paths[i][2] == nil then
         p[i] = paths[i][1]
      else
         p[i] = Path(paths[i][1], paths[i][2])
      end
      if paths[i].roll_ref then
         p[i].roll_ref = paths[i].roll_ref
      end
      p[i].thr_boost = paths[i].thr_boost
   end
   return path_composer(name, p)
end

--[[
   composed trajectories, does as individual aerobatic maneuvers
--]]

function climbing_circle(radius, height, bank_angle, arg4)
   return make_paths("climbing_circle", {
         { path_horizontal_arc(radius, 360, height), roll_angle_entry_exit(bank_angle) },
   })
end

function half_climbing_circle(radius, height, bank_angle, arg4)
   return make_paths("half_climbing_circle", {
         { path_horizontal_arc(radius, 180, height), roll_angle_entry_exit(bank_angle) },
   })
end

function partial_circle(radius, bank_angle, angle)
   return make_paths("partial_circle", {
         { path_horizontal_arc(radius, angle, 0), roll_angle_entry_exit(bank_angle) },
   })
end

function loop(radius, bank_angle, num_loops, arg4)
   if not num_loops or num_loops <= 0 then
      num_loops = 1
   end
   return make_paths("loop", {
         { path_vertical_arc(radius, 360*num_loops), roll_angle_entry_exit(bank_angle) },
   })
end

function straight_roll(length, num_rolls, arg3, arg4)
   return make_paths("straight_roll", {
         { path_straight(length), roll_angle(num_rolls*360) },
   })
end

--[[

   fly straight until we are distance meters from the composite path
   origin in the maneuver frame along the X axis. If we are already
   past that position then return immediately
--]]
function straight_align(distance, arg2, arg3, arg4, start_pos, start_orientation)
   local d2 = distance - start_pos:x()
   local v = quat_earth_to_body(start_orientation, makeVector3f(d2, 0, 0))
   local len = math.max(v:x(),0.01)
   return make_paths("straight_align", {
         { path_straight(len), roll_angle(0) },
   })
end

function immelmann_turn(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("immelmann_turn", {
         { path_vertical_arc(r, 180),      roll_angle(0) },
         { path_straight(rabs/2),          roll_angle(180) },
   })
end

-- immelmann with max roll rate
function immelmann_turn_fast(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   local roll_time = 180.0 / ACRO_ROLL_RATE:get()
   local roll_dist = target_groundspeed() * roll_time
   return make_paths("immelmann_turn_fast", {
         { path_vertical_arc(r, 180),      roll_angle(0) },
         { path_straight(roll_dist),       roll_angle(180) },
   })
end

function humpty_bump(r, h, arg3, arg4)
   assert(h >= 2*r)
   local rabs = math.abs(r)
   return make_paths("humpty_bump", {
            { path_vertical_arc(r, 90),             roll_angle(0) },
            { path_straight((h-2*rabs)/3),          roll_angle(0) },
            { path_straight((h-2*rabs)/3),          roll_angle(180) },
            { path_straight((h-2*rabs)/3),          roll_angle(0) },
            { path_vertical_arc(-r, 180),           roll_angle(0) },
            { path_straight(h-2*rabs),              roll_angle(0) },
            { path_vertical_arc(-r, 90),            roll_angle(0) },
            { path_straight(2*rabs),                roll_angle(0) },
   })
end

function crossbox_humpty(r, h, arg3, arg4)
   assert(h >= 2*r)
   local rabs = math.abs(r)
   return make_paths("crossbox_humpty", {
            { path_vertical_arc(r, 90),          roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(90),  roll_ref=90 },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(-r, 180),        roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(90), roll_ref=-90 },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(r, 90),          roll_angle(0), roll_ref=180 },
   })
end

function laydown_humpty(r, h, arg3, arg4)
   assert(h >= 2*r)
   local rabs = math.abs(r)
   return make_paths("laydown_humpty", {
            { path_vertical_arc(r, 45),          roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(-90), roll_ref=90 },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(r, 180),         roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_straight((h-2*rabs)/3),       roll_angle(90),  roll_ref=-90 },
            { path_straight((h-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(-r, 45),         roll_angle(0), roll_ref=180},
   })
end

function split_s(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("split_s", {
         { path_straight(rabs/2),                roll_angle(180) },
         { path_vertical_arc(-r, 180),           roll_angle(0) },
   })
end

function upline_45(r, height_gain, arg3, arg4)
   --local h = (height_gain - 2*r*(1.0-math.cos(math.rad(45))))/math.sin(math.rad(45))
   local h = (height_gain - (2 * r) + (2 * r * math.cos(math.rad(45)))) / math.cos(math.rad(45))
  assert(h >= 0)
   return make_paths("upline_45", {
         { path_vertical_arc(r, 45),  roll_angle(0) },
         { path_straight(h),          roll_angle(0) },
         { path_vertical_arc(-r, 45), roll_angle(0) },
   })
end

function upline_20(r, height_gain, arg3, arg4)
   local h = (height_gain - 2*r*(1.0-math.cos(math.rad(20))))/math.sin(math.rad(20))
   assert(h >= 0)
   return make_paths("upline_45", {
         { path_vertical_arc(r, 20),  roll_angle(0) },
         { path_straight(h),          roll_angle(0) },
         { path_vertical_arc(-r, 20), roll_angle(0) },
   })
end

function downline_45(r, height_loss, arg3, arg4)
   local h = (height_loss - 2*r*(1.0-math.cos(math.rad(45))))/math.sin(math.rad(45))
   assert(h >= 0)
   return make_paths("downline_45", {
         { path_vertical_arc(-r, 45),  roll_angle(0) },
         { path_straight(h),           roll_angle(0) },
         { path_vertical_arc(r, 45),   roll_angle(0) },
   })
end

function rolling_circle(radius, num_rolls, arg3, arg4)
   return make_paths("rolling_circle", {
         { path_horizontal_arc(radius, 360), roll_angle(360*num_rolls), thr_boost=true },
   })
end


function cylinder(radius, length, num_spirals, arg4)
   return make_paths("cylinder", {
         { path_cylinder(radius, length, num_spirals), roll_angle(0), thr_boost=true },
   })
end

function barrel_roll(radius, length, num_spirals, arg4)
   local gamma_deg = math.deg(math.atan((length/num_spirals)/(2*math.pi*radius)))
   local speed = target_groundspeed()
   local bank = math.deg(math.atan((speed*speed) / (radius * GRAVITY_MSS)))
   local radius2 = radius/(1.0 - math.cos(math.rad(90-gamma_deg)))

   return make_paths("barrel_roll", {
         { path_horizontal_arc(-radius2, 90-gamma_deg, 0), roll_angle_entry_exit(-bank) },
         { path_cylinder(radius, length, num_spirals),    roll_angle(0) },
         { path_horizontal_arc(radius2, 90-gamma_deg, 0),  roll_angle_entry_exit(bank) },
   })
end

function side_step(displacement, length, arg3, arg4)
   local speed = target_groundspeed()
   local radius = (displacement*displacement + length*length)/(4*displacement)
   local angle = math.deg(2*math.atan(displacement, length))
   local sign = sgn(displacement)
   local bank = math.deg(math.atan((speed*speed) / (radius * GRAVITY_MSS)))
   displacement = math.abs(displacement)

   return make_paths("side_step",{
      {path_horizontal_arc(sign*radius, angle, 0), roll_angle_entry_exit(sign*bank)},
      {path_horizontal_arc(-sign*radius, angle, 0) , roll_angle_entry_exit(-sign*bank)},
   })
end

function straight_flight(length, bank_angle, arg3, arg4)
   return make_paths("straight_flight", {
         { path_straight(length), roll_angle_entry_exit(bank_angle) },
   })
end

function straight_hold(length, bank_angle, arg3, arg4)
   return make_paths("straight_hold", {
         { path_straight(length), roll_angle_entry(bank_angle) },
   })
end

function scale_figure_eight(r, bank_angle, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("scale_figure_eight", {
         { path_straight(rabs),             roll_angle(0) },
         { path_horizontal_arc(r,  90),     roll_angle_entry_exit(bank_angle) },
         { path_horizontal_arc(-r, 360),    roll_angle_entry_exit(-bank_angle) },
         { path_horizontal_arc(r,  270),    roll_angle_entry_exit(bank_angle) },
         { path_straight(3*rabs),           roll_angle(0) },
   })
end

function figure_eight(r, bank_angle, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("figure_eight", {
         { path_straight(rabs*math.sqrt(2)), roll_angle(0) },
         { path_horizontal_arc(r,  225),     roll_angle_entry_exit(bank_angle) },
         { path_straight(2*rabs),            roll_angle(0) },
         { path_horizontal_arc(-r,  270),    roll_angle_entry_exit(-bank_angle) },
         { path_straight(2*rabs),            roll_angle(0) },
         { path_horizontal_arc(r,    45),    roll_angle_entry_exit(bank_angle) },
   })
end


--[[
   stall turn is not really correct, as we don't fully stall. Needs to be
   reworked
--]]
function stall_turn(radius, height, direction, min_speed)
   local h = height - radius
   assert(h >= 0)
   return make_paths("stall_turn", {
         { path_vertical_arc(radius, 90),          roll_angle(0) },
         { path_straight(h),                       roll_angle(0), min_speed },
         { path_horizontal_arc(5*direction, 180),  roll_angle(0), min_speed },
         { path_straight(h),                       roll_angle(0) },
         { path_vertical_arc(radius, 90),          roll_angle(0) },
   })
end

function half_cuban_eight(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("half_cuban_eight", {
         { path_straight(2*rabs*math.sqrt(2)), roll_angle(0) },
         { path_vertical_arc(r,  225),         roll_angle(0) },
         { path_straight(2*rabs/3),            roll_angle(0) },
         { path_straight(2*rabs/3),            roll_angle(180) },
         { path_straight(2*rabs/3),            roll_angle(0) },
         { path_vertical_arc(-r, 45),          roll_angle(0) },
   })
end

function cuban_eight(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("cuban_eight", {
         { path_straight(rabs*math.sqrt(2)), roll_angle(0) },
         { path_vertical_arc(r,  225),       roll_angle(0) },
         { path_straight(2*rabs/3),          roll_angle(0) },
         { path_straight(2*rabs/3),          roll_angle(180) },
         { path_straight(2*rabs/3),          roll_angle(0) },
         { path_vertical_arc(-r, 270),       roll_angle(0) },
         { path_straight(2*rabs/3),          roll_angle(0) },
         { path_straight(2*rabs/3),          roll_angle(180) },
         { path_straight(2*rabs/3),          roll_angle(0) },
         { path_vertical_arc(r, 45),         roll_angle(0) },
   })
end

function half_reverse_cuban_eight(r, arg2, arg3, arg4)
   local rabs = math.abs(r)
   return make_paths("half_reverse_cuban_eight", {
         { path_vertical_arc(r,  45),   roll_angle(0) },
         { path_straight(2*rabs/3),     roll_angle(0) },
         { path_straight(2*rabs/3),     roll_angle(180) },
         { path_straight(2*rabs/3),     roll_angle(0) },
         { path_vertical_arc(-r, 225),  roll_angle(0) },
   })
end

function horizontal_rectangle(total_length, total_width, r, bank_angle)
   local l = total_length - 2*r
   local w = total_width - 2*r
      
   return make_paths("horizontal_rectangle", {
         { path_straight(0.5*l),        roll_angle(0) },
         { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle)},
         { path_straight(w),            roll_angle(0) },
         { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
         { path_straight(l),            roll_angle(0) },
         { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
         { path_straight(w),            roll_angle(0) },
         { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
         { path_straight(0.5*l),        roll_angle(0) },
   })
end

function vertical_aerobatic_box(total_length, total_width, r, bank_angle)
   local l = total_length - 2*r
   local w = total_width - 2*r
      
   return make_paths("vertical_aerobatic_box", {
         { path_straight(0.5*l),       roll_angle_entry(bank_angle) },
         { path_vertical_arc(r, 90),   roll_angle(0) },
         { path_straight(w),           roll_angle(0) },
         { path_vertical_arc(r, 90),   roll_angle(0) },
         { path_straight(l),           roll_angle(0) },
         { path_vertical_arc(r, 90),   roll_angle(0) },
         { path_straight(w),           roll_angle(0) },
         { path_vertical_arc(r, 90),   roll_angle(0) },
         { path_straight(0.5*l),       roll_angle_exit(-bank_angle) },
   })
end

function multi_point_roll(length, N, arg3, arg4)
   local paths = {}
   local len_roll = length * (N-1) / (N*4-1)
   local len_hold = length / (N*4-1)
   local ang = 360 / N
   for i = 1, N do
      paths[#paths+1] = { path_straight(len_roll), roll_angle(ang) }
      paths[#paths+1] = { path_straight(len_hold), roll_angle(0) }
   end
   return make_paths("multi_point_roll", paths)
end

function two_point_roll(length, arg2, arg3, arg4)
   return multi_point_roll(length, 2)
end

function four_point_roll(length, arg2, arg3, arg4)
   return multi_point_roll(length, 4)
end

function eight_point_roll(length, arg2, arg3, arg4)
   return multi_point_roll(length, 8)
end

function procedure_turn(radius, bank_angle, step_out, arg4)
   local rabs = math.abs(radius)
   return make_paths("procedure_turn", {
            { path_straight(rabs),                  roll_angle(0) },
            { path_horizontal_arc(radius,  90),     roll_angle_entry_exit(bank_angle) },
            { path_straight(step_out),              roll_angle(0) },
            { path_horizontal_arc(-radius,  270),   roll_angle_entry_exit(-bank_angle) },
            { path_straight(4*rabs),                roll_angle(0) },
      })
end

function derry_turn(radius, bank_angle, arg3, arg4)
   return make_paths("derry_turn", {
            { path_horizontal_arc(radius,  90),     roll_angle_entry_exit(bank_angle) },
            { path_horizontal_arc(-radius,  90),    roll_angle_entry_exit(-bank_angle) },
      })
end

function p23_1(radius, height, width, arg4) -- top hat
   return make_paths("p23_1", {
            { path_vertical_arc(radius, 90),              roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(90) },
            { path_straight((height-2*radius)/9),         roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(90) },
            { path_straight((height-2*radius)*2/9),       roll_angle(0) },            
            { path_vertical_arc(-radius, 90),             roll_angle(0) },
            { path_straight((width-2*radius)/3),          roll_angle(0) },
            { path_straight((width-2*radius)/3),          roll_angle(180) },
            { path_straight((width-2*radius)/3),          roll_angle(0) },
            { path_vertical_arc(-radius, 90),             roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(90) },
            { path_straight((height-2*radius)/9),         roll_angle(0) },
            { path_straight((height-2*radius)*2/9),       roll_angle(90) },
            { path_straight((height-2*radius)*2/9),       roll_angle(0) },            
            { path_vertical_arc(radius, 90),              roll_angle(0) },                        
      })
end

function p23_2(radius, height, arg3, arg4)  -- half square
   return make_paths("p23_2", {
            { path_vertical_arc(-radius, 90),         roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(180) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_vertical_arc(-radius, 90),         roll_angle(0) },                    
      })
end

function p23_3(radius, height, arg3, arg4)   -- humpty
   return make_paths("p23_3", {
            { path_vertical_arc(radius, 90),          roll_angle(0) },
            { path_straight((height-2*radius)/8),     roll_angle(0) },
            { path_straight((height-2*radius)*6/8),   roll_angle(360) },
            { path_straight((height-2*radius)/8),     roll_angle(0) },
            { path_vertical_arc(radius, 180),         roll_angle(0) },    
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(180) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_vertical_arc(radius, 90),          roll_angle(0) },                                
      })
end

function p23_4(radius, height, arg3, arg4)   -- on corner  
   local l = ((height - (2 * radius)) * math.sin(math.rad(45)))                             
   return make_paths("p23_4", {
            { path_vertical_arc(-radius, 45),          roll_angle(0) },
            { path_straight(l/3),                      roll_angle(0) },
            { path_straight(l/3),                      roll_angle(180) },
            { path_straight(l/3),                      roll_angle(0) },
            { path_vertical_arc(-radius, 90),          roll_angle(0) },    
            { path_straight(l/3),                      roll_angle(0) },
            { path_straight(l/3),                      roll_angle(180) },
            { path_straight(l/3),                      roll_angle(0) },
            { path_vertical_arc(-radius, 45),          roll_angle(0) },                                
      })
end

function p23_5(radius, height_gain, arg3, arg4)   -- 45 up - should be 1 1/2 snaps....
    --local l = (height_gain - 2*radius*(1.0-math.cos(math.rad(45))))/math.sin(math.rad(45))
   local l = (height_gain - (2 * radius) + (2 * radius * math.cos(math.rad(45)))) / math.cos(math.rad(45))
   return make_paths("p23_5", {
            { path_vertical_arc(-radius, 45),        roll_angle(0) },
            { path_straight(l/3),                    roll_angle(0) },
            { path_straight(l/3),                    roll_angle(540) },
            { path_straight(l/3),                    roll_angle(0) },
            { path_vertical_arc(radius, 45),         roll_angle(0) },            
      })
end

function p23_6(radius, height_gain, arg3, arg4)   -- 3 sided
   local l = (height_gain - 2*radius) / ((2*math.sin(math.rad(45))) + 1)
   return make_paths("p23_6", {
            { path_vertical_arc(-radius, 45),      roll_angle(0) },
            { path_straight(l),                    roll_angle(0) },
            { path_vertical_arc(-radius, 45),      roll_angle(0) },
            { path_straight(l),                    roll_angle(0) },
            { path_vertical_arc(-radius, 45),      roll_angle(0) },            
            { path_straight(l),                    roll_angle(0) },
            { path_vertical_arc(-radius, 45),      roll_angle(0) },            
      })
end

function p23_7(length, arg2, arg3, arg4) -- roll combination
   return make_paths("p23_7", {
            { path_straight(length*5/22),       roll_angle(180) },
            { path_straight(length*1/22),       roll_angle(0) },
            { path_straight(length*5/22),       roll_angle(180) },
            { path_straight(length*5/22),       roll_angle(-180) },
            { path_straight(length*1/22),       roll_angle(0) },
            { path_straight(length*5/22),       roll_angle(-180) },                 
      })
end

function p23_8(radius, height, arg3, arg4)  -- immelmann
   return make_paths("p23_8", {
         { path_vertical_arc(-radius, 180),           roll_angle(0) },
         { path_straight(radius/2),                   roll_angle(180) },                    
      })
end

function p23_9(radius, height, num_turns, arg4)   -- spin (currently a vert down 1/2 roll)
   return make_paths("p23_9", {
            { path_vertical_arc(radius, 90),          roll_angle(0) },
            { path_straight(height-2*radius),         roll_angle(180) },
            { path_vertical_arc(-radius, 90),         roll_angle(0) },                             
      })
end

function p23_10(radius, height, arg3, arg4)   -- humpty                              
   return make_paths("p23_10", {
            { path_vertical_arc(radius, 90),               roll_angle(0) },
            { path_straight((height-2*radius)/3),          roll_angle(0) },
            { path_straight((height-2*radius)/3),          roll_angle(180) },
            { path_straight((height-2*radius)/3),          roll_angle(0) },
            { path_vertical_arc(-radius, 180),             roll_angle(0) },
            { path_straight((height-2*radius)/3),          roll_angle(0) },
            { path_straight((height-2*radius)/3),          roll_angle(180) },
            { path_straight((height-2*radius)/3),          roll_angle(0) },
            { path_vertical_arc(-radius, 90),              roll_angle(0) },                             
      })
end

function p23_11(radius, height, arg3, arg4)   -- laydown loop
   local rabs = math.abs(radius)
   local vert_length = height - (2 * rabs)               
   local angle_length =  ((2 * rabs) - (2 * (rabs - (rabs * (math.cos(math.rad(45))))))) / math.sin(math.rad(45))                                     
   return make_paths("p23_11", {
            { path_vertical_arc(-radius, 45),            roll_angle(0) },
            { path_straight(angle_length*2/6),           roll_angle(0) },
            { path_straight(angle_length*1/6),           roll_angle(180) },
            { path_straight(angle_length*1/6),           roll_angle(-180) },
            { path_straight(angle_length*2/6),           roll_angle(0) },
            { path_vertical_arc(radius, 315),            roll_angle(0) },         
            { path_straight(vert_length*2/9),            roll_angle(0) },
            { path_straight(vert_length*2/9),            roll_angle(90) },
            { path_straight(vert_length*1/9),            roll_angle(0) },
            { path_straight(vert_length*2/9),            roll_angle(90) },
            { path_straight(vert_length*2/9),            roll_angle(0) },
            { path_vertical_arc(radius, 90),             roll_angle(0) },            
      })
end

function p23_12(radius, height, arg3, arg4)   -- 1/2 square
   return make_paths("p23_12", {
            { path_vertical_arc(-radius, 90),         roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(180) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_vertical_arc(-radius, 90),         roll_angle(0) },              
      })
end

function p23_13(radius, height, arg3, arg4)  -- stall turn
   return make_paths("p23_13", {
            { path_vertical_arc(radius, 90),          roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(90) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_vertical_arc(-radius, 90),         roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_straight((height-2*radius)/3),     roll_angle(90) },
            { path_straight((height-2*radius)/3),     roll_angle(0) },
            { path_vertical_arc(-radius, 90),         roll_angle(0) },          
      })
end

function p23_13a(radius, height, arg3, arg4)  -- stall turn PLACE HOLDER
   assert(height >= 2*radius)
   local rabs = math.abs(radius)
   return make_paths("P23_13a", {
            { path_vertical_arc(radius, 90),          roll_angle(0) },
            { path_straight((height-2*rabs)/3),       roll_angle(0) },
            { path_straight((height-2*rabs)/3),       roll_angle(90),  roll_ref=90 },
            { path_straight((height-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(-radius, 180),        roll_angle(0) },
            { path_straight((height-2*rabs)/3),       roll_angle(0) },
            { path_straight((height-2*rabs)/3),       roll_angle(90),  roll_ref=-90 },
            { path_straight((height-2*rabs)/3),       roll_angle(0) },
            { path_vertical_arc(radius, 90),          roll_angle(0), roll_ref=180 },
     })
end


function p23_14(r, h, arg3, arg4)   -- fighter turn
   assert(h >= 2*r)
   local rabs = math.abs(r)
   local angle_length = (h - ((0.2929 * rabs)) / (math.sin(math.rad(45)))) - rabs
   return make_paths("fighter_turn", {
            { path_vertical_arc(r, 45),              roll_angle(0) },
            { path_straight((angle_length)/3),       roll_angle(0) },
            { path_straight((angle_length)/3),       roll_angle(-90),  roll_ref=90 },
            { path_straight((angle_length)/3),       roll_angle(0) },
            { path_vertical_arc(r, 180),             roll_angle(0) },
            { path_straight((angle_length)/3),       roll_angle(0) },
            { path_straight((angle_length)/3),       roll_angle(90), roll_ref=-90 },
            { path_straight((angle_length)/3),       roll_angle(0) },
            { path_vertical_arc(-r, 45),             roll_angle(0), roll_ref=180 },
   })
end                            

function p23_15(radius, height, arg3, arg4)   -- triangle
   local h1 = radius * math.sin(math.rad(45))
   local h2 = (2 * radius) - (radius * math.cos(math.rad(45)))
   local h3 = height - (2 * radius)
   local side = h3 / math.cos(math.rad(45))
   --local base = (h3 + (2 * (radius - radius * math.cos(math.rad(45))))) - (2 * radius)   
   local base = (2 * (h3 + radius)) - 2 * radius
   return make_paths("p23_15", {
            { path_straight(base * 1/5),                   roll_angle(180) },
            { path_straight(base * 2/5),                   roll_angle(0) },         
            { path_vertical_arc(radius, 135) ,             roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(90) },
            { path_straight(side*1/9),                     roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(90) },
            { path_straight(side*2/9),                     roll_angle(0) },
            { path_vertical_arc(radius, 90),               roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(90) },
            { path_straight(side*1/9),                     roll_angle(0) },
            { path_straight(side*2/9),                     roll_angle(90) },
            { path_straight(side*2/9),                     roll_angle(0) },
            { path_vertical_arc(radius, 135),              roll_angle(0) },
            { path_straight(base * 2/5),                   roll_angle(0) }, 
            { path_straight(base * 1/5),                   roll_angle(180) },
            { path_straight(base * 2/5),                   roll_angle(0) },  
      })
end

function p23_16(radius, height, arg3, arg4)   -- sharks tooth
   local angle_length = (height - 2 * (radius - (radius * math.cos(math.rad(45))))) / math.cos(math.rad(45))
   local vert_length = height - (2 * radius)
   return make_paths("p23_16", {
            { path_vertical_arc(radius, 90),         roll_angle(0) },
            { path_straight((vert_length)/3),        roll_angle(0) },
            { path_straight((vert_length)/3),        roll_angle(180) },
            { path_straight((vert_length)/3),        roll_angle(0) },           
            { path_vertical_arc(radius, 135),        roll_angle(0) },
            { path_straight(angle_length*2/9),       roll_angle(0) },
            { path_straight(angle_length*2/9),       roll_angle(90) },
            { path_straight(angle_length*1/9),       roll_angle(0) },
            { path_straight(angle_length*2/9),       roll_angle(90) },
            { path_straight(angle_length*2/9),       roll_angle(0) },
            { path_vertical_arc(-radius, 45),        roll_angle(0), roll_ref=180 },
      })
end

function p23_17(radius, arg2, arg3, arg4)   -- loop
   return make_paths("p23_17", {
            { path_vertical_arc(radius, 135),      roll_angle(0) },
            { path_vertical_arc(radius, 90),       roll_angle(180) },
            { path_vertical_arc(radius, 135),      roll_angle(0), roll_ref=180 },
      })
end

function half_roll(arg1, arg2, arg3, arg4)   -- half roll for testing inverted manouvers
   return make_paths("half_roll", {
            { path_straight(40),             roll_angle(180) },
            { path_straight(10),             roll_angle(0) },   
      })
end

function fai_f3a_box_l_r()
   return path_composer("f3a_box_l_r", {     -- positioned for a flight line 150m out. Flight line 520m total length.
                                             -- Script start point is ON CENTER, with the model heading DOWNWIND!
          { straight_roll,              { 150,   0 } },
          { half_reverse_cuban_eight,   { 95 } },
          { straight_align,             { 0, 0 } },
          { vertical_aerobatic_box,     { 540, 190, 45,  0 },     message="Starting Box Demo"},
          { vertical_aerobatic_box,     { 540, 190, 45,  0 } },
          { vertical_aerobatic_box,     { 540, 190, 45,  0 } },
          { vertical_aerobatic_box,     { 540, 190, 45,  0 } },
          { straight_roll,              { 50, 0 } }
   })
end

--[[
   NZ clubman schedule
--]]
function nz_clubman()                               -- positioned for a flight line 100m out
                                                    -- Script start point is ON CENTER, with the model heading DOWNWIND!
   return path_composer("nz_clubman_l_r", {
          --[[
          { straight_roll,            { 20,  0 } },
          { procedure_turn,           { 20, 45, 60 } },
          { straight_roll,            { 150, 0 } },
          { half_reverse_cuban_eight, { 40 } },
          { straight_roll,            { 150, 0 } },
          --]]
          { straight_roll,            { 150,   0 } },
          { half_reverse_cuban_eight, { 90 } },
          { straight_align,           { 0, 0 } },
          { cuban_eight,              { 90 },          message="Cuban Eight"},
          { straight_align,           { -100, 0 } },
          { half_reverse_cuban_eight, { 90 } },
          { straight_align,           { 40, 0 } },
          { half_reverse_cuban_eight, { 90 },          message="Half Rev Cuban"},
          { straight_align,           { -180, 0 } },
          { half_reverse_cuban_eight, { 90 } },
          { straight_align,           { -120, 0 } },
          { two_point_roll,           { 240 },         message="Two Point Roll"},
          { straight_align,           { 150, 0 } },
          { half_reverse_cuban_eight, { 90 } },
          { straight_align,           { 106, 0 } },
          { upline_45,                { 40, 180 },     message="45 Upline"},
          { straight_align,           { -180, 0 } },                          -- missing the stall turn
          { split_s,                  { 90, 90 } },
          { straight_align,           { -120, 0 } },
          { straight_roll,            { 240, 1 },      message="Slow Roll"},
          { straight_align,           { 150, 0 } },
          { half_cuban_eight,         { 90 } },
          { straight_align,           { 0, 0 } },
          { loop,                     { 90, 0, 2 },    message="Two Loops"},
          { straight_align,           { -180, 0 } },
          { immelmann_turn,           { 90, 90 } },
          { straight_align,           { -106, 0 } },
          { downline_45,              { 40, 180 },     message="45 Downline"},
          { straight_align,           { 150, 0 } },
          { half_cuban_eight,         { 90 } },
          { straight_roll,            { 100, 0 } },
   })
end

--[[
   F3A p23, preliminary schedule 2023
--]]
function f3a_p23_l_r()
   return path_composer("f3a_p23_l_r", {            -- positioned for a flight line 150m out. Flight line 520m total length.
                                                    -- Script start point is ON CENTER, with the model heading DOWNWIND!
          { straight_roll,              { 160,   0 } },
          { half_reverse_cuban_eight,   { 80 } },
          { straight_align,             { 140, 0 } },
          { p23_1,                      { 40, 200, 200 },  message="Top Hat"}, 
          { straight_align,             { -220, 0 } },
          { p23_2,                      { 40, 200 },       message="Half Square Loop"},     
          { straight_align,             { 0, 0 } },
          { p23_3,                      { 40, 200 },       message="Humpty"},    
          { straight_align,             { 160, 0 } },
          { p23_4,                      { 40, 200 },       message="Half Square on Corner"},   
          { straight_align,             { 116, 0 } },
          { p23_5,                      { 40, 200 },       message="45 Up"},                        -- snap roll
          { straight_align,             { -185, 0 } },
          { p23_6,                      { 40, 200 },       message="Half Eight Sided Loop"},  
          { straight_align,             { -100, 0 } },    
          { p23_7,                      { 200 },           message="Roll Combination"},      
          { straight_align,             { 160, 0 } },
          { p23_8,                      { 100 },           message="Immelmann Turn"},      
          { straight_align,             { 40, 0 } },
          { p23_9,                      { 40, 200 },       message="Should be a Spin"},            -- spin
          { straight_align,             { -140, 0 } },
          { p23_10,                     { 40, 200 },       message="Humpty"}, 
          { straight_align,             { -91, 0 } },
          { p23_11,                     { 50, 200 },       message="Laydown Loop"},  
          { straight_align,             { 220, 0 } },
          { p23_12,                     { 40, 200 },       message="Half Square Loop"},   
          { straight_align,             { 40, 0 } },
          { p23_13a,                    { 40, 200 },       message="Stall Turn"},                  -- stall turn    
          { straight_roll,              { 100,   0 } },      
          { p23_14,                     { 40, 180 },       message="Fighter Turn"},                  
          { straight_align,             { -24, 0 } },
          { p23_15,                     { 40, 200 },       message="Triangle"},   
          { straight_align,             { 220, 0 } },
          { p23_16,                     { 40, 160 },       message="Sharks Tooth"},    
          { straight_align,             { 0, 0 } },
          { p23_17,                     { 100 },           message="Loop"},  
          { straight_roll,              { 100, 0 } },
    })
end

--[[
   F4C Scale Schedule Example
--]]

function f4c_example_l_r()                          -- positioned for a flight line nominally 150m out (some manouvers start 30m out)
                                                    -- Script start point is ON CENTER @ 150m, with the model heading DOWNWIND ie flying Right to Left!
   return path_composer("f4c_example", {    
         { straight_roll,             { 320,   0 } },
         { half_climbing_circle,      { -70, 0, -60 } },            -- come in close for the first two manouvers
         --{ straight_roll,             { 10,   0 } },
         { straight_align,            { 280, 0 } },
         { scale_figure_eight,        { -140, -30 },        message="Scale Figure Eight"},   
         { straight_roll,             { 80,   0 } },
         { immelmann_turn,            { 90       } }, 
         { straight_align,            { 0, 0 } },
         --{ straight_roll,             { 340,   0 } },
         { climbing_circle,           { 140, -205, 30 },    message="Descending 360"},   
         { straight_roll,             { 40,   0 } },
         { upline_20,                 { 80, 25 } },                  -- Climb up 25m to base height
         { straight_roll,             { 50,   0 } },
         { half_climbing_circle,      { 70, 0, 60 } },               -- Go back out to 150m
         { straight_align,            { 0, 0 } },
         { loop,                      { 90,    0, 1 },      message="Loop"},       
         { straight_align,            { -50, 0 } },
         { half_reverse_cuban_eight,  { 90       } },
         { straight_align,            { 0, 0 } },
         { immelmann_turn,            { 90       },         message="Immelmann Turn"},    
         { straight_align,            { -140, 0 } },
         { split_s,                   { 90       } },
         { straight_align,            { 0, 0 } },
         { half_cuban_eight,          { 90       },         message="Half Cuban Eight"},  
         { straight_align,            { -180, 0 } },
         { half_climbing_circle,      { 70, 0, 60 } },  
         --{ straight_roll,             { 115,   0 } }, 
         { straight_align,            { -90, 0 } },         
         { derry_turn,                { 90,   60 },         message="Derry Turn"}, 
         { straight_roll,             { 200,   0 } },           
         { half_climbing_circle,      { -90, 0, -60 } },   
         { straight_align,            { 0, 0 } },         
         { climbing_circle,           { -140,    0, -30 },  message="Gear Demo"}, 
         { straight_roll,             { 250,   0 } },
         { half_climbing_circle,      { -100, 0, -60 } },
         { straight_align,            { -185, 0 } },
         { barrel_roll,               { 90, 240, 1 },      message="Barrel Roll"},  
         { straight_roll,             { 60,    0 } },
         { half_reverse_cuban_eight,  { 90       }},
         { straight_roll,             { 60,    0 } },
   })
end

--[[
   simple air show
--]]

function air_show1()
   return path_composer("AirShow", {
         { loop,                      {  25, 0, 1 }, message="Loop"},
         { straight_align,            {  80, 0 } },
         { half_reverse_cuban_eight,  {  25       }, message="HalfReverseCubanEight" },
         { straight_align,            {  80 } },
         { scale_figure_eight,        { -40, -45 },  message="ScaleFigureEight" },
         { immelmann_turn,            {  30       }, message="Immelmann" },
         { straight_align,            { -40, 0 } },
         { straight_roll,             {  80, 2 },    message="Roll" },
         { straight_align,            { 120, 0 } },
         { split_s,                   {  30       }, message="Split-S"},
         { straight_align,            {   0 } },
         { rolling_circle,            { -50, 3},     message="RollingCircle" },
         { straight_align,            { -50, 0 } },
         { humpty_bump,               {  20, 60 },   message="HumptyBump" },
         { straight_align,            {  80, 0 } },
         { half_cuban_eight,          {  25       }, message="HalfCubanEight" },
         { straight_align,            {  75, 0 } },
         { upline_45,                 {  30, 50 },   message="Upline45", },
         { downline_45,               {  30, 50 },   message="Downline45" },
         { half_reverse_cuban_eight,  {  25       }, message="HalfReverseCubanEight" },
         { straight_align,            {   0 } },
   })
end

function air_show3()
   return path_composer("AirShow3", {
           { air_show1, {}, message="AirShowPt1" },
           { air_show1, {}, message="AirShowPt2" },
           { air_show1, {}, message="AirShowPt3" },
   })
end

function test_all_paths()
   return path_composer("test_all_paths", {
          { figure_eight,             { 100,  45 } },
          { straight_roll,            { 20,    0 } },
          { loop,                     { 30,    0,  1 } },
          { straight_roll,            { 20,    0 } },
          { horizontal_rectangle,     { 100, 100, 20, 45 } },
          { straight_roll,            { 20,    0 } },
          { climbing_circle,          { 100,  20, 45 } },
          { straight_roll,            { 20,    0 } },
          { vertical_aerobatic_box,   { 100, 100, 20,  0 } },
          { straight_roll,            { 20,    0 } },
          { rolling_circle,           { 100,   2,  0,  0 } },
          { straight_roll,            { 20,    0 } },
          { half_cuban_eight,         { 30,      } },
          { straight_roll,            { 20,    0 } },
          { half_reverse_cuban_eight, { 30,      } },
          { straight_roll,            { 20,    0 } },
          { cuban_eight,              { 30,      } },
          { straight_roll,            { 20,    0 } },
          { humpty_bump,              { 30,  100 } },
          { straight_flight,          { 100,  45 } },
          { scale_figure_eight,       { 100,  45 } },
          { straight_roll,            { 20,    0 } },
          { immelmann_turn,           { 30,   60 } },
          { straight_roll,            { 20,    0 } },
          { split_s,                  { 30,   60 } },
          { straight_roll,            { 20,    0 } },
          { upline_45,                { 20,   50 } },
          { straight_roll,            { 20,    0 } },
          { downline_45,              { 20,   50 } },
          { straight_roll,            { 20,    0 } },
          { procedure_turn,           { 40, 45, 20 } },
          { straight_roll,            { 20,    0 } }, 
          { two_point_roll,           { 100      } },
          { straight_roll,            { 20,    0 } },
          { derry_turn,               { 40,   60 } },
          { straight_roll,            { 20,    0 } },
          { half_climbing_circle,     { -65, 0, -60 } },
          { straight_roll,            { 20,    0 } },
          --[[
          { p23_1,                    { 20, 150, 150 } }, 
          { straight_roll,            { 20,    0 } },
          { p23_2,                    { 20,  150 } },      
          { straight_roll,            { 20,    0 } },
          { p23_3,                    { 20,  150 } },      
          { straight_roll,            { 20,    0 } },
          { p23_4,                    { 20,  150 } },      
          { straight_roll,            { 20,    0 } },
          { p23_5,                    { 20,  150 } },   
          { straight_roll,            { 20,    0 } },
          { p23_6,                    { 20,  150 } },       -- now inverted :-)
          { straight_roll,            { 20,    0 } },
          --]]
   })
end

---------------------------------------------------

--[[
   target speed is taken as max of target airspeed and current 3D
   velocity at the start of the maneuver
--]]
function target_groundspeed()
   return math.max(ahrs:get_EAS2TAS()*TRIM_ARSPD_CM:get()*0.01, ahrs:get_velocity_NED():length())
end

--[[
   get ground course from AHRS
--]]
function get_ground_course_deg()
   local vned = ahrs:get_velocity_NED()
   return wrap_180(math.deg(math.atan(vned:y(), vned:x())))
end


--args:
--  path_f: path function returning position 
--  t: normalised [0, 1] time
--  arg1, arg2: arguments for path function
--  orientation: maneuver frame orientation
--returns: requested position, angle and speed in maneuver frame
function rotate_path(path_f, t, orientation, offset)
   local t = constrain(t, 0, 1)
   local point = path_f.get_pos(t)
   local angle = path_f.get_roll(t)
   local roll_correction = path_f.get_roll_correction(t)
   local speed = path_f.get_speed(t)
   local thr_boost = path_f.get_throttle_boost(t)
   local point = quat_earth_to_body(orientation, point)

   local scale = AEROM_PATH_SCALE:get()
   point = point:scale(math.abs(scale))
   if scale < 0 then
      -- we need to mirror the path
      point:y(-point:y())
      roll_correction = -roll_correction
      angle = -angle
      -- compensate path orientation for the mirroring
      local orient = orientation:inverse()
      point = quat_body_to_earth((orient * orient), point)
   end

   return point+offset, math.rad(angle+roll_correction), speed, thr_boost
end

--Given vec1, vec2, returns an (rotation axis, angle) tuple that rotates vec1 to be parallel to vec2
--If vec1 and vec2 are already parallel, returns a zero vector and zero angle
--Note that the rotation will not be unique.
function vectors_to_rotation(vector1, vector2)
   local axis = vector1:cross(vector2)
   if axis:length() < 0.00001 then
      local vec = Vector3f()
      vec:x(1)
      return vec, 0
   end
   axis:normalize()
   local angle = vector1:angle(vector2)
   return axis, angle
end

--returns Quaternion
function vectors_to_rotation_w_roll(vector1, vector2, roll)
   local axis, angle = vectors_to_rotation(vector1, vector2)
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
   local axis, angle = vectors_to_rotation(vector1, vector2)
   local angular_velocity = angle/time_constant
   return axis:scale(angular_velocity)
end

function vectors_to_angular_rate_w_roll(vector1, vector2, time_constant, roll)
   local axis, angle = vectors_to_rotation_w_roll(vector1, vector2, roll)
   local angular_velocity = angle/time_constant
   return axis:scale(angular_velocity)
end

-- convert a quaternion to axis angle form
function to_axis_and_angle(quat)
   local axis_angle = Vector3f()
   quat:to_axis_angle(axis_angle)
   local angle = axis_angle:length()
   if angle < 0.00001 then
      return makeVector3f(1.0, 0.0, 0.0), 0.0
   end
   return axis_angle:scale(1.0/angle), angle
end

--projects x onto the othogonal subspace of span(unit_v)
function ortho_proj(x, unit_v)
   local temp_x = unit_v:cross(x)
   return unit_v:cross(temp_x)
end

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

--[[
   check if a number is Nan.
--]]
function isNaN(value)
   -- NaN is lua is not equal to itself
   return value ~= value
end

function Vec3IsNaN(v)
   return isNaN(v:x()) or isNaN(v:y()) or isNaN(v:z())
end

function qIsNaN(q)
   return isNaN(q:q1()) or isNaN(q:q2()) or isNaN(q:q3()) or isNaN(q:q4())
end

--[[
   return the body y projection down, this is the c.y element of the equivalent rotation matrix
--]]
function quat_projection_ground_plane(q)
   local q1q2 = q:q1() * q:q2()
   local q3q4 = q:q3() * q:q4()
   return 2.0 * (q3q4 + q1q2)
end

path_var.count = 0

function do_path()
   local now = millis():tofloat() * 0.001
   local ahrs_pos_NED = ahrs:get_relative_position_NED_origin()
   local ahrs_pos = ahrs:get_position()
   local ahrs_gyro = ahrs:get_gyro()
   local ahrs_velned = ahrs:get_velocity_NED()
   local ahrs_airspeed = ahrs:airspeed_estimate()
   --[[
      ahrs_quat is the quaterion which when used with quat_earth_to_body() rotates a vector
      from earth to body frame. It needs to be the inverse of ahrs:get_quaternion()
   --]]
   local ahrs_quat = ahrs:get_quaternion():inverse()

   path_var.count = path_var.count + 1
   local target_dt = 1.0/LOOP_RATE
   local path = current_task.fn

   if not current_task.started then
      local initial_yaw_deg = current_task.initial_yaw_deg
      current_task.started = true

      local speed = target_groundspeed()
      path_var.target_speed = speed

      path_var.length = path.get_length() * math.abs(AEROM_PATH_SCALE:get())

      path_var.total_rate_rads_ef = makeVector3f(0.0, 0.0, 0.0)

      --assuming constant velocity
      path_var.total_time = path_var.length/speed

      --deliberately only want yaw component, because the maneuver should be performed relative to the earth, not relative to the initial orientation
      path_var.initial_ori = Quaternion()
      path_var.initial_ori:from_euler(0, 0, math.rad(initial_yaw_deg))
      path_var.initial_ori = path_var.initial_ori
      path_var.initial_ori:normalize()

      path_var.initial_ef_pos = ahrs_pos_NED:copy()

      path_var.start_pos = ahrs_pos:copy()
      path_var.path_int = path_var.start_pos:copy()

      speed_PI.reset()


      path_var.accumulated_orientation_rel_ef = path_var.initial_ori

      path_var.time_correction = 0.0 

      path_var.filtered_angular_velocity = Vector3f()

      path_var.last_time = now - 1.0/LOOP_RATE
      path_var.sideslip_angle_rad = { 0.0, 0.0 }
      path_var.ff_yaw_rate_rads = 0.0
      path_var.last_q_change_t = 1.0 / LOOP_RATE
      path_var.last_ang_rate_dps = ahrs_gyro:scale(math.deg(1))

      path_var.path_t = 0.0

      path_var.pos = path_var.initial_ef_pos:copy()
      path_var.roll = 0.0
      path_var.speed = nil

      -- get initial tangent
      local p1, r1, s1 = rotate_path(path, path_var.path_t + 0.1/(path_var.total_time*LOOP_RATE),
                                     path_var.initial_ori, path_var.initial_ef_pos)
      path_var.tangent = p1 - path_var.pos
      return true
   end
   
   local vel_length = ahrs_velned:length()

   local actual_dt = now - path_var.last_time
   if actual_dt < 0.25 / LOOP_RATE then
      -- the update has been executed too soon
      return true
   end

   path_var.last_time = now

   local local_n_dt = actual_dt/path_var.total_time

   if path_var.path_t + local_n_dt > 1.0 then
      -- all done
      return false
   end

   -- airspeed, assume we don't go below min
   local airspeed_constrained = math.max(ARSPD_FBW_MIN:get(), ahrs_airspeed)

   --[[
      calculate positions and angles at previous, current and next time steps
   --]]

   local p0 = path_var.pos:copy()
   local r0 = path_var.roll
   local s0 = path_var.speed
   local p1, r1, s1 = rotate_path(path, path_var.path_t + local_n_dt,
                                  path_var.initial_ori, path_var.initial_ef_pos)

   local current_measured_pos_ef = ahrs_pos_NED:copy()

   --[[
      get tangents to the path
   --]]
   local tangent1_ef = path_var.tangent:copy()
   local tangent2_ef = p1 - p0

   local tv_unit = tangent2_ef:copy()
   if tv_unit:length() < 0.00001 then
      gcs:send_text(0, string.format("path not advancing %f", tv_unit:length()))
   end
   tv_unit:normalize()

   --[[
      use actual vehicle velocity to calculate how far along the
      path we have progressed
   --]]
   local v = ahrs_velned:copy()
   local path_dist = v:dot(tv_unit)*actual_dt
   if path_dist < 0 then
      gcs:send_text(0, string.format("aborting %.2f at %d tv=(%.2f,%.2f,%.2f) vx=%.2f adt=%.2f",
                                     path_dist, path_var.count,
                                     tangent2_ef:x(),
                                     tangent2_ef:y(),
                                     tangent2_ef:z(),
                                     v:x(), actual_dt))
      return false
   end
   local path_t_delta = constrain(path_dist/path_var.length, 0.2*local_n_dt, 4*local_n_dt)

   --[[
      recalculate the current path position and angle based on actual delta time
   --]]
   local p1, r1, s1, thr_boost = rotate_path(path,
                                             constrain(path_var.path_t + path_t_delta, 0, 1),
                                             path_var.initial_ori, path_var.initial_ef_pos)

   local last_path_t = path_var.path_t
   path_var.path_t = path_var.path_t + path_t_delta

   -- tangents needs to be recalculated
   tangent2_ef = p1 - p0
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

   -- get the path again with the corrected time
   p1, r1, s1 = rotate_path(path,
                            constrain(path_var.path_t, 0, 1),
                            path_var.initial_ori, path_var.initial_ef_pos)

   -- recalculate the tangent to match the amount we advanced the path time
   tangent2_ef = p1 - p0

   -- get the real world time corresponding to the quaternion change
   local q_change_t = (path_var.path_t - last_path_t) * path_var.total_time

   -- low pass filter the demanded roll angle
   r1 = path_var.roll + wrap_pi(r1 - path_var.roll)
   local alpha = calc_lowpass_alpha(q_change_t, AEROM_ANG_TC:get())
   r1 = (1.0 - alpha) * path_var.roll + alpha * r1
   r1 = wrap_pi(r1)

   path_var.tangent = tangent2_ef:copy()
   path_var.pos = p1:copy()
   path_var.roll = r1
   path_var.speed = s1

   
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

   local acc_err_bf = quat_earth_to_body(ahrs_quat, acc_err_ef)

   local TAS = constrain(ahrs:get_EAS2TAS()*airspeed_constrained, 3, 100)
   local corr_rate_bf_y_rads = -acc_err_bf:z()/TAS
   local corr_rate_bf_z_rads = acc_err_bf:y()/TAS

   local cor_ang_vel_bf_rads = makeVector3f(0.0, corr_rate_bf_y_rads, corr_rate_bf_z_rads)
   if Vec3IsNaN(cor_ang_vel_bf_rads) then
      cor_ang_vel_bf_rads = makeVector3f(0,0,0)
   end
   local cor_ang_vel_bf_dps = cor_ang_vel_bf_rads:scale(math.deg(1))

   if path_var.count < 2 then
      cor_ang_vel_bf_dps = Vector3f()
   end

   --[[
      work out body frame path rate, this is based on two adjacent tangents on the path
   --]]
   local path_rate_ef_rads = tangents_to_rate(tangent1_ef, tangent2_ef, actual_dt)
   if Vec3IsNaN(path_rate_ef_rads) then
      gcs:send_text(0,string.format("path_rate_ef_rads: NaN"))
      path_rate_ef_rads = makeVector3f(0,0,0)
   end
   local path_rate_ef_dps = path_rate_ef_rads:scale(math.deg(1))
   if path_var.count < 3 then
      -- cope with small initial misalignment
      path_rate_ef_dps:z(0)
   end
   local path_rate_bf_dps = quat_earth_to_body(ahrs_quat, path_rate_ef_dps)

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

   local mf_axis = quat_earth_to_body(path_var.accumulated_orientation_rel_ef, makeVector3f(1, 0, 0))

   local orientation_rel_mf_with_roll_angle = Quaternion()
   orientation_rel_mf_with_roll_angle:from_axis_angle(mf_axis, r1)
   orientation_rel_ef_with_roll_angle = orientation_rel_mf_with_roll_angle*path_var.accumulated_orientation_rel_ef

   --[[
      calculate the error correction for the roll versus the desired roll
   --]]
   local roll_error = orientation_rel_ef_with_roll_angle * ahrs_quat
   roll_error:normalize()
   local err_axis_ef, err_angle_rad = to_axis_and_angle(roll_error)
   local time_const_roll = ROLL_CORR_TC:get()
   local err_angle_rate_ef_rads = err_axis_ef:scale(err_angle_rad/time_const_roll)
   local err_angle_rate_bf_dps = quat_earth_to_body(ahrs_quat,err_angle_rate_ef_rads):scale(math.deg(1))
   -- zero any non-roll components
   err_angle_rate_bf_dps:y(0)
   err_angle_rate_bf_dps:z(0)

   --[[
      implement lookahead for path rates
   --]]
   if AEROM_LKAHD:get() > 0 then
      local lookahead = AEROM_LKAHD:get()
      local lookahead_vt = lookahead / path_var.total_time
      p2 = rotate_path(path,
                       constrain(path_var.path_t+lookahead_vt, 0, 1),
                       path_var.initial_ori, path_var.initial_ef_pos)
      local tangent3_ef = p2 - p1
      local lk_ef_rads = tangents_to_rate(tangent2_ef, tangent3_ef, 0.5*(lookahead+(1.0/LOOP_RATE)))

      -- scale for airspeed
      lk_ef_rads = lk_ef_rads:scale(sq(vel_length/path_var.target_speed))

      local lookahead_bf_rads = quat_earth_to_body(ahrs_quat, lk_ef_rads)
      local lookahead_bf_dps = lookahead_bf_rads:scale(math.deg(1))
      logger.write('AELK','Py,Ly,Pz,Lz', 'ffff',
                   path_rate_bf_dps:y(),
                   lookahead_bf_dps:y(),
                   path_rate_bf_dps:z(),
                   lookahead_bf_dps:z())
      path_rate_bf_dps:y(lookahead_bf_dps:y())
      path_rate_bf_dps:z(lookahead_bf_dps:z())
   end
   
   --[[
      calculate an additional yaw rate to get us to the right angle of sideslip for knifeedge
   --]]
   local g_force = (path_rate_ef_rads:cross(v)):scale(1.0/GRAVITY_MSS)
   local specific_force_g_ef = g_force - makeVector3f(0,0,-1)
   local specific_force_g_bf = quat_earth_to_body(orientation_rel_ef_with_roll_angle, specific_force_g_ef)
   local airspeed_scaling = SCALING_SPEED:get()/airspeed_constrained

   local sideslip_rad = specific_force_g_bf:y() * (airspeed_scaling*airspeed_scaling) * math.rad(AEROM_KE_ANG:get())
   local ff_yaw_rate_rads1 = -(sideslip_rad - path_var.sideslip_angle_rad[2]) / q_change_t
   local ff_yaw_rate_rads2 = -(path_var.sideslip_angle_rad[2] - path_var.sideslip_angle_rad[1]) / path_var.last_q_change_t
   local ff_yaw_rate_rads = 0.5 * (ff_yaw_rate_rads1 + ff_yaw_rate_rads2)

   if path_var.count <= 4 then
      -- prevent an initialisation issue
      ff_yaw_rate_rads = 0.0
   end

   ff_yaw_rate_rads = 0.8 * path_var.ff_yaw_rate_rads + 0.2 * ff_yaw_rate_rads
   path_var.ff_yaw_rate_rads = ff_yaw_rate_rads

   path_var.sideslip_angle_rad[1] = path_var.sideslip_angle_rad[2]
   path_var.sideslip_angle_rad[2] = sideslip_rad
   path_var.last_q_change_t = q_change_t

   local sideslip_rate_bf_dps = makeVector3f(0, 0, ff_yaw_rate_rads):scale(math.deg(1))

   --[[
      total angular rate is sum of path rate, correction rate and roll correction rate
   --]]
   local tot_ang_vel_bf_dps = path_rate_bf_dps + cor_ang_vel_bf_dps + err_angle_rate_bf_dps + sideslip_rate_bf_dps

   --[[
      apply angular accel limit
   --]]
   local ang_rate_diff_dps = tot_ang_vel_bf_dps - path_var.last_ang_rate_dps
   local max_delta_dps = AEROM_ANG_ACCEL:get() * actual_dt
   local max_delta_yaw_dps = max_delta_dps
   if AEROM_YAW_ACCEL:get() > 0 and
      (AEROM_YAW_ACCEL:get() < AEROM_ANG_ACCEL:get() or AEROM_ANG_ACCEL:get() <= 0) then
      max_delta_yaw_dps = AEROM_YAW_ACCEL:get() * actual_dt
   end
   if max_delta_dps > 0 then
      ang_rate_diff_dps:x(constrain(ang_rate_diff_dps:x(), -max_delta_dps, max_delta_dps))
      ang_rate_diff_dps:y(constrain(ang_rate_diff_dps:y(), -max_delta_dps, max_delta_dps))
   end
   if max_delta_yaw_dps > 0 then
      ang_rate_diff_dps:z(constrain(ang_rate_diff_dps:z(), -max_delta_yaw_dps, max_delta_yaw_dps))
   end

   tot_ang_vel_bf_dps = path_var.last_ang_rate_dps + ang_rate_diff_dps
   path_var.last_ang_rate_dps = tot_ang_vel_bf_dps


   --[[
      log POSM is pose-measured, POST is pose-track, POSB is pose-track without the roll
   --]]
   log_pose('POSM', current_measured_pos_ef, ahrs_quat:inverse())
   log_pose('POST', p1, orientation_rel_ef_with_roll_angle)

   logger.write('AETM', 'T,Terr,QCt,Adt','ffff',
                path_var.path_t,
                path_err_t,
                q_change_t,
                actual_dt)

   logger.write('AERT','Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz,Perr,Aerr,Yff,SS', 'ffffffffffffff',
                cor_ang_vel_bf_dps:x(), cor_ang_vel_bf_dps:y(), cor_ang_vel_bf_dps:z(),
                path_rate_bf_dps:x(), path_rate_bf_dps:y(), path_rate_bf_dps:z(),
                err_angle_rate_bf_dps:x(),
                tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z(),
                pos_error_ef:length(),
                wrap_180(math.deg(err_angle_rad)),
                math.deg(ff_yaw_rate_rads),
                math.deg(sideslip_rad))

   --log_pose('POSB', p1, path_var.accumulated_orientation_rel_ef)

   --[[
      run the throttle based speed controller
   --]]
   if s1 == nil then
      s1 = path_var.target_speed
   end

   --[[
      get the anticipated pitch at the throttle lookahead time
      we use the maximum of the current path pitch and the anticipated pitch
   --]]
   local qchange = Quaternion()
   qchange:from_angular_velocity(path_rate_ef_rads, AEROM_THR_LKAHD:get())
   local qnew = qchange * orientation_rel_ef_with_roll_angle
   local anticipated_pitch_rad = math.max(qnew:get_euler_pitch(), orientation_rel_ef_with_roll_angle:get_euler_pitch())

   local throttle = speed_PI.update(s1, anticipated_pitch_rad)
   local thr_min = AEROM_THR_MIN:get()
   if thr_boost then
      thr_min = math.max(thr_min, AEROM_THR_BOOST:get())
   end
   throttle = constrain(throttle, thr_min, 100.0)

   if isNaN(throttle) or Vec3IsNaN(tot_ang_vel_bf_dps) then
      gcs:send_text(0,string.format("Path NaN - aborting"))
      return false
   end

   vehicle:set_target_throttle_rate_rpy(throttle, tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z())

   if now - last_named_float_t > 1.0 / NAME_FLOAT_RATE then
      last_named_float_t = now
      gcs:send_named_float("PERR", pos_error_ef:length())
   end

   return true
end

--[[
   an object defining a path
--]]
function PathFunction(fn, name)
   local self = {}
   self.fn = fn
   self.name = name
   return self
end

local command_table = {}
command_table[1] = PathFunction(figure_eight, "Figure Eight")
command_table[2] = PathFunction(loop, "Loop")
command_table[3] = PathFunction(horizontal_rectangle, "Horizontal Rectangle")
command_table[4] = PathFunction(climbing_circle, "Climbing Circle")
command_table[5] = PathFunction(vertical_aerobatic_box, "Vertical Box")
command_table[6] = PathFunction(immelmann_turn_fast, "Immelmann Fast")
command_table[7] = PathFunction(straight_roll, "Axial Roll")
command_table[8] = PathFunction(rolling_circle, "Rolling Circle")
command_table[9] = PathFunction(half_cuban_eight, "Half Cuban Eight")
command_table[10]= PathFunction(half_reverse_cuban_eight, "Half Reverse Cuban Eight")
command_table[11]= PathFunction(cuban_eight, "Cuban Eight")
command_table[12]= PathFunction(humpty_bump, "Humpty Bump")
command_table[13]= PathFunction(straight_flight, "Straight Flight")
command_table[14]= PathFunction(scale_figure_eight, "Scale Figure Eight")
command_table[15]= PathFunction(immelmann_turn, "Immelmann Turn")
command_table[16]= PathFunction(split_s, "Split-S")
command_table[17]= PathFunction(upline_45, "Upline-45")
command_table[18]= PathFunction(downline_45, "Downline-45")
command_table[19]= PathFunction(stall_turn, "Stall Turn")
command_table[20]= PathFunction(procedure_turn, "Procedure Turn")
command_table[21]= PathFunction(derry_turn, "Derry Turn")
command_table[22]= PathFunction(two_point_roll, "Two Point Roll")
command_table[23]= PathFunction(half_climbing_circle, "Half Climbing Circle")
command_table[24]= PathFunction(crossbox_humpty, "Crossbox Humpty")
command_table[25]= PathFunction(laydown_humpty, "Laydown Humpty")
command_table[26]= PathFunction(barrel_roll, "Barrel Roll")
command_table[27]= PathFunction(straight_flight, "Straight Hold")
command_table[28]= PathFunction(partial_circle, "Partial Circle")
command_table[29]= PathFunction(four_point_roll, "Four Point Roll")
command_table[30]= PathFunction(eight_point_roll, "Eight Point Roll")
command_table[31]= PathFunction(multi_point_roll, "Multi Point Roll")
command_table[32]= PathFunction(side_step, "Side Step")
command_table[200] = PathFunction(test_all_paths, "Test Suite")
command_table[201] = PathFunction(nz_clubman, "NZ Clubman")
command_table[202] = PathFunction(f3a_p23_l_r, "FAI F3A P23 L to R")
command_table[203] = PathFunction(f4c_example_l_r, "FAI F4C Example L to R")
command_table[204] = PathFunction(air_show1, "AirShow")
command_table[205] = PathFunction(fai_f3a_box_l_r, "FAI F3A Aerobatic Box Demonstration")
command_table[206] = PathFunction(air_show3, "AirShow3")


--[[
   a table of function available in loadable tricks
--]]
local load_table = {}
load_table["loop"] = loop
load_table["horizontal_rectangle"] = horizontal_rectangle
load_table["climbing_circle"] = climbing_circle
load_table["vertical_aerobatic_box"] = vertical_aerobatic_box
load_table["immelmann_turn_fast"] = immelmann_turn_fast
load_table["straight_roll"] = straight_roll
load_table["rolling_circle"] = rolling_circle
load_table["half_cuban_eight"] = half_cuban_eight
load_table["half_reverse_cuban_eight"] = half_reverse_cuban_eight
load_table["cuban_eight"] = cuban_eight
load_table["humpty_bump"] = humpty_bump
load_table["straight_flight"] = straight_flight
load_table["scale_figure_eight"] = scale_figure_eight
load_table["immelmann_turn"] = immelmann_turn
load_table["split_s"] = split_s
load_table["upline_45"] = upline_45
load_table["downline_45"] = downline_45
load_table["stall_turn"] = stall_turn
load_table["procedure_turn"] = procedure_turn
load_table["derry_turn"] = derry_turn
load_table["two_point_roll"] = two_point_roll
load_table["half_climbing_circle"] = half_climbing_circle
load_table["crossbox_humpty"] = crossbox_humpty
load_table["laydown_humpty"] = laydown_humpty
load_table["straight_align"] = straight_align
load_table["figure_eight"] = figure_eight
load_table["barrel_roll"] = barrel_roll
load_table["straight_hold"] = straight_hold
load_table["partial_circle"] = partial_circle
load_table["multi_point_roll"] = multi_point_roll
load_table["side_step"] = side_step
load_table["p23_1a"] = p23_1a
load_table["p23_1"] = p23_1
load_table["p23_13a"] = p23_13a
load_table["p23_14"] = p23_14
load_table["p23_15"] = p23_15
load_table["p23_16"] = p23_16
load_table["p23_17"] = p23_17


--[[
   load a trick description from a text file
--]]
function load_trick(id)
   if command_table[id] ~= nil then
      -- already have it
      return
   end
   -- look in 3 possible locations for the trick, coping with SITL and real boards
   local trickdirs = { "APM/scripts/", "scripts/", "./" }
   local file = nil
   local fname = string.format("trick%u.txt", id)
   for i = 1, #trickdirs do
      file = io.open(trickdirs[i] .. fname, "r")
      if file then
         break
      end
   end
   if file == nil then
      gcs:send_text(0,string.format("Failed to open %s", fname))
      return
   end
   local name = string.format("Trick%u", id)
   local paths = {}
   local message = nil
   while true do
      local line = file:read()
      if not line then
         break
      end
      local _, _, cmd, arg1, arg2, arg3, arg4 = string.find(line, "^([%w_:]+)%s*([-.%d]*)%s*([-.%d]*)%s*([-.%d]*)%s*([-.%d]*)")
      if cmd == "" or cmd == nil or string.sub(cmd,1,1) == "#" then
         -- ignore comments
      elseif cmd == "name:" then
         _, _, name = string.find(line, "^name:%s*([%w_]+)$")
      elseif cmd == "message:" then
         _, _, message = string.find(line, "^message:%s*(.+)$")
      elseif cmd ~= nil then
         arg1 = tonumber(arg1) or 0
         arg2 = tonumber(arg2) or 0
         arg3 = tonumber(arg3) or 0
         arg4 = tonumber(arg4) or 0
         local f = load_table[cmd]
         if f == nil then
            gcs:send_text(0,string.format("Unknown command '%s' in %s", cmd, fname))
         else
            paths[#paths+1] = { f, { arg1, arg2, arg3, arg4 }}
            if message ~= nil then
               paths[#paths].message = message
               message = nil
            end
         end
      end
   end
   local pc = path_composer(name, paths)
   gcs:send_text(0, string.format("Loaded trick%u '%s'", id, name))
   command_table[id] = PathFunction(pc, name)
end

function PathTask(fn, name, id, initial_yaw_deg, arg1, arg2, arg3, arg4)
   self = {}
   if type(fn) == "table" then
      self.fn = fn
   else
      self.fn = fn(arg1, arg2, arg3, arg4)
   end
   self.name = name
   self.id = id
   self.initial_yaw_deg = initial_yaw_deg
   self.started = false
   return self
end

-- see if an auto mission item needs to be run
function check_auto_mission()
   id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
   if not id then
      return
   end
   if id ~= last_id then
      -- we've started a new command
      current_task = nil
      last_id = id
      local initial_yaw_deg = get_ground_course_deg()
      load_trick(cmd)
      gcs:send_text(0, string.format("Starting %s!", command_table[cmd].name ))

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
      local wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
      if math.abs(wrap_180(initial_yaw_deg - wp_yaw_deg)) > 90 then
         gcs:send_text(0, string.format("Doing turnaround!"))
         wp_yaw_deg = wrap_180(wp_yaw_deg + 180)
      end
      initial_yaw_deg = wp_yaw_deg
      current_task = PathTask(command_table[cmd].fn, command_table[cmd].name,
                              id, initial_yaw_deg, arg1, arg2, arg3, arg4)
   end
end

local last_trick_action_state = 0
local trick_sel_chan = nil
local last_trick_selection = nil

--[[
   get selected trick. Trick numbers are 1 .. TRIK_COUNT. A value of 0 is invalid
--]]
function get_trick_selection()
   if trick_sel_chan == nil then
      trick_sel_chan = rc:find_channel_for_option(TRIK_SEL_FN:get())
      if trick_sel_chan == nil then
         return 0
      end
   end
   -- get trick selection based on selection channel input and number of tricks
   local i = math.floor(TRIK_COUNT:get() * constrain(0.5*(trick_sel_chan:norm_input_ignore_trim()+1),0,0.999)+1)
   if TRICKS[i] == nil then
      return 0
   end
   return i
end

--[[
   check for running a trick
--]]
function check_trick()
   local selection = get_trick_selection()
   local action = rc:get_aux_cached(TRIK_ACT_FN:get())
   if action == 0 and current_task ~= nil then
      gcs:send_text(0,string.format("Trick aborted"))
      current_task = nil
      last_trick_selection = nil
      -- use invalid mode to disable script control
      vehicle:nav_scripting_enable(255)
      return
   end
   if selection == 0 then
      return
   end
   if action == 1 and selection ~= last_trick_selection then
      local id = TRICKS[selection].id:get()
      load_trick(id)
      if command_table[id] ~= nil then
         local cmd = command_table[id]
         gcs:send_text(0, string.format("Trick %u selected (%s)", selection, cmd.name))
         last_trick_selection = selection
         return
      end
   end
   if current_task ~= nil then
      -- let the task finish
      return
   end
   if action ~= last_trick_action_state then
      last_trick_selection = selection
      last_trick_action_state = action
      if selection == 0 then
         gcs:send_text(0, string.format("No trick selected"))
         return
      end
      local id = TRICKS[selection].id:get()
      load_trick(id)
      if command_table[id] == nil then
         gcs:send_text(0, string.format("Invalid trick ID %u", id))
         return
      end
      local cmd = command_table[id]
      if action == 1 then
         gcs:send_text(0, string.format("Trick %u selected (%s)", selection, cmd.name))
      end
      if action == 2 then
         last_trick_selection = nil
         local current_mode = vehicle:get_mode()
         if not vehicle:nav_scripting_enable(current_mode) then
            gcs:send_text(0, string.format("Tricks not available in mode"))
            return
         end
         gcs:send_text(0, string.format("Trick %u started (%s)", selection, cmd.name))
         local initial_yaw_deg = get_ground_course_deg()
         current_task = PathTask(cmd.fn,
                                 cmd.name,
                                 nil,
                                 initial_yaw_deg,
                                 TRICKS[selection].args[1]:get(),
                                 TRICKS[selection].args[2]:get(),
                                 TRICKS[selection].args[3]:get(),
                                 TRICKS[selection].args[4]:get())
      end
   end

end

function update()

   if vehicle:get_mode() == MODE_AUTO then
      check_auto_mission()
   elseif TRICKS ~= nil then
      check_trick()
   end

   if current_task ~= nil then
      if not do_path() then
         gcs:send_text(0, string.format("Finishing %s!", current_task.name))
         if current_task.id ~= nil then
            vehicle:nav_script_time_done(current_task.id)
         else
            -- use invalid mode to disable script control
            vehicle:nav_scripting_enable(255)
         end
         current_task = nil
      end
   end
   
   return update, 1000.0/LOOP_RATE
end

return update()
