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
AEROM_BOX_WIDTH = bind_add_param('BOX_WIDTH', 22, 400)
AEROM_STALL_THR = bind_add_param('STALL_THR', 23, 40)
AEROM_STALL_PIT = bind_add_param('STALL_PIT', 24, -20)
AEROM_KE_TC = bind_add_param('KE_TC', 25, 0.5)

-- cope with old param values
if AEROM_ANG_ACCEL:get() < 100 and AEROM_ANG_ACCEL:get() > 0 then
   AEROM_ANG_ACCEL:set_and_save(3000)
end
if AEROM_ANG_TC:get() > 1.0 then
   AEROM_ANG_TC:set_and_save(0.2)
end

ACRO_ROLL_RATE = Parameter("ACRO_ROLL_RATE")
ACRO_YAW_RATE = Parameter('ACRO_YAW_RATE')
ARSPD_FBW_MIN = Parameter("ARSPD_FBW_MIN")
SCALING_SPEED = Parameter("SCALING_SPEED")

GRAVITY_MSS = 9.80665

--[[
   list of attributes that can be added to a path element
--]]
local path_attribs = { "roll_ref", "set_orient", "rate_override", "thr_boost", "pos_corr", "message", "shift_xy" }

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

--[[
   find our rudder channel for stall turns
--]]
local K_RUDDER = 21
local rudder_chan = SRV_Channels:find_channel(K_RUDDER)
local RUDD_REVERSED = Parameter(string.format("SERVO%u_REVERSED", rudder_chan+1))
local RUDD_MIN = Parameter(string.format("SERVO%u_MIN", rudder_chan+1))
local RUDD_MAX = Parameter(string.format("SERVO%u_MAX", rudder_chan+1))

local function TrickDef(id, arg1, arg2, arg3, arg4)
   local self = {}
   self.id = id
   self.args = {arg1, arg2, arg3, arg4}
   return self
end

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

local function sq(x)
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

local LOOP_RATE = 40
local DO_JUMP = 177
local k_throttle = 70
local NAME_FLOAT_RATE = 2

local TRIM_ARSPD_CM = Parameter("TRIM_ARSPD_CM")

local last_id = 0
local current_task = nil
local last_named_float_t = 0

local path_var = {}

local function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

local function wrap_pi(angle)
   local angle_deg = math.deg(angle)
   local angle_wrapped = wrap_180(angle_deg)
   return math.rad(angle_wrapped)
end

local function wrap_2pi(angle)
   local angle_deg = math.deg(angle)
   local angle_wrapped = wrap_360(angle_deg)
   return math.rad(angle_wrapped)
end


--[[
   calculate an alpha for a first order low pass filter
--]]
local function calc_lowpass_alpha(dt, time_constant)
   local rc = time_constant/(math.pi*2)
   return dt/(dt+rc)
end

--[[ get the c.y element of a quaternion, which gives
   the projection of the vehicle y axis in the down direction
   This is equal to sin(roll)*cos(pitch)
--]]
local function get_quat_dcm_c_y(q)
   local q1 = q:q1()
   local q2 = q:q2()
   local q3 = q:q3()
   local q4 = q:q4()
   local q3q4 = q3 * q4
   local q1q2 = q1 * q2
   return 2*(q3q4 + q1q2)
end

--[[ get the c.y element of the DCM body to earth matrix, which gives
   the projection of the vehicle y axis in the down direction
   This is equal to sin(roll)*cos(pitch)
--]]
local function get_ahrs_dcm_c_y()
   return get_quat_dcm_c_y(ahrs:get_quaternion())
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

local function sgn(x)
   local eps = 0.000001
   if (x > eps) then
      return 1.0
   elseif x < eps then
      return -1.0
   else
      return 0.0
   end
end

local function get_wp_location(i)
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

local function resolve_jump(i)
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
local function makeVector3f(x, y, z)
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
local function vectors_to_quat_rotation(vector1, vector2)
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
local function tangents_to_rate(t1, t2, dt)
   local q_delta = vectors_to_quat_rotation(t1, t2)
   local rate_rads = Vector3f()
   q_delta:to_axis_angle(rate_rads)
   rate_rads = rate_rads:scale(1.0/dt)
   return rate_rads
end

--[[
   create a class that inherits from a base class
--]]
local function inheritsFrom(baseClass, _name)
    local new_class = { name = _name }
    local class_mt = { __index = new_class }

    function new_class:create()
        local newinst = {}
        setmetatable( newinst, class_mt )
        return newinst
    end

    if nil ~= baseClass then
        setmetatable( new_class, { __index = baseClass } )
    end

    return new_class
end

--[[
   return a quaternion for a roll, pitch, yaw (321 euler sequence) attitude
--]]
function qorient(roll_deg, pitch_deg, yaw_deg)
   local q = Quaternion()
   q:from_euler(math.rad(roll_deg), math.rad(pitch_deg), math.rad(yaw_deg))
   return q
end

--[[
   rotate a vector by a quaternion
--]]
local function quat_earth_to_body(quat, v)
   local v = v:copy()
   quat:earth_to_body(v)
   return v
end

--[[
   rotate a vector by a inverse quaternion
--]]
local function quat_body_to_earth(quat, v)
   local v = v:copy()
   quat:inverse():earth_to_body(v)
   return v
end

--[[
   copy a quaternion
--]]
local function quat_copy(q)
   return q:inverse():inverse()
end


--[[
   trajectory building blocks. We have two types of building blocks,
   roll blocks and path blocks. These are combined to give composite paths
--]]


--[[
  roll component that goes through a fixed total angle at a fixed roll rate
--]]
local _roll_angle = inheritsFrom(nil, 'roll_angle')
function _roll_angle:get_roll(t)
   if self.angle == nil then
      return 0
   end
   return self.angle * t
end
function roll_angle(angle)
   local self = _roll_angle:create()
   if angle ~= 0 then
      self.angle = angle
   end
   return self
end

--[[
   roll component that banks to _angle over AEROM_ENTRY_RATE
   degrees/s, then holds that angle, then banks back to zero at
   AEROM_ENTRY_RATE degrees/s
--]]
local _roll_angle_entry_exit = inheritsFrom(nil, "roll_angle_entry_exit")
function _roll_angle_entry_exit:get_roll(t, time_s)
   local entry_s = math.abs(self.angle) / AEROM_ENTRY_RATE:get()
   local entry_t = entry_s / time_s
   if entry_t > 0.5 then
      entry_t = 0.5
   end
   if t <= 0 then
      return 0
   end
   if t < entry_t then
      return self.angle * t / entry_t
   end
   if t < 1.0 - entry_t then
      return self.angle
   end
   if self.angle == 0 or t >= 1.0 then
      return 0
   end
   return (1.0 - ((t - (1.0 - entry_t)) / entry_t)) * self.angle
end

function roll_angle_entry_exit(angle)
   local self = _roll_angle_entry_exit:create()
   self.angle = angle
   return self
end

--[[
   roll component that banks to _angle over AEROM_ENTRY_RATE
   degrees/s, then holds that angle
--]]
local _roll_angle_entry = inheritsFrom(nil, "roll_angle_entry")
function _roll_angle_entry:get_roll(t, time_s)
   local entry_s = math.abs(self.angle) / AEROM_ENTRY_RATE:get()
   local entry_t = entry_s / time_s
   if entry_t > 0.5 then
      entry_t = 0.5
   end
   if t < entry_t then
      return self.angle * t / entry_t
   end
   return self.angle
end

function roll_angle_entry(angle)
   local self = _roll_angle_entry:create()
   self.angle = angle
   return self
end

--[[
   roll component that holds angle until the end of the subpath, then
   rolls back to 0 at the AEROM_ENTRY_RATE
--]]
local _roll_angle_exit = inheritsFrom(nil, "roll_angle_exit")
function _roll_angle_exit:get_roll(t, time_s)
   local entry_s = math.abs(self.angle) / AEROM_ENTRY_RATE:get()
   local entry_t = entry_s / time_s
   if t < 1.0 - entry_t then
      return 0
   end
   if self.angle == 0 then
      return 0
   end
   return ((t - (1.0 - entry_t)) / entry_t) * self.angle
end

function roll_angle_exit(angle)
   local self = _roll_angle_exit:create()
   self.angle = angle
   return self
end

--[[
   implement a sequence of rolls, specified as a list of {proportion, roll_angle} pairs
--]]
local _roll_sequence = inheritsFrom(nil, "roll_sequence")
function _roll_sequence:get_roll(t)
   for i = 1, #self.seq do
      if t <= self.end_t[i] then
         local t2 = (t - self.start_t[i])/(self.seq[i][1]/self.total)
         return self.start_ang[i] + t2 * self.seq[i][2]
      end
   end
   -- we've gone past the end
   return self.start_ang[#self.seq] + self.seq[#self.seq][2]
end

function roll_sequence(seq)
   local self = _roll_sequence:create()
   self.seq = seq
   self.total = 0.0
   self.end_t = {}
   self.start_t = {}
   self.start_ang = {}
   for i = 1, #seq do
      self.total = self.total + seq[i][1]
   end
   local t = 0.0
   local angle = 0.0
   for i = 1, #seq do
      self.start_t[i] = t
      self.start_ang[i] = angle
      angle = angle + seq[i][2]
      t = t + seq[i][1]/self.total
      self.end_t[i] = t
   end
   return self
end

--[[ given a path function get_pos() calculate the extents of the path
   along the X axis as a tuple
--]]
local function get_extents_x(obj)
   local p = obj:get_pos(0)
   local min_x = p:x()
   local max_x = min_x
   for t=0, 1, 0.02 do
      p = obj:get_pos(t)
      min_x = math.min(min_x, p:x())
      max_x = math.max(max_x, p:x())
   end
   return { min_x, max_x }
end


--[[
  all path components inherit from PathComponent
--]]
local _PathComponent = inheritsFrom(nil)
function _PathComponent:get_pos(t)
   return makeVector3f(0, 0, 0)
end
function _PathComponent:get_length()
   return 0
end
function _PathComponent:get_final_orientation()
   return Quaternion()
end
function _PathComponent:get_roll_correction(t)
   return 0
end
function _PathComponent:get_attribute(t, attrib)
   return self[attrib]
end
function _PathComponent:get_extents_x()
   if self.extents ~= nil then
      return self.extents
   end
   self.extents = get_extents_x(self)
   return self.extents
end

local function PathComponent()
   local self = _PathComponent:create()
   return self
end

--[[
   path component that does a straight horizontal line
--]]
local _path_straight = inheritsFrom(_PathComponent, "path_straight")
function _path_straight:get_pos(t)
   return makeVector3f(self.distance*t, 0, 0)
end
function _path_straight:get_length()
   return self.distance
end
function path_straight(distance)
   local self = _path_straight:create()
   self.distance = distance
   return self
end

--[[
   path component that does a straight line then reverses direction
--]]
local _path_reverse = inheritsFrom(_PathComponent, "path_reverse")
function _path_reverse:get_pos(t)
   if t < 0.5 then
      return makeVector3f(self.distance*t*2, 0, 0)
   else
      return makeVector3f(self.distance-(self.distance*(t-0.5)*2), 0, 0)
   end
end
function _path_reverse:get_length()
      return self.distance*2
end
function path_reverse(distance)
   local self = _path_reverse:create()
   self.distance = distance
   return self
end

--[[
   path component that aligns to within the aerobatic box
--]]
local _path_align_box = inheritsFrom(_PathComponent, "path_align_box")
function _path_align_box:get_pos(t)
   return makeVector3f(self.distance*t, 0, 0)
end
function _path_align_box:get_length()
   return self.distance
end
function _path_align_box:set_next_extents(extents, start_pos, start_orientation)
   local box_half = AEROM_BOX_WIDTH:get()/2
   local start_x = start_pos:x()
   local next_max_x = extents[2]
   if math.abs(math.deg(start_orientation:get_euler_yaw())) > 90 then
      -- we are on a reverse path
      self.distance = (box_half * self.alignment) + start_x - next_max_x
   else
      -- we are on a forward path
      self.distance = (box_half * self.alignment) - start_x - next_max_x
   end
   self.distance = math.max(self.distance, 0.01)
end
function path_align_box(alignment)
   local self = _path_align_box:create()
   self.distance = nil
   self.alignment = alignment
   return self
end

--[[
   path component that aligns so the center of the next maneuver is
   centered within the aerobatic box
--]]
local _path_align_center = inheritsFrom(_PathComponent, "path_align_center")
function _path_align_center:get_pos(t)
   return makeVector3f(self.distance*t, 0, 0)
end
function _path_align_center:get_length()
   return self.distance
end
function _path_align_center:set_next_extents(extents, start_pos, start_orientation)
   local start_x = start_pos:x()
   local next_mid_x = (extents[1]+extents[2])*0.5
   if math.abs(math.deg(start_orientation:get_euler_yaw())) > 90 then
      -- we are on a reverse path
      self.distance = start_x - next_mid_x
   else
      -- we are on a forward path
      self.distance = - start_x - next_mid_x
   end
   self.distance = math.max(self.distance, 0.01)
end

function path_align_center()
   local self = _path_align_center:create()
   self.distance = nil
   return self
end

--[[
   path component that does a vertical arc over a given angle
--]]
local _path_vertical_arc = inheritsFrom(_PathComponent, "path_vertical_arc")
function _path_vertical_arc:get_pos(t)
   local t2ang = wrap_2pi(t * math.rad(self.angle))
   return makeVector3f(math.abs(self.radius)*math.sin(t2ang), 0, -self.radius*(1.0 - math.cos(t2ang)))
end
function _path_vertical_arc:get_length()
   return math.abs(self.radius) * 2 * math.pi * math.abs(self.angle) / 360.0
end
function _path_vertical_arc:get_final_orientation()
   local q = Quaternion()
   q:from_axis_angle(makeVector3f(0,1,0), sgn(self.radius)*math.rad(wrap_180(self.angle)))
   q:normalize()
   return q
end
function path_vertical_arc(radius, angle)
   local self = _path_vertical_arc:create()
   self.radius = radius
   self.angle = angle
   return self
end

--[[
   path component that does a horizontal arc over a given angle
--]]
local _path_horizontal_arc = inheritsFrom(_PathComponent, "path_horizontal_arc")
function _path_horizontal_arc:get_pos(t)
   local t2ang = t * math.rad(self.angle)
   return makeVector3f(math.abs(self.radius)*math.sin(t2ang), self.radius*(1.0 - math.cos(t2ang)), -self.height_gain*t)
end
function _path_horizontal_arc:get_length()
   local circumference = 2 * math.pi * math.abs(self.radius)
   local full_circle_height_gain = self.height_gain * 360.0 / math.abs(self.angle)
   local helix_length = math.sqrt(full_circle_height_gain*full_circle_height_gain + circumference*circumference)
   return helix_length * math.abs(self.angle) / 360.0
end
function _path_horizontal_arc:get_final_orientation()
   local q = Quaternion()
   q:from_axis_angle(makeVector3f(0,0,1), sgn(self.radius)*math.rad(self.angle))
   return q
end

--[[
   roll correction for the rotation caused by height gain
--]]
function _path_horizontal_arc:get_roll_correction(t)
   if self.height_gain == 0 then
      return 0
   end
   local gamma=math.atan(self.height_gain*(360/self.angle)/(2*math.pi*self.radius))
   return -t*self.angle*math.sin(gamma)
end

function path_horizontal_arc(radius, angle, height_gain)
   local self = _path_horizontal_arc:create()
   self.radius = radius
   self.angle = angle
   self.height_gain = height_gain or 0
   return self
end

--[[
   path component that does a cylinder for a barrel roll
--]]
local _path_cylinder = inheritsFrom(_PathComponent, "path_cylinder")
function _path_cylinder:get_pos(t)
   local t2ang = t * self.num_spirals * math.pi * 2
   local v = makeVector3f(self.length*t, math.abs(self.radius)*math.sin(t2ang+math.pi), -self.radius*(1.0 - math.cos(t2ang)))
   local qrot = Quaternion()
   qrot:from_axis_angle(makeVector3f(0,0,1), (0.5*math.pi)-self.gamma)
   return quat_earth_to_body(qrot, v)
end

function _path_cylinder:get_length()
   local circumference = 2 * math.pi * math.abs(self.radius)
   local length_per_spiral = self.length / self.num_spirals
   local helix_length = math.sqrt(length_per_spiral*length_per_spiral + circumference*circumference)
   return helix_length * self.num_spirals
end

--[[
   roll correction for the rotation caused by the path
--]]
function _path_cylinder:get_roll_correction(t)
   return t*360*math.sin(self.gamma)*self.num_spirals
end

function path_cylinder(radius, length, num_spirals)
   local self = _path_cylinder:create()
   self.radius = radius
   self.length = length
   self.num_spirals = num_spirals
   self.gamma = math.atan((length/num_spirals)/(2*math.pi*radius))
   return self
end

--[[
   a Path has the methods of both RollComponent and
   PathComponent allowing for a complete description of a subpath
--]]
local _Path = inheritsFrom(nil)
function _Path:get_roll(t, time_s)
   return wrap_180(self.roll_component:get_roll(t, time_s))
end
function _Path:get_roll_correction(t)
   return self.path_component:get_roll_correction(t)
end
function _Path:get_pos(t)
   return self.path_component:get_pos(t)
end
function _Path:get_length()
   return self.path_component:get_length()
end
function _Path:get_final_orientation()
   return self.path_component:get_final_orientation()
end
function _Path:get_attribute(t, attrib)
   return self[attrib]
end
function _Path:set_next_extents(extents, start_pos, start_orientation)
   self.path_component:set_next_extents(extents, start_pos, start_orientation)
end
local function Path(path_component, roll_component)
   local self = _Path:create()
   self.name = string.format("%s|%s", path_component.name, roll_component.name)
   assert(path_component)
   assert(roll_component)
   self.path_component = path_component
   self.roll_component = roll_component
   return self
end


--[[
   componse multiple sub-paths together to create a full trajectory
--]]
local _path_composer = inheritsFrom(nil)
-- return the subpath with index i. Used to cope with two ways of calling path_composer
function _path_composer:subpath(i)
   if i == self.cache_i then
      return self.cache_sp
   end
   self.cache_i = i
   local sp = self.subpaths[i]
   if sp.name then
      -- we are being called with a list of Path objects
      self.cache_sp = sp
   else
      -- we are being called with a list function/argument tuples
      local args = self.subpaths[i][2]
      self.cache_sp = self.subpaths[i][1](args[1], args[2], args[3], args[4], self.start_pos[i], self.start_orientation[i])
      -- copy over path attributes
      for k, v in pairs(path_attribs) do
         self.cache_sp[v] = self.subpaths[i][v]
      end
   end
   return self.cache_sp
end

function _path_composer:end_time(i)
   local proportion = self.lengths[i] / self.total_length
   return self.start_time[i] + proportion
end

function _path_composer:get_subpath_t(t)
   if self.last_subpath_t[1] == t then
      -- use cached value
      return self.last_subpath_t[2], self.last_subpath_t[3]
   end
   local i = 1
   while t >= self:end_time(i) and i < self.num_sub_paths do
      i = i + 1
   end
   local proportion = self.lengths[i]/self.total_length
   local subpath_t = (t - self.start_time[i]) / proportion
   self.last_subpath_t = { t, subpath_t, i }
   local sp = self:subpath(i)
   if i > self.highest_i and t < 1.0 and t > 0 then
      self.highest_i = i
      if sp.message ~= nil then
         gcs:send_text(0, sp.message)
      end
      if AEROM_DEBUG:get() > 0 then
         gcs:send_text(0, string.format("starting %s[%d] %s", self.name, i, sp.name))
      end
   end
   return subpath_t, i
end

-- return position at time t
function _path_composer:get_pos(t)
   local subpath_t, i = self:get_subpath_t(t)
   local sp = self:subpath(i)
   return quat_earth_to_body(self.start_orientation[i], sp:get_pos(subpath_t)) + self.start_pos[i]
end

-- return angle for the composed path at time t
function _path_composer:get_roll(t, time_s)
   local subpath_t, i = self:get_subpath_t(t)
   local speed = target_groundspeed()
   local sp = self:subpath(i)
   local angle = sp:get_roll(subpath_t, self.lengths[i]/speed)
   return angle + self.start_angle[i]
end

function _path_composer:get_roll_correction(t)
   local subpath_t, i = self:get_subpath_t(t)
   local sp = self:subpath(i)
   return sp:get_roll_correction(subpath_t) + (self.start_roll_correction[i] or 0)
end
   
function _path_composer:get_length()
   return self.total_length
end

function _path_composer:get_final_orientation()
   return self.final_orientation
end

function _path_composer:get_attribute(t, attrib)
   local subpath_t, i = self:get_subpath_t(t)
   local sp = self:subpath(i)
   return sp[attrib] or sp:get_attribute(subpath_t, attrib)
end

function _path_composer:get_extents_x()
   if self.extents ~= nil then
      return self.extents
   end
   self.extents = get_extents_x(self)
   return self.extents
end

--[[
   get the time that the next segment starts
--]]
function _path_composer:get_next_segment_start(t)
   local subpath_t, i = self:get_subpath_t(t)
   local sp = self:subpath(i)
   if sp.get_next_segment_start ~= nil then
      return self.start_time[i] + (sp:get_next_segment_start(subpath_t) * (self:end_time(i) - self.start_time[i]))
   end
   return self:end_time(i)
end

local function path_composer(name, subpaths)
   local self = _path_composer:create()
   self.name = name
   self.subpaths = subpaths
   self.lengths = {}
   self.start_time = {}
   self.start_orientation = {}
   self.start_pos = {}
   self.start_angle = {}
   self.start_roll_correction = {}
   self.total_length = 0
   self.num_sub_paths = #subpaths
   self.last_subpath_t = { -1, 0, 0 }
   self.highest_i = 0

   local orientation = Quaternion()
   local pos = makeVector3f(0,0,0)
   local angle = 0
   local roll_correction = 0
   local speed = target_groundspeed()
   local cache_i = -1
   local cache_sp = nil

   for i = 1, self.num_sub_paths do
      -- accumulate orientation, position and angle
      self.start_orientation[i] = quat_copy(orientation)
      self.start_pos[i] = pos:copy()
      self.start_angle[i] = angle
      if roll_correction ~= 0 then
         self.start_roll_correction[i] = roll_correction
      end

      local sp = self:subpath(i)

      self.lengths[i] = sp:get_length()
      if self.lengths[i] == nil and i < self.num_sub_paths then
         local sp2 = self:subpath(i+1)
         local next_extents = sp2:get_extents_x()
         if next_extents ~= nil then
            sp:set_next_extents(next_extents, self.start_pos[i], self.start_orientation[i])
            self.lengths[i] = sp:get_length()
            -- solidify this subpath now that it has its length calculated
            self.subpaths[i] = sp
         end
      end

      self.total_length = self.total_length + self.lengths[i]

      local spos = quat_earth_to_body(orientation, sp:get_pos(1.0))

      pos = pos + spos
      orientation = sp:get_final_orientation() * orientation
      orientation:normalize()

      angle = angle + sp:get_roll(1.0, self.lengths[i]/speed)
      roll_correction = roll_correction + sp:get_roll_correction(1.0)

      if sp.set_orient ~= nil then
         -- override orientation at this point in the sequence
         orientation = sp.set_orient
      end

      if sp.roll_ref ~= nil then
         local q = Quaternion()
         q:from_axis_angle(makeVector3f(1,0,0), math.rad(sp.roll_ref))
         orientation = orientation * q
         orientation:normalize()
      end
   end

   -- get our final orientation, including roll
   self.final_orientation = quat_copy(orientation)
   local q = Quaternion()
   q:from_axis_angle(makeVector3f(1,0,0), math.rad(wrap_180(angle)))
   self.final_orientation = q * self.final_orientation

   -- work out the proportion of the total time we will spend in each sub path
   local total_time = 0
   for i = 1, self.num_sub_paths do
      self.start_time[i] = total_time
      local proportion = self.lengths[i]/self.total_length
      total_time = total_time + proportion
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
      -- copy over path attributes
      for k, v in pairs(path_attribs) do
         if paths[i][v] ~= nil then
            p[i][v] = paths[i][v]
         end
      end
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

--[[
   fly straight so that the next maneuver in the sequence ends at
   the given proportion of the aerobatic box
--]]
function align_box(alignment, arg2, arg3, arg4)
   return Path(path_align_box(alignment), roll_angle(0))
end

--[[
   fly straight so that the next maneuver in the sequence is centered
   in the aerobatic box
--]]
function align_center(arg1, arg2, arg3, arg4)
   return Path(path_align_center(), roll_angle(0))
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
   perform a rudder over maneuver
--]]
function rudder_over(_direction, _min_speed)
   local self = {}
   local direction = _direction
   local min_speed = _min_speed
   local reached_speed = false
   local kick_started = false
   local pitch2_done = false
   local descent_done = false
   local target_q = nil
   local initial_q = nil
   local last_t = nil
   local initial_z = nil

   --[[
      the update() method is called during the rudder over, it
      should return true when the maneuver is completed
   --]]
   function self.update(path, t, target_speed)
      if descent_done then
         -- we're all done
         return true
      end

      local ahrs_quat = ahrs:get_quaternion()
      local ahrs_pos = ahrs:get_relative_position_NED_origin()
      local ahrs_gyro = ahrs:get_gyro()
      local now = millis():tofloat() * 0.001
      local pitch_threshold = 60.0

      if target_q == nil then
         -- initialising
         initial_z = ahrs_pos:z()
         target_q = quat_copy(ahrs_quat)
         initial_q = quat_copy(target_q)
         last_t = now
      end

      local dt = now - last_t
      last_t = now

      local error_quat = ahrs_quat:inverse() * target_q
      local rate_rads = Vector3f()
      error_quat:to_axis_angle(rate_rads)
      local tc = ROLL_CORR_TC:get()
      local rate_dps = rate_rads:scale(math.deg(1)/tc)

      -- use user set throttle for achieving the stall
      local throttle = AEROM_STALL_THR:get()
      local pitch_deg = math.deg(ahrs:get_pitch())
      if reached_speed and not kick_started and math.abs(math.deg(ahrs_gyro:z())) > ACRO_YAW_RATE:get()/3 then
         kick_started = true
      end
      if kick_started then
         -- when we have established some yaw rate cut the throttle to minimum
         throttle = AEROM_THR_MIN:get()
      end

      vehicle:set_target_throttle_rate_rpy(throttle, rate_dps:x(), rate_dps:y(), rate_dps:z())

      log_pose('POSM', ahrs_pos, ahrs:get_quaternion())
      log_pose('POST', ahrs_pos, target_q)

      local current_speed_up = -ahrs:get_velocity_NED():z()
      if not reached_speed and current_speed_up <= min_speed then
         reached_speed = true
      end

      if not reached_speed then
         return false
      end

      -- integrate desired attitude through yaw
      local q_rate_rads = makeVector3f(0,0,ahrs_gyro:z())
      if pitch2_done then
         -- stop adding yaw
         q_rate_rads:z(0)
      end
      local rotation = Quaternion()
      rotation:from_angular_velocity(q_rate_rads, dt)
      target_q = target_q * rotation
      target_q:normalize()

      --[[
         override rudder to maximum, basing PWM on the MIN/MAX of the channel
         according to the desired direction
      --]]
      local rudd_pwm = nil
      local desired_direction = direction
      if desired_direction == 0 then
         local c_y = get_ahrs_dcm_c_y()
         if c_y > 0 then
            desired_direction = 1
         else
            desired_direction = -1
         end
      end
      if desired_direction * (RUDD_REVERSED:get()*2-1) < 0 then
         rudd_pwm = RUDD_MAX:get()
      else
         rudd_pwm = RUDD_MIN:get()
      end
      if not pitch2_done then
         SRV_Channels:set_output_pwm_chan_timeout(rudder_chan, rudd_pwm, math.floor(4*1000/LOOP_RATE))
      end

      if not kick_started then
         return false
      end

      -- see if we are nose down
      if kick_started and pitch_deg < AEROM_STALL_PIT:get() and not pitch2_done then
         -- lock onto a descent path
         pitch2_done = true
         target_q = initial_q * qorient(0, 0, 180)
         --[[ correct the attitude to the opposite correction that we
            had at the start of the slowdown, so we fight the wind on
            the way down
         --]]
         local error_q = initial_q:inverse() * qorient(0, 90, math.deg(initial_q:get_euler_yaw()))
         local error_pitch = error_q:get_euler_pitch()
         local error_yaw = error_q:get_euler_yaw()
         target_q = target_q * qorient(0, math.deg(-2*error_pitch), math.deg(2*error_yaw))
         target_q:normalize()
         return false
      end

      if not pitch2_done or ahrs_pos:z() < initial_z then
         -- haven't finished the descent
         return false
      end

      -- all done, update state
      descent_done = true
      path_var.tangent = path_var.tangent:scale(-1)
      path_var.path_t = path:get_next_segment_start(t)
      path_var.accumulated_orientation_rel_ef = path_var.accumulated_orientation_rel_ef * qorient(0,0,180)
      path_var.last_time = now
      path_var.last_ang_rate_dps = ahrs_gyro:scale(math.deg(1))
      path_var.pos = rotate_path(path, path_var.path_t, path_var.initial_ori, path_var.initial_ef_pos)
      -- ensure that the path will move fwd on the next step
      path_var.pos:z(path_var.pos:z()-10)

      -- cancel rudder override
      SRV_Channels:set_output_pwm_chan_timeout(rudder_chan, rudd_pwm, 0)

      return false
   end

   return self
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
         { path_straight(h),                       roll_angle(0) },
         { path_reverse(h/4),                      roll_angle(0), rate_override=rudder_over(direction,min_speed), set_orient=qorient(0,-90,0) },
         { path_straight(h),                       roll_angle(0), pos_corr=0.5, shift_xy=true },
         { path_vertical_arc(-radius, 90),         roll_angle(0), set_orient=qorient(0,0,180) },
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

--[[
   a multi-point roll
     - length = total length of straight flight
     - N = number of points of roll for full 360
     - hold_frac = proportion of each segment to hold attitude, will use 0.2 if 0
     - num_points = number of points of the N point roll to do, will use N if 0

   Note that num_points can be greater than N, for example do 6 points
   of a 4 point roll, resulting in inverted flight
--]]
function multi_point_roll(length, N, hold_frac, num_points)
   if hold_frac <= 0 then
      hold_frac = 0.2
   end
   if num_points <= 0 then
      num_points = N
   end
   --[[
      construct a roll sequence to use over the full length
   --]]
   local seq = {}
   local roll_frac = 1.0 - hold_frac
   for i = 1, num_points do
      seq[#seq+1] = { roll_frac, 360 / N }
      if i < num_points then
         seq[#seq+1] = { hold_frac, 0 }
      end
   end
   return make_paths("multi_point_roll", {{ path_straight(length), roll_sequence(seq) }})
end

function eight_point_roll(length, arg2, arg3, arg4)
   return multi_point_roll(length, 8, 0.5)
end

function procedure_turn(radius, bank_angle, step_out, arg4)
   local rabs = math.abs(radius)
   return make_paths("procedure_turn", {
            { path_horizontal_arc(radius,  90),     roll_angle_entry_exit(bank_angle) },
            { path_straight(step_out),              roll_angle(0) },
            { path_horizontal_arc(-radius,  270),   roll_angle_entry_exit(-bank_angle) },
            { path_straight(3*rabs),                roll_angle(0) },
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
   local point = path_f:get_pos(t)
   local angle = path_f:get_roll(t)
   local roll_correction = path_f:get_roll_correction(t)

   local attrib = {}
   for k, v in pairs(path_attribs) do
      attrib[v] = path_f:get_attribute(t, v)
   end
   point = point + path_var.path_shift
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

   return point+offset, math.rad(angle+roll_correction), attrib
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

      path_var.length = path:get_length() * math.abs(AEROM_PATH_SCALE:get())

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
      path_var.ff_yaw_rate_rads = 0.0
      path_var.last_ang_rate_dps = ahrs_gyro:scale(math.deg(1))

      path_var.path_t = 0.0

      path_var.pos = path_var.initial_ef_pos:copy()
      path_var.roll = 0.0
      path_var.last_shift_xy = nil
      path_var.path_shift = Vector3f()

      -- get initial tangent
      local p1, r1 = rotate_path(path, path_var.path_t + 0.1/(path_var.total_time*LOOP_RATE),
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

   local local_n_dt = (1.0/LOOP_RATE)/path_var.total_time

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
   local p1, r1, attrib = rotate_path(path, path_var.path_t + local_n_dt,
                                      path_var.initial_ori, path_var.initial_ef_pos)

   local current_measured_pos_ef = ahrs_pos_NED:copy()

   if attrib.rate_override ~= nil then
      if not attrib.rate_override.update(path, path_var.path_t + local_n_dt, path_var.target_speed) then
         -- not done yet
         path_var.pos = current_measured_pos_ef
         return true
      end
   end

   --[[
      see if this path element has a shift_xy attribute
   --]]
   local shift_xy = attrib.shift_xy
   if shift_xy and not path_var.last_shift_xy then
      --[[
         we have entered a new sub-element with a shift_xy
      --]]
      local curpos_mf = quat_body_to_earth(path_var.initial_ori, current_measured_pos_ef)
      local pathpos_mf = quat_body_to_earth(path_var.initial_ori, p1)
      local shift = curpos_mf - pathpos_mf
      shift:z(0)
      path_var.path_shift = path_var.path_shift + shift
      local shift_ef = quat_earth_to_body(path_var.initial_ori, shift)
      p1 = p1 + shift_ef
      p0:y(p1:y())
      p0:x(p1:x())
   end
   path_var.last_shift_xy = shift_xy

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
   local p1, r1, attrib = rotate_path(path,
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
   p1, r1, attrib = rotate_path(path,
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

   -- scale by per-maneuver error correction scale factor
   acc_err_ef = acc_err_ef:scale(attrib.pos_corr or 1.0)

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
   -- look ahead by AEROM_KE_TC seconds to get predicted attitude
   local lookahead_rotation = Quaternion()
   lookahead_rotation:from_angular_velocity(path_rate_bf_dps:scale(math.rad(1)), AEROM_KE_TC:get())
   local lkahead_q = orientation_rel_ef_with_roll_angle * lookahead_rotation
   -- get sin(roll)*cos(pitch) for scaling the KE angle
   local ke_angle = get_quat_dcm_c_y(lkahead_q)
   -- scale by square of airspeed
   local airspeed_scaling = SCALING_SPEED:get()/airspeed_constrained
   local ff_yaw_rate_rads = math.rad(AEROM_KE_ANG:get()) * (-ke_angle) * sq(airspeed_scaling)

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

   logger.write('AERT','Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz,Perr,Aerr,Yff', 'fffffffffffff',
                cor_ang_vel_bf_dps:x(), cor_ang_vel_bf_dps:y(), cor_ang_vel_bf_dps:z(),
                path_rate_bf_dps:x(), path_rate_bf_dps:y(), path_rate_bf_dps:z(),
                err_angle_rate_bf_dps:x(),
                tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z(),
                pos_error_ef:length(),
                wrap_180(math.deg(err_angle_rad)),
                math.deg(ff_yaw_rate_rads))

   --log_pose('POSB', p1, path_var.accumulated_orientation_rel_ef)

   --[[
      run the throttle based speed controller

      get the anticipated pitch at the throttle lookahead time
      we use the maximum of the current path pitch and the anticipated pitch
   --]]
   local qchange = Quaternion()
   qchange:from_angular_velocity(path_rate_ef_rads, AEROM_THR_LKAHD:get())
   local qnew = qchange * orientation_rel_ef_with_roll_angle
   local anticipated_pitch_rad = math.max(qnew:get_euler_pitch(), orientation_rel_ef_with_roll_angle:get_euler_pitch())

   local throttle = speed_PI.update(path_var.target_speed, anticipated_pitch_rad)
   local thr_min = AEROM_THR_MIN:get()
   if attrib.thr_boost then
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
command_table[23]= PathFunction(half_climbing_circle, "Half Climbing Circle")
command_table[25]= PathFunction(laydown_humpty, "Laydown Humpty")
command_table[26]= PathFunction(barrel_roll, "Barrel Roll")
command_table[27]= PathFunction(straight_flight, "Straight Hold")
command_table[28]= PathFunction(partial_circle, "Partial Circle")
command_table[31]= PathFunction(multi_point_roll, "Multi Point Roll")
command_table[32]= PathFunction(side_step, "Side Step")

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
load_table["two_point_roll"] = two_point_roll
load_table["half_climbing_circle"] = half_climbing_circle
load_table["laydown_humpty"] = laydown_humpty
load_table["straight_align"] = straight_align
load_table["figure_eight"] = figure_eight
load_table["barrel_roll"] = barrel_roll
load_table["straight_hold"] = straight_hold
load_table["partial_circle"] = partial_circle
load_table["multi_point_roll"] = multi_point_roll
load_table["side_step"] = side_step
load_table["align_box"] = align_box
load_table["align_center"] = align_center

--[[
   interpret an attribute value, coping with special cases
--]]
function interpret_attrib(v)
   if v == "true" then
      return true
   end
   if v == "false" then
      return false
   end
   -- could be a number
   local n = tonumber(v)
   if n ~= nil then
      return n
   end
   -- assume a string
   return v
end

--[[
   parse a function definition in a txt load file, adding it to the load table so
   it can be used in schedules
--]]
function parse_function(line, file)
   _, _, funcname = string.find(line, "^function%s*([%w_]+).*$")
   if not funcname then
      gcs:send_text(0, string.format("Parse error: %s", line))
      return
   end
   local funcstr = line .. "\n"
   while true do
      local line = file:read()
      if not line then
         gcs:send_text(0, string.format("Error: end of file in %s", funcname))
         break
      end
      funcstr = funcstr .. line .. "\n"
      if string.sub(line,1,3) == "end" then
         break
      end
   end
   local f, errloc, err = load(funcstr, funcname, "t", _ENV)
   if not f then
      gcs:send_text(0,string.format("Error %s: %s", errloc, err))
      return
   end
   -- fun the function code, which creates the function
   local success, err = pcall(f)
   if not success then
      gcs:send_text(0,string.format("Error %s: %s", funcname, err))
   end
   load_table[funcname] = _ENV[funcname]
end

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
   local attrib = {}
   local paths = {}
   while true do
      local line = file:read()
      if not line then
         break
      end
      -- trim trailing spaces
      line = string.gsub(line, '^(.-)%s*$', '%1')
      local _, _, cmd, arg1, arg2, arg3, arg4 = string.find(line, "^([%w_:]+)%s*([-.%d]*)%s*([-.%d]*)%s*([-.%d]*)%s*([-.%d]*)")
      if cmd == "" or cmd == nil or string.sub(cmd,1,1) == "#" then
         -- ignore comments
      elseif cmd == "name:" then
         _, _, name = string.find(line, "^name:%s*([%w_]+)$")
      elseif string.sub(cmd,-1) == ":" then
         _, _, a, s = string.find(line, "^([%w_]+):%s*([%w_:%s-]+)$")
         if a ~= nil then
            attrib[a] = interpret_attrib(s)
         else
            gcs:send_text(0, string.format("Bad line: '%s'", line))
         end
      elseif cmd == "function" then
         parse_function(line, file)
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
            for k, v in pairs(attrib) do
               paths[#paths][k] = v
            end
            attrib = {}
         end
      end
   end
   local pc = path_composer(name, paths)
   gcs:send_text(0, string.format("Loaded trick%u '%s'", id, name))
   command_table[id] = PathFunction(pc, name)
end

function PathTask(fn, name, id, initial_yaw_deg, arg1, arg2, arg3, arg4)
   local self = {}
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
      if command_table[cmd] == nil then
         gcs:send_text(0, string.format("Trick %u not found", cmd))
         return
      end
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

gcs:send_text(0, string.format("Loaded plane_aerobatics.lua"))
return update()
