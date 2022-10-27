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

HGT_P = bind_add_param('HGT_P', 1, 1)
HGT_I = bind_add_param('HGT_I', 2, 2)
HGT_KE_BIAS = bind_add_param('HGT_KE_ADD', 3, 20) -- unused
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
AEROM_ENTRY_RATE = bind_add_param('ENTRY_RATE', 14, 60)
AEROM_THR_LKAHD = bind_add_param('THR_LKAHD', 15, 1)

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

local TRIM_ARSPD_CM = Parameter("TRIM_ARSPD_CM")

local last_id = 0
local current_task = nil

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

local function speed_controller(kP_param,kI_param, kFF_pitch_param, Imax)
   local self = {}
   local kFF_pitch = kFF_pitch_param
   local PI = PI_controller(kP_param:get(), kI_param:get(), Imax)

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

local speed_PI = speed_controller(SPD_P, SPD_I, THR_PIT_FF, 100.0)

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

--------Trajectory definitions---------------------

function climbing_circle(t, radius, height, arg3, arg4)
   local angle = t*math.pi*2
   local vec = makeVector3f(radius*math.sin(angle), radius*(1.0-math.cos(angle)), -math.sin(0.5*angle)*height)
   return vec, 0.0
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

--[[
  roll component that goes through a fixed total angle at a fixed roll rate
--]]
function roll_angle(_angle)
   local self = {}
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
   local self = {}
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
      if t < 1.0 - entry_t then
         return angle
      end
      return (1.0 - ((t - (1.0 - entry_t)) / entry_t)) * angle
   end
   return self
end

--[[
   path component that does a straight horizontal line
--]]
function path_straight(_distance)
   local self = {}
   local distance = _distance
   function self.get_pos(t)
      return makeVector3f(distance*t, 0, 0)
   end
   function self.get_length()
      return distance
   end
   function self.get_final_orientation()
      return Quaternion()
   end
   return self
end

--[[
   path component that does a vertical arc over a given angle
--]]
function path_vertical_arc(_radius, _angle)
   local self = {}
   local radius = _radius
   local angle = _angle
   function self.get_pos(t)
      local t2ang = t * math.rad(angle)
      return makeVector3f(math.abs(radius)*math.sin(t2ang), 0, -radius*(1.0 - math.cos(t2ang)))
   end
   function self.get_length()
      return math.abs(radius) * 2 * math.pi * math.abs(angle) / 360.0
   end
   function self.get_final_orientation()
      local q = Quaternion()
      q:from_axis_angle(makeVector3f(0,1,0), sgn(radius)*math.rad(angle))
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
   local self = {}
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
      if _height_gain == 0 then
         return math.abs(radius) * 2 * math.pi * angle / 360.0
      end
      -- otherwise integrate to get path length
      return integrate_length(self.get_pos)
   end
   function self.get_final_orientation()
      local q = Quaternion()
      q:from_axis_angle(makeVector3f(0,0,1), sgn(radius)*math.rad(angle))
      return q
   end
   return self
end

--Wrapper to construct a Vector3f{x, y, z} from (x, y, z)
function makeVector3f(x, y, z)
   local vec = Vector3f()
   vec:x(x)
   vec:y(y)
   vec:z(z)
   return vec
end


--[[
   componse multuple sub-paths together
--]]
function path_composer(_subpaths)
   local self = {}
   local subpaths = _subpaths
   local lengths = {}
   local proportions = {}
   local start_time = {}
   local end_time = {}
   local start_orientation = {}
   local start_pos = {}
   local start_angle = {}
   local end_speed = {}
   local start_speed = {}
   local total_length = 0
   local num_sub_paths = #subpaths

   local orientation = Quaternion()
   local pos = makeVector3f(0,0,0)
   local angle = 0
   local speed = path_var.target_speed

   for i = 1, num_sub_paths do
      lengths[i] = subpaths[i][1].get_length()
      total_length = total_length + lengths[i]

      -- accumulate orientation, position and angle
      start_orientation[i] = orientation
      start_pos[i] = pos
      start_angle[i] = angle
      local spos = subpaths[i][1].get_pos(1.0)
      orientation:earth_to_body(spos)
      pos = pos + spos
      orientation = orientation * subpaths[i][1].get_final_orientation()
      angle = angle + subpaths[i][2].get_roll(1.0, lengths[i]/speed)

      start_speed[i] = speed
      end_speed[i] = subpaths[i][3]
      if end_speed[i] == nil then
         end_speed[i] = path_var.target_speed
      end
      speed = end_speed[i]
   end

   -- work out the proportion of the total time we will spend in each sub path
   local total_time = 0
   for i = 1, num_sub_paths do
      proportions[i] = lengths[i] / total_length
      start_time[i] = total_time
      end_time[i] = total_time + proportions[i]
      total_time = total_time + proportions[i]
   end

   -- return position and angle for the composed path at time t
   function self.run(t)
      -- work out which subpath we are in
      local i = 1
      while t >= end_time[i] and i < num_sub_paths do
         i = i + 1
      end
      local subpath_t = (t - start_time[i]) / proportions[i]
      local speed = start_speed[i] + subpath_t * (end_speed[i] - start_speed[i])
      pos   = subpaths[i][1].get_pos(subpath_t)
      angle = subpaths[i][2].get_roll(subpath_t, lengths[i]/speed)
      start_orientation[i]:earth_to_body(pos)
      return pos + start_pos[i], math.rad(angle + start_angle[i]), speed
   end

   return self
end

function immelmann_turn(t, r, roll_rate, arg3, arg4)
   if t == 0 then
      local speed = path_var.target_speed
      path_var.composer = path_composer({
            { path_vertical_arc(r, 180),            roll_angle(0) },
            { path_straight(speed*180.0/roll_rate), roll_angle(180) },
      })
   end
   return path_var.composer.run(t)
end

function humpty_bump(t, r, h, arg3, arg4)
   assert(h >= 2*r)
   if t == 0 then
      path_var.composer = path_composer({
            { path_vertical_arc(r, 90),          roll_angle(0) },
            { path_straight((h-2*r)/3),          roll_angle(0) },
            { path_straight((h-2*r)/3),          roll_angle(180) },
            { path_straight((h-2*r)/3),          roll_angle(0) },
            { path_vertical_arc(-r, 180),        roll_angle(0) },
            { path_straight(h-2*r),              roll_angle(0) },
            { path_vertical_arc(-r, 90),         roll_angle(0) },
            { path_straight(2*r),                roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function split_s(t, r, roll_rate, arg3, arg4)
   if t == 0 then
      local speed = path_var.target_speed
      path_var.composer = path_composer({
            { path_straight(speed*180.0/roll_rate), roll_angle(180) },
            { path_vertical_arc(-r, 180),           roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function upline_45(t, r, height_gain, arg3, arg4)
   if t == 0 then
      local h = height_gain - 2*r*math.sin(math.rad(45))
      assert(h >= 0)
      path_var.composer = path_composer({
            { path_vertical_arc(r, 45),  roll_angle(0) },
            { path_straight(h),          roll_angle(0) },
            { path_vertical_arc(-r, 45), roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function downline_45(t, r, height_loss, arg3, arg4)
   if t == 0 then
      local h = height_loss - 2*r*math.sin(math.rad(45))
      assert(h >= 0)
      path_var.composer = path_composer({
            { path_vertical_arc(-r, 45),  roll_angle(0) },
            { path_straight(h),           roll_angle(0) },
            { path_vertical_arc(r, 45),   roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function rolling_circle(t, radius, num_rolls, arg3, arg4)
   if t == 0 then
      path_var.composer = path_composer({
            { path_horizontal_arc(radius, 360), roll_angle(360*num_rolls) },
      })
   end
   return path_var.composer.run(t)
end

function straight_flight(t, length, bank_angle, arg3, arg4)
   if t == 0 then
      local entry_s = bank_angle / AEROM_ENTRY_RATE:get()
      local entry_dist = path_var.target_speed * entry_s
      if length < 2*entry_dist then
         entry_dist = length*0.5
      end
      path_var.composer = path_composer({
            { path_straight(entry_dist),            roll_angle(bank_angle) },
            { path_straight(length - 2*entry_dist), roll_angle(0) },
            { path_straight(entry_dist),            roll_angle(-bank_angle) },
      })
   end
   return path_var.composer.run(t)
end

function banked_circle(t, radius, bank_angle, height, arg4)
   if t == 0 then
      local circumference = 2 * radius * math.pi
      local circle_s = circumference / path_var.target_speed
      local entry_s = bank_angle / AEROM_ENTRY_RATE:get()
      local entry_angle = (entry_s / circle_s) * 360.0
      if entry_angle > 180 then
         entry_angle = 180
      end
      path_var.composer = path_composer({
            { path_horizontal_arc(radius, 360, height), roll_angle_entry_exit(bank_angle) },
      })
   end
   return path_var.composer.run(t)
end

function scale_figure_eight(t, r, bank_angle, arg3, arg4)
   if t == 0 then
      path_var.composer = path_composer({
            { path_straight(r),             roll_angle(0) },
            { path_horizontal_arc(r,  90),  roll_angle_entry_exit(bank_angle) },
            { path_horizontal_arc(-r, 360), roll_angle_entry_exit(-bank_angle) },
            { path_horizontal_arc(r,  270), roll_angle_entry_exit(bank_angle) },
            { path_straight(3*r),           roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function figure_eight(t, r, bank_angle, arg3, arg4)
   if t == 0 then
      local rabs = math.abs(r)
      path_var.composer = path_composer({
            { path_straight(rabs*math.sqrt(2)), roll_angle(0) },
            { path_horizontal_arc(r,  225),    roll_angle_entry_exit(bank_angle) },
            { path_straight(2*rabs),           roll_angle(0) },
            { path_horizontal_arc(-r,  270),   roll_angle_entry_exit(-bank_angle) },
            { path_straight(2*rabs),           roll_angle(0) },
            { path_horizontal_arc(r,    45),   roll_angle_entry_exit(bank_angle) },
      })
   end
   return path_var.composer.run(t)
end


--[[
   stall turn is not really correct, as we don't fully stall. Needs to be
   reworked
--]]
function stall_turn(t, radius, height, direction, min_speed)
   if t == 0 then
      local h = height - radius
      assert(h >= 0)
      path_var.composer = path_composer({
            { path_vertical_arc(radius, 90),          roll_angle(0) },
            { path_straight(h),                       roll_angle(0), min_speed },
            { path_horizontal_arc(5*direction, 180),  roll_angle(0), min_speed },
            { path_straight(h),                       roll_angle(0) },
            { path_vertical_arc(radius, 90),          roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function half_cuban_eight(t, r, arg2, arg3, arg4)
   if t == 0 then
      local rabs = math.abs(r)
      path_var.composer = path_composer({
            { path_straight(2*rabs*math.sqrt(2)), roll_angle(0) },
            { path_vertical_arc(r,  225),  roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(180) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_vertical_arc(-r, 45),   roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function cuban_eight(t, r, arg2, arg3, arg4)
   if t == 0 then
      local rabs = math.abs(r)
      path_var.composer = path_composer({
            { path_straight(rabs*math.sqrt(2)), roll_angle(0) },
            { path_vertical_arc(r,  225),  roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(180) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_vertical_arc(-r, 270),  roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(180) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_vertical_arc(r, 45),    roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function half_reverse_cuban_eight(t, r, arg2, arg3, arg4)
   if t == 0 then
      local rabs = math.abs(r)
      path_var.composer = path_composer({
            { path_vertical_arc(r,  45),   roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_straight(2*rabs/3),     roll_angle(180) },
            { path_straight(2*rabs/3),     roll_angle(0) },
            { path_vertical_arc(-r, 225),  roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function horizontal_rectangle(t, total_length, total_width, r, bank_angle)
   if t == 0 then
      local l = total_length - 2*r
      local w = total_width - 2*r
      
      path_var.composer = path_composer({
            { path_straight(0.5*l),     roll_angle(0) },
            { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle)},
            { path_straight(w),     roll_angle(0) },
            { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
            { path_straight(l),     roll_angle(0) },
            { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
            { path_straight(w),     roll_angle(0) },
            { path_horizontal_arc(r, 90),  roll_angle_entry_exit(bank_angle) },
            { path_straight(0.5*l),     roll_angle(0) },
      })
   end
   return path_var.composer.run(t)
end

function vertical_aerobatic_box(t, total_length, total_width, r, bank_angle)
   if t == 0 then
      local l = total_length - 2*r
      local w = total_width - 2*r
      
      --TODO: this doesn't seem to work for roll_angle_entry_exit?
      path_var.composer = path_composer({
            { path_straight(0.5*l),     roll_angle(0) },
            { path_vertical_arc(r, 90),   roll_angle(0)},
            { path_straight(w),     roll_angle(0) },
            { path_vertical_arc(r, 90),   roll_angle(0) },
            { path_straight(l),     roll_angle(0) },
            { path_vertical_arc(r, 90),   roll_angle(0)},
            { path_straight(w),     roll_angle(0) },
            { path_vertical_arc(r, 90),   roll_angle(0)},
            { path_straight(0.5*l),     roll_angle(0) },
      })
      
   end
   return path_var.composer.run(t)
end

---------------------------------------------------

--[[
   target speed is taken as max of target airspeed and current 3D
   velocity at the start of the maneuver
--]]
function target_groundspeed()
   return math.max(ahrs:get_EAS2TAS()*TRIM_ARSPD_CM:get()*0.01, ahrs:get_velocity_NED():length())
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
--returns: requested position, angle and speed in maneuver frame
function rotate_path(path_f, t, arg1, arg2, arg3, arg4, orientation, offset)
   t = constrain(t, 0, 1)
   point, angle, speed = path_f(t, arg1, arg2, arg3, arg4)
   orientation:earth_to_body(point)
   --TODO: rotate angle?
   return point+offset, angle, speed
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

path_var.count = 0
path_var.initial_ori = Quaternion()
path_var.initial_maneuver_to_earth = Quaternion()

function do_path()
   local now = millis():tofloat() * 0.001

   path_var.count = path_var.count + 1
   local target_dt = 1.0/LOOP_RATE
   local path = current_task.fn
   local arg1 = current_task.args[1]
   local arg2 = current_task.args[2]
   local arg3 = current_task.args[3]
   local arg4 = current_task.args[4]

   if not current_task.started then
      local initial_yaw_deg = current_task.initial_yaw_deg
      current_task.started = true

      local speed = target_groundspeed()
      path_var.target_speed = speed

      path_var.length = path_length(path, arg1, arg2, arg3, arg4)

      path_var.total_rate_rads_ef = makeVector3f(0.0, 0.0, 0.0)

      --assuming constant velocity
      path_var.total_time = path_var.length/speed
      path_var.last_pos, last_angle = path(0.0, arg1, arg2, arg3, arg4) --position at t0

      --deliberately only want yaw component, because the maneuver should be performed relative to the earth, not relative to the initial orientation
      path_var.initial_ori:from_euler(0, 0, math.rad(initial_yaw_deg))

      path_var.initial_maneuver_to_earth:from_euler(0, 0, -math.rad(initial_yaw_deg))
      
      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()


      local corrected_position_t0_ef, angle_t0, s0 = rotate_path(path, target_dt/path_var.total_time,
                                                                 arg1, arg2, arg3, arg4,
                                                                 path_var.initial_ori, path_var.initial_ef_pos)
      local corrected_position_t1_ef, angle_t1, s0 = rotate_path(path, 2*target_dt/path_var.total_time,
                                                                 arg1, arg2, arg3, arg4,
                                                                 path_var.initial_ori, path_var.initial_ef_pos)

      path_var.start_pos = ahrs:get_position()
      path_var.path_int = path_var.start_pos:copy()

      speed_PI.reset()


      path_var.accumulated_orientation_rel_ef = path_var.initial_ori

      path_var.time_correction = 0.0 

      path_var.filtered_angular_velocity = Vector3f()

      path_var.start_time = now + target_dt
      path_var.last_time = now
      path_var.average_dt = target_dt
      path_var.scaled_dt = target_dt

      path_var.path_t = 0
      return true
   end
   

   local vel_length = ahrs:get_velocity_NED():length()

   local actual_dt = now - path_var.last_time

   local local_n_dt = actual_dt/path_var.total_time

   path_var.last_time = now

   if path_var.path_t + 2*local_n_dt > 1.0 then
      return false
   end

   --[[
      calculate positions and angles at previous, current and next time steps
   --]]

   next_target_pos_ef = next_target_pos_ef
   local p0, r0, s0 = rotate_path(path, path_var.path_t + 0*local_n_dt,
                              arg1, arg2, arg3, arg4,
                              path_var.initial_ori, path_var.initial_ef_pos)
   local p1, r1, s1 = rotate_path(path, path_var.path_t + 1*local_n_dt,
                              arg1, arg2, arg3, arg4,
                              path_var.initial_ori, path_var.initial_ef_pos)
   local p2, r2, s2 = rotate_path(path, path_var.path_t + 2*local_n_dt,
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
   p2, r2, s2 = rotate_path(path,
                            constrain(path_var.path_t + path_t_delta, 0, 1),
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

   throttle = speed_PI.update(s1, anticipated_pitch_rad)
   throttle = constrain(throttle, 0, 100.0)

   if isNaN(throttle) or isNaN(tot_ang_vel_bf_dps:x()) or isNaN(tot_ang_vel_bf_dps:y()) or isNaN(tot_ang_vel_bf_dps:z()) then
      gcs:send_text(0,string.format("Path NaN - aborting"))
      return false
   end

   vehicle:set_target_throttle_rate_rpy(throttle, tot_ang_vel_bf_dps:x(), tot_ang_vel_bf_dps:y(), tot_ang_vel_bf_dps:z())
   
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

command_table = {}
command_table[1] = PathFunction(figure_eight, "Figure Eight")
command_table[2] = PathFunction(loop, "Loop")
command_table[3] = PathFunction(horizontal_rectangle, "Horizontal Rectangle")
command_table[4] = PathFunction(climbing_circle, "Climbing Circle")
command_table[5] = PathFunction(vertical_aerobatic_box, "Vertical Box")
command_table[6] = PathFunction(banked_circle, "Banked Circle")
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
-- command_table[100] = PathFunction(clubman_schedule, "Clubman Schedule")

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

function PathTask(fn, name, id, initial_yaw_deg, arg1, arg2, arg3, arg4)
   self = {}
   self.fn = fn
   self.name = name
   self.id = id
   self.initial_yaw_deg = initial_yaw_deg
   self.args = { arg1, arg2, arg3, arg4 }
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
      local initial_yaw_deg = math.deg(ahrs:get_yaw())
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
         local initial_yaw_deg = math.deg(ahrs:get_yaw())
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

   -- check if we should create a mission
   if AUTO_MIS:get() > 0 then
      create_auto_mission()
      AUTO_MIS:set_and_save(0)
   end

   check_auto_mission()
   if TRICKS ~= nil and vehicle:get_mode() ~= MODE_AUTO then
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
