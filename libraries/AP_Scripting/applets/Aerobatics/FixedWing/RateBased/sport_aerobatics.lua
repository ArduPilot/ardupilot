--[[ perform simple aerobatic manoeuvres in AUTO mode
cmd = 1: axial rolls, arg1 = roll rate dps, arg2 = number of rolls
cmd = 2: loops or 180deg return, arg1 = pitch rate dps, arg2 = number of loops, if zero do a 1/2 cuban8-like return
cmd = 3: rolling circle, arg1 = yaw rate, arg2 = roll rate
cmd = 4: knife edge at any angle, arg1 = roll angle to hold, arg2 = duration
cmd = 5: pause, holding heading and alt to allow stabilization after a move, arg1 = duration in seconds
]]--
-- luacheck: only 0

DO_JUMP = 177
k_throttle = 70
MODE_AUTO = 10
-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 72
local PARAM_TABLE_PREFIX = "AEROR_"
assert(param:add_table(PARAM_TABLE_KEY, "AEROR_", 30), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local HGT_P = bind_add_param('HGT_P',1,1) -- height P gain
local HGT_I = bind_add_param("HGT_I",2,1) -- height I gain
local HGT_KE_BIAS = bind_add_param("KE_ADD",3,20) -- height knifeedge addition for pitch
local THR_PIT_FF = bind_add_param("PIT_FF",4,80) -- throttle FF from pitch
local SPD_P = bind_add_param("SPD_P",5,5) -- speed P gain
local SPD_I = bind_add_param("SPD_I",6,25) -- speed I gain
local TRIM_THROTTLE = Parameter("TRIM_THROTTLE")
local RLL2SRV_TCONST = Parameter("RLL2SRV_TCONST")
local PITCH_TCONST = Parameter("PTCH2SRV_TCONST")

local last_roll_err = 0.0
local last_id = 0
local initial_yaw_deg = 0
local wp_yaw_deg = 0
local initial_height = 0
local repeat_count = 0
local running = false
local trick_stage = 0

local function TrickDef(id, arg1, arg2, arg3, arg4)
   local self = {}
   self.id = id
   self.args = {arg1, arg2, arg3, arg4}
   return self
end


--[[
   Aerobatic tricks on a switch support - allows for tricks to be initiated outside AUTO mode
--]]
-- 2nd param table for tricks on a switch
local PARAM_TABLE_KEY2 = 73
local PARAM_TABLE_PREFIX2 = "TRIKR"
assert(param:add_table(PARAM_TABLE_KEY2, PARAM_TABLE_PREFIX2, 63), 'could not add param table2')

-- add a parameter and bind it to a variable in table2
function bind_add_param2(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY2, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX2 .. name)
end
local TRICKS = nil
local TRIK_SEL_FN = nil
local TRIK_ACT_FN = nil
local TRIK_COUNT  = nil

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


-- setup parameters for tricks
TRIK_SEL_FN = bind_add_param2("_SEL_FN", 2, 301)
TRIK_ACT_FN = bind_add_param2("_ACT_FN", 3, 300)
TRIK_COUNT  = bind_add_param2("_COUNT",  4, 3)
TRICKS = {}
local last_trick_action_state = rc:get_aux_cached(TRIK_ACT_FN:get())


function tricks_exist()
   if rc:find_channel_for_option(TRIK_SEL_FN:get())  
            and math.floor(TRIK_COUNT:get()) then
      return true
   else
      return false
   end
end   
   
-- setup parameters for tricks
count = math.floor(TRIK_COUNT:get())
count = constrain(count,0,11)
if tricks_exist() then
   for i = 1, count do
      local k = 5*i
      local prefix = string.format("%u", i)
      TRICKS[i] = TrickDef(bind_add_param2(prefix .. "_ID",   k+0, i),
                          bind_add_param2(prefix .. "_ARG1", k+1, 30),
                           bind_add_param2(prefix .. "_ARG2", k+2, 0),
                           bind_add_param2(prefix .. "_ARG3", k+3, 0),
                           bind_add_param2(prefix .. "_ARG4", k+4, 0))
   end
   gcs:send_text(5, string.format("Enabled %u aerobatic tricks", count))   
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
local speed_PI = PI_controller(SPD_P:get(), SPD_I:get(), 100.0)

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

function set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
   -- we don't want a rudder offset, and we do want yaw rate
   vehicle:set_rudder_offset(0, true)
   vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
end

--  start of trick routines---------------------------------------------------------------------------------
function do_axial_roll(arg1, arg2)
   -- constant roll rate axial roll, arg1 roll rate, arg2 is number of rolls
   if not running then
      running = true
      roll_num = 1
      repeat_count = arg2 -1
      trick_stage = 0
      height_PI.reset()
      gcs:send_text(5, string.format("Starting %d Roll(s)", arg2))
   end
   local roll_rate = arg1
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())

   if trick_stage == 0 then
      if roll_deg > 45 then
         trick_stage = 1
      end
   elseif trick_stage == 1 then
      if roll_deg > -5 and roll_deg < 5 then
         -- we're done with a roll
         gcs:send_text(5, string.format("Finished Roll %d", roll_num))
         if repeat_count > 0 then
            trick_stage = 0
            repeat_count = repeat_count - 1
            roll_num = roll_num + 1
         else
         running = false
         trick = 0
         if vehicle:get_mode() == MODE_AUTO then
            vehicle:nav_script_time_done(last_id)
         else
            vehicle:nav_scripting_enable(255)
         end
         trick_stage = 2
         return
         end
      end
   end
   if trick_stage < 2 then
      throttle = throttle_controller()
      target_pitch = height_PI.update(initial_height)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
      set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end


local target_vel

function do_loop(arg1, arg2)
   -- do one loop with controllable pitch rate arg1 is pitch rate, arg2 number of loops, 0 or less indicates 1/2 cuban8 reversal style turnaround
   if not running then
      running = true
      trick_stage = 0 
      loop_number = 1
      repeat_count = arg2 -1  
      target_vel = ahrs:get_velocity_NED():length()
      if arg2 > 0 then
         gcs:send_text(5, string.format("Starting %d Loop(s)",arg2))
      else
        gcs:send_text(5, string.format("Starting Turnaround"))
      end
   end

   local throttle = throttle_controller()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local vel = ahrs:get_velocity_NED():length()
   local pitch_rate = arg1
   local pitch_rate = pitch_rate * (1+ 2*((vel/target_vel)-1)) --increase/decrease rate based on velocity to round loop
   pitch_rate = constrain(pitch_rate,.5 * arg1, 3 * arg1)
   
   if trick_stage == 0 then
      if pitch_deg > 60 then
         trick_stage = 1
      end
   elseif trick_stage == 1 then
      if (math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5 and repeat_count >= 0) then 
         -- we're done with loop
         gcs:send_text(5, string.format("Finished Loop %d", loop_number))
         trick_stage = 2  --now recover stage
         height_PI.reset()
      elseif (math.abs(roll_deg) > 90 and pitch_deg > -5 and pitch_deg < 5 and repeat_count < 0) then
         gcs:send_text(5, string.format("Finished Turnaround ")) 
         trick_stage = 2  --now recover stage
         height_PI.reset()
      end
   elseif trick_stage == 2 then
         -- recover alt if above or below start and terminate
    if math.abs(ahrs:get_position():alt()*0.01 - initial_height) > 3 then
       throttle, pitch_rate, yaw_rate = recover_alt()
    elseif repeat_count > 0 then
    trick_stage = 0
    repeat_count = repeat_count - 1
    loop_number = loop_number + 1
    else 
       running = false
       if vehicle:get_mode() == MODE_AUTO then
          vehicle:nav_script_time_done(last_id)
       else
          vehicle:nav_scripting_enable(255)
       end
       return
    end
  end
  throttle = throttle_controller()
  if trick_stage == 2 or trick_stage == 0 then 
     level_type = 0
  else
     level_type = 1
  end
  if math.abs(pitch_deg) > 85 and  math.abs(pitch_deg) < 95 then
     roll_rate = 0
  else
     roll_rate = earth_frame_wings_level(level_type)
  end
  set_rate_targets(throttle, roll_rate, pitch_rate, 0)
end


local circle_yaw_deg = 0
local circle_last_ms = 0

function do_rolling_circle(arg1, arg2)
   -- constant roll rate circle roll, arg1 yaw rate, positive to right, neg to left, arg2 is roll rate
   if not running then
      running = true
      trick_stage = 0
      circle_yaw_deg = 0
      circle_last_ms = millis()
      height_PI.reset()
      gcs:send_text(5, string.format("Starting Rolling Circle"))
   end
   local yaw_rate_dps = arg1
   local roll_rate_dps = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local now_ms = millis()
   local dt = (now_ms - circle_last_ms):tofloat() * 0.001
   circle_last_ms = now_ms

   circle_yaw_deg = circle_yaw_deg + yaw_rate_dps * dt

   if trick_stage == 0 then
      if math.abs(circle_yaw_deg) > 10.0 then
         trick_stage = 1
      end
   elseif trick_stage == 1 then
      if math.abs(circle_yaw_deg) >= 360.0 then
         running = false
         -- we're done
         gcs:send_text(5,"Finished Rolling Circle")
         if vehicle:get_mode() == MODE_AUTO then
            vehicle:nav_script_time_done(last_id)
         else
            vehicle:nav_scripting_enable(255)
         end
         trick_stage = 2
         return
      end
   end
   if trick_stage < 2 then
      target_pitch = height_PI.update(initial_height)
      vel = ahrs:get_velocity_NED()
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wrap_360(circle_yaw_deg+initial_yaw_deg), PITCH_TCONST:get())
      throttle = throttle_controller()
      throttle = constrain(throttle, 0, 100.0)
      set_rate_targets(throttle, roll_rate_dps, pitch_rate, yaw_rate)
   end
end

local knife_edge_ms = 0
function do_knife_edge(arg1,arg2)
  -- arg1 is angle +/-180, duration is arg2
    local now = millis():tofloat() * 0.001
    if not running then
        running = true
        height_PI.reset()
        knife_edge_s = now
        gcs:send_text(5, string.format("Starting %d Knife Edge", arg1))
    end
    local i=0
    if (now - knife_edge_s) < arg2 then
        local roll_deg = math.deg(ahrs:get_roll())
        local roll_angle_error = (arg1 - roll_deg)
        if math.abs(roll_angle_error) > 180 then
         if roll_angle_error > 0 then
           roll_angle_error = roll_angle_error - 360
         else 
           roll_angle_error= roll_angle_error +360
         end 
        end
        roll_rate = roll_angle_error/RLL2SRV_TCONST:get()
        target_pitch = height_PI.update(initial_height)
        pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
        throttle = throttle_controller()
        set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
    else
        gcs:send_text(5, string.format("Finished Knife Edge", arg1))
        running = false
        if vehicle:get_mode() == MODE_AUTO then
           vehicle:nav_script_time_done(last_id)
        else
           vehicle:nav_scripting_enable(255)
        end
    return
    end
end

-- fly level for a time..allows full altitude recovery after trick
function do_pause(arg1,arg2)
    -- arg1 is time of pause in sec, arg2 is unused
    local now = millis():tofloat() * 0.001
    if not running then
        running = true
        height_PI.reset()
        knife_edge_s = now
        gcs:send_text(5, string.format("%d sec Pause", arg1))
    end
    local i=0
    if (now - knife_edge_s) < arg1 then
        roll_rate = earth_frame_wings_level(0)
        target_pitch = height_PI.update(initial_height)
        pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
        throttle = throttle_controller()
        set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
    else 
        running = false
        gcs:send_text(5, string.format("Pause Over"))
        if vehicle:get_mode() == MODE_AUTO then
           vehicle:nav_script_time_done(last_id)
        else
           vehicle:nav_scripting_enable(255)
        end
    return
    end
end


local circle_yaw = 0
local circle_last_ms = 0

function do_knifedge_circle(arg1, arg2)
   -- constant roll angle circle , arg1 yaw rate, positive to right, neg to left, arg2 is not used
   if not running then
      running = true
      trick_stage = 0
      circle_yaw_deg = 0
      circle_last_ms = millis()
      height_PI.reset()
      gcs:send_text(5, string.format("Staring KnifeEdge Circle"))
   end
   local yaw_rate_dps = arg1
   local pitch_deg = math.deg(ahrs:get_pitch())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local now_ms = millis()
   local dt = (now_ms - circle_last_ms):tofloat() * 0.001
   circle_last_ms = now_ms
   circle_yaw_deg = circle_yaw_deg + yaw_rate_dps * dt
   if trick_stage == 0 then
      if math.abs(circle_yaw_deg) > 10.0 then
         trick_stage = 1
      end
   elseif trick_stage == 1 then
      if math.abs(circle_yaw_deg) >= 360.0 then
         running = false
         -- we're done
         gcs:send_text(5,"Finished KnifeEdge Circle")
         if vehicle:get_mode() == MODE_AUTO then
            vehicle:nav_script_time_done(last_id)
         else
            vehicle:nav_scripting_enable(255)
         end
         trick_stage = 2
         return
      end
   end
   if trick_stage < 2 then
        local roll_deg = math.deg(ahrs:get_roll())
        if arg1 >0 then
           angle = 90
        else
           angle = -90
        end
        local roll_angle_error = (angle - roll_deg)
        if math.abs(roll_angle_error) > 180 then
         if roll_angle_error > 0 then
           roll_angle_error = roll_angle_error - 360
         else 
           roll_angle_error= roll_angle_error +360
         end 
        end
      roll_rate_dps = roll_angle_error/RLL2SRV_TCONST:get()
      target_pitch = height_PI.update(initial_height)
      vel = ahrs:get_velocity_NED()
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wrap_360(circle_yaw_deg+initial_yaw_deg), PITCH_TCONST:get())
      throttle = throttle_controller()
      throttle = constrain(throttle, 0, 100.0)
      set_rate_targets(throttle, roll_rate_dps, pitch_rate, yaw_rate)
   end
end

function hold_roll_angle (angle)
   local roll_deg = math.deg(ahrs:get_roll())
   local roll_angle_error = (angle - roll_deg)
   if math.abs(roll_angle_error) > 180 then
      if roll_angle_error > 0 then
         roll_angle_error = roll_angle_error - 360
      else 
         roll_angle_error= roll_angle_error +360
      end 
   end
   return roll_angle_error/RLL2SRV_TCONST:get()
end

local roll_ms = 0
function do_4point_roll(arg1, arg2)
   -- constant roll rate axial roll with pauses at 90 deg intervals, arg1 roll rate, arg2 is pause time in sec
   arg2 = arg2 * 1000
   if not running then
      running = true
      trick_stage = 0
      height_PI.reset()
      gcs:send_text(5, string.format("Starting 4pt Roll"))
   end
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())

   if trick_stage == 0 then
      roll_rate = arg1
      if roll_deg >= 90 then
         trick_stage = 1
         roll_ms = millis()
      end
   elseif trick_stage == 1 then
      roll_rate = hold_roll_angle(90)
      if (millis() - roll_ms > arg2) then
         trick_stage = 2
      end
   elseif trick_stage == 2 then
      roll_rate = arg1
      if roll_deg >= 175 or roll_deg <= -175 then
         trick_stage = 3
         roll_ms = millis()
      end
   elseif trick_stage == 3 then
      roll_rate = hold_roll_angle(180)
      if (millis() - roll_ms >arg2) then
         trick_stage = 4
      end
   elseif trick_stage == 4 then
      roll_rate = arg1
      if roll_deg >=-90 and roll_deg < 175 then
         trick_stage = 5
         roll_ms = millis()
      end
   elseif trick_stage == 5 then   
      roll_rate = hold_roll_angle(-90)
      if (millis() - roll_ms >arg2) then
         trick_stage = 6
      end
   elseif trick_stage == 6 then
      roll_rate = arg1 
      if roll_deg > -5 and roll_deg < 5 then
         -- we're done with a roll
         trick_stage = 7
         gcs:send_text(5, "Finished 4 pt Roll")
         running = false
         if vehicle:get_mode() == MODE_AUTO then
            vehicle:nav_script_time_done(last_id)
         else
            vehicle:nav_scripting_enable(255)
         end
      end
   end
   if trick_stage < 7 then
      throttle = throttle_controller()
      target_pitch = height_PI.update(initial_height)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
      set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end

function do_split_s(arg1, arg2)
   -- roll inverted at arg2 rate, then 1/2 loop at arg1 pitch rate
   if not running then
      running = true
      trick_stage = 0   --roll inverted, pitch level
      height_PI.reset()
      gcs:send_text(5, string.format("Starting Split-S"))
   end
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if trick_stage == 0 then 
      roll_rate = arg2
      wp_yaw_deg = math.deg(ahrs:get_yaw())
      target_pitch = height_PI.update(initial_height)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())   
      if roll_deg >=175 or roll_deg <= -175 then
         trick_stage = 1 --start the pitch rate, wings level
      end
   elseif trick_stage == 1 then
      if math.abs(pitch_deg) > 85 and  math.abs(pitch_deg) < 95 then
         roll_rate = 0
      else
         roll_rate = earth_frame_wings_level(1)
      end
      pitch_rate = arg1
      if (math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5) then
          trick_stage = 2 --trick over
          gcs:send_text(5, string.format("Finished Split-S ")) 
          height_PI.reset()
          running = false
          if vehicle:get_mode() == MODE_AUTO then
             vehicle:nav_script_time_done(last_id)
          else
             vehicle:nav_scripting_enable(255)
          end
      end
   end 
   throttle = throttle_controller() 
   set_rate_targets(throttle, roll_rate, pitch_rate, 0)
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

-- see if an auto mission item needs to be run
function check_auto_mission()
   id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
   if id then        -- we are running a scripting mission 
      if id ~= last_id then
         -- we've started a new command
         running = false
         last_id = id
         repeat_count = 0
         initial_yaw_deg = math.deg(ahrs:get_yaw())
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
      end
   do_trick(cmd,arg1,arg2)
   end
end
  
local trick_sel_chan = nil
local last_trick_selection = 0

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

--[[ trick name table
--]]
 local name = {}
 name[1] = "Roll(s)"
 name[2] = "Loop(s)/Turnaround"
 name[3] = "Rolling Circle"
 name[4] = "Straight Hold"
 name[5] = "Pause"
 name[6] = "Knife Edge Circle"
 name[7] = "4pt Roll"
 name[8] = "Split-S"
--[[
   check for running a trick
--]]

function check_trick()
   local selection = get_trick_selection()
   local action = rc:get_aux_cached(TRIK_ACT_FN:get())
   local id =TRICKS[selection].id:get()
   
   if action == 0 and running then
      gcs:send_text(0,string.format("Trick aborted"))
      running = false
      last_trick_selection = 0
      -- use invalid mode to disable script control
      vehicle:nav_scripting_enable(255)
      return 0
   end
   if selection == 0 then
      return 0
   end
   if action == 1 and selection ~= last_trick_selection then
         gcs:send_text(5, string.format("Trick %u selected (%s)", id, name[id]))
         last_trick_action_state = action
         last_trick_selection = selection
         return 0
   end
   if running then
      -- let the task finish
      return selection
   end
   -- trick action changed state
   if action ~= last_trick_action_state then
      last_trick_selection = selection
      last_trick_action_state = action
      if selection == 0 then
         gcs:send_text(0, string.format("No trick selected"))
         return 0
      end
      local id = TRICKS[selection].id:get()    
      if action == 1 then
         gcs:send_text(5, string.format("Trick %u selected (%s)", id, name[id]))
         return 0
      end
      -- action changed to execute
      if action == 2 and vehicle:get_likely_flying() then
         last_trick_selection = selection
         local current_mode = vehicle:get_mode()
         if not vehicle:nav_scripting_enable(current_mode) then
            gcs:send_text(0, string.format("Tricks not available in this mode"))
            return 0
         end
         wp_yaw_deg = math.deg(ahrs:get_yaw())
         initial_height = ahrs:get_position():alt()*0.01
         return selection     
      end
   end
   return 0
end

function do_trick(cmd,arg1,arg2)
      if cmd == 1 then
         do_axial_roll(arg1, arg2)
      elseif cmd == 2 then
         do_loop(arg1, arg2)
      elseif cmd == 3 then
         do_rolling_circle(arg1, arg2)
      elseif cmd ==4 then
         do_knife_edge(arg1,arg2)
      elseif cmd == 5 then
         do_pause(arg1,arg2)
      elseif cmd == 6 then
         do_knifedge_circle(arg1,arg2)
      elseif cmd == 7 then
         do_4point_roll(arg1,arg2)
      elseif cmd == 8 then
         do_split_s(arg1,arg2)
      end
end

function update()
   if ahrs:get_velocity_NED() == nil  or ahrs:get_EAS2TAS() == nil or ahrs:get_relative_position_NED_origin() == nil then
      -- don't start till we have valid ahrs estimates
      return update, 10
   end
   if vehicle:get_mode() == MODE_AUTO then
      check_auto_mission() --run a trick mission item
   elseif tricks_exist() then
      trick = check_trick() 
      if trick ~=0 then 
      id = TRICKS[trick].id:get()
      arg1 = TRICKS[trick].args[1]:get()
      arg2 = TRICKS[trick].args[2]:get()
      do_trick(id,arg1,arg2)
      end
   end
  return update, 10
end

gcs:send_text(5, string.format("Loaded sport_aerobatics.lua"))
last_trick_selection = get_trick_selection() --prevents immediate activation on boot or restart
return update()
