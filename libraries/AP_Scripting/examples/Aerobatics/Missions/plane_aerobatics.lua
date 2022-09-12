--[[ perform simple aerobatic manoeuvres in AUTO mode
cmd = 1: axial rolls, arg1 = roll rate dps, arg2 = number of rolls
cmd = 2: loops or 180deg return, arg1 = pitch rate dps, arg2 = number of loops, if zero do a 1/2 cuban8-like return
cmd = 3: rolling circle, arg1 = earth frame yaw rate, dps, arg2 = roll rate, dps
cmd = 4: knife edge at any angle, arg1 = roll angle to hold, arg2 = duration
cmd = 5: pause, holding heading and alt to allow stabilization after a move, arg1 = duration in seconds
]]--

-- setup param block for aerobatics, reserving 30 params beginning with AERO_
local PARAM_TABLE_KEY = 70
assert(param:add_table(PARAM_TABLE_KEY, "AEROM_", 30), 'could not add param table')

-- this control script uses AERO_TRICK_ID to report the selected trick number from the scripting_rc_selection rc channel
assert(param:add_param(PARAM_TABLE_KEY, 1,  'HGT_P', 1), 'could not add param4') -- height P gain
assert(param:add_param(PARAM_TABLE_KEY, 2,  'HGT_I', 2), 'could not add param5') -- height I gain
assert(param:add_param(PARAM_TABLE_KEY, 3,  'HGT_KE_ADD', 20), 'could not add param6') --height knife-edge addition for pitch
assert(param:add_param(PARAM_TABLE_KEY, 4,  'THR_PIT_FF', 80), 'could not add param67') --throttle FF from pitch
assert(param:add_param(PARAM_TABLE_KEY, 5,  'SPD_P', 5), 'could not add param8') -- speed P gain
assert(param:add_param(PARAM_TABLE_KEY, 6,  'SPD_I', 25), 'could not add param9')  -- speed I gain

DO_JUMP = 177
k_throttle = 70

local HGT_P = Parameter("AEROM_HGT_P") -- height P gain
local HGT_I = Parameter("AEROM_HGT_I") -- height I gain
local HGT_KE_BIAS = Parameter("AEROM_HGT_KE_ADD") -- height knifeedge addition for pitch
local THR_PIT_FF = Parameter("AEROM_THR_PIT_FF") -- throttle FF from pitch
local SPD_P = Parameter("AEROM_SPD_P") -- speed P gain
local SPD_I = Parameter("AEROM_SPD_I") -- speed I gain
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

--  start of trick routines---------------------------------------------------------------------------------
function do_axial_roll(arg1, arg2)
   -- constant roll rate axial roll, arg1 roll rate, arg2 is number of rolls
   if not running then
      running = true
      repeat_count = arg2 -1
      roll_stage = 0
      height_PI.reset()
      gcs:send_text(0, string.format("Starting roll"))
   end
   local roll_rate = arg1
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if roll_stage == 0 then
      if roll_deg > 45 then
         roll_stage = 1
      end
   elseif roll_stage == 1 then
      if roll_deg > -5 and roll_deg < 5 then
         -- we're done with a roll
         gcs:send_text(0, string.format("Finished roll r=%.1f p=%.1f", roll_deg, pitch_deg))
         if repeat_count > 0 then
            roll_stage = 0
            repeat_count = repeat_count - 1
         else
         running = false
         vehicle:nav_script_time_done(last_id)
         roll_stage = 2
         return
         end
      end
   end
   if roll_stage < 2 then
      throttle = throttle_controller()
      target_pitch = height_PI.update(initial_height)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end

local loop_stage = 0
local target_vel

function do_loop(arg1, arg2)
   -- do one loop with controllable pitch rate arg1 is pitch rate, arg2 number of loops, 0 indicates 1/2 cuban8 reversal
   if not running then
      running = true
      loop_stage = 0 
      repeat_count = arg2 -1  
      target_vel = ahrs:get_velocity_NED():length()
      if arg2 ~=0 then
        gcs:send_text(0, string.format("Starting loop"))
      else
        gcs:send_text(0, string.format("Starting immelman"))
      end
   end

   local throttle = throttle_controller()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local vel = ahrs:get_velocity_NED():length()
   local pitch_rate = arg1
   local pitch_rate = pitch_rate * (1+ 2*((vel/target_vel)-1)) --increase/decrease rate based on velocity to round loop
   pitch_rate = constrain(pitch_rate,.5 * arg1, 3 * arg1)
   
   if loop_stage == 0 then
      if pitch_deg > 60 then
         loop_stage = 1
      end
   elseif loop_stage == 1 then
      if (math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5 and repeat_count >= 0) then 
         -- we're done with loop
         gcs:send_text(0, string.format("Finished loop p=%.1f", pitch_deg))
         loop_stage = 2  --now recover stage
         height_PI.reset()
      elseif (math.abs(roll_deg) > 90 and pitch_deg > -5 and pitch_deg < 5 and repeat_count < 0) then
         gcs:send_text(0, string.format("Finished immelman p=%.1f", pitch_deg)) 
         loop_stage = 2  --now recover stage
         height_PI.reset()
      end
   elseif loop_stage == 2 then
         -- recover alt if above or below start and terminate
    if math.abs(ahrs:get_position():alt()*0.01 - initial_height) > 3 then
       throttle, pitch_rate, yaw_rate = recover_alt()
    elseif repeat_count > 0 then
    loop_stage = 0
    repeat_count = repeat_count - 1
    else 
       running = false
       --gcs:send_text(0, string.format("Recovered entry alt"))
       vehicle:nav_script_time_done(last_id)
       return
    end
  end
  throttle = throttle_controller()
  if loop_stage == 2 or loop_stage == 0 then 
     level_type = 0
  else
     level_type = 1
  end
  if math.abs(pitch_deg) > 85 and  math.abs(pitch_deg) < 95 then
     roll_rate = 0
  else
     roll_rate = earth_frame_wings_level(level_type)
  end
   vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, 0)
end

local rolling_circle_stage = 0
local rolling_circle_yaw = 0
local rolling_circle_last_ms = 0

function do_rolling_circle(arg1, arg2)
   -- constant roll rate circle roll, arg1 = earth frame yaw rate, dps, positive to right, neg to left, arg2 = roll rate, dps
   if not running then
      running = true
      rolling_circle_stage = 0
      rolling_circle_yaw_deg = 0
      rolling_circle_last_ms = millis()
      height_PI.reset()
      gcs:send_text(0, string.format("Starting rolling circle"))
   end
   local yaw_rate_dps = arg1
   local roll_rate_dps = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local now_ms = millis()
   local dt = (now_ms - rolling_circle_last_ms):tofloat() * 0.001
   rolling_circle_last_ms = now_ms

   rolling_circle_yaw_deg = rolling_circle_yaw_deg + yaw_rate_dps * dt

   if rolling_circle_stage == 0 then
      if math.abs(rolling_circle_yaw_deg) > 10.0 then
         rolling_circle_stage = 1
      end
   elseif rolling_circle_stage == 1 then
      if math.abs(rolling_circle_yaw_deg) >= 360.0 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished rollcircle r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         rolling_circle_stage = 2
         return
      end
   end
   if rolling_circle_stage < 2 then
      target_pitch = height_PI.update(initial_height)
      vel = ahrs:get_velocity_NED()
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wrap_360(rolling_circle_yaw_deg+initial_yaw_deg), PITCH_TCONST:get())
      throttle = throttle_controller()
      throttle = constrain(throttle, 0, 100.0)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate_dps, pitch_rate, yaw_rate)
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
        gcs:send_text(0, string.format("%d Knife edge", arg1))
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
        vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
    else
        gcs:send_text(0, string.format("Finished Knife edge", arg1))
        vehicle:nav_script_time_done(last_id)
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
        gcs:send_text(0, string.format("%dsec Pause", arg1))
    end
    local i=0
    if (now - knife_edge_s) < arg1 then
        roll_rate = earth_frame_wings_level(0)
        target_pitch = height_PI.update(initial_height)
        pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST:get())
        throttle = throttle_controller()
        vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
    else
        vehicle:nav_script_time_done(last_id)
    return
    end
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

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
   if id then
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
      end
   else
      running = false
   end
   return update, 10
end

return update()
