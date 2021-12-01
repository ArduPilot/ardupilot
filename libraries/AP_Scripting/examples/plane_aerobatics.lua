-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

local ROLL_TCONST = param:get('RLL2SRV_TCONST') * 0.5
local PITCH_TCONST = param:get('PTCH2SRV_TCONST') * 0.5
local TRIM_THROTTLE = param:get('TRIM_THROTTLE') * 1.0

DO_JUMP = 177
NAV_WAYPOINT = 16

local scr_user1_param = Parameter()
local scr_user2_param = Parameter()
local scr_user3_param = Parameter()
local scr_user4_param = Parameter()
assert(scr_user1_param:init('SCR_USER1'), 'could not find SCR_USER1 parameter')
assert(scr_user2_param:init('SCR_USER2'), 'could not find SCR_USER2 parameter')
assert(scr_user3_param:init('SCR_USER3'), 'could not find SCR_USER3 parameter')
assert(scr_user4_param:init('SCR_USER4'), 'could not find SCR_USER4 parameter')

local last_roll_err = 0.0
local last_id = 0
local initial_yaw_deg = 0
local wp_yaw_deg = 0

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

-- a controller to target a zero roll angle, coping with inverted flight
-- output is a body frame roll rate, with convergence over time tconst in seconds
function roll_zero_controller(tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_err = 0.0
   if math.abs(pitch_deg) > 85 then
      -- close to 90 we retain the last roll rate
      roll_err = last_roll_err
   elseif roll_deg > 90 then
      roll_err = 180 - roll_deg
   elseif roll_deg < -90 then
      roll_err = (-180) - roll_deg
   else
      roll_err = -roll_deg
   end
   last_roll_err = roll_err
   return roll_err / tconst
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
function throttle_controller(tconst)
   local pitch_rad = ahrs:get_pitch()
   local thr_ff = scr_user3_param:get()
   local throttle = TRIM_THROTTLE + math.sin(pitch_rad) * thr_ff
   return constrain(throttle, 0.0, 100.0)
end

function do_axial_roll(arg1, arg2)
   -- constant roll rate axial roll
   if not running then
      running = true
      roll_stage = 0
      gcs:send_text(0, string.format("Starting roll"))
   end
   local roll_rate = arg1
   local throttle = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if roll_stage == 0 then
      if roll_deg > 45 then
         roll_stage = 1
      end
   elseif roll_stage == 1 then
      if roll_deg > -5 and roll_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished roll r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         roll_stage = 2
         return
      end
   end
   if roll_stage < 2 then
      target_pitch = scr_user2_param:get()
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end

local loop_stage = 0

function do_loop(arg1, arg2)
   -- do one loop with controllable pitch rate and throttle
   if not running then
      running = true
      loop_stage = 0
      gcs:send_text(0, string.format("Starting loop"))
   end
   local pitch_rate = arg1
   local throttle = throttle_controller()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if loop_stage == 0 then
      if pitch_deg > 60 then
         loop_stage = 1
      end
   elseif loop_stage == 1 then
      if math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished loop p=%.1f", pitch_deg))
         vehicle:nav_script_time_done(last_id)
         loop_stage = 2
         return
      end
   end
   if loop_stage < 2 then
      local roll_rate = roll_zero_controller(ROLL_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, 0)
   end
end

local rolling_circle_stage = 0
local rolling_circle_yaw = 0
local rolling_circle_last_ms = 0

function do_rolling_circle(arg1, arg2)
   -- constant roll rate circle roll
   if not running then
      running = true
      rolling_circle_stage = 0
      rolling_circle_yaw = initial_yaw_deg
      rolling_circle_last_ms = millis()
      gcs:send_text(0, string.format("Starting rolling circle"))
   end
   local roll_rate = arg1
   local throttle = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local gspeed = ahrs:groundspeed_vector():length()
   local radius = scr_user4_param:get()
   if radius < 1 then
      radius = 50.0
   end
   local circumference = math.pi * 2.0 * radius
   local circle_time = circumference / gspeed
   local yaw_rate_dps = 360.0 / circle_time
   local now_ms = millis()
   local dt = (now_ms - rolling_circle_last_ms):tofloat() * 0.001
   rolling_circle_last_ms = now_ms

   rolling_circle_yaw = rolling_circle_yaw + yaw_rate_dps * dt
   if rolling_circle_stage == 0 then
      if wrap_180(yaw_deg - initial_yaw_deg) > 45 then
         rolling_circle_stage = 1
      end
   elseif rolling_circle_stage == 1 then
      if math.abs(wrap_180(yaw_deg - initial_yaw_deg)) < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished rollcircle r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         rolling_circle_stage = 2
         return
      end
   end
   if rolling_circle_stage < 2 then
      target_pitch = scr_user2_param:get()
      pitch_rate, yaw_rate = pitch_controller(target_pitch, rolling_circle_yaw, PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end

--- get a location object for a given WP number
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
         initial_yaw_deg = math.deg(ahrs:get_yaw())

         -- work out yaw between previous WP and next WP
         local cnum = mission:get_current_nav_index()
         local loc_prev = get_wp_location(cnum-1)
         local loc_next = get_wp_location(resolve_jump(cnum+1))
         wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
      end
      if cmd == 1 then
         do_axial_roll(arg1, arg2)
      elseif cmd == 2 then
         do_loop(arg1, arg2)
      elseif cmd == 3 then
         do_rolling_circle(arg1, arg2)
      end
   else
      running = false
   end
   return update, 10
end

return update()
