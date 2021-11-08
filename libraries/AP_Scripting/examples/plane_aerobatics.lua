-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

local ROLL_TCONST = param:get('RLL2SRV_TCONST') * 0.5
local PITCH_TCONST = param:get('PTCH2SRV_TCONST') * 0.5
local TRIM_THROTTLE = param:get('TRIM_THROTTLE') * 1.0

local scr_user1_param = Parameter()
local scr_user2_param = Parameter()
local scr_user3_param = Parameter()
assert(scr_user1_param:init('SCR_USER1'), 'could not find SCR_USER1 parameter')
assert(scr_user2_param:init('SCR_USER2'), 'could not find SCR_USER2 parameter')
assert(scr_user3_param:init('SCR_USER3'), 'could not find SCR_USER3 parameter')

local last_roll_err = 0.0
local last_id = 0

-- find our rudder channel
local RUDDER_CHAN = SRV_Channels:find_channel(21)
local RUDDER_TRIM = param:get("SERVO" .. RUDDER_CHAN + 1 .. "_TRIM")
local RUDDER_REVERSED = param:get("SERVO" .. RUDDER_CHAN + 1 .. "_REVERSED")
local RUDDER_MIN = param:get("SERVO" .. RUDDER_CHAN + 1 .. "_MIN")
local RUDDER_MAX = param:get("SERVO" .. RUDDER_CHAN + 1 .. "_MAX")
local RUDDER_THROW = (RUDDER_MAX - RUDDER_MIN) * 0.5

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

-- a controller to target a zero pitch angle
-- output is a body frame pitch rate, with convergence over time tconst in seconds
function pitch_controller(target_pitch_deg, tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local pitch_rate = (target_pitch_deg - pitch_deg) * math.cos(math.rad(roll_deg)) / tconst
   RUDDER_GAIN = scr_user1_param:get()
   local rudder = pitch_deg * RUDDER_GAIN * math.sin(math.rad(roll_deg)) / tconst
   return pitch_rate, rudder
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
      pitch_rate, rudder = pitch_controller(target_pitch, PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, 0)
      if RUDDER_REVERSED == 1 then
         rudder = -rudder
      end
      local rudder_out = math.floor(RUDDER_TRIM + RUDDER_THROW * rudder)
      rudder_out = constrain(rudder_out, RUDDER_MIN, RUDDER_MAX)
      SRV_Channels:set_output_pwm_chan_timeout(RUDDER_CHAN, rudder_out, 50)
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

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
   if id then
      if id ~= last_id then
         -- we've started a new command
         running = false
         last_id = id
      end
      if cmd == 1 then
         do_axial_roll(arg1, arg2)
      elseif cmd == 2 then
         do_loop(arg1, arg2)
      end
   else
      running = false
   end
   return update, 10
end

return update()
