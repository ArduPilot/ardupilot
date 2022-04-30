-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

local ROLL_TCONST = param:get('RLL2SRV_TCONST') * 0.5
local PITCH_TCONST = param:get('PTCH2SRV_TCONST') * 0.5

local LOOP_RATE = 10 --Hz

DO_JUMP = 177
NAV_WAYPOINT = 16

k_throttle = 70

-- the table key must be used by only one script on a particular flight
-- controller. If you want to re-use it then you need to wipe your old parameters
-- the key must be a number between 0 and 200. The key is persistent in storage
local PARAM_TABLE_KEY = 59
local PARAM_TABLE_PREFIX = "AERO_"

function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup SHIP specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')
HEIGHT_P     = bind_add_param('HEIGHT_P', 1, 2.0)
HEIGHT_I     = bind_add_param('HEIGHT_I', 2, 2.0)
THR_FF       = bind_add_param('THR_FF', 3, 40.0) --percentage
PITCH_FF     = bind_add_param('PITCH_FF', 4, 15.0) --degrees
SPEED_P      = bind_add_param('SPEED_P', 5, 2.0)
SPEED_I      = bind_add_param('SPEED_I', 6, 2.0)


local TRIM_THROTTLE = bind_param("TRIM_THROTTLE")
local TRIM_ARSPD_CM = bind_param("TRIM_ARSPD_CM")

local last_roll_err = 0.0
local last_id = 0
local initial_yaw_deg = 0
local wp_yaw_deg = 0
local initial_height = 0
local MIN_SPEED = 0.1

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

function wrap_pi(angle_rad)
   return math.rad(wrap_180(math.deg(angle_rad)))
end

function convert_ef_rates_to_bf_rates(roll, pitch, yaw, ef_roll_rate, ef_pitch_rate, ef_yaw_rate)
   local cr = math.cos(roll)
   local sr = math.sin(roll)
   local cp = math.cos(pitch)
   local sp = math.sin(pitch) 
   local cy = math.cos(yaw)
   local sy = math.sin(yaw)
   
   local bf_roll_rate  =  ef_roll_rate                   -    sp*ef_yaw_rate
   local bf_pitch_rate =                cr*ef_pitch_rate + sr*cp*ef_yaw_rate
   local bf_yaw_rate   =               -sr*ef_pitch_rate + cr*cp*ef_yaw_rate

   return bf_roll_rate, bf_pitch_rate, bf_yaw_rate
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
      local target_pitch = PI.update(initial_height, ahrs:get_position():alt()*0.01)
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

local height_PI = height_controller(HEIGHT_P, HEIGHT_I, PITCH_FF, 20.0)
local speed_PI = PI_controller(SPEED_P:get(), SPEED_I:get(), 100.0)

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
   local thr_ff = THR_FF:get()
   local throttle = TRIM_THROTTLE:get() + math.sin(pitch_rad) * thr_ff
   return constrain(throttle, 0.0, 100.0)
end

function do_axial_roll(arg1, arg2)
   -- constant roll rate axial roll
   if not running then
      running = true
      roll_stage = 0
      height_PI.reset()
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
      target_pitch = height_PI.update(initial_height)
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
      rolling_circle_yaw = 0
      rolling_circle_last_ms = millis()
      height_PI.reset()

      speed_PI.set_P(SPEED_P:get())
      speed_PI.set_I(SPEED_I:get())
      speed_PI.reset(math.max(SRV_Channels:get_output_scaled(k_throttle), TRIM_THROTTLE:get()))
      gcs:send_text(0, string.format("Starting rolling circle"))
   end
   local radius = arg1
   local num_rolls = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local gspeed = ahrs:groundspeed_vector():length()
   local circumference = math.abs(math.pi * 2.0 * radius)
   local circle_time = circumference / gspeed
   local yaw_rate_dps = 360.0 / circle_time
   local now_ms = millis()
   local dt = (now_ms - rolling_circle_last_ms):tofloat() * 0.001
   rolling_circle_last_ms = now_ms

   if radius < 0.0 then
      yaw_rate_dps = -yaw_rate_dps
   end

   local roll_rate = (360.0 * num_rolls) / circle_time

   rolling_circle_yaw = rolling_circle_yaw + yaw_rate_dps * dt

   if rolling_circle_stage == 0 then
      if math.abs(rolling_circle_yaw) > 10.0 then
         rolling_circle_stage = 1
      end
   elseif rolling_circle_stage == 1 then
      if math.abs(rolling_circle_yaw) >= 360.0 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished rollcircle r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         rolling_circle_stage = 2
         return
      end
   end

   local target_roll = num_rolls * math.abs(rolling_circle_yaw)
   local roll_error = wrap_180(target_roll - roll_deg)
   local roll_error_P = 0.5
   local roll_rate_corrected = roll_rate + roll_error * roll_error_P

   if rolling_circle_stage < 2 then
      target_pitch = height_PI.update(initial_height)
      vel = ahrs:get_velocity_NED()
      throttle = speed_PI.update(TRIM_ARSPD_CM:get()*0.01, vel:length())
      throttle = constrain(throttle, 0, 100.0)
      --saturation_error = constrained_throttle - target_throttle
      --gcs:send_text(0, string.format("saturation_error: e=%.1f", saturation_error))
      speed_PI.log("SPI", 0.0)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wrap_360(rolling_circle_yaw+initial_yaw_deg), PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate_corrected, pitch_rate, yaw_rate)
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


--------Trajectory definitions---------------------
function path_circle(t, radius, unused)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 1.0-math.cos(t), 0)
   return vec:scale(radius)
end

--TODO: fix this to have initial tangent 0
function path_figure_eight(t, radius)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), math.sin(t)*math.cos(t), 0)
   return vec:scale(radius)
end

function path_vertical_circle(t, radius, unused)
   t = t*math.pi*2
   local vec = makeVector3f(math.sin(t), 0.0, -1.0 + math.cost(t))
   return vec:scale(radius)
end
---------------------------------------------------

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
function rotate_path(path_f, t, arg1, arg2, orientation)
   point = path_f(t, arg1, arg2)
   return orientation:rotate_vector(point)
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

--Given vec1, vec2, returns an angular velocity  tuple that rotates vec1 to be parallel to vec2
--If vec1 and vec2 are already parallel, returns a zero vector and zero angle 
function vectors_to_angular_rate(vector1, vector2, time_constant)
   axis, angle = vectors_to_rotation(vector1, vector2)
   angular_velocity = angle/time_constant
   return axis:scale(angular_velocity)
end

--Just used this to test the above function, can probably delete now.
function test_angular_rate()
   local vector1 = makeVector3f(1.0, 0.0, 0.0)
   local vector2 = makeVector3f(1.0, 1.0, 0.0)
   local angular_rate = vectors_to_angular_rate(vector1, vector2, 1.0)
   gcs:send_text(0, string.format("angular rate: %.1f %.1f %.1f", math.deg(angular_rate:x()), math.deg(angular_rate:y()), math.deg(angular_rate:z())))
end

--returns body-fixed angular rate as Vec3f
function path_error_correction(current_pos, target_pos, forward_velocity)
   
   if forward_velocity <= MIN_SPEED then
      return makeVector3f(0.0, 0.0, 0.0)
   end
   
   --time over which to correct position error
   local time_const_pos_to_vel = 5.0
   --time over which to achieve desired velocity
   local time_const_vel_to_acc = 1.0

   local pos_error = target_pos - current_pos
   local vel_error = pos_error:scale(1.0/time_const_pos_to_vel)

   local vel_error_bf = ahrs:earth_to_body(vel_error)
   --TODO: can't change throttle at the moment
   vel_error_bf:x(0.0)
   local acc_err_bf = vel_error_bf:scale(1.0/time_const_vel_to_acc)

   return makeVector3f(0.0, -acc_err_bf:z()/forward_velocity, acc_err_bf:y()/forward_velocity)
end

test_angular_rate()

local path_var = {}
path_var.count = 0
path_var.positions = {}
path_var.initial_ori = Quaternion()

last_time = 0.0

function do_path(path, initial_yaw_deg, arg1, arg2)

   local now = millis():tofloat() * 0.001

   last_time = now

   path_var.count = path_var.count + 1
   local target_dt = 1.0/LOOP_RATE

   if not running then
      running = true
      path_var.length = path_length(path, arg1, arg2)
      local speed = TRIM_ARSPD_CM:get()*0.01

      --assuming constant velocity
      path_var.total_time = path_var.length/speed
      path_var.start_time = now
      path_var.last_time = now
      path_var.last_pos = path(0.0, arg1, arg2) --position at t0

      local path_delta = path(target_dt, arg1, arg2) --position at t1
      --deliberately only want yaw component, because the maneuver should be performed relative to the earth, not relative to the initial orientation
      path_var.initial_ori:from_euler(0, 0, math.rad(initial_yaw_deg))
      
      local corrected_position_t0 = rotate_path(path, 0, arg1, arg2, path_var.initial_ori)
      local corrected_position_t1 = rotate_path(path, target_dt, arg1, arg2, path_var.initial_ori)

      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()
      path_var.start_pos = ahrs:get_position()

      height_PI.reset()

      speed_PI.set_P(SPEED_P:get())
      speed_PI.set_I(SPEED_I:get())
      speed_PI.reset(math.max(SRV_Channels:get_output_scaled(k_throttle), TRIM_THROTTLE:get()))

      --path_var.positions[-1] is not used in initial runthrough
      path_var.positions[0] = corrected_position_t0
      path_var.positions[1] = corrected_position_t1

      path_var.time_correction = 0.0 

      path_var.filtered_angular_velocity = Vector3f()
   end

   --TODO: dt taken from actual loop rate or just desired loop rate?
   --local dt = now - path_var.last_time
   local dt = target_dt

   --normalise current time to [0, 1]
   local t = (now - path_var.start_time + path_var.time_correction)/path_var.total_time

   --TODO: Fix this exit condition
   if t > 1.0 then --done
      running = false
      vehicle:nav_script_time_done(last_id)
      return
   end

   --where we aim to be on the path at this timestamp
   --rotate from maneuver frame to 'local' EF
   local next_target_pos = rotate_path(path, t+dt, arg1, arg2, path_var.initial_ori)

   path_var.positions[-1] = path_var.positions[0]:copy()
   path_var.positions[0] = path_var.positions[1]:copy()
   path_var.positions[1] = next_target_pos:copy()

   local current_target_pos = path_var.positions[0]

   logger.write("TPOS",'t,x,y,z','ffff',t, current_target_pos:x(),current_target_pos:y(),current_target_pos:z())

   local current_measured_pos = ahrs:get_relative_position_NED_origin() - path_var.initial_ef_pos

   local path_error = {}
   path_error[-1] = (current_measured_pos - path_var.positions[-1]):length()
   path_error[0] = (current_measured_pos - path_var.positions[0]):length() 
   path_error[1] = (current_measured_pos - path_var.positions[1]):length()

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

   local position_error = path_var.positions[0]- current_measured_pos
   
   local path_loc = path_var.start_pos:copy()
   path_loc:offset(path_var.positions[0]:x(), path_var.positions[0]:y())
   path_loc:alt(path_loc:alt() - math.floor(path_var.positions[0]:z()*100))

   logger.write("POSM",'x,y,z','fff',current_measured_pos:x(),current_measured_pos:y(),current_measured_pos:z())
   logger.write("PERR",'x,y,z,tc,Lat,Lng,Alt','ffffLLf',position_error:x(),position_error:y(),position_error:z(), path_var.time_correction, path_loc:lat(), path_loc:lng(), path_loc:alt()*0.01)

   --velocity required to travel along trajectory
   local trajectory_velocity = (path_var.positions[1] - path_var.positions[-1]):scale(0.5/dt) --derivative from -dt to dt for more accuracy
   local tangent1 = (path_var.positions[0] - path_var.positions[-1])
   local tangent2 = (path_var.positions[1] - path_var.positions[0])
   local path_rate_rads = vectors_to_angular_rate(tangent1, tangent2, 2*dt)

   path_var.prev_target_pos = current_target_pos
   path_var.last_measured_pos = current_pos
   path_var.last_time = now

   local angular_velocity = path_rate_rads:scale(math.deg(1))

   --Smooth the angular velocities
   local cutoff_hz = 0.2;
   local alpha = calc_lowpass_alpha_dt(dt, cutoff_hz);
   local filtered_angular_velocity_x = path_var.filtered_angular_velocity:x()
   local filtered_angular_velocity_y = path_var.filtered_angular_velocity:y()
   local filtered_angular_velocity_z = path_var.filtered_angular_velocity:z()
   filtered_angular_velocity_x = filtered_angular_velocity_x + (angular_velocity:x() - filtered_angular_velocity_x) * alpha
   filtered_angular_velocity_y = filtered_angular_velocity_y + (angular_velocity:y() - filtered_angular_velocity_y) * alpha
   filtered_angular_velocity_z = filtered_angular_velocity_z + (angular_velocity:z() - filtered_angular_velocity_z) * alpha
   path_var.filtered_angular_velocity:x(filtered_angular_velocity_x)
   path_var.filtered_angular_velocity:y(filtered_angular_velocity_y)
   path_var.filtered_angular_velocity:z(filtered_angular_velocity_z)

   logger.write("AV",'x,y,z,fx,fy,fz','ffffff',angular_velocity:x(),angular_velocity:y(),angular_velocity:z(), filtered_angular_velocity_x,filtered_angular_velocity_y,filtered_angular_velocity_z)
   
   local vel_length = ahrs:get_velocity_NED():length()

   local err_corr_rate = path_error_correction(current_measured_pos, current_target_pos, vel_length)
   local err_corr_rate_deg = err_corr_rate:scale(math.deg(1))
   angular_velocity = angular_velocity + err_corr_rate_deg
   --angular_velocity = path_var.filtered_angular_velocity + err_corr_rate_deg

   logger.write("AVEC",'x,y,z','fff',err_corr_rate_deg:x(),err_corr_rate_deg:y(),err_corr_rate_deg:z())
   logger.write("CAV",'x,y,z','fff',angular_velocity:x(),angular_velocity:y(),angular_velocity:z())

   local target_speed = TRIM_ARSPD_CM:get()*0.01
   throttle = speed_PI.update(target_speed, vel_length)
   throttle = constrain(throttle, 0, 100.0)
   vehicle:set_target_throttle_rate_rpy(throttle, angular_velocity:x(), angular_velocity:y(), angular_velocity:z())

end

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
   if id then
      if id ~= last_id then
         -- we've started a new command
         running = false
         last_id = id
         initial_yaw_deg = math.deg(ahrs:get_yaw())
         initial_height = ahrs:get_position():alt()*0.01

         -- work out yaw between previous WP and next WP
         local cnum = mission:get_current_nav_index()
         local loc_prev = get_wp_location(cnum-1)
         local loc_next = get_wp_location(resolve_jump(cnum+1))
         wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
      end
      gcs:send_text(0, string.format("user args arg1=%.1f arg2=%.1f", arg1, arg2))

      if cmd == 1 then
         do_axial_roll(arg1, arg2)
      elseif cmd == 2 then
         do_loop(arg1, arg2)
      elseif cmd == 3 then
         do_rolling_circle(arg1, arg2)
      elseif cmd == 4 then
         do_path(path_circle, initial_yaw_deg, arg1, arg2)
      elseif cmd == 5 then
         do_path(path_figure_eight, initial_yaw_deg, arg1, arg2)
      elseif cmd == 6 then
         do_path(path_vertical_circle, initial_yaw_deg, arg1, arg2)
      end
   else
      running = false
   end
   return update, 1000/LOOP_RATE
end

return update()
