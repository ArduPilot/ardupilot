-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

local ROLL_TCONST = param:get('RLL2SRV_TCONST') * 0.5
local PITCH_TCONST = param:get('PTCH2SRV_TCONST') * 0.5

local LOOP_RATE = 100 --Hz

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

function path_circle(t, radius, unused)
   t = t*math.pi*2
   --TODO: include * overload in lua bindings
   local vec = Vector3f()
   vec:x(1.0-math.cos(t))
   vec:y(-math.sin(t))
   return vec:scale(radius), 0.0
end

function path_figure_eight(t, radius, unused)
   t = t*math.pi*2
   local vec = Vector3f()
   vec:x(math.sin(t))
   vec:y(math.sin(t)*math.cos(t))
   return vec:scale(radius), 0.0
end

--path_vertical_circle returns rate
function path_vertical_circle(t, radius, unused)
   t = t*math.pi*2
   --TODO: include * overload in lua bindings
   local vec = Vector3f()
   vec:x(math.sin(t))
   vec:y(0.0)
   vec:z(-1.0+math.cos(t))
   return vec:scale(radius), 0.0
end

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

function rotate_position(position, yaw)
   local rotated_point = Vector3f()
   local cs = math.cos(yaw)
   local sn = math.sin(yaw)

   rotated_point:x(position:x() * cs - position:y() * sn)
   rotated_point:y(position:x() * sn + position:y() * cs)
   rotated_point:z(position:z())
   return rotated_point
end


function rotate_path(path_f, t, arg1, arg2, yaw)
   point, roll_rate = path_f(t, arg1, arg2)
   return rotate_position(point, yaw), roll_rate
end


--look for where this is used 
-- float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
-- {
--     if (dt <= 0.0f || cutoff_freq <= 0.0f) {
--         return 1.0;
--     }
--     float rc = 1.0f/(M_2PI*cutoff_freq);
--     return constrain_float(dt/(dt+rc), 0.0f, 1.0f);
-- }

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

local path_var = {}
path_var.count = 0
path_var.positionAndRoll = {}
path_var.positions = {}

function do_path(path, arg1, arg2)

   local now = millis():tofloat() * 0.001

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
      path_var.last_pos = path(0.0, arg1, arg2)
      local path_delta = path(target_dt, arg1, arg2)
      local init_tangent = path_delta - path_var.last_pos
      path_var.init_yaw = wrap_pi(math.atan(init_tangent:y(), init_tangent:x()))
      path_var.init_yaw = -wrap_pi(path_var.init_yaw - math.rad(wp_yaw_deg))
      
      local corrected_position_t0 = rotate_path(path, 0, arg1, arg2, path_var.init_yaw)
      local corrected_position_t1 = rotate_path(path, target_dt, arg1, arg2, path_var.init_yaw)

      --path_var.prev_target_pos = corrected_position_t0

      local tangent = corrected_position_t1 - corrected_position_t0
      local corrected_yaw = wrap_pi(math.atan(tangent:y(), tangent:x()))

      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()
      path_var.start_pos = ahrs:get_position()

      height_PI.reset()

      speed_PI.set_P(SPEED_P:get())
      speed_PI.set_I(SPEED_I:get())
      speed_PI.reset(math.max(SRV_Channels:get_output_scaled(k_throttle), TRIM_THROTTLE:get()))

      --path_var.positions[-1] = prev_target_pos
      path_var.positions[0] = corrected_position_t0
      path_var.positions[1] = corrected_position_t1


      path_var.time_correction = 0.0 

      path_var.filtered_target_velocity = Vector3f()
   end

   --local dt = now - path_var.last_time
   local dt = target_dt
   --actually want ahrs last measured time or ahrs update frequency

   --normalise current time to [0, 1]
   local t = (now - path_var.start_time + path_var.time_correction)/path_var.total_time

   --TODO: Fix this exit condition
   if t > 1.0 then --done
      running = false
      vehicle:nav_script_time_done(last_id)
      return
   end

   --where we aim to be on the path at this timestamp
   --TODO: make more clear, yaw and initial translation -> SE3 offset of frame
   --rotate from maneuver frame to 'local' EF
   local next_target_pos = rotate_path(path, t+dt, arg1, arg2, path_var.init_yaw)

   path_var.positions[-1] = path_var.positions[0]:copy()
   path_var.positions[0] = path_var.positions[1]:copy()
   path_var.positions[1] = next_target_pos:copy()

   local current_target_pos = path_var.positions[0]

   logger.write("TPOS",'t,x,y,z','ffff',t, current_target_pos:x(),current_target_pos:y(),current_target_pos:z())

   -- local ori = Quaternion()
   -- ahrs:get_quat_body_to_ned(ori)
   -- -- gcs:send_text(0, string.format("logging orientation %.1f %.1f %.1f %.1f", ori:q1(), ori:q2(), ori:q3(), ori:q4()))
   -- qp = ori:get_euler_pitch()
   -- qr = ori:get_euler_roll()
   -- qy = ori:get_euler_yaw()

   -- logger.write("QUAT",'r,p,y','fff',qr, qp, qy)

   local current_measured_pos = ahrs:get_relative_position_NED_origin() - path_var.initial_ef_pos

   local path_error = {}
   path_error[-1] = (current_measured_pos - path_var.positions[-1]):length()
   path_error[0] = (current_measured_pos - path_var.positions[0]):length() 
   path_error[1] = (current_measured_pos - path_var.positions[1]):length()

   local smallest_error_index = -1
   for i = 0,1,1
   do
      if(path_error[i] < path_error[smallest_error_index]) then
         smallest_error_index = i
      end
   end

   if(smallest_error_index == 1) then
     path_var.positions[-1] = path_var.positions[0]
     path_var.positions[0] = path_var.positions[1]
     path_var.positions[1] = rotate_path(path, t + 2*dt, arg1, arg2, path_var.init_yaw)
   end

   if(smallest_error_index == -1) then
     path_var.positions[1] = path_var.positions[0] 
     path_var.positions[0] = path_var.positions[-1]
     path_var.positions[-1] = rotate_path(path, t - 2*dt, arg1, arg2, path_var.init_yaw)
   end

   path_var.time_correction = path_var.time_correction + smallest_error_index*target_dt

   local position_error = path_var.positions[0]- current_measured_pos
   --arbitrary time constant larger than dt over which to correct the position error
   
   local path_loc = path_var.start_pos:copy()
   path_loc:offset(path_var.positions[0]:x(), path_var.positions[0]:y())
   --gcs:send_text(0, string.format("path loc alt %.1f %.1f", path_loc:alt(), path_var.positions[0]:z()))

   path_loc:alt(path_loc:alt() - math.floor(path_var.positions[0]:z()*100))

   --gcs:send_text(0, string.format("position: %d", smallest_error_index))
   logger.write("POSM",'x,y,z','fff',current_measured_pos:x(),current_measured_pos:y(),current_measured_pos:z())
   logger.write("PERR",'x,y,z,tc,Lat,Lng,Alt','ffffLLf',position_error:x(),position_error:y(),position_error:z(), path_var.time_correction, path_loc:lat(), path_loc:lng(), path_loc:alt()*0.01)

   --turn into AERO_PATH_TCONST
   local correction_time = 0.75
   --velocity required to correct current position error
   local position_error_velocity = (position_error):scale(1.0/correction_time)
   --velocity required to travel along trajectory
   local trajectory_velocity = (path_var.positions[1] - path_var.positions[-1]):scale(0.5/dt) --derivative from -dt to dt for more accuracy
   --local trajectory_velocity = (path_var.positions[1] - path_var.positions[0]):scale(1.0/dt) --derivative from -dt to dt for more accuracy

   --gcs:send_text(0, string.format("position_error_velocity %.1f %.1f %.1f", position_error_velocity:x(), position_error_velocity:y(), position_error_velocity:z()))

   logger.write("VTRA",'x,y,z','fff',trajectory_velocity:x(),trajectory_velocity:y(),trajectory_velocity:z())

   local target_velocity = position_error_velocity + trajectory_velocity
   --gcs:send_text(0, string.format("VTAR %.1f %.1f %.1f", target_velocity:x(),target_velocity:y(),target_velocity:z()))

   local cutoff_hz = 2.0;
   local alpha = calc_lowpass_alpha_dt(dt, cutoff_hz);
   --if (is_positive(_RISI[i].delta_angle_dt)) {
   --gyro_filtered[i] += ((_RISI[i].delta_angle/_RISI[i].delta_angle_dt) - gyro_filtered[i]) * alpha;
   --local alpha = 0.1
   --gcs:send_text(0, string.format("alpha value %.1f ", alpha))

   local filtered_target_velocity_x = path_var.filtered_target_velocity:x()
   local filtered_target_velocity_y = path_var.filtered_target_velocity:y()
   local filtered_target_velocity_z = path_var.filtered_target_velocity:z()

   filtered_target_velocity_x = filtered_target_velocity_x + (target_velocity:x() - filtered_target_velocity_x) * alpha
   filtered_target_velocity_y = filtered_target_velocity_y + (target_velocity:y() - filtered_target_velocity_y) * alpha
   filtered_target_velocity_z = filtered_target_velocity_z + (target_velocity:z() - filtered_target_velocity_z) * alpha


   path_var.filtered_target_velocity:x(filtered_target_velocity_x)
   path_var.filtered_target_velocity:y(filtered_target_velocity_y)
   path_var.filtered_target_velocity:z(filtered_target_velocity_z)

   logger.write("VTAR",'t,x,y,z','ffff',t,target_velocity:x(),target_velocity:y(),target_velocity:z())
   logger.write("VTFI",'x,y,z','fff',path_var.filtered_target_velocity:x(),path_var.filtered_target_velocity:y(),path_var.filtered_target_velocity:z())


   --smooth velocity with LPF

   --smooth dt with LPF. tc of 1Hz

   path_var.prev_target_pos = current_target_pos
   path_var.last_measured_pos = current_pos
   path_var.last_time = now

   local target_yaw_rad = wrap_pi(math.atan(target_velocity:y(), target_velocity:x()))   
   local current_yaw_rad = wrap_pi(ahrs:get_yaw())
   local yaw_error = wrap_pi(target_yaw_rad - current_yaw_rad)
   local yaw_tconst = dt
   -- want to correct yaw rate with P control
   local ef_yaw_rate = yaw_error/yaw_tconst  

   -- local orientation = ahrs:get_rotation_body_to_ned()

   -- (cos(pi/2), 0, sin(pi/2), 0)
   local target_pitch_rad = wrap_pi(math.atan(-target_velocity:z(), math.sqrt(target_velocity:x()^2 + target_velocity:y()^2)))


   --old
   -- target rd,pd,yd, derived from vectors (where we're pointing)
   -- get current r,p,y (converted from global ori est) (gimbal lock)
   -- form errors (rd - r, pd - p, yd - y)

   --new target rd,pd,yd <- combine into global orientation q_d
   --current ori q
   --error orientation q*q_d^{}

   local current_pitch_rad = wrap_pi(ahrs:get_pitch())
   local pitch_error = wrap_pi(target_pitch_rad - current_pitch_rad)
   local pitch_tconst = dt
   local ef_pitch_rate = pitch_error/pitch_tconst

   logger.write("TPR",'rad,error,error_rate','fff',target_pitch_rad, pitch_error, ef_pitch_rate)

   --assume roll 0 for now, should work for 2D paths like circle and figure eight
   local target_roll_rad = 0.0
   local current_roll_rad = wrap_pi(ahrs:get_roll())
   local roll_error = wrap_pi(target_roll_rad - current_roll_rad)
   local roll_tconst = dt
   --local ef_roll_rate = roll_error/roll_tconst
   local ef_roll_rate = 0.0

   logger.write("RATS",'roll,pitch,yaw', 'fff',ef_roll_rate, ef_pitch_rate, ef_yaw_rate)

   --convert ef rates to bf rates, I assume there is a function we can bind in Ardupilot to do this already, but couldn't find it.
   bf_roll_rate, bf_pitch_rate, bf_yaw_rate = convert_ef_rates_to_bf_rates(current_roll_rad,
                                                                        current_pitch_rad,
                                                                        current_yaw_rad,
                                                                        ef_roll_rate, 
                                                                        ef_pitch_rate, 
                                                                        ef_yaw_rate)

   vel = ahrs:get_velocity_NED()
   local target_speed = TRIM_ARSPD_CM:get()*0.01
   throttle = speed_PI.update(target_speed, vel:length())
   throttle = constrain(throttle, 0, 100.0)
   vehicle:set_target_throttle_rate_rpy(throttle, bf_roll_rate, bf_pitch_rate, bf_yaw_rate)
end

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
   --gcs:send_text(0, string.format("user args arg1=%.1f arg2=%.1f", arg1, arg2))

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
         do_path(path_circle, arg1, arg2)
      elseif cmd == 5 then
         do_path(path_figure_eight, arg1, arg2)
      elseif cmd == 6 then
         do_path(path_vertical_circle, arg1, arg2)
      end
   else
      running = false
   end
   return update, 1000/LOOP_RATE
end

return update()
