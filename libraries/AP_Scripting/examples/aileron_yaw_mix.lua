--[[
mix aileron on a tilt wing to give VTOL yaw
--]]


local UPDATE_RATE_HZ = 300
local k_aileron = 4
local k_motor_tilt = 41
local k_scripting1 = 94

local MAV_SEVERITY_ERROR = 3

-- Create Parameter Table
-- Note KEYS (9 & 10) are used by the CAN_ECU sccript

local PARAM_TABLE_KEY = 23
local PARAM_TABLE_PREFIX = "AMIX_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

local AMIX_YAW_FF      = bind_add_param('YAW_FF',        1, 0)
local AMIX_YAW_P       = bind_add_param('YAW_P',         2, 0)
local AMIX_YAW_I       = bind_add_param('YAW_I',         3, 0)
local AMIX_YAW_IMAX    = bind_add_param('YAW_IMAX',      4, 0)
local AMIX_TELEM_RATE  = bind_add_param('TELEM_RATE',    5, 0)

function constrain(v, minv, maxv)
   -- constrain a value between two limits
   if v < minv then
      return minv
   end
   if v > maxv then
      return maxv
   end
   return v
end

--[[
 a PI controller with feed-forward implemented as a Lua object, using
 closure style object
--]]
local function PIFF(kFF,kP,kI,iMax)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kFF = kFF
   local _kP  = kP
   local _kI  = kI

   local _iMax = iMax
   local _last_t = nil
   local _log_data = {}
   local _I = 0.0
   local _counter = 0

   local _pid_out = {}
   _pid_out.FF = 0.0
   _pid_out.P = 0.0
   _pid_out.I = 0.0
   _pid_out.target = 0.0
   _pid_out.actual = 0.0

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

      local FF = _kFF:get() * target
      local P = _kP:get() * err
      _I = _I + _kI:get() * err * dt
      imax = _iMax:get()
      _I = constrain(_I, -imax, imax)
      local I = _I
      local ret = FF + P + I

      _log_data = { target, current, FF, P, I, ret }

      _pid_out.FF = FF
      _pid_out.P = P
      _pid_out.I = I
      _pid_out.target = target
      _pid_out.actual = current

      return ret
   end

   -- log the controller internals
   function self.log(name)
      logger.write(name,'Targ,Curr,FF,P,I,Total','ffffff',table.unpack(_log_data))
   end

   function self.send_telemetry()
      gcs:send_named_float(string.format('YAW_FF'), _pid_out.FF)
      gcs:send_named_float(string.format('YAW_P'), _pid_out.P)
      gcs:send_named_float(string.format('YAW_I'), _pid_out.I)
      gcs:send_named_float(string.format('YAW_TG'), math.deg(_pid_out.target))
      gcs:send_named_float(string.format('YAW_AC'), math.deg(_pid_out.actual))
   end

   -- return the instance
   return self
end

local yaw_pid = PIFF(AMIX_YAW_FF, AMIX_YAW_P, AMIX_YAW_I, AMIX_YAW_IMAX)
local last_telem_send = 0.0

function update()
   -- get the current tilt
   local tilt_scaled = SRV_Channels:get_output_scaled(k_motor_tilt)
   if not tilt_scaled then
      return
   end

   -- get the current body frame yaw rate
   local ygyro_rads = ahrs:get_gyro():z()
   local roll_trate, pitch_trate, yaw_trate = AC_AttitudeControl:get_rpy_rate()
   local yaw_pid_out = yaw_pid.update(yaw_trate, ygyro_rads)

   -- tilt from 0 to 1
   local tilt = tilt_scaled * 0.001

   -- get fixed wing aileron
   local aileron_scaled = SRV_Channels:get_output_scaled(k_aileron)
   if not aileron_scaled then
      gcs:send_text(0,string.format("no aileron function", idx))
      return
   end

   -- aileron from -1 to 1
   local aileron = aileron_scaled / 4500.0

   -- get VTOL yaw outputs, sum of FF, P, I and D
   local yaw_total = yaw_pid_out

   -- tilt angle in radians, 0 means VTOL mode, 1 means fixed wing fwd
   local tilt_angle_rad = math.pi * 0.5 * tilt

   -- output surface from -1 to 1
   -- note minus sign on yaw for the frame change
   local surface = constrain(math.cos(tilt_angle_rad) * (-yaw_total) + math.sin(tilt_angle_rad) * aileron, -1, 1)

   SRV_Channels:set_angle(k_scripting1, 4500)
   SRV_Channels:set_output_scaled(k_scripting1, surface * 4500)

   yaw_pid.log("AMIX")

   local now = millis():tofloat() * 0.001
   local dt = now - last_telem_send
   if dt >= (1.0 / AMIX_TELEM_RATE:get()) then
      last_telem_send = now
      yaw_pid.send_telemetry()
   end

   return update, 1000/UPDATE_RATE_HZ
end

gcs:send_text(0,string.format("AMIX Loaded"))

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
