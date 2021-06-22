-- this is an example of how to do object oriented programming in Lua

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

-- a PI controller with feed-forward implemented as a Lua object
local function PIFF(kFF,kP,kI,iMax)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kFF = kFF
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
   local _iMax = iMax
   local _last_t = nil
   local _log_data = {}
   local _I = 0
   local _counter = 0

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

      local FF = _kFF * target
      local P = _kP * err
      _I = _I + _kI * err * dt
      if _iMax then
         _I = constrain(_I, -_iMax, iMax)
      end
      local I = _I
      local ret = FF + P + I

      _log_data = { target, current, FF, P, I, ret }
      return ret
   end

   -- log the controller internals
   function self.log(name)
      logger.write(name,'Targ,Curr,FF,P,I,Total','ffffff',table.unpack(_log_data))
   end

   -- return the instance
   return self
end

-- declare two PI controllers
local PI_elevator = PIFF(1.1, 0.0, 0.0, 20.0)
local PI_rudder   = PIFF(1.1, 0.0, 0.0, 20.0)

function test()
  elevator = PI_elevator.update(1.0, 0.5)
  rudder = PI_rudder.update(2.0, 0.7)

  PI_elevator.log("PEL")
  PI_rudder.log("PRD")

  gcs:send_text(0, "tick: " .. tostring(millis()))

  return test, 500
end

return test()
