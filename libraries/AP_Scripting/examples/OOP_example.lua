-- this is an example of how to do object oriented programming in Lua
-- luacheck: only 0

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
         _I = constrain(_I, -_iMax, _iMax)
      end
      local I = _I
      local ret = FF + P + I

      _log_data = { target, current, FF, P, I, ret }
      return ret
   end

   -- log the controller internals
   function self.log(name)
      logger:write(name,'Targ,Curr,FF,P,I,Total','ffffff',table.unpack(_log_data))
   end

   -- return the instance
   return self
end


--[[
 another example of a PIFF controller as an object, this time using
 metatables. Using metatables uses less memory and object creation is
 faster, but access to variables is slower
--]]
local PIFF2 = {}
PIFF2.__index = PIFF2

function PIFF2.new(kFF,kP,kI,iMax)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = setmetatable({},PIFF2)
   self.kFF = kFF
   self.kP = kP
   self.kI = kI
   self.iMax = iMax
   self.last_t = nil
   self.log_data = {}
   self.I = 0
   self.counter = 0
   return self
end

function PIFF2.update(self, target, current)
   local now = millis():tofloat() * 0.001
   if not self.last_t then
      self.last_t = now
   end
   local dt = now - self.last_t
   self.last_t = now
   local err = target - current
   self.counter = self.counter + 1
   local FF = self.kFF * target
   local P = self.kP * err
   self.I = self.I + self.kI * err * dt
   if self.iMax then
      self.I = constrain(self.I, -self.iMax, self.iMax)
   end
   local ret = FF + P + self.I

   self.log_data = { target, current, FF, P, self.I, ret }
   return ret
end

function PIFF2.log(self, name)
   logger:write(name,'Targ,Curr,FF,P,I,Total','ffffff',table.unpack(self.log_data))
end

--[[
 declare two PI controllers, using one of each style. Note the use of new() for the metatables style
--]]
local PI_elevator = PIFF(1.1, 0.0, 0.0, 20.0)
local PI_rudder   = PIFF2.new(1.1, 0.0, 0.0, 20.0)

function test()
  -- note the different syntax for the two varients
  elevator = PI_elevator.update(1.0, 0.5)
  rudder = PI_rudder:update(2.0, 0.7)

  PI_elevator.log("PEL")
  PI_rudder:log("PRD")

  gcs:send_text(0, "tick: " .. tostring(millis()))

  return test, 500
end

return test()
