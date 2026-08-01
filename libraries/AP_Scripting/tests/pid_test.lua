--[[
   tests for the shared pid.lua module

   The regression this mainly guards is a NaN escaping the controller: a NaN
   propagates silently through every later output, and for a controller driving
   an airspeed demand it ends up in a MAVLink command and traps in SITL.
--]]

local pid = require("pid")

-- true only for a real, usable number
local function is_finite(v)
   return v == v and v ~= math.huge and v ~= -math.huge
end

gcs:send_text(6, "testing pid module")

-------------------------------------------------------------------------------
-- constrain()
-------------------------------------------------------------------------------

-- ordinary clamping still behaves
assert(pid.constrain(5, 0, 10) == 5)
assert(pid.constrain(-1, 0, 10) == 0)
assert(pid.constrain(11, 0, 10) == 10)

-- a NaN fails every comparison, so an unguarded constrain() would hand it
-- straight back. It must be clamped to something usable instead.
local nan = 0.0/0.0
assert(nan ~= nan) -- sanity: this really is a NaN
local constrained_nan = pid.constrain(nan, 10, 20)
assert(is_finite(constrained_nan))
assert(constrained_nan >= 10 and constrained_nan <= 20)

-- infinities must be clamped by the normal comparisons
assert(pid.constrain(math.huge, 10, 20) == 20)
assert(pid.constrain(-math.huge, 10, 20) == 10)

-------------------------------------------------------------------------------
-- PID_controller update() must never emit a non-finite value
-------------------------------------------------------------------------------

--[[
   Two updates within the same microsecond give dt == 0. The derivative term
   divides by dt, which is 0/0 (NaN) when the error is unchanged and +/-Inf
   otherwise; either poisons the controller for the rest of the flight. Freeze
   micros() to reproduce that deterministically rather than relying on timing.
--]]
local real_micros = micros
local frozen_us = real_micros()
micros = function() return frozen_us end -- luacheck: ignore

local controller = pid.PID_controller(0.1, 0.1, 0.1, 1.0, 0, 100)

-- first call initialises the filter, the rest run with dt == 0
for _ = 1, 5 do
   assert(is_finite(controller.update(0, 0)))
end
for i = 1, 5 do
   assert(is_finite(controller.update(i * 2, i)))
end

micros = real_micros -- luacheck: ignore

--[[
   The damage from dividing by dt == 0 is not just the one bad output: it is
   latched into the derivative state and every later update inherits it. A
   clamp on the way out hides that, so check the controller still *responds*
   to its input rather than just that the number looks reasonable - a poisoned
   controller returns the same clamped value no matter what it is asked for.
--]]
local low = controller.update(10, 0)
local high = controller.update(90, 0)
assert(is_finite(low) and is_finite(high))
assert(low ~= high, "controller stopped responding to its input")

function update() -- luacheck: ignore
   gcs:send_text(6, 'PID tests passed')
   return update, 1000
end

return update()
