--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   SpeedPI
   A simple "PI" controller for airspeed. Copied from Andrew Tridgell's original
   work on the ArduPilot Aerobatics Lua scripts.

    Usage: 
    1. drop it in the scripts/modules directory
    2. include in your own script using 
         local speedpi = requre("speedpi.lua")
    3. create an instance - you may need to tune the 
        local speed_controller = speedpi.speed_controller(0.1, 0.1, 2.5, airspeed_min, airspeed_max)
    4. call it's update() from your update() with the current airspeed and airspeed error
        local airspeed_new = speed_controller.update(vehicle_airspeed, desired_airspeed - vehicle_airspeed)
    5. Set the vehicle airspeed based on the airspeed_new value returned from speedpi    

--]]

local SpeedPI = {}

SpeedPI.SCRIPT_VERSION = "4.6.0-002"
SpeedPI.SCRIPT_NAME = "Speed PI Controller"
SpeedPI.SCRIPT_NAME_SHORT = "SpeedPI"

-- constrain a value between limits
function SpeedPI.constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
end

function SpeedPI.PI_controller(kP,kI,iMax,min,max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}

    -- private fields as locals
    local _kP = kP or 0.0
    local _kI = kI or 0.0
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t = nil
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0
    local nowPI = millis():tofloat() * 0.001
 
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
       if ((_total < _max and _total > _min) or 
          (_total >= _max and err < 0) or 
             (_total <= _min and err > 0)) then
          _I = _I + _kI * err * dt
       end
       if _iMax then
        _I = SpeedPI.constrain(_I, -_iMax, iMax)
       end
       local I = _I
       local ret = target + P + I
       if math.floor(now) ~= math.floor(nowPI) then
           --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": target %f err %f P %f I %f ret %f", target,err, P, I, ret) )
           nowPI = millis():tofloat() * 0.001
       end
       _target = target
       _current = current
       _P = P
 
       ret = SpeedPI.constrain(ret, _min, _max)
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
       -- Targ = Current + error ( target airspeed )
       -- Curr = Current airspeed input to the controller
       -- P = calculated Proportional component
       -- I = calculated Integral component
       -- Total = calculated new Airspeed 
       -- Add - passed in as 0 
       logger.write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
    end
 
    -- return the instance
    return self
 end
 
function SpeedPI.speed_controller(kP_param, kI_param, iMax, sMin, sMax)
    local self = {}
    local speedpi = SpeedPI.PI_controller(kP_param, kI_param, iMax, sMin, sMax)

    function self.update(spd_current, spd_error)
       local adjustment = speedpi.update(spd_current + spd_error, spd_current)
       speedpi.log("ZSPI", 0) -- Z = scripted, S = speed, PI = PI controller
       return adjustment
    end

    function self.reset()
       speedpi.reset(0)
    end

    return self
end

gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s %s module loaded", SpeedPI.SCRIPT_NAME, SpeedPI.SCRIPT_VERSION) )

return SpeedPI