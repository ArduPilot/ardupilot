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

   PIDcontroller
   A simple "PID" controller for airspeed. Copied from Andrew Tridgell's original
   work on the ArduPilot Aerobatics Lua scripts.

    Usage: 
    1. drop it in the scripts/modules directory
    2. include in your own script using 
         local speedpi = requre("speedpid.lua")
    3. create an instance - you may need to tune the gains
        local speed_controller = speedpid.speed_controller(0.1, 0.1, 2.5, airspeed_min, airspeed_max)
    4. call it's update() from your update() with the current airspeed and airspeed error
        local airspeed_new = speed_controller.update(vehicle_airspeed, desired_airspeed - vehicle_airspeed)
    5. Set the vehicle airspeed based on the airspeed_new value returned from speedpi    

--]]

local PIDcontroller = {}

PIDcontroller.SCRIPT_VERSION = "4.7.0-001"
PIDcontroller.SCRIPT_NAME = "PID Controller"
PIDcontroller.SCRIPT_NAME_SHORT = "PID"

PIDcontroller.MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- constrain a value between limits
function PIDcontroller.constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
end

function PIDcontroller.is_zero(x)
    return math.abs(x) < 1.1920928955078125e-7
end

function PIDcontroller.PID_controller(kP,kI,kD,iMax,min,max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}

    -- private fields as locals
    local _kP = kP or 0.0
    local _kI = kI or 0.0
    local _kD = kD or 0.0
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t_us = nil
    local _error = 0.0
    local _derivative = 0.0
    local _reset_filter = true
    local _filt_E_hz = 0.01
    local _filt_D_hz = 0.005
    local _D = 0
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0

    function self.calc_lowpass_alpha_dt(dt, cutoff_freq)
        if (dt <= 0.0 or cutoff_freq <= 0.0) then
            --INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
            return 1.0
        end
        if (PIDcontroller.is_zero(cutoff_freq)) then
            return 1.0
        end
        if (PIDcontroller.is_zero(dt)) then
            return 0.0
        end
        local rc = 1.0 / (math.pi * 2.0 * cutoff_freq)
        return dt / (dt + rc);
    end

    function self.get_filt_E_alpha(dt)
        return self.calc_lowpass_alpha_dt(dt, _filt_E_hz);
    end

    function self.get_filt_D_alpha(dt)
        return self.calc_lowpass_alpha_dt(dt, _filt_D_hz);
    end

    -- update the controller.
    function self.update(target, current)
        local now_us = micros()
        if not _last_t_us then
           _last_t_us = now_us
        end
        local dt = (now_us - _last_t_us):tofloat() * 0.000001
        _last_t_us = now_us
        local err = target - current
        _counter = _counter + 1

        -- reset input filter to value received
        if (_reset_filter) then
            _reset_filter = false
            _error = _target - current
            _derivative = 0.0
        else 
            local error_last = _error;
            _error = _error + self.get_filt_E_alpha(dt) * ((_target - current) - _error);

            -- calculate and filter derivative
            if (dt >= 0) then
                local derivative = (_error - error_last) / dt;
                _derivative = _derivative + self.get_filt_D_alpha(dt) * (derivative - _derivative);
            end
        end
        local D = _derivative * _kD
        _D = D

        local P = _kP * err
        if ((_total < _max and _total > _min) or
            (_total >= _max and err < 0) or
            (_total <= _min and err > 0)) then
            _I = _I + _kI * err * dt
        end
        if _iMax then
            _I = PIDcontroller.constrain(_I, -_iMax, iMax)
        end
        local I = _I
        local ret = target + P + I + D
        _target = target
        _current = current
        _P = P

        ret = PIDcontroller.constrain(ret, _min, _max)
        _total = ret
        return ret
    end

    -- reset integrator to an initial value
    function self.reset(integrator)
       _I = integrator
       _reset_filter = true
    end
 
    function self.set_D(D)
        _D = D
        _D_error = 0
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
        logger:write(name,'Targ,Curr,P,I,D,Total,Add','fffffff',_target,_current,_P,_I,_D,_total,add_total)
    end

    -- return the instance
    return self
 end
 
function PIDcontroller.speed_controller(kP_param, kI_param, kD_param, iMax, sMin, sMax)
    local self = {}
    local speedpid = PIDcontroller.PID_controller(kP_param, kI_param, kD_param, iMax, sMin, sMax)

    function self.update(spd_current, spd_error)
        local adjustment = speedpid.update(spd_current + spd_error, spd_current)
        speedpid.log("ZSPI", 0) -- Z = scripted, S = speed, PI = PI controller
        return adjustment
    end

    function self.reset()
        speedpid.reset(0)
    end

    return self
end

gcs:send_text(PIDcontroller.MAV_SEVERITY.INFO, string.format("%s %s module loaded", PIDcontroller.SCRIPT_NAME, PIDcontroller.SCRIPT_VERSION) )

return PIDcontroller
