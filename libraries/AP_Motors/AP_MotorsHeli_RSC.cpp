// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_RSC.h"

extern const AP_HAL::HAL& hal;

// recalc_scalers - recalculates various scalers used.  Should be called at about 1hz to allow users to see effect of changing parameters
void AP_MotorsHeli_RSC::recalc_scalers()
{
    // recalculate rotor ramp up increment
    if (_ramp_time <= 0) {
        _ramp_time = 1;
    }

    _ramp_increment = 1000.0f / (_ramp_time * _loop_rate);

    // recalculate rotor runup increment
    if (_runup_time <= 0 ) {
        _runup_time = 1;
    }
    
    if (_runup_time < _ramp_time) {
        _runup_time = _ramp_time;
    }

    _runup_increment = 1000.0f / (_runup_time * _loop_rate);
}

void AP_MotorsHeli_RSC::output_armed()
{
    // ramp rotor esc output towards target
    if (_speed_out < _desired_speed) {
        // allow rotor out to jump to rotor's current speed
        if (_speed_out < _estimated_speed) {
            _speed_out = _estimated_speed;
        }

        // ramp up slowly to target 
        _speed_out += _ramp_increment;
        if (_speed_out > _desired_speed) {
            _speed_out = _desired_speed;
        }
    } else {
        // ramping down happens instantly
        _speed_out = _desired_speed;
    }

    // ramp rotor speed estimate towards speed out
    if (_estimated_speed < _speed_out) {
        _estimated_speed += _runup_increment;
        if (_estimated_speed > _speed_out) {
            _estimated_speed = _speed_out;
        }
    } else {
        _estimated_speed -= _runup_increment;
        if (_estimated_speed < _speed_out) {
            _estimated_speed = _speed_out;
        }
    }

    // set runup complete flag
    if (!_runup_complete && _desired_speed > 0 && _estimated_speed >= _desired_speed) {
        _runup_complete = true;
    }

    if (_runup_complete && _estimated_speed <= _critical_speed) {
        _runup_complete = false;
    }

    // output to rsc servo
    write_rsc(_speed_out);
}

void AP_MotorsHeli_RSC::output_disarmed()
{
    write_rsc(0);
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1000
void AP_MotorsHeli_RSC::write_rsc(int16_t servo_out)
{
    _servo_output.servo_out = servo_out;
    _servo_output.calc_pwm();

    hal.rcout->write(_servo_output_channel, _servo_output.radio_out);
}