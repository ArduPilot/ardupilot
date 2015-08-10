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

// output - update value to send to ESC/Servo
void AP_MotorsHeli_RSC::output(uint8_t state)
{
    switch (state){
        case ROTOR_CONTROL_STOP:
            _control_speed = 0;                             // ramp input to zero
            _control_out = 0;                               // force ramp output to zero
            _estimated_speed = 0;                           // force speed estimate to zero
            break;

        case ROTOR_CONTROL_IDLE:
            _control_speed = _idle_speed;                   // set control speed to idle speed
            if (_control_out < _idle_speed){
                _control_out = _idle_speed;                 // if control output is less than idle speed, force ramp function to jump to idle speed
            }
            break;

        case ROTOR_CONTROL_ACTIVE:
            _control_speed = _desired_speed;                // set control speed to desired speed
            break;
    }

    // run speed ramp function to slew output smoothly
    speed_ramp(_control_speed);

    // update rotor speed estimate
    update_speed_estimate();

    // output to rsc servo
    write_rsc(_control_out);
}

// speed_ramp - ramps speed towards target, result put in _control_out 
void AP_MotorsHeli_RSC::speed_ramp(int16_t speed_target)
{
    // range check speed_target
    speed_target = constrain_int16(speed_target,0,1000);

    // ramp output upwards towards target
    if (_control_out < speed_target) {
        // allow control output to jump to estimated speed
        if (_control_out < _estimated_speed) {
            _control_out = _estimated_speed;
        }
        // ramp up slowly to target
        _control_out += _ramp_increment;
        if (_control_out > speed_target) {
            _control_out = speed_target;
        }
    }else{
        // ramping down happens instantly
        _control_out = speed_target;
    }


}

// update_speed_estimate - function to estimate speed
void AP_MotorsHeli_RSC::update_speed_estimate()
{
    // ramp speed estimate towards control out
    if (_estimated_speed < _control_out) {
        _estimated_speed += _runup_increment;
        if (_estimated_speed > _control_out) {
            _estimated_speed = _control_out;
        }
    }else{
        _estimated_speed -= _runup_increment;
        if (_estimated_speed < _control_out) {
            _estimated_speed = _control_out;
        }
    }

    // update run-up complete flag
    if (!_runup_complete && _control_out > _idle_speed && _estimated_speed >= _control_out) {
        _runup_complete = true;
    }
    if (_runup_complete && _estimated_speed <= _critical_speed) {
        _runup_complete = false;
    }
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1000
void AP_MotorsHeli_RSC::write_rsc(int16_t servo_out)
{
    _servo_output.servo_out = servo_out;
    _servo_output.calc_pwm();

    hal.rcout->write(_servo_output_channel, _servo_output.radio_out);
}