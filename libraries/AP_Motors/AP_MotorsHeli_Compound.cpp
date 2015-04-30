// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdlib.h>
#include <AP_HAL.h>
#include "AP_MotorsHeli_Compound.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Compound::var_info[] PROGMEM = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli_Single, 0),

    AP_GROUPEND
};

// init_servos
void AP_MotorsHeli_Compound::init_servos ()
{
    AP_MotorsHeli_Single::init_servos();
    
    _servo_yaw_2.set_angle(4500);
}

// set_boost
void AP_MotorsHeli_Compound::set_boost(int16_t boost_in)
{
   _boost_in = boost_in;    
}

// output_yaw
void AP_MotorsHeli_Compound::output_yaw(int16_t yaw_in)
{
    // constrain yaw and update limits
    int16_t yaw_out = constrain_int16(yaw_in, -4500, 4500);
  
    if (_servo_yaw.servo_out != yaw_out) {
        limit.yaw = true;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        int16_t boost_available = 4500 - abs(yaw_out);
        int16_t boost_out = _boost_in * boost_available / 1000;

        _servo_yaw_1.servo_out = boost_out + yaw_out;
        _servo_yaw_2.servo_out = boost_out - yaw_out;

        _servo_yaw_1.calc_pwm();
        _servo_yaw_2.calc_pwm();

        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _servo_yaw_1.radio_out); 
        hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_5]), _servo_yaw_2.radio_out);
    } else if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH) {
        int16_t boost_available = 4500 - max(yaw_out, 0);
        int16_t boost_out = _boost_in * boost_available / 1000;
        
        _servo_aux_1.servo_out = boost_out + yaw_out;
        _servo_aux_2.servo_out = boost_out;

        _servo_aux_1.calc_pwm();
        _servo_aux_2.calc_pwm();

        hal.rcout->write(AP_MOTORS_HELI_COMPOUND_AUX_1, _servo_aux_1.radio_out);
        hal.rcout->write(AP_MOTORS_HELI_COMPOUND_AUX_2, _servo_aux_2.radio_out);
    }
}