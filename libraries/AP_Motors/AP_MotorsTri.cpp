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

/*
 *       AP_MotorsTri.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_MotorsTri.h"

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsTri::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
}

// set update rate to motors - a value in hertz
void AP_MotorsTri::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_1] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_2] |
	    1U << _motor_to_channel_map[AP_MOTORS_MOT_4];
    hal.rcout->set_freq(mask, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsTri::enable()
{
    // enable output channels
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_1]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_2]);
    hal.rcout->enable_ch(_motor_to_channel_map[AP_MOTORS_MOT_4]);
    hal.rcout->enable_ch(AP_MOTORS_CH_TRI_YAW);
}

// output_min - sends minimum values out to the motors
void AP_MotorsTri::output_min()
{
    // set lower limit flag
    limit.throttle_lower = true;

    // set all motors to minimum
    motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_min;
    motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_min;

    // send minimum value to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_CH_TRI_YAW], _rc_yaw->radio_trim);
}

// output_armed - sends commands to the motors
void AP_MotorsTri::output_armed()
{
    int16_t out_min = _rc_throttle->radio_min + _min_throttle;
    int16_t out_max = _rc_throttle->radio_max;

    // initialize lower limit flag
    limit.throttle_lower = false;

    // Throttle is 0 to 1000 only
    _rc_throttle->servo_out = constrain_int16(_rc_throttle->servo_out, 0, _max_throttle);

    // capture desired roll, pitch, yaw and throttle from receiver
    _rc_roll->calc_pwm();
    _rc_pitch->calc_pwm();
    _rc_throttle->calc_pwm();
    _rc_yaw->calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle->servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed_ramped < 0) {
            _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }
        motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_min + _spin_when_armed_ramped;
        motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_min + _spin_when_armed_ramped;

        // Every thing is limited
        limit.throttle_lower = true;

    }else{
        int16_t roll_out            = (float)_rc_roll->pwm_out * 0.866f;
        int16_t pitch_out           = _rc_pitch->pwm_out / 2;

        // check if throttle is below limit
        if (_rc_throttle->radio_out <= out_min) {
            limit.throttle_lower = true;
        }
        //left front
        motor_out[AP_MOTORS_MOT_2] = _rc_throttle->radio_out + roll_out + pitch_out;
        //right front
        motor_out[AP_MOTORS_MOT_1] = _rc_throttle->radio_out - roll_out + pitch_out;
        // rear
        motor_out[AP_MOTORS_MOT_4] = _rc_throttle->radio_out - _rc_pitch->pwm_out;

        // Tridge's stability patch
        if(motor_out[AP_MOTORS_MOT_1] > out_max) {
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_1] - out_max);
            motor_out[AP_MOTORS_MOT_1] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_2] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_4] -= (motor_out[AP_MOTORS_MOT_2] - out_max);
            motor_out[AP_MOTORS_MOT_2] = out_max;
        }

        if(motor_out[AP_MOTORS_MOT_4] > out_max) {
            motor_out[AP_MOTORS_MOT_1] -= (motor_out[AP_MOTORS_MOT_4] - out_max);
            motor_out[AP_MOTORS_MOT_2] -= (motor_out[AP_MOTORS_MOT_4] - out_max);
            motor_out[AP_MOTORS_MOT_4] = out_max;
        }

        // adjust for throttle curve
        if( _throttle_curve_enabled ) {
            motor_out[AP_MOTORS_MOT_1] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_1]);
            motor_out[AP_MOTORS_MOT_2] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_2]);
            motor_out[AP_MOTORS_MOT_4] = _throttle_curve.get_y(motor_out[AP_MOTORS_MOT_4]);
        }

        // ensure motors don't drop below a minimum value and stop
        motor_out[AP_MOTORS_MOT_1] = max(motor_out[AP_MOTORS_MOT_1],    out_min);
        motor_out[AP_MOTORS_MOT_2] = max(motor_out[AP_MOTORS_MOT_2],    out_min);
        motor_out[AP_MOTORS_MOT_4] = max(motor_out[AP_MOTORS_MOT_4],    out_min);
    }

    // send output to each motor
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], motor_out[AP_MOTORS_MOT_1]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], motor_out[AP_MOTORS_MOT_2]);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], motor_out[AP_MOTORS_MOT_4]);

    // also send out to tail command (we rely on any auto pilot to have updated the rc_yaw->radio_out to the correct value)
    // note we do not save the radio_out to the motor_out array so it may not appear in the ch7out in the status screen of the mission planner
    // note: we use _rc_tail's (aka channel 7's) REV parameter to control whether the servo is reversed or not but this is a bit nonsensical.
    //       a separate servo object (including min, max settings etc) would be better or at least a separate parameter to specify the direction of the tail servo
    if( _rc_tail->get_reverse() == true ) {
        hal.rcout->write(AP_MOTORS_CH_TRI_YAW, _rc_yaw->radio_trim - (_rc_yaw->radio_out - _rc_yaw->radio_trim));
    }else{
        hal.rcout->write(AP_MOTORS_CH_TRI_YAW, _rc_yaw->radio_out);
    }
}

// output_disarmed - sends commands to the motors
void AP_MotorsTri::output_disarmed()
{
    // fill the motor_out[] array for HIL use
    for (unsigned char i = AP_MOTORS_MOT_1; i < AP_MOTORS_MOT_4; i++) {
        motor_out[i] = _rc_throttle->radio_min;
    }

    // Send minimum values to all motors
    output_min();
}

// output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
void AP_MotorsTri::output_test()
{
    // Send minimum values to all motors
    output_min();

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min);
    hal.scheduler->delay(4000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_1], _rc_throttle->radio_min);
    hal.scheduler->delay(2000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_4], _rc_throttle->radio_min);
    hal.scheduler->delay(2000);
    hal.rcout->write(_motor_to_channel_map[AP_MOTORS_MOT_2], _rc_throttle->radio_min + _min_throttle);
    hal.scheduler->delay(300);

    // Send minimum values to all motors
    output_min();
}
