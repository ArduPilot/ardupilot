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
 *       AP_Motors6DOF.cpp - ArduSub motors library
 *
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Motors6DOF.h"

extern const AP_HAL::HAL& hal;

//void AP_Motors6DOF::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float strafe_fac, uint8_t testing_order) {
//	//Parent takes care of enabling output and setting up masks
//	AP_MotorsMatrix::add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);
//
//	//These are additional parameters for an ROV
//	_throttle_factor[motor_num] = throttle_fac;
//	_forward_factor[motor_num] = forward_fac;
//	_strafe_factor[motor_num] = strafe_fac;
//}



// output_min - sends minimum values out to the motors
void AP_Motors6DOF::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    //limit.throttle_lower = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    // ToDo find a field to store the minimum pwm instead of hard coding 1500
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, 1500);
            //rc_write(i, _throttle_radio_min);
        }
    }
    hal.rcout->push();
}

//ToDo: call in control_rov, to mix inputs with no stabilization control
void AP_Motors6DOF::output_armed_not_stabilizing()
{
	output_min(); //cut the motors for now
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/strafe commands
void AP_Motors6DOF::output_armed_stabilizing()
{
    int8_t i;
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t yaw_pwm;                                                // yaw pwm value, initially calculated by calc_yaw_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t forward_pwm;
    int16_t strafe_pwm;
    int16_t out_min_pwm = 1100;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = 1900;                      // maximum pwm value we can send to the motors
//    int16_t out_mid_pwm = 1500;              // mid pwm value we can send to the motors
                                      // the is the best throttle we can come up which provides good control without climbing

//    float rpy_scale = 1.0;                                          // this is used to scale the roll, pitch and yaw to fit within the motor limits



    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    int16_t linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

    int16_t rpy_low = 0;    // lowest motor value
    int16_t rpy_high = 0;   // highest motor value
    int16_t yaw_allowed;    // amount of yaw we can fit in
    int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Ensure throttle is within bounds of 0 to 1000
    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    roll_pwm = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    yaw_pwm = calc_yaw_pwm();
    throttle_radio_output = calc_throttle_radio_output();
    forward_pwm = get_forward();
    strafe_pwm = get_strafe();

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	rpy_out[i] = roll_pwm * _roll_factor[i] +
        					pitch_pwm * _pitch_factor[i] +
							yaw_pwm * _yaw_factor[i];

        }
    }

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    // ToDo calculate linear commands in a function to ensure that any input > 1500 before scaling turns into an output > 1500 after scaling
    // and any input < 1500 turns into an output < 1500
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	linear_out[i] = throttle_radio_output * _throttle_factor[i] +
        					forward_pwm * _forward_factor[i] +
							strafe_pwm * _strafe_factor[i];

        }
    }

    // Calculate final pwm output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	motor_out[i] = rpy_out[i] + linear_out[i];

        }
    }

//    // apply thrust curve and voltage scaling
//    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
//        if (motor_enabled[i]) {
//            motor_out[i] = apply_thrust_curve_and_volt_scaling(motor_out[i], out_min_pwm, out_max_pwm);
//        }
//    }

    // clip motor output if required (shouldn't be)
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
        }
    }

    // send output to each motor
    hal.rcout->cork();
    for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
        if( motor_enabled[i] ) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}
