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
 *       AP_MotorsVectoredROV.cpp
 *
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsVectoredROV.h"

extern const AP_HAL::HAL& hal;

// setup_motors - configures the motors for the BlueROV
void AP_MotorsVectoredROV::setup_motors()
{
	// call parent
	AP_Motors6DOF::setup_motors();

	//For a vectored configuration, the numbers would look something like this:
	float
		//Front left
		MOT_1_ROLL_FACTOR = 0.0,
		MOT_1_PITCH_FACTOR = 0.0,
		MOT_1_YAW_FACTOR = -1.0,
		MOT_1_THROTTLE_FACTOR = 0.0,
		MOT_1_FORWARD_FACTOR = -1,
		MOT_1_STRAFE_FACTOR = -1,

		//Left, facing up
		MOT_2_ROLL_FACTOR = -1.0,
		MOT_2_PITCH_FACTOR = 0.0,
		MOT_2_YAW_FACTOR = 0.0,
		MOT_2_THROTTLE_FACTOR = -1,
		MOT_2_FORWARD_FACTOR = 0.0,
		MOT_2_STRAFE_FACTOR = 0.0,

		//Back left
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = 1.0,
		MOT_3_THROTTLE_FACTOR = 0.0,
		MOT_3_FORWARD_FACTOR = 1,
		MOT_3_STRAFE_FACTOR = -1,

		//Back right
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = -1.0,
		MOT_4_THROTTLE_FACTOR = 0.0,
		MOT_4_FORWARD_FACTOR = 1,
		MOT_4_STRAFE_FACTOR = 1,

		//Right, facing up
		MOT_5_ROLL_FACTOR = 1.0,
		MOT_5_PITCH_FACTOR = 0.0,
		MOT_5_YAW_FACTOR = 0.0,
		MOT_5_THROTTLE_FACTOR = -1,
		MOT_5_FORWARD_FACTOR = 0.0,
		MOT_5_STRAFE_FACTOR = 0.0,

		//Front right
		MOT_6_ROLL_FACTOR = 0.0,
		MOT_6_PITCH_FACTOR = 0.0,
		MOT_6_YAW_FACTOR = 1.0,
		MOT_6_THROTTLE_FACTOR = 0.0,
		MOT_6_FORWARD_FACTOR = -1,
		MOT_6_STRAFE_FACTOR = 1;


    add_motor_raw_6dof(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR,1);
    add_motor_raw_6dof(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR,2);
    add_motor_raw_6dof(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR,3);
    add_motor_raw_6dof(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR,4);
    add_motor_raw_6dof(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR,5);
    add_motor_raw_6dof(AP_MOTORS_MOT_6, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, MOT_6_THROTTLE_FACTOR, MOT_6_FORWARD_FACTOR, MOT_6_STRAFE_FACTOR,6);
}


// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/strafe commands
void AP_MotorsVectoredROV::output_armed_stabilizing()
{
    int8_t i;
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t yaw_pwm;                                                // yaw pwm value, initially calculated by calc_yaw_pwm() but may be modified after, +/- 400
    int16_t throttle_radio_output;                                  // throttle pwm value, +/- 400
    int16_t forward_pwm;                                            // forward pwm value, +/- 400
    int16_t strafe_pwm;                                             // forward pwm value, +/- 400
    int16_t out_min_pwm = 1100;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = 1900;                      // maximum pwm value we can send to the motors

    int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    int16_t linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

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
    throttle_radio_output = (calc_throttle_radio_output()-_throttle_radio_min-(_throttle_radio_max-_throttle_radio_min)/2);
    forward_pwm = get_forward()*0.4;
    strafe_pwm = get_strafe()*0.4;

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	rpy_out[i] = roll_pwm * _roll_factor[i] +
        					pitch_pwm * _pitch_factor[i] +
							yaw_pwm * _yaw_factor[i];

        }
    }

    int16_t forward_coupling_limit = 400-float(_forwardVerticalCouplingFactor)*fabs(throttle_radio_output);
    if ( forward_coupling_limit < 0 ) {
    	forward_coupling_limit = 0;
    }
    int8_t forward_coupling_direction[] = {-1,0,1,1,0,-1,0,0};

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	float forward_pwm_limited = forward_pwm;

        	// The following statements decouple forward/vertical hydrodynamic coupling on
        	// vectored ROVs. This is done by limiting the maximum output of the "rear" vectored
        	// thruster (where "rear" depends on direction of travel).
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define sign(x) ((x>0)-(x<0))
        	if ( sign(forward_pwm) == sign(forward_coupling_direction[i]) && forward_coupling_direction[i] != 0 ) {
        		forward_pwm_limited = constrain(forward_pwm,-forward_coupling_limit,forward_coupling_limit);
        	}
#undef constrain

        	linear_out[i] = throttle_radio_output * _throttle_factor[i] +
        					forward_pwm_limited * _forward_factor[i] +
							strafe_pwm * _strafe_factor[i];

        }
    }

    // Calculate final pwm output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	motor_out[i] = 1500 + _motor_reverse[i]*(rpy_out[i] + linear_out[i]);

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

