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

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors6DOF::var_info[] = {
	AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_Motors6DOF, _motor_reverse[0], 1),

	// @Param: 2_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("2_DIRECTION", 2, AP_Motors6DOF, _motor_reverse[1], 1),

	// @Param: 3_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("3_DIRECTION", 3, AP_Motors6DOF, _motor_reverse[2], 1),

	// @Param: 4_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("4_DIRECTION", 4, AP_Motors6DOF, _motor_reverse[3], 1),

	// @Param: 5_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("5_DIRECTION", 5, AP_Motors6DOF, _motor_reverse[4], 1),

	// @Param: 6_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("6_DIRECTION", 6, AP_Motors6DOF, _motor_reverse[5], 1),

	// @Param: 7_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("7_DIRECTION", 7, AP_Motors6DOF, _motor_reverse[6], 1),

	// @Param: 8_DIRECTION
	// @DisplayName: Motor normal or reverse
	// @Description: Used to change motor rotation directions without changing wires
	// @Values: 1:normal,-1:reverse
	// @User: Standard
	AP_GROUPINFO("8_DIRECTION", 8, AP_Motors6DOF, _motor_reverse[7], 1),

	// @Param: FV_CPLNG_K
	// @DisplayName: Forward/vertical to pitch decoupling factor
	// @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("FV_CPLNG_K", 9, AP_Motors6DOF, _forwardVerticalCouplingFactor, 1.0),

    AP_GROUPEND
};

void AP_Motors6DOF::add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac, uint8_t testing_order) {
	//Parent takes care of enabling output and setting up masks
	add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

	//These are additional parameters for an ROV
	_throttle_factor[motor_num] = throttle_fac;
	_forward_factor[motor_num] = forward_fac;
	_lateral_factor[motor_num] = lat_fac;
}

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

int16_t AP_Motors6DOF::calc_thrust_to_pwm(float thrust_in) const
{
    return constrain_int16(1500 + thrust_in * 400, _throttle_radio_min, _throttle_radio_max);
}

void AP_Motors6DOF::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = 1500;
                }
            }
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = 1500;
                }
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                }
            }
            break;
    }

    // send output to each motor
    hal.rcout->cork();
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
    hal.rcout->push();
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

	roll_thrust = _roll_in;
	pitch_thrust = _pitch_in;
	yaw_thrust = _yaw_in;
	throttle_thrust = get_throttle_bidirectional();
	forward_thrust = _forward_in;
	lateral_thrust = _lateral_in;

    float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	rpy_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         yaw_thrust * _yaw_factor[i];

        }
    }

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	linear_out[i] = throttle_thrust * _throttle_factor[i] +
        					forward_thrust * _forward_factor[i] +
							lateral_thrust * _lateral_factor[i];
        }
    }

    // Calculate final output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	_thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]),-1.0f,1.0f);
        }
    }
}
