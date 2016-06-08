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
		//Front right
		MOT_1_ROLL_FACTOR = 0.0,
		MOT_1_PITCH_FACTOR = 0.0,
		MOT_1_YAW_FACTOR = 1.0,
		MOT_1_THROTTLE_FACTOR = 0.0,
		MOT_1_FORWARD_FACTOR = -1,
		MOT_1_STRAFE_FACTOR = 1,

		//Front left
		MOT_2_ROLL_FACTOR = 0.0,
		MOT_2_PITCH_FACTOR = 0.0,
		MOT_2_YAW_FACTOR = -1.0,
		MOT_2_THROTTLE_FACTOR = 0.0,
		MOT_2_FORWARD_FACTOR = -1,
		MOT_2_STRAFE_FACTOR = -1,

		//Back right
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = -1.0,
		MOT_3_THROTTLE_FACTOR = 0.0,
		MOT_3_FORWARD_FACTOR = 1,
		MOT_3_STRAFE_FACTOR = 1,

		//Back left
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = 1.0,
		MOT_4_THROTTLE_FACTOR = 0.0,
		MOT_4_FORWARD_FACTOR = 1,
		MOT_4_STRAFE_FACTOR = -1,

		//Right, facing up
		MOT_5_ROLL_FACTOR = 1.0,
		MOT_5_PITCH_FACTOR = 0.0,
		MOT_5_YAW_FACTOR = 0.0,
		MOT_5_THROTTLE_FACTOR = -1,
		MOT_5_FORWARD_FACTOR = 0.0,
		MOT_5_STRAFE_FACTOR = 0.0,

		//Left, facing up
		MOT_6_ROLL_FACTOR = -1.0,
		MOT_6_PITCH_FACTOR = 0.0,
		MOT_6_YAW_FACTOR = 0.0,
		MOT_6_THROTTLE_FACTOR = -1,
		MOT_6_FORWARD_FACTOR = 0.0,
		MOT_6_STRAFE_FACTOR = 0.0;

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
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_MotorsVectoredROV::output_armed_stabilizing()
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

    float forward_coupling_limit = 1-_forwardVerticalCouplingFactor*float(fabs(throttle_thrust));
    if ( forward_coupling_limit < 0 ) {
    	forward_coupling_limit = 0;
    }
    int8_t forward_coupling_direction[] = {-1,-1,1,1,0,0,0,0};

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

        	float forward_thrust_limited = forward_thrust;

        	// The following statements decouple forward/vertical hydrodynamic coupling on
        	// vectored ROVs. This is done by limiting the maximum output of the "rear" vectored
        	// thruster (where "rear" depends on direction of travel).
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define sign(x) ((x>0)-(x<0))
        	if ( sign(forward_thrust) == sign(forward_coupling_direction[i]) && forward_coupling_direction[i] != 0 ) {
        		forward_thrust_limited = constrain(forward_thrust,-forward_coupling_limit,forward_coupling_limit);
        	}
#undef constrain

        	linear_out[i] = throttle_thrust * _throttle_factor[i] +
        					forward_thrust_limited * _forward_factor[i] +
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

