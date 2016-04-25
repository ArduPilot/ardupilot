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
 *       AP_MotorsVectored6DOF.cpp
 *
 *
 */

#include "AP_MotorsVectored6DOF.h"

// setup_motors - configures the motors for the BlueROV
void AP_MotorsVectored6DOF::setup_motors()
{
	// call parent
	AP_Motors6DOF::setup_motors();

	// hard coded config for Vectored 6DOF configuration (Triton)
	// Positive values: strafe right, move forward, heave up 
	//				    pitch up, roll right, yaw right
	float
		// Front right pushing forwards and left
		MOT_1_ROLL_FACTOR = 0.0,
		MOT_1_PITCH_FACTOR = 0.0,
		MOT_1_YAW_FACTOR = 1.0,
		MOT_1_THROTTLE_FACTOR = 0.0,
		MOT_1_FORWARD_FACTOR = 1.0,
		MOT_1_STRAFE_FACTOR = -1.0,

		// Front left pushing forwards and right
		MOT_2_ROLL_FACTOR = 0.0,
		MOT_2_PITCH_FACTOR = 0.0,
		MOT_2_YAW_FACTOR = -1.0,
		MOT_2_THROTTLE_FACTOR = 0.0,
		MOT_2_FORWARD_FACTOR = 1.0,
		MOT_2_STRAFE_FACTOR = 1.0,

		// Rear right pushing backwards and left
		MOT_3_ROLL_FACTOR = 0.0,
		MOT_3_PITCH_FACTOR = 0.0,
		MOT_3_YAW_FACTOR = -1.0,
		MOT_3_THROTTLE_FACTOR = 0.0,
		MOT_3_FORWARD_FACTOR = -1.0,
		MOT_3_STRAFE_FACTOR = -1.0,

		// Rear left pushing backwards and right
		MOT_4_ROLL_FACTOR = 0.0,
		MOT_4_PITCH_FACTOR = 0.0,
		MOT_4_YAW_FACTOR = 1.0,
		MOT_4_THROTTLE_FACTOR = 0.0,
		MOT_4_FORWARD_FACTOR = -1.0,
		MOT_4_STRAFE_FACTOR = 1.0,

		// Front right pushing down
		MOT_5_ROLL_FACTOR = -1.0,
		MOT_5_PITCH_FACTOR = 1.0,
		MOT_5_YAW_FACTOR = 0.0,
		MOT_5_THROTTLE_FACTOR = -1.0,
		MOT_5_FORWARD_FACTOR = 0.0,
		MOT_5_STRAFE_FACTOR = 0.0,

		// Front left pushing down
		MOT_6_ROLL_FACTOR = 1.0,
		MOT_6_PITCH_FACTOR = 1.0,
		MOT_6_YAW_FACTOR = 0.0,
		MOT_6_THROTTLE_FACTOR = -1.0,
		MOT_6_FORWARD_FACTOR = 0.0,
		MOT_6_STRAFE_FACTOR = 0.0,

		// Rear right pushing down
		MOT_7_ROLL_FACTOR = -1.0,
		MOT_7_PITCH_FACTOR = -1.0,
		MOT_7_YAW_FACTOR = 0.0,
		MOT_7_THROTTLE_FACTOR = -1.0,
		MOT_7_FORWARD_FACTOR = 0.0,
		MOT_7_STRAFE_FACTOR = 0.0,

		// Rear left pushing down
		MOT_8_ROLL_FACTOR = 1.0,
		MOT_8_PITCH_FACTOR = -1.0,
		MOT_8_YAW_FACTOR = 0.0,
		MOT_8_THROTTLE_FACTOR = -1.0,
		MOT_8_FORWARD_FACTOR = 0.0,
		MOT_8_STRAFE_FACTOR = 0.0;

	add_motor_raw_6dof(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR,1);
	add_motor_raw_6dof(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR,2);
	add_motor_raw_6dof(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR,3);
	add_motor_raw_6dof(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR,4);
	add_motor_raw_6dof(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR,5);
	add_motor_raw_6dof(AP_MOTORS_MOT_6, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, MOT_6_THROTTLE_FACTOR, MOT_6_FORWARD_FACTOR, MOT_6_STRAFE_FACTOR,6);
	add_motor_raw_6dof(AP_MOTORS_MOT_7, MOT_7_ROLL_FACTOR, MOT_7_PITCH_FACTOR, MOT_7_YAW_FACTOR, MOT_7_THROTTLE_FACTOR, MOT_7_FORWARD_FACTOR, MOT_7_STRAFE_FACTOR,7);
	add_motor_raw_6dof(AP_MOTORS_MOT_8, MOT_8_ROLL_FACTOR, MOT_8_PITCH_FACTOR, MOT_8_YAW_FACTOR, MOT_8_THROTTLE_FACTOR, MOT_8_FORWARD_FACTOR, MOT_8_STRAFE_FACTOR,8);

}


// Band Aid fix for motor normalization issues.
// TODO: find a global solution for managing saturation that works for all vehicles
void AP_MotorsVectored6DOF::output_armed_stabilizing()
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

    float rpt_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float yfl_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    float rpt_max;
    float yfl_max;

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

    // calculate roll, pitch and Throttle for each motor (only used by vertical thrusters)
    rpt_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	rpt_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         throttle_thrust * _throttle_factor[i];
            if (fabs(rpt_out[i]) > rpt_max) {
                rpt_max = fabs(rpt_out[i]);
            }   
        }
    }

    // calculate linear/yaw command for each motor (only used for translational thrusters)
    // linear factors should be 0.0 or 1.0 for now
    yfl_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	yfl_out[i] = yaw_thrust * _yaw_factor[i] +
        					forward_thrust * _forward_factor[i] +
							lateral_thrust * _lateral_factor[i];
            if (fabs(yfl_out[i]) > yfl_max) {
                yfl_max = fabs(yfl_out[i]);
            }                
        }
    }

    // Calculate final output for each motor and normalize if necessary 
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	_thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
        }
    }
}
