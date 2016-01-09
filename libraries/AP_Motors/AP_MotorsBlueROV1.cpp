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
 *       AP_MotorsBlueROV.cpp
 *
 *
 */

#include "AP_MotorsBlueROV.h"

// setup_motors - configures the motors for the BlueROV
void AP_MotorsBlueROV::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();

    // hard coded config for BlueROV
    float
		//Front left downward facing
		MOT_1_ROLL_FACTOR = -0.5,
		MOT_1_PITCH_FACTOR = 0.5,
		MOT_1_YAW_FACTOR = 0,
		MOT_1_THROTTLE_FACTOR = 0.5,
		MOT_1_FORWARD_FACTOR = 0.0,
		MOT_1_STRAFE_FACTOR = 0.0,

		//Front right downward facing
		MOT_2_ROLL_FACTOR = 0.5,
		MOT_2_PITCH_FACTOR = 0.5,
		MOT_2_YAW_FACTOR = 0,
		MOT_2_THROTTLE_FACTOR = 0.5,
		MOT_2_FORWARD_FACTOR = 0.0,
		MOT_2_STRAFE_FACTOR = 0.0,

		//Mid left backward facing
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = 1.0,
		MOT_3_THROTTLE_FACTOR = 0.0,
		MOT_3_FORWARD_FACTOR = 0.8,
		MOT_3_STRAFE_FACTOR = 0.0,

		//Mid right backward facing
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = -1.0,
		MOT_4_THROTTLE_FACTOR = 0.0,
		MOT_4_FORWARD_FACTOR = 0.8,
		MOT_4_STRAFE_FACTOR = 0.0,

		//Rear motor downward facing
		MOT_5_ROLL_FACTOR = 0,
		MOT_5_PITCH_FACTOR = -1.0,
		MOT_5_YAW_FACTOR = 0.0,
		MOT_5_THROTTLE_FACTOR = 0.5,
		MOT_5_FORWARD_FACTOR = 0.0,
		MOT_5_STRAFE_FACTOR = 0.0,

		//Bottom motor facing left
		MOT_6_ROLL_FACTOR = 0.25,
		MOT_6_PITCH_FACTOR = 0.0,
		MOT_6_YAW_FACTOR = 0.0,
		MOT_6_THROTTLE_FACTOR = 0.0,
		MOT_6_FORWARD_FACTOR = 0.0,
		MOT_6_STRAFE_FACTOR = 1.0;


    //For a vectored configuration, the numbers would look something like this.
//    float
//		//Front left
//		MOT_1_ROLL_FACTOR = 0.0,
//		MOT_1_PITCH_FACTOR = 0.0,
//		MOT_1_YAW_FACTOR = 0.25,
//		MOT_1_THROTTLE_FACTOR = 0.0,
//		MOT_1_FORWARD_FACTOR = 0.25,
//		MOT_1_STRAFE_FACTOR = 0.25,
//
//		//Front right
//		MOT_2_ROLL_FACTOR = 0.0,
//		MOT_2_PITCH_FACTOR = 0.0,
//		MOT_2_YAW_FACTOR = -0.25,
//		MOT_2_THROTTLE_FACTOR = 0.0,
//		MOT_2_FORWARD_FACTOR = 0.25,
//		MOT_2_STRAFE_FACTOR = -0.25,
//
//		//Back left
//		MOT_3_ROLL_FACTOR = 0,
//		MOT_3_PITCH_FACTOR = 0,
//		MOT_3_YAW_FACTOR = -0.25,
//		MOT_3_THROTTLE_FACTOR = 0.0,
//		MOT_3_FORWARD_FACTOR = 0.25,
//		MOT_3_STRAFE_FACTOR = -0.25,
//
//		//Back right
//		MOT_4_ROLL_FACTOR = 0,
//		MOT_4_PITCH_FACTOR = 0,
//		MOT_4_YAW_FACTOR = 0.25,
//		MOT_4_THROTTLE_FACTOR = 0.0,
//		MOT_4_FORWARD_FACTOR = 0.25,
//		MOT_4_STRAFE_FACTOR = 0.25,
//
//		//Left, facing up
//		MOT_5_ROLL_FACTOR = -1.0,
//		MOT_5_PITCH_FACTOR = 0.0,
//		MOT_5_YAW_FACTOR = 0.0,
//		MOT_5_THROTTLE_FACTOR = 1.0,
//		MOT_5_FORWARD_FACTOR = 0.0,
//		MOT_5_STRAFE_FACTOR = 0.0,
//
//		//Right, facing up
//		MOT_6_ROLL_FACTOR = 1.0,
//		MOT_6_PITCH_FACTOR = 0.0,
//		MOT_6_YAW_FACTOR = 0.0,
//		MOT_6_THROTTLE_FACTOR = 1.0,
//		MOT_6_FORWARD_FACTOR = 0.0,
//		MOT_6_STRAFE_FACTOR = 0.0;


	add_motor_raw(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, 1);
	add_motor_raw(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, 2);
	add_motor_raw(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, 3);
	add_motor_raw(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, 4);
	add_motor_raw(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, 5);
	add_motor_raw(AP_MOTORS_MOT_6, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, 6);
}

AP_MotorsBlueROV::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float strafe_fac, uint8_t testing_order) {
	//Parent takes care of enabling output and setting up masks
	AP_MotorsMatrix::add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

	//These are additional parameters for an ROV
	_throttle_factor[motor_num] = throttle_fac;
	_forward_factor[motor_num] = forward_fac;
	_strafe_factor[motor_num] = strafe_fac;
}

