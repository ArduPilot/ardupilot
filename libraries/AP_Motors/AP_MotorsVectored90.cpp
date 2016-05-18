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
 *       AP_MotorsVectored90.cpp
 *
 *
 */

#include "AP_MotorsVectored90.h"

/*
|5    <1>    6|
|             |
|3           4|
|             |
|7    <2>    8|
*/

// setup_motors - configures the motors for the frame
void AP_MotorsVectored90::setup_motors()
{
	// call parent
	AP_Motors6DOF::setup_motors();

	// hard coded config for 90 degree vectored 6DOF frame
	float
		//Lateral
		MOT_1_ROLL_FACTOR = 0,
		MOT_1_PITCH_FACTOR = 0,
		MOT_1_YAW_FACTOR = 0,
		MOT_1_THROTTLE_FACTOR = 0,
		MOT_1_FORWARD_FACTOR = 0,
		MOT_1_STRAFE_FACTOR = 1.0,

		//Lateral
		MOT_2_ROLL_FACTOR = 0,
		MOT_2_PITCH_FACTOR = 0,
		MOT_2_YAW_FACTOR = 0,
		MOT_2_THROTTLE_FACTOR = 0,
		MOT_2_FORWARD_FACTOR = 0,
		MOT_2_STRAFE_FACTOR = 1.0,

		//Forward/yaw
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = 1.0,
		MOT_3_THROTTLE_FACTOR = 0,
		MOT_3_FORWARD_FACTOR = 1.0,
		MOT_3_STRAFE_FACTOR = 0,

		//Forward/yaw
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = -1.0,
		MOT_4_THROTTLE_FACTOR = 0,
		MOT_4_FORWARD_FACTOR = 1.0,
		MOT_4_STRAFE_FACTOR = 0,

		//Front Left
		MOT_5_ROLL_FACTOR = 1.0,
		MOT_5_PITCH_FACTOR = 1.0,
		MOT_5_YAW_FACTOR = 0,
		MOT_5_THROTTLE_FACTOR = 1.0,
		MOT_5_FORWARD_FACTOR = 0,
		MOT_5_STRAFE_FACTOR = 0,

		//Front Right
		MOT_6_ROLL_FACTOR = -1.0,
		MOT_6_PITCH_FACTOR = 1.0,
		MOT_6_YAW_FACTOR = 0,
		MOT_6_THROTTLE_FACTOR = 1.0,
		MOT_6_FORWARD_FACTOR = 0,
		MOT_6_STRAFE_FACTOR = 0,

		//Back Left
		MOT_7_ROLL_FACTOR = 1.0,
		MOT_7_PITCH_FACTOR = -1.0,
		MOT_7_YAW_FACTOR = 0,
		MOT_7_THROTTLE_FACTOR = 1.0,
		MOT_7_FORWARD_FACTOR = 0,
		MOT_7_STRAFE_FACTOR = 0,

		//Back Right
		MOT_8_ROLL_FACTOR = -1.0,
		MOT_8_PITCH_FACTOR = -1.0,
		MOT_8_YAW_FACTOR = 0,
		MOT_8_THROTTLE_FACTOR = 1.0,
		MOT_8_FORWARD_FACTOR = 0,
		MOT_8_STRAFE_FACTOR = 0;

	add_motor_raw_6dof(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR,1);
	add_motor_raw_6dof(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR,2);
	add_motor_raw_6dof(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR,3);
	add_motor_raw_6dof(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR,4);
	add_motor_raw_6dof(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR,5);
	add_motor_raw_6dof(AP_MOTORS_MOT_6, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, MOT_6_THROTTLE_FACTOR, MOT_6_FORWARD_FACTOR, MOT_6_STRAFE_FACTOR,6);
	add_motor_raw_6dof(AP_MOTORS_MOT_7, MOT_7_ROLL_FACTOR, MOT_7_PITCH_FACTOR, MOT_7_YAW_FACTOR, MOT_7_THROTTLE_FACTOR, MOT_7_FORWARD_FACTOR, MOT_7_STRAFE_FACTOR,7);
	add_motor_raw_6dof(AP_MOTORS_MOT_8, MOT_8_ROLL_FACTOR, MOT_8_PITCH_FACTOR, MOT_8_YAW_FACTOR, MOT_8_THROTTLE_FACTOR, MOT_8_FORWARD_FACTOR, MOT_8_STRAFE_FACTOR,8);

}
