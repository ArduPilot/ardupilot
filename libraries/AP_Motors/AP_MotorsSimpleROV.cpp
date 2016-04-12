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
 *       AP_MotorsSimple ROV.cpp
 *
 *
 */

#include "AP_MotorsSimpleROV.h"

// setup_motors - configures the motors for the BlueROV
void AP_MotorsSimpleROV::setup_motors()
{
	// call parent
	AP_Motors6DOF::setup_motors();

	// hard coded config for Simple ROV
	float
		// Right forward thruster
		MOT_1_ROLL_FACTOR = 0,
		MOT_1_PITCH_FACTOR = 0,
		MOT_1_YAW_FACTOR = -1,
		MOT_1_THROTTLE_FACTOR = 0,
		MOT_1_FORWARD_FACTOR = 1,
		MOT_1_STRAFE_FACTOR = 0,

		// Left forward thruster
		MOT_2_ROLL_FACTOR = 0,
		MOT_2_PITCH_FACTOR = 0,
		MOT_2_YAW_FACTOR = 1,
		MOT_2_THROTTLE_FACTOR = 0,
		MOT_2_FORWARD_FACTOR = 1,
		MOT_2_STRAFE_FACTOR = 0,

		// Right vertical thruster
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = 0,
		MOT_3_THROTTLE_FACTOR = -1,
		MOT_3_FORWARD_FACTOR = 0,
		MOT_3_STRAFE_FACTOR = 0,

		// Left Vertical thruster
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = 0,
		MOT_4_THROTTLE_FACTOR = -1,
		MOT_4_FORWARD_FACTOR = 0,
		MOT_4_STRAFE_FACTOR = 0,

		// Lateral thruster
		MOT_5_ROLL_FACTOR = 0,
		MOT_5_PITCH_FACTOR = 0,
		MOT_5_YAW_FACTOR = 0,
		MOT_5_THROTTLE_FACTOR = 0,
		MOT_5_FORWARD_FACTOR = 0,
		MOT_5_STRAFE_FACTOR = 1;

	add_motor_raw_6dof(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR,1);
	add_motor_raw_6dof(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR,2);
	add_motor_raw_6dof(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR,3);
	add_motor_raw_6dof(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR,4);
	add_motor_raw_6dof(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR,5);
}
