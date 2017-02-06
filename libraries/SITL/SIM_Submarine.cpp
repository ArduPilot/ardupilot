/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  Submarine simulator class
*/

#include "SIM_Submarine.h"
#include <AP_Motors/AP_Motors.h>
#include "Frame_Vectored.h"

#include <stdio.h>

using namespace SITL;

static Thruster vectored_thrusters[] =
{
	    Thruster(0, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, MOT_1_THROTTLE_FACTOR, MOT_1_FORWARD_FACTOR, MOT_1_STRAFE_FACTOR),
	    Thruster(1, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, MOT_2_THROTTLE_FACTOR, MOT_2_FORWARD_FACTOR, MOT_2_STRAFE_FACTOR),
	    Thruster(2, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, MOT_3_THROTTLE_FACTOR, MOT_3_FORWARD_FACTOR, MOT_3_STRAFE_FACTOR),
	    Thruster(3, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, MOT_4_THROTTLE_FACTOR, MOT_4_FORWARD_FACTOR, MOT_4_STRAFE_FACTOR),
	    Thruster(4, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, MOT_5_THROTTLE_FACTOR, MOT_5_FORWARD_FACTOR, MOT_5_STRAFE_FACTOR),
	    Thruster(5, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, MOT_6_THROTTLE_FACTOR, MOT_6_FORWARD_FACTOR, MOT_6_STRAFE_FACTOR)

};

Submarine::Submarine(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
//    frame = Frame::find_frame(frame_str);
//    if (frame == NULL) {
//        printf("Frame '%s' not found", frame_str);
//        exit(1);
//    }
//    if (strstr(frame_str, "-fast")) {
//        frame->init(1.5, 0.5, 85, 4*radians(360));
//    } else {
//        frame->init(1.5, 0.51, 15, 4*radians(360));
//    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NONE;
}

// calculate rotational and linear accelerations
void Submarine::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
//    frame->calculate_forces(*this, input, rot_accel, body_accel);
	rot_accel = Vector3f(0,0,0);
	body_accel = Vector3f(0,0,-9.8);
	for(int i = 0; i < 6; i++) {
		Thruster t = vectored_thrusters[i];
		int16_t pwm = input.servos[t.servo];
		float output = 0;
		if(pwm < 2000 && pwm > 1000) {
			output = (pwm - 1500) / 400.0f; // range -1~1
		}

//		printf("input [%d] : %f\n", t.servo, input);
		body_accel += t.linear * output;
		rot_accel += t.rotational * output;
	}

//	printf("\n\n");
}
    
/*
  update the Submarine simulation by one time step
 */
void Submarine::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();

    // update magnetic field
    update_mag_field_bf();
}


