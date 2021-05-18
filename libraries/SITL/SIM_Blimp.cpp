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
  Blimp simulator class
*/

#include "SIM_Blimp.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

Blimp::Blimp(const char *frame_str) :
    Aircraft(frame_str)
{
    frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NONE;
    lock_step_scheduled = true;
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    // float fin_back  = filtered_servo_angle(input, 0);
    // float fin_front = filtered_servo_angle(input, 1);
    // float fin_right = filtered_servo_angle(input, 2);
    // float fin_left  = filtered_servo_angle(input, 3);

    // ::printf("FINS (%.1f %.1f %.1f %.1f)\n",
    //          fin_back, fin_front, fin_right, fin_left);
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
