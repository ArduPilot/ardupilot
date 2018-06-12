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
  balloon simulator class
*/

#include "SIM_Balloon.h"

#include <stdio.h>

namespace SITL {

Balloon::Balloon(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    mass = 5.0f;
}

/*
  update the balloon simulation by one time step
 */
void Balloon::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    if (!released && input.servos[6] > 1800) {
        ::printf("Balloon released\n");
        released = true;
    }

    if (!burst && input.servos[7] > 1800) {
        ::printf("Balloon burst\n");
        burst = true;
    }

    // rotational air resistance
    Vector3f rot_accel = -gyro * radians(400) / terminal_rotation_rate;

    // air resistance
    Vector3f air_resistance = -velocity_air_ef * (GRAVITY_MSS/terminal_velocity);

    float lift_accel = 0;
    if (!burst && released) {
        float air_resistance_at_climb_rate = climb_rate * (GRAVITY_MSS/terminal_velocity);
        lift_accel = air_resistance_at_climb_rate + GRAVITY_MSS * dcm.c.z;
    }

    accel_body = Vector3f(0, 0, -lift_accel);
    accel_body += dcm.transposed() * air_resistance;
    
    update_dynamics(rot_accel);

    if (position.z < -burst_altitude) {
        ::printf("Balloon burst at %.1f\n", -position.z);
        burst = true;
    }
    
    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
