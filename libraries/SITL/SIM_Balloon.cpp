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
  balloon simulator class
*/

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "SIM_Balloon.h"
#include <stdio.h>

/*
  constructor
 */
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
    if (!released && input.servos[6] > 1800) {
        ::printf("Balloon released\n");
        released = true;
    }

    if (!burst && input.servos[7] > 1800) {
        ::printf("Balloon burst\n");
        burst = true;
    }

    float delta_time = frame_time_us * 1.0e-6f;

    // rotational air resistance
    Vector3f rot_accel = -gyro * radians(400) / terminal_rotation_rate;

    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // air resistance
    Vector3f air_resistance = -velocity_ef * (GRAVITY_MSS/terminal_velocity);

    float lift_accel = 0;
    if (!burst && released) {
        float air_resistance_at_climb_rate = climb_rate * (GRAVITY_MSS/terminal_velocity);
        lift_accel = air_resistance_at_climb_rate + GRAVITY_MSS * dcm.c.z;
    }

    accel_body = Vector3f(0, 0, -lift_accel);
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);
    accel_earth += air_resistance;

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground(position) && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // add some noise
    add_noise(velocity_ef.length() / terminal_velocity);

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    Vector3f old_position = position;
    position += velocity_ef * delta_time;

    if (position.z < -burst_altitude) {
        ::printf("Balloon burst at %.1f\n", -position.z);
        burst = true;
    }

    // assume zero wind for now
    airspeed = velocity_ef.length();

    // constrain height to the ground
    if (on_ground(position)) {
        if (!on_ground(old_position)) {
            printf("Hit ground at %f m/s\n", velocity_ef.z);

            velocity_ef.zero();

            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            dcm.from_euler(0, 0, y);

            position.z = -(ground_level + frame_height - home.alt*0.01f);
        }
    }

    // update lat/lon/altitude
    update_position();
}
#endif // CONFIG_HAL_BOARD
