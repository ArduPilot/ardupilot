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
  simple quadplane simulator class
*/

#include "SIM_QuadPlane.h"

#include <stdio.h>

using namespace SITL;

static const Motor quad_motors[4] =
{
    Motor(45,  false,  5),
    Motor(225, false,  6),
    Motor(315, true,   7),
    Motor(135, true,   8)
};

static Frame quad_frame("x", 4, quad_motors);

QuadPlane::QuadPlane(const char *home_str, const char *frame_str) :
    Plane(home_str, frame_str)
{
    frame = &quad_frame;
    frame->init(mass, 0.51, 50, 20*radians(360));
}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    float delta_time = frame_time_us * 1.0e-6f;

    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel, accel_body);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;
    printf("%u %u %u %u\n",
           input.servos[5],
           input.servos[6],
           input.servos[7],
           input.servos[8]);
    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body);

    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;
    
    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground(position) && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    Vector3f old_position = position;
    position += velocity_ef * delta_time;

    // assume zero wind for now
    airspeed = velocity_ef.length();

    // constrain height to the ground
    if (on_ground(position)) {
        if (!on_ground(old_position)) {
            printf("Hit ground at %f m/s\n", velocity_ef.z);
            position.z = -(ground_level + frame_height - home.alt*0.01f);
        }
    }

    // update lat/lon/altitude
    update_position();
}
