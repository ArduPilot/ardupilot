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

QuadPlane::QuadPlane(const char *home_str, const char *frame_str) :
    Plane(home_str, frame_str)
{
    // see if we want a tiltrotor
    if (strstr(frame_str, "-tilt")) {
        tiltrotors = true;
    }
    // default to X frame
    const char *frame_type = "x";

    if (strstr(frame_str, "-octa-quad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octa")) {
        frame_type = "octa";
    } else if (strstr(frame_str, "-hexax")) {
        frame_type = "hexax";
    } else if (strstr(frame_str, "-hexa")) {
        frame_type = "hexa";
    } else if (strstr(frame_str, "-plus")) {
        frame_type = "+";
    }
    frame = Frame::find_frame(frame_type);
    if (frame == nullptr) {
        printf("Failed to find frame '%s'\n", frame_type);
        exit(1);
    }

    // leave first 4 servos free for plane
    frame->motor_offset = 4;

    // we use zero terminal velocity to let the plane model handle the drag
    frame->init(mass, 0.51, 0, 0);

}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel, accel_body);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;

    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body);

    if (tiltrotors) {
        // get rotor tilt from channel 9
        float tilt = constrain_float(-90 * (input.servos[8]-1000)*0.001, -90, 0);

        // rotate the rotational accel and body accel from multicopter
        // model to take account of rotor tilt
        Matrix3f rotation;
        rotation.identity();
        rotation.rotate(Vector3f(0, radians(tilt), 0));
        quad_rot_accel = rotation * quad_rot_accel;
        quad_accel_body = rotation * quad_accel_body;
    }
    
    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();
}
