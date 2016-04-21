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
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
void Motor::calculate_forces(const Aircraft::sitl_input &input,
                             const float thrust_scale,
                             uint8_t motor_offset,
                             Vector3f &rot_accel,
                             Vector3f &thrust) const
{
    float motor_speed = constrain_float((input.servos[motor_offset+servo]-1100)/900.0, 0, 1);
    rot_accel.x = -radians(5000.0) * sinf(radians(angle)) * motor_speed;
    rot_accel.y =  radians(5000.0) * cosf(radians(angle)) * motor_speed;
    rot_accel.z = yaw_factor * motor_speed * radians(400.0);
    thrust(0, 0, motor_speed * thrust_scale); // newtons
}

