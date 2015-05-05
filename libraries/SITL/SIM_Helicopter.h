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
  helicopter simulator class
*/

#ifndef _SIM_HELICOPTER_H
#define _SIM_HELICOPTER_H

#include "SIM_Aircraft.h"

/*
  a helicopter simulator
 */
class Helicopter : public Aircraft
{
public:
    Helicopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Helicopter(home_str, frame_str);
    }

private:
    float terminal_rotation_rate;
    float hover_throttle;
    float terminal_velocity;
    float hover_lean;
    float yaw_zero;
    float rotor_rot_accel;
    float roll_rate_max;
    float pitch_rate_max;
    float yaw_rate_max;
    float rsc_setpoint;
    float thrust_scale;
    float tail_thrust_scale;
};


#endif // _SIM_HELICOPTER_H
