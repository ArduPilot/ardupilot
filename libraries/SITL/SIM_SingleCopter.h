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
  single copter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include <AP_Motors/AP_Motors.h>

namespace SITL {

/*
  a single copter simulator
 */
class SingleCopter : public Aircraft {
public:
    SingleCopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new SingleCopter(home_str, frame_str);
    }

private:
    float terminal_rotation_rate = 4*radians(360.0f);
    float hover_throttle = 0.65f;
    float terminal_velocity = 40;
    float rotor_rot_accel = radians(20) * AP_MOTORS_MATRIX_YAW_FACTOR_CW;
    float roll_rate_max = radians(700);
    float pitch_rate_max = radians(700);
    float yaw_rate_max = radians(700);
    float thrust_scale;

    enum {
        FRAME_SINGLE,
        FRAME_COAX,
    } frame_type;
};

} // namespace SITL
