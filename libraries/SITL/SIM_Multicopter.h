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
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"

#include "SIM_Sprayer.h"
#include "SIM_Gripper_Servo.h"
#include "SIM_Gripper_EPM.h"

namespace SITL {

/*
  a multicopter simulator
 */
class MultiCopter : public Aircraft {
public:
    MultiCopter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new MultiCopter(home_str, frame_str);
    }

protected:
    // calculate rotational and linear accelerations
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Frame *frame;

    // The numbers below are the pwm output channels with "0" meaning the first output (aka RC1)
    Sprayer sprayer{6, 7};
    Gripper_Servo gripper{8};
    Gripper_EPM gripper_epm{9};

    float gross_mass() const override;

};

}
