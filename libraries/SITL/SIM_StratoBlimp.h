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
  a stratospheric blimp simulator class
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_STRATOBLIMP_ENABLED

#include "SIM_Aircraft.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  a stratospheric blimp simulator
 */

class StratoBlimp : public Aircraft {
public:
    StratoBlimp(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new StratoBlimp(frame_str);
    }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);

private:
    void calculate_coefficients();
    void handle_motor(float throttle, float tilt, Vector3f &body_acc, Vector3f &rot_accel, float lateral_position);
    void get_drag(const Vector3f &velocity_linear,
                  const Vector3f &velocity_rot,
                  float altitude,
                  Vector3f &drag_linear, Vector3f &drag_rotaccel);
    float get_lift(float altitude);

    float air_density;
    float EAS2TAS;
    float drag_yaw;
    bool released;
    bool helper_balloon_attached = true;

    AP_Float mass;
    AP_Float helium_mass;
    AP_Float arm_length;
    AP_Float motor_thrust;
    AP_Float drag_fwd;
    AP_Float drag_side;
    AP_Float drag_up;
    AP_Float altitude_target;
    AP_Float target_climb_rate;
    AP_Float turn_rate;
    AP_Float motor_angle;
    AP_Float yaw_rate_max;
    AP_Float moi_roll;
    AP_Float moi_yaw;
    AP_Float moi_pitch;
    AP_Float center_of_lift;
    AP_Float center_of_drag;
    AP_Float free_lift_rate;
};

}

#endif // AP_SIM_STRATOBLIMP_ENABLED
