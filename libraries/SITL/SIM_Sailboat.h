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
  sailboat simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a sailboat simulator
 */
class Sailboat : public Aircraft {
public:
    Sailboat(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Sailboat(home_str, frame_str);
    }

private:

    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag);
    float turn_circle(float steering);
    float calc_yaw_rate(float steering, float speed);
    float calc_lat_accel(float steering_angle, float speed);

    float max_wheel_turn;
    float turning_circle;

    // 10 point curves for lift and drag.  index is angle/10deg
    // angle-of-attack      0      10     20     30     40     50     60     70     80     90     100    110    120    130    140    150    160    170+
    float lift_curve[18] = {0.00f, 0.00f, 0.80f, 1.00f, 0.95f, 0.75f, 0.60f, 0.40f, 0.20f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};
    float drag_curve[18] = {0.10f, 0.10f, 0.12f, 0.15f, 0.20f, 0.27f, 0.35f, 0.50f, 0.70f, 1.00f, 0.70f, 0.50f, 0.35f, 0.27f, 0.20f, 0.15f, 0.12f, 0.10f};
};

} // namespace SITL
