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
    Sailboat(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Sailboat(frame_str);
    }

    bool on_ground() const override {return true;};

protected:
    bool motor_connected;       // true if this frame has a motor
    float sail_area; // 1.0 for normal area

private:

    // calculate the lift and drag as values from 0 to 1 given an apparent wind speed in m/s and angle-of-attack in degrees
    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const;

    // return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
    float get_turn_circle(float steering) const;

    // return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
    float get_yaw_rate(float steering, float speed) const;

    // return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
    float get_lat_accel(float steering, float speed) const;

    // simulate waves and swell
    void update_wave(float delta_time);

    float steering_angle_max;   // vehicle steering mechanism's max angle in degrees
    float turning_circle;       // vehicle minimum turning circle diameter in meters

    // lift and drag curves.  index is angle/10deg
    // angle-of-attack            0      10     20     30     40     50     60     70     80     90      100     110     120     130     140     150     160     170+
    const float lift_curve[18] = {0.00f, 0.50f, 1.00f, 1.10f, 0.95f, 0.75f, 0.60f, 0.40f, 0.20f, 0.00f, -0.20f, -0.40f, -0.60f, -0.75f, -0.95f, -1.10f, -1.00f, -0.50f};
    const float drag_curve[18] = {0.10f, 0.10f, 0.20f, 0.40f, 0.80f, 1.20f, 1.50f, 1.70f, 1.90f, 1.95f,  1.90f,  1.70f,  1.50f,  1.20f,  0.80f,  0.40f,  0.20f,  0.10f};

    const float mass = 2.0f;

    Vector3f velocity_ef_water; // m/s
    Vector3f wave_gyro;         // rad/s
    float wave_heave;           // m/s/s
    float wave_phase;           // rads
};

} // namespace SITL
