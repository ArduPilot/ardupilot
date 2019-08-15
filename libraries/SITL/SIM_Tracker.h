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
  antenna-tracker simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a antenna tracker simulator
 */
class Tracker : public Aircraft {
public:
    Tracker(const char *frame_str);
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Tracker(frame_str);
    }

private:

    const bool onoff = false;
    const float yawrate = 9.0f;
    const float pitchrate = 1.0f;
    const float pitch_range = 45;
    const float yaw_range = 170;
    const float zero_yaw = 270;  // yaw direction at startup
    const float zero_pitch = 10; // pitch at startup
    uint64_t last_debug_us = 0;

    float pitch_input;
    float yaw_input;
    float yaw_current_relative;
    float pitch_current_relative;

    void update_position_servos(float delta_time, float &yaw_rate, float &pitch_rate);
    void update_onoff_servos(float &yaw_rate, float &pitch_rate);
};

} // namespace SITL
