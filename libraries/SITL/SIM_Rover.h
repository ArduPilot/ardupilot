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
  rover simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a rover simulator
 */
class SimRover : public Aircraft {
public:
    SimRover(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new SimRover(frame_str);
    }

private:
    static constexpr float MAX_YAW_RATE = 360.0;  // MAX 360 deg/s yaw rate speed to not nuke the ekf gsf
    float max_speed = 20.0f;            // vehicle's maximum forward speed in m/s
    float max_accel = 10.0f;            // vehicle's maximum forward acceleration in m/s/s
    float max_wheel_turn = 35.0f;       // ackermann steering vehicle's maximum steering angle
    float turning_circle = 1.8f;        // ackermann steering vehicle's minimum turn diameter in meters
    float skid_turn_rate = 140.0f;      // skid-steering vehicle's maximum turn rate in deg/sec
    bool skid_steering;                 // true if this vehicle is a skid-steering vehicle

    // vectored thrust related members
    bool vectored_thrust;                   // true if vehicle uses vectored thrust (i.e. steering controls direction of thrust)
    float vectored_angle_max = 90.0f;       // maximum angle (in degrees) to which thrust can be turned
    float vectored_turn_rate_max = 90.0f;   // maximum turn rate (in deg/sec) with full throttle angled at 90deg

    float turn_circle(float steering) const;
    float calc_yaw_rate(float steering, float speed);
    float calc_lat_accel(float steering_angle, float speed);
};

} // namespace SITL
