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
  gimbal simulator class
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_GIMBAL_ENABLED

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

namespace SITL {

class Gimbal {
public:

    void update(const class Aircraft &aircraft);
    void set_demanded_rates(const Vector3f &rates) {
        demanded_angular_rate = rates;
    }

    void get_deltas(Vector3f &_delta_angle, Vector3f &_delta_velocity, uint32_t &_delta_time_us);
    void get_joint_angles(Vector3f &_angles) { _angles = joint_angles; }

private:

    // rotation matrix (gimbal body -> earth)
    Matrix3f dcm;
    bool init_done;

    // time of last update
    uint32_t last_update_us;

    // true angular rate of gimbal in body frame (rad/s)
    Vector3f gimbal_angular_rate;

    // observed angular rate (including biases)
    Vector3f gyro;

    /* joint angles, in radians. in yaw/roll/pitch order. Relative to fwd.
       So 0,0,0 points forward.
       Pi/2,0,0 means pointing right
       0, Pi/2, 0 means pointing fwd, but rolled 90 degrees to right
       0, 0, -Pi/2, means pointing down
    */
    Vector3f joint_angles;

    // physical constraints on joint angles in (roll, pitch, azimuth) order
    Vector3f lower_joint_limits{
        float(radians(-40)),
        float(radians(-135)),
        float(radians(-7.5))
    };
    Vector3f upper_joint_limits{
        float(radians(40)),
        float(radians(45)),
        float(radians(7.5))
    };

    const float travelLimitGain = 20;

    // true gyro bias
    Vector3f true_gyro_bias;

    // time since delta angles/velocities returned
    uint32_t delta_start_us;

    // integral of gyro vector over last time interval. In radians
    Vector3f delta_angle;

    // integral of accel vector over last time interval. In m/s
    Vector3f delta_velocity;

    /*
      control variables from the vehicle
    */
    // angular rate in rad/s. In body frame of gimbal
    Vector3f demanded_angular_rate;

    // gyro bias provided by EKF on vehicle. In rad/s.
    // Should be subtracted from the gyro readings to get true body
    // rotatation rates
    // Vector3f supplied_gyro_bias;
};

}  // namespace SITL

#endif  // AP_SIM_GIMBAL_ENABLED
