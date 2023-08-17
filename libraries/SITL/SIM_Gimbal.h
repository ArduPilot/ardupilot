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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_GIMBAL_ENABLED
#define HAL_SIM_GIMBAL_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if HAL_SIM_GIMBAL_ENABLED

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

class Gimbal {
public:
    Gimbal(const struct sitl_fdm &_fdm);
    void update(void);

private:
    const struct sitl_fdm &fdm;
    const char *target_address;
    const uint16_t target_port;

    // rotation matrix (gimbal body -> earth)
    Matrix3f dcm;

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
    Vector3f lower_joint_limits;
    Vector3f upper_joint_limits;

    const float travelLimitGain;

    // true gyro bias
    Vector3f true_gyro_bias;

    // reporting variables. gimbal pushes these to vehicle code over
    // MAVLink at approx 100Hz

    // reporting period in ms
    const float reporting_period_ms;

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
    Vector3f supplied_gyro_bias;

    uint32_t last_report_us;
    uint32_t last_heartbeat_ms;
    bool seen_heartbeat;
    bool seen_gimbal_control;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;

    SocketAPM mav_socket;
    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;

    uint32_t param_send_last_ms;
    uint8_t param_send_idx;

    void send_report(void);
    void param_send(const struct gimbal_param *p);
    struct gimbal_param *param_find(const char *name);
};

}  // namespace SITL

#endif  // HAL_SIM_GIMBAL_ENABLED
