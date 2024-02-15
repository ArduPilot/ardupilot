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
  simulator connection for Webots 2023a
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_WEBOTSPYTHON_ENABLED
#define HAL_SIM_WEBOTSPYTHON_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_WEBOTSPYTHON_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket_native.h>

namespace SITL {

/*
  WebotsPython simulator
 */
class WebotsPython : public Aircraft {
public:
    WebotsPython(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new WebotsPython(frame_str);
    }

    /*  Create and set in/out socket for Webots simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    /// @brief packet to send to Webots
    /// python struct code 'f'*16
    struct servo_packet {
      float motor_speed[16];
    };

    /// @brief packet from Webots (the Flight Dynamics Model)
    /// python struct code 'd'*(1+3+3+3+3+3)
    struct fdm_packet {
      double timestamp;  // in seconds
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_rpy[3];
      double velocity_xyz[3];
      double position_xyz[3];
    };

    /// @brief receive sensor packet from Webots
    void recv_fdm(const struct sitl_input &input);

    /// @brief send servo packet to Webots
    void send_servos(const struct sitl_input &input);

    /// @brief drain all pending packets from Webots
    void drain_sockets();

    double last_timestamp;

    SocketAPM_native socket_sitl;
    const char* _webots_address = "127.0.0.1";
    int _webots_port = 9002;
    static const uint64_t WEBOTS_TIMEOUT_US = 5000000;
};

}  // namespace SITL


#endif  // HAL_SIM_WEBOTSPYTHON_ENABLED
