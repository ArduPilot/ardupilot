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
  simulator connection for ardupilot version of Gazebo
*/

#ifndef _SIM_GAZEBO_H
#define _SIM_GAZEBO_H

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>

/*
  Gazebo simulator
 */
class Gazebo : public Aircraft
{
public:
    Gazebo(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Gazebo(home_str, frame_str);
    }

private:
    /*
      packet sent to Gazebo
     */
    struct servo_packet {
      float motor_speed[4];
    };
    
    /*
      reply packet sent from Gazebo to ArduPilot
     */
    struct fdm_packet {
      double timestamp;
      double imu_angular_velocity_rpy[3];
      double imu_linear_acceleration_xyz[3];
      double imu_orientation_quat[4];
      double velocity_xyz[3];
      double position_xyz[3];
    };

    void send_servos_heli(const struct sitl_input &input);
    void send_servos_fixed_wing(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    double last_timestamp;
    SocketAPM sock;
};


#endif // _SIM_GAZEBO_H
