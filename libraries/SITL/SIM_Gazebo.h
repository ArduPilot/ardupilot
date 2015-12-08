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

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

#define NB_GAZEBO_SERVOS    16

/*
  Gazebo simulator
 */
class Gazebo : public Aircraft {
public:
    Gazebo(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Gazebo(home_str, frame_str);
    }

    /* Calls the ROS start */
    virtual void finalize_creation(void);

    /* fill a sitl_fdm structure from the simulator state */
    void fill_fdm(struct sitl_fdm &fdm) const;


private:
    /*
      packet sent to Gazebo
      Make sure the exact same message struct is defined in the Gazebo plugin
     */
    struct servo_packet {
      float motor_speed[NB_GAZEBO_SERVOS];    // ranges from 0 (no rotation) to 1 (full throttle)
    };

    /*
      reply packet sent from Gazebo to ArduPilot
      Make sure the exact same message struct is defined in the Gazebo plugin
     */
    struct fdm_packet {
      double timestamp;                             // [seconds] simulation time
      double imu_angular_velocity_rpy[3];           // [rad/s]
      double imu_linear_acceleration_xyz[3];        // [m/s/s] in NED, body frame
      double imu_orientation_quat[4];               // rotation quaternion, APM conventions, from body to earth
      double velocity_xyz[3];                       // [m/s] in NED
      double position_xyz[3];                       // [m] in NED, from Gazebo's map origin (0,0,0)
      double position_latlonalt[3];                 // [degrees], altitude is Up

      // You can add here extra sensors to pass along
      double sonar_down;                            // [m] downward facing range finder
      double sonar_front;                           // [m]
    };

    /*
      Describes the Gazebo send/receive link status.
      It is used to avoid repeating identical error messages on each update.
     */
    enum link_status {
        RUNNING = 0,            // send/receive are currently running
        NOT_INITIALIZED,        // when any of the send/receive socket is not yet opened
        ERR_BYTES_LOST,         // when the send socket fails to transmit all message bytes
        ERR_FATAL               // other errors, like closed connection
    };


    // -----------------------------------------
    // ROS/Gazebo launch methods / variables

    /* Starts ROS as a child process in a new terminal */
    void start_ros_gazebo(void);

    /* Indicates the success or failure of starting ROS/Gazebo (if defined to be launched by Ardupilot) */
    bool _is_gazebo_started;


    // -----------------------------------------
    // Communication methods / variables

    bool open_servos_socket();
    bool open_fdm_socket();

    void recv_fdm(const struct sitl_input &input);
    void send_servos_heli(const struct sitl_input &input);              // Not yet implemented
    void send_servos_fixed_wing(const struct sitl_input &input);        // Not yet implemented
    void send_servos(const struct sitl_input &input);
    void drain_servos_socket();

    /* Socket to send commands to the UAV in the Gazebo simulation (motor commands, ...) */
    SocketAPM _sock_servos_to_gazebo;
    /* Socket to receive sensor measurements from the Gazebo simulation (IMU, GPS, ...) */
    SocketAPM _sock_fdm_from_gazebo;

    bool _is_servos_socket_open;
    bool _is_fdm_socket_open;

    link_status _link_status;


    // -----------------------------------------
    // Other methods / variables

    double _last_timestamp;     // [seconds] date of the last fdm message received

protected:

    /* fill a sitl_fdm_extras with the current extras sensors measures, for the
       simulators that support them. */
    virtual void fill_fdm_extras(struct sitl_fdm_extras &fdm_extras) const;

    // You can add here data handling for extra sensors not covered by SIM_Aircraft

    /* Whether the downward facing range finder is simulated or not */
    bool _is_sonar_down_present;
    /* Last range measurement by the downward facing range finder */
    double _sonar_down;
};

} // namespace SITL
