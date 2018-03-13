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
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Gazebo.h"

#include <stdio.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Gazebo::Gazebo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0),
    socket_sitl{true}
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    fprintf(stdout, "Starting SITL Gazebo\n");
}

/*
  Create and set in/out socket
*/
void Gazebo::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    if (!socket_sitl.bind("0.0.0.0", port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Abording launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _gazebo_address = address;
    _gazebo_port = port_out;
    printf("Setting Gazebo interface to %s:%d \n", _gazebo_address, _gazebo_port);
}

/*
  decode and send servos
*/
void Gazebo::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    // should rename servo_command
    // 16 because struct sitl_input.servos is 16 large in SIM_Aircraft.h
    for (unsigned i = 0; i < 16; ++i)
    {
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_sitl.sendto(&pkt, sizeof(pkt), _gazebo_address, _gazebo_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Gazebo::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch gazebo reset
        if (get_wall_time_us() > last_wall_time_us + GAZEBO_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    const double deltat = pkt.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old paquet
        time_now_us += 1;
        return;
    }
    // get imu stuff
    accel_body = Vector3f(static_cast<float>(pkt.imu_linear_acceleration_xyz[0]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[1]),
                          static_cast<float>(pkt.imu_linear_acceleration_xyz[2]));

    gyro = Vector3f(static_cast<float>(pkt.imu_angular_velocity_rpy[0]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[1]),
                    static_cast<float>(pkt.imu_angular_velocity_rpy[2]));

    // compute dcm from imu orientation
    Quaternion quat(static_cast<float>(pkt.imu_orientation_quat[0]),
                    static_cast<float>(pkt.imu_orientation_quat[1]),
                    static_cast<float>(pkt.imu_orientation_quat[2]),
                    static_cast<float>(pkt.imu_orientation_quat[3]));
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(static_cast<float>(pkt.velocity_xyz[0]),
                           static_cast<float>(pkt.velocity_xyz[1]),
                           static_cast<float>(pkt.velocity_xyz[2]));

    position = Vector3f(static_cast<float>(pkt.position_xyz[0]),
                        static_cast<float>(pkt.position_xyz[1]),
                        static_cast<float>(pkt.position_xyz[2]));


    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.timestamp;

}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void Gazebo::drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do {
        received = socket_sitl.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);

}

/*
  update the Gazebo simulation by one time step
 */
void Gazebo::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
}

}  // namespace SITL
