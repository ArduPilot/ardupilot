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

#include "SIM_Webots_Python.h"

#if HAL_SIM_WEBOTSPYTHON_ENABLED

#include <stdio.h>
#include <errno.h>

namespace SITL {

WebotsPython::WebotsPython(const char *frame_str) :
        Aircraft(frame_str),
        last_timestamp(0),
        socket_sitl{true}{

    // disable time sync and sensor smoothing to allow for faster than realtime simulation
    use_time_sync = false;   
    use_smoothing = false;

    printf("Starting SITL Webots\n");
}

/*
  Create and set in/out socket
*/
void WebotsPython::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Webots keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!socket_sitl.bind("0.0.0.0", port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _webots_address = address;
    _webots_port = port_out;
    printf("Setting Webots interface to %s:%d \n", _webots_address, _webots_port);
}

/*
  Decode and send SITL outputs to FDM aka Webots (scaled to be 0-1 instead of 1000-2000)
*/
void WebotsPython::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    for (unsigned i = 0; i < 16; ++i){
      pkt.motor_speed[i] = (input.servos[i]-1000) / 1000.0f;
    }
    socket_sitl.sendto(&pkt, sizeof(pkt), _webots_address, _webots_port);
}

/*
  Receive sensor data from Webots (the Flight Dynamics Model)
*/
void WebotsPython::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (socket_sitl.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch webots reset
        if (get_wall_time_us() > last_wall_time_us + WEBOTS_TIMEOUT_US) {
            last_timestamp = 0;
        }
    }

    const double deltat = pkt.timestamp - last_timestamp;  // in seconds
    if (deltat < 0) {  // don't use old packet
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
    dcm.from_euler(static_cast<float>(pkt.imu_orientation_rpy[0]),
                   static_cast<float>(pkt.imu_orientation_rpy[1]),
                   static_cast<float>(pkt.imu_orientation_rpy[2]));

    velocity_ef = Vector3f(static_cast<float>(pkt.velocity_xyz[0]),
                           static_cast<float>(pkt.velocity_xyz[1]),
                           static_cast<float>(pkt.velocity_xyz[2]));

    position = Vector3d(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);
    position.xy() += origin.get_distance_NE_double(home);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(static_cast<float>(1.0/deltat));
    }
    last_timestamp = pkt.timestamp;

}

/*
  Drain remaining data on the socket to prevent phase lag
 */
void WebotsPython::drain_sockets()
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
  Update the Webots simulation by one time step
 */
void WebotsPython::update(const struct sitl_input &input)
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


#endif  // HAL_SIM_WEBOTSPYTHON_ENABLED
