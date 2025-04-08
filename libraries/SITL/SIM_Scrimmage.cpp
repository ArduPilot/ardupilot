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
  simulator connector for Scrimmage simulator
*/

#include "SIM_Scrimmage.h"

#if HAL_SIM_SCRIMMAGE_ENABLED

#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

Scrimmage::Scrimmage(const char *_frame_str) :
    Aircraft(_frame_str),
    prev_timestamp_us(0),
    recv_sock(true),
    send_sock(true)
{
}

void Scrimmage::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    fdm_port_in = port_in;
    fdm_port_out = port_out;
    fdm_address = address;
    printf("ArduPilot sending to scrimmage on %s:%d\n",fdm_address, fdm_port_out);
    printf("ArduPilot listening to scrimmage on %s:%d\n",fdm_address, fdm_port_in);

    recv_sock.bind(fdm_address, fdm_port_in);

    recv_sock.reuseaddress();
    recv_sock.set_blocking(false);

    send_sock.reuseaddress();
    send_sock.set_blocking(false);
}

void Scrimmage::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;

    for (int i = 0; i < MAX_NUM_SERVOS; i++) {
        pkt.servos[i] = input.servos[i];
    }
    send_sock.sendto(&pkt, sizeof(servo_packet), fdm_address, fdm_port_out);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Scrimmage::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    // wait for packet from scrimmage
    while (recv_sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt));

    // auto-adjust to simulation frame rate
    uint64_t dt_us = 0;
    if (pkt.timestamp_us > prev_timestamp_us)
        dt_us = pkt.timestamp_us - prev_timestamp_us;
    time_now_us += dt_us;

    float dt_inv = 1.0e6 / ((float)dt_us);
    if ( dt_inv > 100) {
        adjust_frame_time(dt_inv);
    }
    prev_timestamp_us = pkt.timestamp_us;

    // dcm_bl: dcm from body to local frame
    dcm.from_euler(pkt.roll, pkt.pitch, pkt.yaw);
    dcm.normalize();

    // subtract gravity to get specific force measuremnt of the IMU
    accel_body = Vector3f(pkt.xAccel, pkt.yAccel, pkt.zAccel) - dcm.transposed()*Vector3f(0.0f, 0.0f, GRAVITY_MSS);
    gyro = Vector3f(pkt.rollRate, pkt.pitchRate, pkt.yawRate);

    velocity_ef = Vector3f(pkt.speedN, pkt.speedE, pkt.speedD);

    location.lat = pkt.latitude * 1.0e7;
    location.lng = pkt.longitude * 1.0e7;
    location.alt = pkt.altitude * 1.0e2;
    position.z = (home.alt - location.alt) * 1.0e-2;


    // velocity relative to air mass, in earth frame TODO
    velocity_air_ef = velocity_ef - wind_ef;

    // velocity relative to airmass in body frame TODO
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    battery_voltage = 0;
    battery_current = 0;
    rpm[0] = 0;
    rpm[1] = 0;

    airspeed = pkt.airspeed;
    airspeed_pitot = pkt.airspeed;
}

/*
  update the Scrimmage simulation by one time step
 */
void Scrimmage::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_mag_field_bf();
}

} // namespace SITL

#endif
