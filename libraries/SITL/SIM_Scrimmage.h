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
  simulator connection for Scrimmage Simulator
*/

#pragma once

#include <string>

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a last_letter simulator
 */
class Scrimmage : public Aircraft {
public:
    Scrimmage(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Scrimmage(frame_str);
    }

    /*  Create and set in/out socket for extenal simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    uint16_t fdm_port_in;
    uint16_t fdm_port_out;
    const char* fdm_address;

    /*
      packet sent to Scrimmage from ArduPilot
     */
    static const int MAX_NUM_SERVOS = 16;
    struct servo_packet {
        uint16_t servos[MAX_NUM_SERVOS];
    };

    /*
      state packet sent from Scrimmage to ArduPilot
     */
    struct fdm_packet {
        uint64_t timestamp_us; // simulation time in microseconds
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    uint64_t prev_timestamp_us;
    SocketAPM recv_sock;
    SocketAPM send_sock;

    const char *frame_str;
};

} // namespace SITL
