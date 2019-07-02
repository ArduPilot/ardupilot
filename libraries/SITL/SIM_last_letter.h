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
  simulator connection for ardupilot version of last_letter
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a last_letter simulator
 */
class last_letter : public Aircraft {
public:
    last_letter(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new last_letter(home_str, frame_str);
    }

private:
    static const uint16_t fdm_port = 5002;

    /*
      packet sent to last_letter
     */
    struct servo_packet {
        uint16_t servos[16];
    };

    /*
      reply packet sent from last_letter to ArduPilot
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
    void start_last_letter(void);

    uint64_t last_timestamp_us;
    SocketAPM sock;
};

} // namespace SITL
