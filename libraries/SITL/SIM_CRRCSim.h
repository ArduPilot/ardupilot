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
  simulator connection for ardupilot version of CRRCSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a CRRCSim simulator
 */
class CRRCSim : public Aircraft {
public:
    CRRCSim(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new CRRCSim(frame_str);
    }

private:
    /*
      packet sent to CRRCSim
     */
    struct servo_packet {
        float roll_rate;
        float pitch_rate;
        float throttle;
        float yaw_rate;
        float col_pitch;
    };

    /*
      reply packet sent from CRRCSim to ArduPilot
     */
    struct fdm_packet {
        double timestamp;
        double latitude, longitude;
        double altitude;
        double heading;
        double speedN, speedE, speedD;
        double xAccel, yAccel, zAccel;
        double rollRate, pitchRate, yawRate;
        double roll, pitch, yaw;
        double airspeed;
    };

    void send_servos_heli(const struct sitl_input &input);
    void send_servos_fixed_wing(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);

    bool heli_servos;
    double last_timestamp;
    SocketAPM sock;
};

} // namespace SITL
