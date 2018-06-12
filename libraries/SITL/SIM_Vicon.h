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
  simple particle sensor simulation
*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include <AP_HAL/utility/RingBuffer.h>

namespace SITL {

class Vicon {
public:

    Vicon();

    // update state
    void update(const Location &loc, const Vector3f &position, const Quaternion &attitude);

    // return fd on which data from the device can be read, and data
    // to the device can be written
    int fd() { return fd_their_end; }

private:

    SITL *_sitl;

    // TODO: make these parameters:
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    // we share channels with the ArduPilot binary!
    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    int fd_their_end;
    int fd_my_end;

    uint64_t last_observation_usec;
    uint64_t time_send_us;
    uint64_t time_offset_us;
    mavlink_message_t obs_msg;

    struct obs_elements {
        uint32_t    time_ms;        // measurement timestamp (msec)
        Vector3f    position;
        Quaternion  attitude;
    };

    void update_vicon_position_estimate(const Location &loc,
                                        const Vector3f &position,
                                        const Quaternion &attitude);

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;

    bool init_sitl_pointer();
};

}
