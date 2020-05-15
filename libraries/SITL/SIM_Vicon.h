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

#include "SIM_SerialDevice.h"

namespace SITL {

class Vicon : public SerialDevice {
public:

    Vicon();

    // update state
    void update(const Location &loc, const Vector3f &position, const Quaternion &attitude);

private:

    // TODO: make these parameters:
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    // we share channels with the ArduPilot binary!
    // Beware: the mavlink rangefinder shares this channel.
    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    uint64_t last_observation_usec;
    uint64_t time_send_us;
    uint64_t time_offset_us;
    mavlink_message_t obs_msg;


    void update_vicon_position_estimate(const Location &loc,
                                        const Vector3f &position,
                                        const Quaternion &attitude);

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;
};

}
