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
    void update(const Location &loc, const Vector3f &position, const Vector3f &velocity, const Quaternion &attitude);

private:

    // TODO: make these parameters:
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    // we share channels with the ArduPilot binary!
    // Beware: the mavlink rangefinder shares this channel.
    const mavlink_channel_t mavlink_ch = (mavlink_channel_t)(MAVLINK_COMM_0+5);

    uint64_t last_observation_usec; // time last observation was sent
    uint64_t time_offset_us;        // simulated timeoffset between external system and autopilot

    // buffer of messages to send
    struct {
        uint64_t time_send_us;      // system time this message should be sent or 0 if no message to send
        mavlink_message_t obs_msg;  // message to be sent
    } msg_buf[3];

    // SIM_VICON_TYPE parameter bit values
    enum class ViconTypeMask : uint8_t {
        VISION_POSITION_ESTIMATE    = (1 << 0),
        VISION_SPEED_ESTIMATE       = (1 << 1),
        VICON_POSITION_ESTIMATE     = (1 << 2)
    };

    // return true if the given message type should be sent
    bool should_send(ViconTypeMask type_mask) const { return (((uint8_t)type_mask & _sitl->vicon_type_mask.get()) > 0); }

    // get unused index in msg_buf
    bool get_free_msg_buf_index(uint8_t &index);

    void update_vicon_position_estimate(const Location &loc,
                                        const Vector3f &position,
                                        const Vector3f &velocity,
                                        const Quaternion &attitude);

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;
};

}
