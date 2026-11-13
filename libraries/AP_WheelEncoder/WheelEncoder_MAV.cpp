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

#include "AP_WheelEncoder_config.h"

#if AP_WHEELENCODER_MAV_ENABLED

#include "WheelEncoder_MAV.h"

#include <AP_HAL/AP_HAL.h>

bool AP_WheelEncoder_MAV::healthy() const
{
    return _state.total_count > 0 &&
           AP_HAL::millis() - _state.last_reading_ms < WHEELENCODER_STREAM_TIMEOUT_MS;
}

void AP_WheelEncoder_MAV::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_WHEEL_DISTANCE) {
        return;
    }

    mavlink_wheel_distance_t packet;
    mavlink_msg_wheel_distance_decode(&msg, &packet);

    const uint8_t instance = _state.instance;
    if (instance >= packet.count || instance >= ARRAY_SIZE(packet.distance)) {
        // sender is not reporting this wheel
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // the library stores distance as encoder counts, so convert using
    // this instance's counts per revolution and radius.  These cancel
    // when the frontend converts back to distance
    const float radius = _frontend.get_wheel_radius(instance);
    const uint16_t cpr = _frontend.get_counts_per_revolution(instance);
    if (!is_positive(radius) || cpr == 0) {
        copy_state_to_frontend(_state.distance_count, _state.total_count, _state.error_count + 1, now_ms);
        return;
    }
    const float counts_per_m = cpr / (M_2PI * radius);
    const int32_t distance_count = float_to_int32(roundf(packet.distance[instance] * counts_per_m));

    // timestamp with our own clock, as the sender's time_usec is not
    // synchronised with ours
    copy_state_to_frontend(distance_count, _state.total_count + 1, _state.error_count, now_ms);
}

#endif  // AP_WHEELENCODER_MAV_ENABLED
