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

#include "AP_VisualOdom_Backend.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_VisualOdom_Backend::AP_VisualOdom_Backend(AP_VisualOdom &frontend) :
    _frontend(frontend)
{
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom_Backend::healthy() const
{
    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _last_update_ms) < AP_VISUALODOM_TIMEOUT_MS);
}

// consume vision_position_delta mavlink messages
void AP_VisualOdom_Backend::handle_vision_position_delta_msg(const mavlink_message_t &msg)
{
    // decode message
    mavlink_vision_position_delta_t packet;
    mavlink_msg_vision_position_delta_decode(&msg, &packet);

    // apply rotation to angle and position delta
    const enum Rotation rot = _frontend.get_orientation();
    Vector3f angle_delta = Vector3f(packet.angle_delta[0], packet.angle_delta[1], packet.angle_delta[2]);
    angle_delta.rotate(rot);
    Vector3f position_delta = Vector3f(packet.position_delta[0], packet.position_delta[1], packet.position_delta[2]);
    position_delta.rotate(rot);

    const uint32_t now_ms = AP_HAL::millis();
    _last_update_ms = now_ms;

    // send to EKF
    const float time_delta_sec = packet.time_delta_usec / 1000000.0f;
    AP::ahrs_navekf().writeBodyFrameOdom(packet.confidence,
                                         position_delta,
                                         angle_delta,
                                         time_delta_sec,
                                         now_ms,
                                         _frontend.get_delay_ms(),
                                         _frontend.get_pos_offset());

    // log sensor data
    AP::logger().Write_VisualOdom(time_delta_sec,
                                  angle_delta,
                                  position_delta,
                                  packet.confidence);
}

// returns the system time of the last reset if reset_counter has not changed
// updates the reset timestamp to the current system time if the reset_counter has changed
uint32_t AP_VisualOdom_Backend::get_reset_timestamp_ms(uint8_t reset_counter)
{
    // update reset counter and timestamp if reset_counter has changed
    if (reset_counter != _last_reset_counter) {
        _last_reset_counter = reset_counter;
        _reset_timestamp_ms = AP_HAL::millis();
    }
    return _reset_timestamp_ms;
}

#endif
