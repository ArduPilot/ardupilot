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

#include "AP_VisualOdom_MAV.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_VisualOdom_MAV::AP_VisualOdom_MAV(AP_VisualOdom &frontend) :
    AP_VisualOdom_Backend(frontend)
{
}

// consume VISION_POSITION_DELTA MAVLink message
void AP_VisualOdom_MAV::handle_msg(const mavlink_message_t &msg)
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
    AP::ahrs_navekf().writeBodyFrameOdom(packet.confidence,
                                         angle_delta,
                                         position_delta,
                                         packet.time_delta_usec,
                                         now_ms,
                                         _frontend.get_pos_offset());

    // log sensor data
    AP::logger().Write_VisualOdom(packet.time_delta_usec / 1000000.0f,
                                  angle_delta,
                                  position_delta,
                                  packet.confidence);
}
