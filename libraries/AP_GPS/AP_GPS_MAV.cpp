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

//
//  MAVLINK GPS driver
//
#include "AP_GPS_MAV.h"
#include <stdint.h>

AP_GPS_MAV::AP_GPS_MAV(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
}

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_MAV::read(void)
{
    if (_new_data) {
        _new_data = false;
        return true;
    }

    return false;
}

// handles an incoming mavlink message (HIL_GPS) and sets
// corresponding gps data appropriately;
void AP_GPS_MAV::handle_msg(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

        case MAVLINK_MSG_ID_GPS_INPUT: {
            mavlink_gps_input_t packet;
            mavlink_msg_gps_input_decode(msg, &packet);

            bool have_alt    = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_ALT) == 0);
            bool have_hdop   = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_HDOP) == 0);
            bool have_vdop   = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_VDOP) == 0);
            bool have_vel_h  = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_VEL_HORIZ) == 0);
            bool have_vel_v  = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_VEL_VERT) == 0);
            bool have_sa     = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY) == 0);
            bool have_ha     = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY) == 0);
            bool have_va     = ((packet.ignore_flags & GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY) == 0);

            state.time_week     = packet.time_week;
            state.time_week_ms  = packet.time_week_ms;
            state.status = (AP_GPS::GPS_Status)packet.fix_type;

            Location loc = {};
            loc.lat = packet.lat;
            loc.lng = packet.lon;
            if (have_alt) {
                loc.alt = packet.alt * 100; // convert to centimeters
            }
            state.location = loc;
            state.location.options = 0;

            if (have_hdop) {
                state.hdop = packet.hdop * 100; // convert to centimeters
            }

            if (have_vdop) {
                state.vdop = packet.vdop * 100; // convert to centimeters
            }

            if (have_vel_h) {
                Vector3f vel(packet.vn, packet.ve, 0);
                if (have_vel_v) {
                    vel.z = packet.vd;
                    state.have_vertical_velocity = true;
                }

                state.velocity = vel;
                state.ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
                state.ground_speed = norm(vel.x, vel.y);
            }

            if (have_sa) {
                state.speed_accuracy = packet.speed_accuracy;
                state.have_speed_accuracy = true;
            }

            if (have_ha) {
                state.horizontal_accuracy = packet.horiz_accuracy;
                state.have_horizontal_accuracy = true;
            }

            if (have_va) {
                state.vertical_accuracy = packet.vert_accuracy;
                state.have_vertical_accuracy = true;
            }

            state.num_sats = packet.satellites_visible;
            state.last_gps_time_ms = AP_HAL::millis();
            _new_data = true;
            break;
            }

        case MAVLINK_MSG_ID_HIL_GPS: {
            mavlink_hil_gps_t packet;
            mavlink_msg_hil_gps_decode(msg, &packet);

            state.time_week = 0;
            state.time_week_ms  = packet.time_usec/1000;
            state.status = (AP_GPS::GPS_Status)packet.fix_type;

            Location loc = {};
            loc.lat = packet.lat;
            loc.lng = packet.lon;
            loc.alt = packet.alt * 0.1f;
            state.location = loc;
            state.location.options = 0;
            state.hdop = MIN(packet.eph, GPS_UNKNOWN_DOP);
            state.vdop = MIN(packet.epv, GPS_UNKNOWN_DOP);
            if (packet.vel < 65535) {
                state.ground_speed = packet.vel / 100.0f;
            }
            Vector3f vel(packet.vn/100.0f, packet.ve/100.0f, packet.vd/100.0f);
            state.velocity = vel;
            if (packet.vd != 0) {
                state.have_vertical_velocity = true;
            }
            if (packet.cog < 36000) {
                state.ground_course = packet.cog / 100.0f;
            }
            state.have_speed_accuracy = false;
            state.have_horizontal_accuracy = false;
            state.have_vertical_accuracy = false;
            if (packet.satellites_visible < 255) {
                state.num_sats = packet.satellites_visible;
            }
            state.last_gps_time_ms = AP_HAL::millis();
            _new_data = true;
            break;
            }
        default:
            // ignore all other messages
            break;
    }
}
