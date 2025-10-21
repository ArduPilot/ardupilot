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

#include "AP_RangeFinder_MAVLink.h"

#if AP_RANGEFINDER_MAVLINK_ENABLED

#include <AP_HAL/AP_HAL.h>

/*
   Set the distance based on a MAVLINK message
*/
void AP_RangeFinder_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(&msg, &packet);

    // only accept distances for the configured orientation
    if (packet.orientation == orientation()) {
        state.last_reading_ms = AP_HAL::millis();
        distance_cm = packet.current_distance;
        _max_distance_cm = packet.max_distance;
        _min_distance_cm = packet.min_distance;
        sensor_type = (MAV_DISTANCE_SENSOR)packet.type;
        signal_quality = packet.signal_quality;
        if (signal_quality == 0) {
            // MAVLink's 0 means invalid/unset, so we map it to -1
            signal_quality = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
        } else if (signal_quality == 1) {
            // Map 1 to 0 as that is what ardupilot uses as the worst signal quality
            signal_quality = RangeFinder::SIGNAL_QUALITY_MIN;
        }
    }
}

int16_t AP_RangeFinder_MAVLink::max_distance_cm() const
{
    if (_max_distance_cm == 0 && _min_distance_cm == 0) {
        // we assume if both of these are zero that we ignore both
        return params.max_distance_cm;
    }

    if (params.max_distance_cm < _max_distance_cm) {
        return params.max_distance_cm;
    }
    return _max_distance_cm;
}
int16_t AP_RangeFinder_MAVLink::min_distance_cm() const
{
    if (_max_distance_cm == 0 && _min_distance_cm == 0) {
        // we assume if both of these are zero that we ignore both
        return params.min_distance_cm;
    }
    if (params.min_distance_cm > _min_distance_cm) {
        return params.min_distance_cm;
    }
    return _min_distance_cm;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MAVLink::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_MAVLINK_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        state.distance_m = 0.0f;
        state.signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    } else {
        state.distance_m = distance_cm * 0.01f;
        state.signal_quality_pct = signal_quality;
        update_status();
    }
}

#endif
