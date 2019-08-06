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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_MAV.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAV_TIMEOUT_MS    500 // distance messages must arrive within this many milliseconds

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_MAV::AP_Proximity_MAV(AP_Proximity &_frontend,
                                   AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
}

// update the state of the sensor
void AP_Proximity_MAV::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_MAV_TIMEOUT_MS)) &&
        (_last_upward_update_ms == 0 || (AP_HAL::millis() - _last_upward_update_ms > PROXIMITY_MAV_TIMEOUT_MS))) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_MAV::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_MAV_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity_MAV::handle_msg(mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        mavlink_distance_sensor_t packet;
        mavlink_msg_distance_sensor_decode(msg, &packet);

        // store distance to appropriate sector based on orientation field
        if (packet.orientation <= MAV_SENSOR_ROTATION_YAW_315) {
            uint8_t sector = packet.orientation;
            _angle[sector] = sector * 45;
            _distance[sector] = packet.current_distance / 100.0f;
            _distance_min = packet.min_distance / 100.0f;
            _distance_max = packet.max_distance / 100.0f;
            _distance_valid[sector] = (_distance[sector] >= _distance_min) && (_distance[sector] <= _distance_max);
            _last_update_ms = AP_HAL::millis();
            update_boundary_for_sector(sector);
        }

        // store upward distance
        if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_90) {
            _distance_upward = packet.current_distance / 100.0f;
            _last_upward_update_ms = AP_HAL::millis();
        }
        return;
    }

    if (msg->msgid == MAVLINK_MSG_ID_OBSTACLE_DISTANCE) {
        mavlink_obstacle_distance_t packet;
        mavlink_msg_obstacle_distance_decode(msg, &packet);

        // check increment (message's sector width)
        float increment;
        if (packet.increment_f > 0) {
            // use increment float
            increment = packet.increment_f;
        } else if (packet.increment > 0) {
            // use increment uint8_t
            increment = packet.increment;
        } else {
            // invalid increment
            return;
        }

        const float MAX_DISTANCE = 9999.0f;
        const float total_distances = MIN(360.0f / increment, MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN); // usually 72

        // set distance min and max
        _distance_min = packet.min_distance / 100.0f;
        _distance_max = packet.max_distance / 100.0f;
        _last_update_ms = AP_HAL::millis();

        // get user configured yaw correction from front end
        const float param_yaw_offset = constrain_float(frontend.get_yaw_correction(state.instance), -360.0f, +360.0f);
        const float yaw_correction = wrap_360(param_yaw_offset + packet.angle_offset);
        if (frontend.get_orientation(state.instance) != 0) {
            increment *= -1;
        }

        // initialise updated array and proximity sector angles (to closest object) and distances
        bool sector_updated[_num_sectors];
        float sector_width_half[_num_sectors];
        for (uint8_t i = 0; i < _num_sectors; i++) {
            sector_updated[i] = false;
            sector_width_half[i] = _sector_width_deg[i] * 0.5f;
            _angle[i] = _sector_middle_deg[i];
            _distance[i] = MAX_DISTANCE;
        }

        // iterate over message's sectors
        for (uint8_t j = 0; j < total_distances; j++) {
            const float packet_distance_m = packet.distances[j] * 0.01f;
            const float mid_angle = wrap_360(j * increment + yaw_correction);

            // iterate over proximity sectors
            for (uint8_t i = 0; i < _num_sectors; i++) {
                float angle_diff = fabsf(wrap_180(_sector_middle_deg[i] - mid_angle));
                // update distance array sector with shortest distance from message
                if ((angle_diff <= sector_width_half[i]) && (packet_distance_m < _distance[i])) {
                    _distance[i] = packet_distance_m;
                    _angle[i] = mid_angle;
                    sector_updated[i] = true;
                }
            }
        }

        // update proximity sectors validity and boundary point
        for (uint8_t i = 0; i < _num_sectors; i++) {
            _distance_valid[i] = (_distance[i] >= _distance_min) && (_distance[i] <= _distance_max);
            if (sector_updated[i]) {
                update_boundary_for_sector(i);
            }
        }
    }
}
