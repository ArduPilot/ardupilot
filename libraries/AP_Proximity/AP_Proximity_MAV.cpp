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
    if ((_last_update_ms == 0) || (AP_HAL::millis() - _last_update_ms > PROXIMITY_MAV_TIMEOUT_MS)) {
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
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(msg, &packet);

    // store distance to appropriate sector based on orientation field
    if (packet.orientation <= MAV_SENSOR_ROTATION_YAW_315) {
        uint8_t sector = packet.orientation;
        _angle[sector] = sector * 45;
        _distance[sector] = packet.current_distance / 100.0f;
        _distance_valid[sector] = true;
        _distance_min = packet.min_distance / 100.0f;
        _distance_max = packet.max_distance / 100.0f;
        _last_update_ms = AP_HAL::millis();
        update_boundary_for_sector(sector);
    }

    // store upward distance
    if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_90) {
        _distance_upward = packet.current_distance / 100.0f;
        _last_upward_update_ms = AP_HAL::millis();
    }
}
