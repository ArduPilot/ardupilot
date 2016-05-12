// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include <AP_HAL/AP_HAL.h>



extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MAVLink::AP_RangeFinder_MAVLink(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    last_update_ms = AP_HAL::millis();
    distance_cm = 0;
}

/*
   detect if a MAVLink rangefinder is connected. We'll detect by
   checking a parameter.
*/
bool AP_RangeFinder_MAVLink::detect(RangeFinder &_ranger, uint8_t instance)
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MAVLink,
    // there is an attached MAVLink rangefinder
    return true;
}

/*
   Set the distance based on a MAVLINK message
*/
void AP_RangeFinder_MAVLink::handle_msg(mavlink_message_t *msg)
{
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(msg, &packet);

    last_update_ms = AP_HAL::millis();
    distance_cm = packet.current_distance;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MAVLink::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if(AP_HAL::millis() - last_update_ms > AP_RANGEFINDER_MAVLINK_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
        state.distance_cm = 0;
    } else {
        set_status(RangeFinder::RangeFinder_Good);
        state.distance_cm = distance_cm;
    }
}
