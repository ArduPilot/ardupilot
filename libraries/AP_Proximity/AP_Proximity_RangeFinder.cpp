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
#include "AP_Proximity_RangeFinder.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_RangeFinder::AP_Proximity_RangeFinder(AP_Proximity &_frontend,
                                   AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
}

// update the state of the sensor
void AP_Proximity_RangeFinder::update(void)
{
    // exit immediately if no rangefinder object
    const RangeFinder *rngfnd = frontend.get_rangefinder();
    if (rngfnd == nullptr) {
        set_status(AP_Proximity::Proximity_NoData);
        return;
    }

    // look through all rangefinders
    for (uint8_t i=0; i<rngfnd->num_sensors(); i++) {
        if (rngfnd->has_data(i)) {
            // check for horizontal range finders
            if (rngfnd->get_orientation(i) <= ROTATION_YAW_315) {
                uint8_t sector = (uint8_t)rngfnd->get_orientation(i);
                _angle[sector] = sector * 45;
                _distance[sector] = rngfnd->distance_cm(i) / 100.0f;
                _distance_valid[sector] = true;
                _distance_min = rngfnd->min_distance_cm(i) / 100.0f;
                _distance_max = rngfnd->max_distance_cm(i) / 100.0f;
                _last_update_ms = AP_HAL::millis();
                update_boundary_for_sector(sector);
            }
            // check upward facing range finder
            if (rngfnd->get_orientation(i) == ROTATION_PITCH_90) {
                _distance_upward = rngfnd->distance_cm(i) / 100.0f;
                _last_upward_update_ms = AP_HAL::millis();
            }
        }
    }

    // check for timeout and set health status
    if ((_last_update_ms == 0) || (AP_HAL::millis() - _last_update_ms > PROXIMITY_RANGEFIDER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_RangeFinder::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}
