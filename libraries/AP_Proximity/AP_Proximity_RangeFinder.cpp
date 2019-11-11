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
#include <ctype.h>
#include <stdio.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

// update the state of the sensor
void AP_Proximity_RangeFinder::update(void)
{
    // exit immediately if no rangefinder object
    const RangeFinder *rngfnd = AP::rangefinder();
    if (rngfnd == nullptr) {
        set_status(AP_Proximity::Status::NoData);
        return;
    }

    uint32_t now = AP_HAL::millis();

    // look through all rangefinders
    for (uint8_t i=0; i < rngfnd->num_sensors(); i++) {
        AP_RangeFinder_Backend *sensor = rngfnd->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
        if (sensor->has_data()) {
            // check for horizontal range finders
            if (sensor->orientation() <= ROTATION_YAW_315) {
                uint8_t sector = (uint8_t)sensor->orientation();
                _angle[sector] = sector * 45;
                _distance[sector] = sensor->distance_cm() * 0.01f;
                _distance_min = sensor->min_distance_cm() * 0.01f;
                _distance_max = sensor->max_distance_cm() * 0.01f;
                _distance_valid[sector] = (_distance[sector] >= _distance_min) && (_distance[sector] <= _distance_max);
                _last_update_ms = now;
                update_boundary_for_sector(sector, true);
            }
            // check upward facing range finder
            if (sensor->orientation() == ROTATION_PITCH_90) {
                int16_t distance_upward = sensor->distance_cm();
                int16_t up_distance_min = sensor->min_distance_cm();
                int16_t up_distance_max = sensor->max_distance_cm();
                if ((distance_upward >= up_distance_min) && (distance_upward <= up_distance_max)) {
                    _distance_upward = distance_upward * 0.01f;
                } else {
                    _distance_upward = -1.0; // mark an valid reading
                }
                _last_upward_update_ms = now;
            }
        }
    }

    // check for timeout and set health status
    if ((_last_update_ms == 0) || (now - _last_update_ms > PROXIMITY_RANGEFIDER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_RangeFinder::get_upward_distance(float &distance) const
{
    if ((AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS) &&
        is_positive(_distance_upward)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}
