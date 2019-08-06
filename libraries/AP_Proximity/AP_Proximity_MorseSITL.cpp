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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Proximity_MorseSITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 200.0f
#define PROXIMITY_ACCURACY 0.1f

/* 
   The constructor also initialises the proximity sensor. 
*/
AP_Proximity_MorseSITL::AP_Proximity_MorseSITL(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state):
    AP_Proximity_Backend(_frontend, _state),
    sitl(AP::sitl())
{
}

// update the state of the sensor
void AP_Proximity_MorseSITL::update(void)
{
    SITL::vector3f_array &points = sitl->state.scanner.points;
    SITL::float_array &ranges = sitl->state.scanner.ranges;
    if (points.length != ranges.length ||
        points.length == 0) {
        set_status(AP_Proximity::Proximity_NoData);
        return;
    }

    set_status(AP_Proximity::Proximity_Good);

    memset(_distance_valid, 0, sizeof(_distance_valid));
    memset(_angle, 0, sizeof(_angle));
    memset(_distance, 0, sizeof(_distance));

    // only use 8 sectors to match RPLidar
    const uint8_t nsectors = MIN(8, PROXIMITY_SECTORS_MAX);
    const uint16_t degrees_per_sector = 360 / nsectors;

    for (uint16_t i=0; i<points.length; i++) {
        Vector3f &point = points.data[i];
        float &range = ranges.data[i];
        distance_maximum = MAX(distance_maximum, range);
        if (point.is_zero()) {
            continue;
        }
        float angle_deg = wrap_360(degrees(atan2f(-point.y, point.x)));
        uint16_t angle_rounded = uint16_t(angle_deg+0.5);
        uint8_t sector = wrap_360(angle_rounded + 22.5f) / degrees_per_sector;
        if (!_distance_valid[sector] || range < _distance[sector]) {
            _distance_valid[sector] = true;
            _distance[sector] = range;
            _angle[sector] = angle_deg;
            update_boundary_for_sector(sector);
        }
    }

#if 0
    printf("npoints=%u\n", points.length);
    for (uint16_t i=0; i<nsectors; i++) {
        printf("sector[%u] ang=%.1f dist=%.1f\n", i, _angle[i], _distance[i]);
    }
#endif
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_MorseSITL::distance_max() const
{
    // we don't have a data field from Morse for max range, so we use the max
    // we've ever seen
    return distance_maximum;
}

float AP_Proximity_MorseSITL::distance_min() const
{
    return 0.0f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_MorseSITL::get_upward_distance(float &distance) const
{
    // we don't have an upward facing laser
    return false;
}

#endif // CONFIG_HAL_BOARD
