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
#include "AP_Proximity_AirSimSITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 100.0f
#define PROXIMITY_ACCURACY 0.1f

// update the state of the sensor
void AP_Proximity_AirSimSITL::update(void)
{
    SITL::vector3f_array &points = sitl->state.scanner.points;
    if (points.length == 0) {
        set_status(AP_Proximity::Status::NoData);
        return;
    }

    set_status(AP_Proximity::Status::Good);

    memset(_distance_valid, 0, sizeof(_distance_valid));

    for (uint16_t i=0; i<points.length; i++) {
        Vector3f &point = points.data[i];
        if (point.is_zero()) {
            continue;
        }
        const float angle_deg = wrap_360(degrees(atan2f(-point.y, point.x)));
        const uint8_t sector = convert_angle_to_sector(angle_deg);

        const Vector2f v = Vector2f(point.x, point.y);
        const float distance_m = v.length();

        if (distance_m > distance_min()) {
            if (_last_sector == sector) {
                if (_distance_m_last > distance_m) {
                    _distance_m_last = distance_m;
                    _angle_deg_last = angle_deg;
                }
            } else {
                // new sector started, previous one can be updated
                _distance_valid[_last_sector] = true;
                _angle[_last_sector] = _angle_deg_last;
                _distance[_last_sector] = _distance_m_last;

                // update boundary
                update_boundary_for_sector(_last_sector, true);

                // initialize new sector
                _last_sector = sector;
                _distance_m_last = INT16_MAX;
                _angle_deg_last = angle_deg;
            }
        } else {
            _distance_valid[sector] = false;
        }
    }

#if 0
    printf("npoints=%u\n", points.length);
    for (uint16_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
        printf("sector[%u] ang=%.1f dist=%.1f\n", i, _angle[i], _distance[i]);
    }
#endif
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_AirSimSITL::distance_max() const
{
    return PROXIMITY_MAX_RANGE;
}

float AP_Proximity_AirSimSITL::distance_min() const
{
    return 0.0f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_AirSimSITL::get_upward_distance(float &distance) const
{
    // we don't have an upward facing laser
    return false;
}

#endif // CONFIG_HAL_BOARD
