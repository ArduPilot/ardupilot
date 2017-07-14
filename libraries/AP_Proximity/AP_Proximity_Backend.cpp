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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
        frontend(_frontend),
        state(_state)
{
    // initialise sector edge vector used for building the boundary fence
    init_boundary();
}

// get distance in meters in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
bool AP_Proximity_Backend::get_horizontal_distance(float angle_deg, float &distance) const
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        if (_distance_valid[sector]) {
            distance = _distance[sector];
            return true;
        }
    }
    return false;
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_Backend::get_closest_object(float& angle_deg, float &distance) const
{
    bool sector_found = false;
    uint8_t sector = 0;

    // check all sectors for shorter distance
    for (uint8_t i=0; i<_num_sectors; i++) {
        if (_distance_valid[i]) {
            if (!sector_found || (_distance[i] < _distance[sector])) {
                sector = i;
                sector_found = true;
            }
        }
    }

    if (sector_found) {
        angle_deg = _angle[sector];
        distance = _distance[sector];
    }
    return sector_found;
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity_Backend::get_object_count() const
{
    return _num_sectors;
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity_Backend::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if (object_number < _num_sectors && _distance_valid[object_number]) {
        angle_deg = _angle[object_number];
        distance = _distance[object_number];
        return true;
    }
    return false;
}

// get distances in PROXIMITY_MAX_DIRECTION directions. used for sending distances to ground station
bool AP_Proximity_Backend::get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const
{
    // exit immediately if we have no good ranges
    bool valid_distances = false;
    for (uint8_t i=0; i<_num_sectors; i++) {
        if (_distance_valid[i]) {
            valid_distances = true;
        }
    }
    if (!valid_distances) {
        return false;
    }

    // initialise orientations and directions
    //  see MAV_SENSOR_ORIENTATION for orientations (0 = forward, 1 = 45 degree clockwise from north, etc)
    //  distances initialised to maximum distances
    bool dist_set[PROXIMITY_MAX_DIRECTION];
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        prx_dist_array.orientation[i] = i;
        prx_dist_array.distance[i] = distance_max();
        dist_set[i] = false;
    }

    // cycle through all sectors filling in distances
    for (uint8_t i=0; i<_num_sectors; i++) {
        if (_distance_valid[i]) {
            // convert angle to orientation
            int16_t orientation = static_cast<int16_t>(_angle[i] * (PROXIMITY_MAX_DIRECTION / 360.0f));
            if ((orientation >= 0) && (orientation < PROXIMITY_MAX_DIRECTION) && (_distance[i] < prx_dist_array.distance[orientation])) {
                prx_dist_array.distance[orientation] = _distance[i];
                dist_set[orientation] = true;
            }
        }
    }

    // fill in missing orientations with average of adjacent orientations if necessary and possible
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        if (!dist_set[i]) {
            uint8_t orient_before = (i==0) ? (PROXIMITY_MAX_DIRECTION - 1) : (i-1);
            uint8_t orient_after = (i==(PROXIMITY_MAX_DIRECTION - 1)) ? 0 : (i+1);
            if (dist_set[orient_before] && dist_set[orient_after]) {
                prx_dist_array.distance[i] = (prx_dist_array.distance[orient_before] + prx_dist_array.distance[orient_after]) / 2.0f;
            }
        }
    }
    return true;
}

// get boundary points around vehicle for use by avoidance
//   returns nullptr and sets num_points to zero if no boundary can be returned
const Vector2f* AP_Proximity_Backend::get_boundary_points(uint16_t& num_points) const
{
    // high-level status check
    if (state.status != AP_Proximity::Proximity_Good) {
        num_points = 0;
        return nullptr;
    }

    // check at least one sector has valid data, if not, exit
    bool some_valid = false;
    for (uint8_t i=0; i<_num_sectors; i++) {
        if (_distance_valid[i]) {
            some_valid = true;
            break;
        }
    }
    if (!some_valid) {
        num_points = 0;
        return nullptr;
    }

    // return boundary points
    num_points = _num_sectors;
    return _boundary_point;
}

// initialise the boundary and sector_edge_vector array used for object avoidance
//   should be called if the sector_middle_deg or _setor_width_deg arrays are changed
void AP_Proximity_Backend::init_boundary()
{
    for (uint8_t sector=0; sector < _num_sectors; sector++) {
        float angle_rad = radians((float)_sector_middle_deg[sector]+(float)_sector_width_deg[sector]/2.0f);
        _sector_edge_vector[sector].x = cosf(angle_rad) * 100.0f;
        _sector_edge_vector[sector].y = sinf(angle_rad) * 100.0f;
        _boundary_point[sector] = _sector_edge_vector[sector] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
    }
}

// update boundary points used for object avoidance based on a single sector's distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Backend::update_boundary_for_sector(uint8_t sector)
{
    // sanity check
    if (sector >= _num_sectors) {
        return;
    }

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= _num_sectors) {
        next_sector = 0;
    }

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[sector] && _distance_valid[next_sector]) {
        shortest_distance = MIN(_distance[sector], _distance[next_sector]);
    } else if (_distance_valid[sector]) {
        shortest_distance = _distance[sector];
    } else if (_distance_valid[next_sector]) {
        shortest_distance = _distance[next_sector];
    }
    if (shortest_distance < PROXIMITY_BOUNDARY_DIST_MIN) {
        shortest_distance = PROXIMITY_BOUNDARY_DIST_MIN;
    }
    _boundary_point[sector] = _sector_edge_vector[sector] * shortest_distance;

    // if the next sector (clockwise) has an invalid distance, set boundary to create a cup like boundary
    if (!_distance_valid[next_sector]) {
        _boundary_point[next_sector] = _sector_edge_vector[next_sector] * shortest_distance;
    }

    // repeat for edge between sector and previous sector
    uint8_t prev_sector = (sector == 0) ? _num_sectors-1 : sector-1;
    shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[prev_sector] && _distance_valid[sector]) {
        shortest_distance = MIN(_distance[prev_sector], _distance[sector]);
    } else if (_distance_valid[prev_sector]) {
        shortest_distance = _distance[prev_sector];
    } else if (_distance_valid[sector]) {
        shortest_distance = _distance[sector];
    }
    _boundary_point[prev_sector] = _sector_edge_vector[prev_sector] * shortest_distance;

    // if the sector counter-clockwise from the previous sector has an invalid distance, set boundary to create a cup like boundary
    uint8_t prev_sector_ccw = (prev_sector == 0) ? _num_sectors-1 : prev_sector-1;
    if (!_distance_valid[prev_sector_ccw]) {
        _boundary_point[prev_sector_ccw] = _sector_edge_vector[prev_sector_ccw] * shortest_distance;
    }
}

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Proximity_Status status)
{
    state.status = status;
}

bool AP_Proximity_Backend::convert_angle_to_sector(float angle_degrees, uint8_t &sector) const
{
    // sanity check angle
    if (angle_degrees > 360.0f || angle_degrees < -180.0f) {
        return false;
    }

    // convert to 0 ~ 360
    if (angle_degrees < 0.0f) {
        angle_degrees += 360.0f;
    }

    bool closest_found = false;
    uint8_t closest_sector;
    float closest_angle;

    // search for which sector angle_degrees falls into
    for (uint8_t i = 0; i < _num_sectors; i++) {
        float angle_diff = fabsf(wrap_180(_sector_middle_deg[i] - angle_degrees));

        // record if closest
        if (!closest_found || angle_diff < closest_angle) {
            closest_found = true;
            closest_sector = i;
            closest_angle = angle_diff;
        }

        if (fabsf(angle_diff) <= _sector_width_deg[i] / 2.0f) {
            sector = i;
            return true;
        }
    }

    // angle_degrees might have been within a gap between sectors
    if (closest_found) {
        sector = closest_sector;
        return true;
    }

    return false;
}

// get ignore area info
uint8_t AP_Proximity_Backend::get_ignore_area_count() const
{
    // count number of ignore sectors
    uint8_t count = 0;
    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            count++;
        }
    }
    return count;
}

// get next ignore angle
bool AP_Proximity_Backend::get_ignore_area(uint8_t index, uint16_t &angle_deg, uint8_t &width_deg) const
{
    if (index >= PROXIMITY_MAX_IGNORE) {
        return false;
    }
    angle_deg = frontend._ignore_angle_deg[index];
    width_deg = frontend._ignore_width_deg[index];
    return true;
}

// retrieve start or end angle of next ignore area (i.e. closest ignore area higher than the start_angle)
// start_or_end = 0 to get start, 1 to retrieve end
bool AP_Proximity_Backend::get_next_ignore_start_or_end(uint8_t start_or_end, int16_t start_angle, int16_t &ignore_start) const
{
    bool found = false;
    int16_t smallest_angle_diff = 0;
    int16_t smallest_angle_start = 0;

    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            int16_t offset = start_or_end == 0 ? -frontend._ignore_width_deg[i] : +frontend._ignore_width_deg[i];
            int16_t ignore_start_angle = wrap_360(frontend._ignore_angle_deg[i] + offset/2.0f);
            int16_t ang_diff = wrap_360(ignore_start_angle - start_angle);
            if (!found || ang_diff < smallest_angle_diff) {
                smallest_angle_diff = ang_diff;
                smallest_angle_start = ignore_start_angle;
                found = true;
            }
        }
    }

    if (found) {
        ignore_start = smallest_angle_start;
    }
    return found;
}
