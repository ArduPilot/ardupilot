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
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>
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

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_Backend::get_closest_object(float& angle_deg, float &distance) const
{
    bool sector_found = false;
    uint8_t sector = 0;

    // check all sectors for shorter distance
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
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
    return PROXIMITY_NUM_SECTORS;
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity_Backend::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if (object_number < PROXIMITY_NUM_SECTORS && _distance_valid[object_number]) {
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
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
        if (_distance_valid[i]) {
            valid_distances = true;
            break;
        }
    }
    if (!valid_distances) {
        return false;
    }

    // initialise orientations and directions
    //  see MAV_SENSOR_ORIENTATION for orientations (0 = forward, 1 = 45 degree clockwise from north, etc)
    //  distances initialised to maximum distances
    bool dist_set[PROXIMITY_MAX_DIRECTION]{};
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        prx_dist_array.orientation[i] = i;
        prx_dist_array.distance[i] = distance_max();
    }

    // cycle through all sectors filling in distances
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
        if (_distance_valid[i]) {
            // convert angle to orientation
            int16_t orientation = static_cast<int16_t>((_angle[i]+PROXIMITY_SECTOR_WIDTH_DEG*0.5) * (PROXIMITY_MAX_DIRECTION / 360.0f));
            orientation %= PROXIMITY_MAX_DIRECTION;
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
    if (state.status != AP_Proximity::Status::Good) {
        num_points = 0;
        return nullptr;
    }

    // check at least one sector has valid data, if not, exit
    bool some_valid = false;
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
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
    num_points = PROXIMITY_NUM_SECTORS;
    return _boundary_point;
}

// initialise the boundary and sector_edge_vector array used for object avoidance
//   should be called if the sector_middle_deg or _setor_width_deg arrays are changed
void AP_Proximity_Backend::init_boundary()
{
    for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
        float angle_rad = radians((float)_sector_middle_deg[sector]+(PROXIMITY_SECTOR_WIDTH_DEG/2.0f));
        _sector_edge_vector[sector].x = cosf(angle_rad) * 100.0f;
        _sector_edge_vector[sector].y = sinf(angle_rad) * 100.0f;
        _boundary_point[sector] = _sector_edge_vector[sector] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
    }
}

// update boundary points used for object avoidance based on a single sector's distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Backend::update_boundary_for_sector(const uint8_t sector, const bool push_to_OA_DB)
{
    // sanity check
    if (sector >= PROXIMITY_NUM_SECTORS) {
        return;
    }

    if (push_to_OA_DB && _distance_valid[sector]) {
        database_push(_angle[sector], _distance[sector]);
    }

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= PROXIMITY_NUM_SECTORS) {
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
    uint8_t prev_sector = (sector == 0) ? PROXIMITY_NUM_SECTORS-1 : sector-1;
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
    uint8_t prev_sector_ccw = (prev_sector == 0) ? PROXIMITY_NUM_SECTORS - 1 : prev_sector - 1;
    if (!_distance_valid[prev_sector_ccw]) {
        _boundary_point[prev_sector_ccw] = _sector_edge_vector[prev_sector_ccw] * shortest_distance;
    }
}

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Status status)
{
    state.status = status;
}

uint8_t AP_Proximity_Backend::convert_angle_to_sector(float angle_degrees) const
{
    return wrap_360(angle_degrees + (PROXIMITY_SECTOR_WIDTH_DEG * 0.5f)) / 45.0f;
}

// check if a reading should be ignored because it falls into an ignore area
bool AP_Proximity_Backend::ignore_reading(uint16_t angle_deg) const
{
    // check angle vs each ignore area
    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            if (abs(angle_deg - frontend._ignore_angle_deg[i]) <= (frontend._ignore_width_deg[i]/2)) {
                return true;
            }
        }
    }
    return false;
}

// returns true if database is ready to be pushed to and all cached data is ready
bool AP_Proximity_Backend::database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
        return false;
    }

    body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    return true;
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float distance)
{
    Vector3f current_pos;
    Matrix3f body_to_ned;

    if (database_prepare_for_push(current_pos, body_to_ned)) {
        database_push(angle, distance, AP_HAL::millis(), current_pos, body_to_ned);
    }
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }
    
    //Assume object is angle bearing and distance meters away from the vehicle 
    Vector2f object_2D = {0.0f,0.0f};
    object_2D.offset_bearing(wrap_180(angle), distance);	
    
    //rotate that vector to a 3D vector in NED frame
    const Vector3f object_3D = {object_2D.x,object_2D.y,0.0f};
    const Vector3f rotated_object_3D = body_to_ned * object_3D;
    
    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;
    
    oaDb->queue_push(temp_pos, timestamp_ms, distance);
}