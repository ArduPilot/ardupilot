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

#include "AP_Proximity_Boundary_3D.h"

#define PROXIMITY_BOUNDARY_3D_TIMEOUT_MS 750 // we should check the 3D boundary faces after this many ms

/*
  Constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Boundary_3D::AP_Proximity_Boundary_3D() 
{
    // initialise sector edge vector used for building the boundary fence
    init();
}

// initialise the boundary and sector_edge_vector array used for object avoidance
//   should be called if the sector_middle_deg or _sector_width_deg arrays are changed
void AP_Proximity_Boundary_3D::init()
{
    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        const float pitch = ((float)_pitch_middle_deg[layer]);
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            const float angle_rad = ((float)_sector_middle_deg[sector]+(PROXIMITY_SECTOR_WIDTH_DEG/2.0f));
            _sector_edge_vector[layer][sector].offset_bearing(angle_rad, pitch, 100.0f);
            _boundary_points[layer][sector] = _sector_edge_vector[layer][sector] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
        }
    }
}

// returns face corresponding to the provided yaw and (optionally) pitch
// pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
// yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
AP_Proximity_Boundary_3D::Face AP_Proximity_Boundary_3D::get_face(float pitch, float yaw) const
{
    const uint8_t sector = wrap_360(yaw + (PROXIMITY_SECTOR_WIDTH_DEG * 0.5f)) / 45.0f;
    const float pitch_limited = constrain_float(pitch, -75.0f, 74.9f);
    const uint8_t layer = (pitch_limited + 75.0f)/PROXIMITY_PITCH_WIDTH_DEG;
    return Face{layer, sector};
}

// Set the actual body-frame angle(yaw), pitch, and distance of the detected object.
// This method will also mark the sector and layer to be "valid",
// This distance can then be used for Obstacle Avoidance
// Assume detected obstacle is horizontal (zero pitch), if no pitch is passed
// prx_instance should be set to the proximity sensor backend instance number
void AP_Proximity_Boundary_3D::set_face_attributes(const Face &face, float pitch, float angle, float distance, uint8_t prx_instance)
{
    if (!face.valid()) {
        return;
    }

    // ignore update if another instance has provided a shorter distance within the last 0.2 seconds
    if ((prx_instance != _prx_instance[face.layer][face.sector]) && _distance_valid[face.layer][face.sector] && (_filtered_distance[face.layer][face.sector].get() < distance)) {
        // check if recent
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_update_ms[face.layer][face.sector] < PROXIMITY_FACE_RESET_MS) {
            return;
        }
    }

    _angle_deg[face.layer][face.sector] = angle;
    _pitch_deg[face.layer][face.sector] = pitch;
    _distance[face.layer][face.sector] = distance;
    _distance_valid[face.layer][face.sector] = true;
    _prx_instance[face.layer][face.sector] = prx_instance;

    // apply filter
    set_filtered_distance(face, distance);

    // update boundary used for simple avoidance
    update_boundary(face);
}

// apply a new cutoff_freq to low-pass filter
void AP_Proximity_Boundary_3D::apply_filter_freq(float cutoff_freq)
{
    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            _filtered_distance[layer][sector].set_cutoff_frequency(cutoff_freq);
        }
    }
}

// Apply low pass filter on the raw distance
void AP_Proximity_Boundary_3D::set_filtered_distance(const Face &face, float distance)
{
    if (!face.valid()) {
        return;
    }
    if (!is_equal(_filtered_distance[face.layer][face.sector].get_cutoff_freq(), _filter_freq)) {
        // cutoff freq has changed
        apply_filter_freq(_filter_freq);
    }

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt = now_ms - _last_update_ms[face.layer][face.sector];
    if (dt < PROXIMITY_FILT_RESET_TIME) {
        _filtered_distance[face.layer][face.sector].apply(distance, dt* 0.001f);
    } else {
        // reset filter since last distance was passed a long time back
        _filtered_distance[face.layer][face.sector].reset(distance);
    }
    _last_update_ms[face.layer][face.sector] = now_ms;
}

// update boundary points used for object avoidance based on a single sector and pitch distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Boundary_3D::update_boundary(const Face &face)
{
    // sanity check
    if (!face.valid()) {
        return;
    }

    const uint8_t layer = face.layer;
    const uint8_t sector = face.sector;

    // find adjacent sector (clockwise)
    const uint8_t next_sector = get_next_sector(sector);

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[layer][sector] && _distance_valid[layer][next_sector]) {
        shortest_distance = MIN(_filtered_distance[layer][sector].get(), _filtered_distance[layer][next_sector].get());
    } else if (_distance_valid[layer][sector]) {
        shortest_distance = _filtered_distance[layer][sector].get();
    } else if (_distance_valid[layer][next_sector]) {
        shortest_distance = _filtered_distance[layer][next_sector].get();
    }
    if (shortest_distance < PROXIMITY_BOUNDARY_DIST_MIN) {
        shortest_distance = PROXIMITY_BOUNDARY_DIST_MIN;
    }
    _boundary_points[layer][sector] = _sector_edge_vector[layer][sector] * shortest_distance;

    // if the next sector (clockwise) has an invalid distance, set boundary to create a cup like boundary
    if (!_distance_valid[layer][next_sector]) {
        _boundary_points[layer][next_sector] = _sector_edge_vector[layer][next_sector] * shortest_distance;
    }

    // repeat for edge between sector and previous sector
    const uint8_t prev_sector = get_prev_sector(sector);
    shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[layer][prev_sector] && _distance_valid[layer][sector]) {
        shortest_distance = MIN(_filtered_distance[layer][prev_sector].get(), _filtered_distance[layer][sector].get());
    } else if (_distance_valid[layer][prev_sector]) {
        shortest_distance = _filtered_distance[layer][prev_sector].get();
    } else if (_distance_valid[layer][sector]) {
        shortest_distance = _filtered_distance[layer][sector].get();
    }
    _boundary_points[layer][prev_sector] = _sector_edge_vector[layer][prev_sector] * shortest_distance;

    // if the sector counter-clockwise from the previous sector has an invalid distance, set boundary to create a cup-like boundary
    const uint8_t prev_sector_ccw = get_prev_sector(prev_sector);
    if (!_distance_valid[layer][prev_sector_ccw]) {
        _boundary_points[layer][prev_sector_ccw] = _sector_edge_vector[layer][prev_sector_ccw] * shortest_distance;
    }
}

// reset boundary.  marks all distances as invalid
void AP_Proximity_Boundary_3D::reset()
{
    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            _distance_valid[layer][sector] = false;
        }
    }
}

// Reset this location, specified by Face object, back to default
// i.e Distance is marked as not-valid, and set to a large number.
// prx_instance should be set to the proximity sensor's backend instance number
void AP_Proximity_Boundary_3D::reset_face(const Face &face, uint8_t prx_instance)
{
    if (!face.valid()) {
        return;
    }

    // return immediately if face already has no valid distance
    if (!_distance_valid[face.layer][face.sector]) {
        return;
    }

    // ignore reset if another instance provided this face's distance within the last 0.2 seconds
    if (prx_instance != _prx_instance[face.layer][face.sector]) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_update_ms[face.layer][face.sector] < 200) {
            return;
        }
    }

    _distance_valid[face.layer][face.sector] = false;

    // update simple avoidance boundary
    update_boundary(face);
}

// check if a face has valid distance even if it was updated a long time back
void AP_Proximity_Boundary_3D::check_face_timeout()
{
    // exit immediately if already checked recently
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_check_face_timeout_ms) < PROXIMITY_BOUNDARY_3D_TIMEOUT_MS) {
        return;
    }
    _last_check_face_timeout_ms = now_ms;

    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            if (_distance_valid[layer][sector]) {
                if ((now_ms - _last_update_ms[layer][sector]) > PROXIMITY_FACE_RESET_MS) {
                    // this face has a valid distance but wasn't updated for a long time, reset it
                    _distance_valid[layer][sector] = false;
                    update_boundary(AP_Proximity_Boundary_3D::Face{layer, sector});
                }
            }
        }
    }
}

// get distance for a face.  returns true on success and fills in distance argument with distance in meters
bool AP_Proximity_Boundary_3D::get_distance(const Face &face, float &distance) const
{
    if (!face.valid()) {
        return false;
    }
    if (_distance_valid[face.layer][face.sector]) {
        distance = _distance[face.layer][face.sector];
        return true;
    }

    return false;
}

// get the total number of obstacles 
uint8_t AP_Proximity_Boundary_3D::get_obstacle_count() const
{
    return PROXIMITY_NUM_LAYERS * PROXIMITY_NUM_SECTORS;
}

// Converts obstacle_num passed from avoidance library into appropriate face of the boundary
// Returns false if the face is invalid
// "update_boundary" method manipulates two sectors ccw and one sector cw from any valid face.
// Any boundary that does not fall into these manipulated faces are useless, and will be marked as false
// The resultant is packed into a Boundary Location object and returned by reference as "face"
bool AP_Proximity_Boundary_3D::convert_obstacle_num_to_face(uint8_t obstacle_num, Face& face) const
{
    // obstacle num is just "flattened layers, and sectors"
    const uint8_t layer = obstacle_num / PROXIMITY_NUM_SECTORS;
    const uint8_t sector = obstacle_num % PROXIMITY_NUM_SECTORS;
    face.sector = sector;
    face.layer = layer;

    uint8_t valid_sector = sector;
    // check for 3 adjacent sectors
    for (uint8_t i=0; i < 3; i++) {
        if (_distance_valid[layer][valid_sector]) {
            // update boundary has manipulated this face
            return true;
        }
        valid_sector = get_next_sector(valid_sector);
    }

    // this face was not manipulated by "update_boundary" and is stale. Don't use it
    return false;
}

// Appropriate layer and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given layer
// Then returns the closest point on this line from vehicle, in body-frame. 
// Used by GPS based Simple Avoidance  
// False is returned if the obstacle_num provided does not produce a valid obstacle 
bool AP_Proximity_Boundary_3D::get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const
{
    Face face;
    if (!convert_obstacle_num_to_face(obstacle_num, face)) {
        // not a valid face
        return false;
    }
    const uint8_t sector_end = face.sector;
    const uint8_t sector_start = get_next_sector(face.sector);
    
    const Vector3f start = _boundary_points[face.layer][sector_start];
    const Vector3f end = _boundary_points[face.layer][sector_end];
    vec_to_obstacle = Vector3f::point_on_line_closest_to_other_point(start, end, Vector3f{});
    return true;
}

// Appropriate layer and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given layer
// Then returns the closest point on this line from the segment that was passed, in body-frame.
// Addionally a 3-D plane is constructed using the closest point found above as normal, and a point on the line segment in the boundary.
// True is returned when the passed line segment intersects this plane.
// This helps us know if the passed line segment was in the direction of the boundary, or going in a different direction.
// Used by GPS based Simple Avoidance  - for "brake mode"
// False is returned if the obstacle_num provided does not produce a valid obstacle
bool AP_Proximity_Boundary_3D::closest_point_from_segment_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{
    Face face;
    if (!convert_obstacle_num_to_face(obstacle_num, face)) {
        // not a valid a face
        return false;
    }

    const uint8_t sector_end = face.sector;
    const uint8_t sector_start = get_next_sector(face.sector);
    const Vector3f start = _boundary_points[face.layer][sector_start];
    const Vector3f end = _boundary_points[face.layer][sector_end];

    // closest point between passed line segment and boundary
    Vector3f::segment_to_segment_closest_point(seg_start, seg_end, start, end, closest_point);
    if (closest_point == start) {
        // draw a plane using the closest point as normal vector, and a point on the boundary
        // return false if the passed segment does not intersect the plane
        return Vector3f::segment_plane_intersect(seg_start, seg_end, closest_point, end);
    }
    return Vector3f::segment_plane_intersect(seg_start, seg_end, closest_point, start);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_Boundary_3D::get_closest_object(float& angle_deg, float &distance) const
{
    bool closest_found = false;
    uint8_t closest_sector = 0;
    uint8_t closest_layer = 0;

    // check boundary for shortest distance
    // only check for middle layers and higher
    // lower layers might contain ground, which will give false pre-arm failure
    for (uint8_t layer=PROXIMITY_MIDDLE_LAYER; layer<PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector<PROXIMITY_NUM_SECTORS; sector++) {
            if (_distance_valid[layer][sector]) {
                if (!closest_found || (_distance[layer][sector] < _distance[closest_layer][closest_sector])) {
                    closest_layer = layer;
                    closest_sector = sector;
                    closest_found = true;
                }
            }
        }
    }

    if (closest_found) {
        angle_deg = _angle_deg[closest_layer][closest_sector];
        distance = _distance[closest_layer][closest_sector];
    }
    return closest_found;
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity_Boundary_3D::get_horizontal_object_count() const
{
    return PROXIMITY_NUM_SECTORS;
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity_Boundary_3D::get_horizontal_object_angle_and_distance(uint8_t object_number, float &angle_deg, float &distance) const
{
    if ((object_number < PROXIMITY_NUM_SECTORS) && _distance_valid[PROXIMITY_MIDDLE_LAYER][object_number]) {
        angle_deg = _angle_deg[PROXIMITY_MIDDLE_LAYER][object_number];
        distance = _filtered_distance[PROXIMITY_MIDDLE_LAYER][object_number].get();
        return true;
    }
    return false;
}

// get an obstacle info for AP_Periph
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity_Boundary_3D::get_obstacle_info(uint8_t obstacle_num, float &angle_deg, float &pitch_deg, float &distance) const
{
    // obstacle num is just "flattened layers, and sectors"
    const uint8_t layer = obstacle_num / PROXIMITY_NUM_SECTORS;
    const uint8_t sector = obstacle_num % PROXIMITY_NUM_SECTORS;
    if (_distance_valid[layer][sector]) {
        angle_deg = _angle_deg[layer][sector];
        pitch_deg = _pitch_deg[layer][sector];
        distance = _filtered_distance[layer][sector].get();
        return true;
    }

    return false;
}

// Return filtered distance for the passed in face
bool AP_Proximity_Boundary_3D::get_filtered_distance(const Face &face, float &distance) const
{
    if (!face.valid()) {
        return false;
    }

    if (!_distance_valid[face.layer][face.sector]) {
        // invalid distace
        return false;
    }

    distance = _filtered_distance[face.layer][face.sector].get();
    return true;
}

// Get raw and filtered distances in 8 directions per layer
bool AP_Proximity_Boundary_3D::get_layer_distances(uint8_t layer_number, float dist_max, Proximity_Distance_Array &prx_dist_array, Proximity_Distance_Array &prx_filt_dist_array) const
{
    // cycle through all sectors filling in distances and orientations
    // see MAV_SENSOR_ORIENTATION for orientations (0 = forward, 1 = 45 degree clockwise from north, etc)
    bool valid_distances = false;
    prx_dist_array.offset_valid = 0;
    prx_filt_dist_array.offset_valid = 0;
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        prx_dist_array.orientation[i] = i;
        const AP_Proximity_Boundary_3D::Face face(layer_number, i);
        if (!face.valid()) {
            return false;
        }
        if (get_distance(face, prx_dist_array.distance[i]) && get_filtered_distance(face, prx_filt_dist_array.distance[i])) {
            valid_distances = true;
            prx_dist_array.offset_valid |= (1U << i);
            prx_filt_dist_array.offset_valid |= (1U << i);
        } else {
            prx_dist_array.distance[i] = dist_max;
            prx_filt_dist_array.distance[i] = dist_max;
        }
    }

    return valid_distances;
}

// reset the temporary boundary. This fills in distances with FLT_MAX
void AP_Proximity_Temp_Boundary::reset()
{
    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            _distances[layer][sector] = FLT_MAX;
        }
    }
}

// add a distance to the temp boundary if it is shorter than any other provided distance since the last time the boundary was reset
// pitch and yaw are in degrees, distance is in meters
void AP_Proximity_Temp_Boundary::add_distance(const AP_Proximity_Boundary_3D::Face &face, float pitch_deg, float yaw_deg, float distance_m)
{
    if (face.valid() && distance_m < _distances[face.layer][face.sector]) {
        _distances[face.layer][face.sector] = distance_m;
        _angle_deg[face.layer][face.sector] = yaw_deg;
        _pitch_deg[face.layer][face.sector] = pitch_deg;
    }
}

// fill the original 3D boundary with the contents of this temporary boundary
// prx_instance should be set to the proximity sensor's backend instance number
void AP_Proximity_Temp_Boundary::update_3D_boundary(uint8_t prx_instance, AP_Proximity_Boundary_3D &boundary)
{
    for (uint8_t layer=0; layer < PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            if (_distances[layer][sector] < FLT_MAX) {
                AP_Proximity_Boundary_3D::Face face{layer, sector};
                boundary.set_face_attributes(face, _pitch_deg[layer][sector], _angle_deg[layer][sector], _distances[layer][sector], prx_instance);
            }
        }
    }
}

