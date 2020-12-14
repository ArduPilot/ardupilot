#include "AP_Proximity_Backend.h"
#include "AP_Proximity_Boundary_3D.h"

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
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            float angle_rad = ((float)_sector_middle_deg[sector]+(PROXIMITY_SECTOR_WIDTH_DEG/2.0f));
            float pitch = ((float)_pitch_middle_deg[layer]);
            _sector_edge_vector[layer][sector].offset_bearing(angle_rad, pitch, 100.0f);
            _boundary_points[layer][sector] = _sector_edge_vector[layer][sector] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
        }
    }
}

// returns face corresponding to the provided yaw and (optionally) pitch
// pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle?)
// yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
AP_Proximity_Boundary_3D::Face AP_Proximity_Boundary_3D::get_face(float pitch, float yaw) const
{   
    const uint8_t sector = wrap_360(yaw + (PROXIMITY_SECTOR_WIDTH_DEG * 0.5f)) / 45.0f;
    const float pitch_limited = constrain_float(pitch, -75.0f, 74.9f);
    const uint8_t layer = (pitch_limited + 75.0f)/PROXIMITY_PITCH_WIDTH_DEG;
    return Face(layer, sector);
}

// Set the actual body-frame angle(yaw), pitch, and distance of the detected object.
// This method will also mark the sector and layer to be "valid", so this distance can be used for Obstacle Avoidance
void AP_Proximity_Boundary_3D::set_face_attributes(Face face, float angle, float pitch, float distance)
{
    if (!face.valid()) {
        return;
    }

    _angle[face.layer][face.sector] = angle;
    _pitch[face.layer][face.sector] = pitch;
    _distance[face.layer][face.sector] = distance;
    _distance_valid[face.layer][face.sector] = true;

    // update boundary used for simple avoidance
    update_boundary(face);
}

// add a distance to the boundary if it is shorter than any other provided distance since the last time the boundary was reset
// pitch and yaw are in degrees, distance is in meters
void AP_Proximity_Boundary_3D::add_distance(float pitch, float yaw, float distance)
{
    Face face = get_face(pitch, yaw);
    if (!_distance_valid[face.layer][face.sector] || (distance < _distance[face.layer][face.sector])) {
        _distance[face.layer][face.sector] = distance;
        _distance_valid[face.layer][face.sector] = true;
    }
}

// update boundary points used for object avoidance based on a single sector and pitch distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Boundary_3D::update_boundary(const Face face)
{
    // sanity check
    if (!face.valid()) {
        return;
    }

    const uint8_t layer = face.layer;
    const uint8_t sector = face.sector;

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= PROXIMITY_NUM_SECTORS) {
        next_sector = 0;
    }

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[layer][sector] && _distance_valid[layer][next_sector]) {
        shortest_distance = MIN(_distance[layer][sector], _distance[layer][next_sector]);
    } else if (_distance_valid[layer][sector]) {
        shortest_distance = _distance[layer][sector];
    } else if (_distance_valid[layer][next_sector]) {
        shortest_distance = _distance[layer][next_sector];
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
    uint8_t prev_sector = (sector == 0) ? PROXIMITY_NUM_SECTORS-1 : sector-1;
    shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[layer][prev_sector] && _distance_valid[layer][sector]) {
        shortest_distance = MIN(_distance[layer][prev_sector], _distance[layer][sector]);
    } else if (_distance_valid[layer][prev_sector]) {
        shortest_distance = _distance[layer][prev_sector];
    } else if (_distance_valid[layer][sector]) {
        shortest_distance = _distance[layer][sector];
    }
    _boundary_points[layer][prev_sector] = _sector_edge_vector[layer][prev_sector] * shortest_distance;

    // if the sector counter-clockwise from the previous sector has an invalid distance, set boundary to create a cup like boundary
    uint8_t prev_sector_ccw = (prev_sector == 0) ? PROXIMITY_NUM_SECTORS - 1 : prev_sector - 1;
    if (!_distance_valid[layer][prev_sector_ccw]) {
        _boundary_points[layer][prev_sector_ccw] = _sector_edge_vector[layer][prev_sector_ccw] * shortest_distance;
    }
}

// update middle layer boundary points
void AP_Proximity_Boundary_3D::update_middle_boundary()
{
    for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
        update_boundary(Face(PROXIMITY_MIDDLE_LAYER, sector));
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
void AP_Proximity_Boundary_3D::reset_face(Face face)
{
    if (!face.valid()) {
        return;
    }
    _distance_valid[face.layer][face.sector] = false;

    // update simple avoidance boundary
    update_boundary(face);
}

// get distance for a face.  returns true on success and fills in distance argument with distance in meters
bool AP_Proximity_Boundary_3D::get_distance(Face face, float &distance) const
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
// this method iterates through the entire 3-D boundary and checks which layer has at least one valid distance
uint8_t AP_Proximity_Boundary_3D::get_obstacle_count()
{
    uint8_t obstacle_count = 0;
    // reset entire array to false
    memset(_active_layer, 0, sizeof(_active_layer));
    // check if this layer has atleast one valid sector
    for (uint8_t layer=0; layer<PROXIMITY_NUM_LAYERS; layer++) {
        for (uint8_t sector=0; sector<PROXIMITY_NUM_SECTORS; sector++ ) {
            if (_distance_valid[layer][sector]) {
                _active_layer[layer] = true;
                obstacle_count += PROXIMITY_NUM_SECTORS;
                break;
            }
        }
    }
    return obstacle_count;
}

// Converts obstacle_num passed from avoidance library into appropriate layer and sector
// This is packed into a Boundary Location object and returned 
AP_Proximity_Boundary_3D::Face AP_Proximity_Boundary_3D::convert_obstacle_num_to_face(uint8_t obstacle_num) const
{
    const uint8_t active_layer = obstacle_num / PROXIMITY_NUM_SECTORS;
    uint8_t layer_count = 0;
    uint8_t layer = 0;
    for (uint8_t i=0; i < PROXIMITY_NUM_LAYERS; i++) {
        if (_active_layer[i]) {
            layer_count++;
        }
        if (layer_count == (active_layer + 1)) {
            layer = i;
            break;
        }
    }
    const uint8_t sector = obstacle_num % PROXIMITY_NUM_SECTORS;

    return AP_Proximity_Boundary_3D::Face(layer, sector);
}

// WARNING: This requires get_obstacle_count() to be called before calling this method
// Appropriate layer and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given layer
// Then returns the closest point on this line from vehicle, in body-frame. 
// Used by GPS based Simple Avoidance  
void AP_Proximity_Boundary_3D::get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const
{
    const AP_Proximity_Boundary_3D::Face face = convert_obstacle_num_to_face(obstacle_num);
    const uint8_t sector_end = face.sector;
    uint8_t sector_start = face.sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[face.layer][sector_start];
    const Vector3f end = _boundary_points[face.layer][sector_end];
    vec_to_obstacle = Vector3f::closest_point_between_line_and_point(start, end, Vector3f{0.0f, 0.0f, 0.0f});
}

// WARNING: This requires get_obstacle_count() to be called before calling this method
// Appropriate layer and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given layer
// Then returns the closest point on this line from the segment that was passed, in body-frame.
// Used by GPS based Simple Avoidance  - for "brake mode" 
float AP_Proximity_Boundary_3D::distance_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{   
    const AP_Proximity_Boundary_3D::Face face = convert_obstacle_num_to_face(obstacle_num);
    const uint8_t sector_end = face.sector;
    uint8_t sector_start = face.sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[face.layer][sector_start];
    const Vector3f end = _boundary_points[face.layer][sector_end];
    return Vector3f::segment_to_segment_dist(seg_start, seg_end, start, end, closest_point);
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
        angle_deg = _angle[closest_layer][closest_sector];
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
bool AP_Proximity_Boundary_3D::get_horizontal_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if ((object_number < PROXIMITY_NUM_SECTORS) && _distance_valid[PROXIMITY_MIDDLE_LAYER][object_number]) {
        angle_deg = _angle[PROXIMITY_MIDDLE_LAYER][object_number];
        distance = _distance[PROXIMITY_MIDDLE_LAYER][object_number];
        return true;
    }
    return false;
}
