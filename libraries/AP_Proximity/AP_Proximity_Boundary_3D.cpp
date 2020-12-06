#include "AP_Proximity_Backend.h"
#include "AP_Proximity_Boundary_3D.h"

/*
  Constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Boundary_3D::AP_Proximity_Boundary_3D() 
{
    // initialise sector edge vector used for building the boundary fence
    init_boundary();
}

// initialise the boundary and sector_edge_vector array used for object avoidance
//   should be called if the sector_middle_deg or _sector_width_deg arrays are changed
void AP_Proximity_Boundary_3D::init_boundary()
{
    for (uint8_t stack = 0; stack < PROXIMITY_NUM_LAYERS; stack ++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            float angle_rad = ((float)_sector_middle_deg[sector]+(PROXIMITY_SECTOR_WIDTH_DEG/2.0f));
            float pitch = ((float)_pitch_middle_deg[stack]);
            _sector_edge_vector[sector][stack].offset_bearing(angle_rad, pitch, 100.0f);
            _boundary_points[sector][stack] = _sector_edge_vector[sector][stack] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
        }
    }
}

// returns Boundary_Location object consisting of appropriate stack and sector corresponding to the yaw and pitch.
// Pitch defaults to zero if only yaw is passed to this method
// Yaw is the horizontal body-frame angle the detected object makes with the vehicle
// Pitch is the vertical body-frame angle the detected object makes with the vehicle 
boundary_location AP_Proximity_Boundary_3D::get_sector(float yaw, float pitch) 
{   
    const uint8_t sector = wrap_360(yaw + (PROXIMITY_SECTOR_WIDTH_DEG * 0.5f)) / 45.0f;
    const float pitch_degrees = constrain_float(pitch, -75.0f, 74.9f);
    const uint8_t stack = (pitch_degrees + 75.0f)/PROXIMITY_PITCH_WIDTH_DEG;
    return boundary_location{sector, stack};
}

// Set the actual body-frame angle(yaw), pitch, and distance of the detected object.
// This method will also mark the sector and stack to be "valid", so this distance can be used for Obstacle Avoidance
void AP_Proximity_Boundary_3D::set_attributes(const Boundary_Location& bnd_loc, float angle, float pitch, float distance)
{   
    const uint8_t sector = bnd_loc.sector;
    const uint8_t stack = bnd_loc.stack;
    _angle[sector][stack] = angle;
    _pitch[sector][stack] = pitch;
    _distance[sector][stack] = distance;    
    _distance_valid[sector][stack] = true;
}

// update boundary points used for object avoidance based on a single sector and pitch distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Boundary_3D::update_boundary(const Boundary_Location& bnd_loc) 
{
    const uint8_t sector = bnd_loc.sector;
    const uint8_t layer = bnd_loc.stack;

    // sanity check
    if (sector >= PROXIMITY_NUM_SECTORS) {
        return;
    }

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= PROXIMITY_NUM_SECTORS) {
        next_sector = 0;
    }

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[sector][layer] && _distance_valid[next_sector][layer]) {
        shortest_distance = MIN(_distance[sector][layer], _distance[next_sector][layer]);
    } else if (_distance_valid[sector][layer]) {
        shortest_distance = _distance[sector][layer];
    } else if (_distance_valid[next_sector][layer]) {
        shortest_distance = _distance[next_sector][layer];
    }
    if (shortest_distance < PROXIMITY_BOUNDARY_DIST_MIN) {
        shortest_distance = PROXIMITY_BOUNDARY_DIST_MIN;
    }
    _boundary_points[sector][layer] = _sector_edge_vector[sector][layer] * shortest_distance;

    // if the next sector (clockwise) has an invalid distance, set boundary to create a cup like boundary
    if (!_distance_valid[next_sector][layer]) {
        _boundary_points[next_sector][layer] = _sector_edge_vector[next_sector][layer] * shortest_distance;
    }

    // repeat for edge between sector and previous sector
    uint8_t prev_sector = (sector == 0) ? PROXIMITY_NUM_SECTORS-1 : sector-1;
    shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[prev_sector][layer] && _distance_valid[sector][layer]) {
        shortest_distance = MIN(_distance[prev_sector][layer], _distance[sector][layer]);
    } else if (_distance_valid[prev_sector][layer]) {
        shortest_distance = _distance[prev_sector][layer];
    } else if (_distance_valid[sector][layer]) {
        shortest_distance = _distance[sector][layer];
    }
    _boundary_points[prev_sector][layer] = _sector_edge_vector[prev_sector][layer] * shortest_distance;

    // if the sector counter-clockwise from the previous sector has an invalid distance, set boundary to create a cup like boundary
    uint8_t prev_sector_ccw = (prev_sector == 0) ? PROXIMITY_NUM_SECTORS - 1 : prev_sector - 1;
    if (!_distance_valid[prev_sector_ccw][layer]) {
        _boundary_points[prev_sector_ccw][layer] = _sector_edge_vector[prev_sector_ccw][layer] * shortest_distance;
    }
}

// Reset this location, specified by Boundary_Location object, back to default
// i.e Distance is marked as not-valid, and set to a large number.
void AP_Proximity_Boundary_3D::reset_sector(const Boundary_Location& bnd_loc)
{
    _distance[bnd_loc.sector][bnd_loc.stack] = DISTANCE_MAX;
    _distance_valid[bnd_loc.sector][bnd_loc.stack] = false;
}

// Reset all horizontal sectors
// i.e Distance is marked as not-valid, and set to a large number for all horizontal sectors.
void AP_Proximity_Boundary_3D::reset_all_horizontal_sectors()
{
    for (uint8_t i=0; i < PROXIMITY_NUM_SECTORS; i++) {
        const Boundary_Location bnd_loc{i};
        reset_sector(bnd_loc);
    }
}

// Reset all stacks and sectors
// i.e Distance is marked as not-valid, and set to a large number for all stacks and sectors.
void AP_Proximity_Boundary_3D::reset_all_sectors_and_stacks()
{
    for (uint8_t j=0; j < PROXIMITY_NUM_LAYERS; j++) {
        for (uint8_t i=0; i < PROXIMITY_NUM_SECTORS; i++) {
            const Boundary_Location bnd_loc{i, j};
            reset_sector(bnd_loc);
        }
    }
}

// get the total number of obstacles 
// this method iterates through the entire 3-D boundary and checks which layer has atleast one valid distance
uint8_t AP_Proximity_Boundary_3D::get_obstacle_count()
{
    uint8_t obstacle_count = 0;
    // reset entire array to false
    memset(_active_layer, 0, sizeof(_active_layer));
    // check if this layer has atleast one valid sector
    for (uint8_t j=0; j<PROXIMITY_NUM_LAYERS; j++) {
        for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++ ) {
            if (_distance_valid[i][j]) {
                _active_layer[j] = true;
                obstacle_count += PROXIMITY_NUM_SECTORS;
                break;
            }
        }
    }
    return obstacle_count;
}

// Converts obstacle_num passed from avoidance library into appropriate stack and sector
// This is packed into a Boundary Location object and returned 
boundary_location AP_Proximity_Boundary_3D::convert_obstacle_num_to_boundary_loc(uint8_t obstacle_num) const
{
    const uint8_t active_layer = obstacle_num / PROXIMITY_NUM_SECTORS;
    uint8_t layer_count = 0;
    uint8_t stack = 0;
    for (uint8_t i=0; i < PROXIMITY_NUM_LAYERS; i++) {
        if (_active_layer[i]) {
            layer_count++;
        }
        if (layer_count == (active_layer + 1)) {
            stack = i;
            break;
        }
    }
    const uint8_t sector = obstacle_num % PROXIMITY_NUM_SECTORS;

    return boundary_location{sector, stack};
}

// WARNING: This requires get_obstacle_count() to be called before calling this method
// Appropriate stack and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given stack 
// Then returns the closest point on this line from vehicle, in body-frame. 
// Used by GPS based Simple Avoidance  
void AP_Proximity_Boundary_3D::get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const
{   
    const boundary_location bnd_loc = convert_obstacle_num_to_boundary_loc(obstacle_num);
    const uint8_t sector_end = bnd_loc.sector;
    uint8_t sector_start = bnd_loc.sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[sector_start][bnd_loc.stack];
    const Vector3f end = _boundary_points[sector_end][bnd_loc.stack];
    vec_to_obstacle = Vector3f::closest_point_between_line_and_point(start, end, Vector3f{0.0f, 0.0f, 0.0f});
}

// WARNING: This requires get_obstacle_count() to be called before calling this method
// Appropriate stack and sector are found from the passed obstacle_num
// This function then draws a line between this sector, and sector + 1 at the given stack  
// Then returns the closest point on this line from the segment that was passed, in body-frame.
// Used by GPS based Simple Avoidance  - for "brake mode" 
float AP_Proximity_Boundary_3D::distance_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{   
    const boundary_location bnd_loc = convert_obstacle_num_to_boundary_loc(obstacle_num);
    const uint8_t sector_end = bnd_loc.sector;
    uint8_t sector_start = bnd_loc.sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[sector_start][bnd_loc.stack];
    const Vector3f end = _boundary_points[sector_end][bnd_loc.stack];
    return Vector3f::segment_to_segment_dist(seg_start, seg_end, start, end, closest_point);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_Boundary_3D::get_closest_object(float& angle_deg, float &distance) const
{
    bool sector_found = false;
    uint8_t sector = 0;
    uint8_t stack = 0;

    // check boundary for shortest distance
    // only check for middle layers and higher
    // lower layers might contain ground, which will give false pre-arm failure
    for (uint8_t j=PROXIMITY_MIDDLE_LAYER; j<PROXIMITY_NUM_LAYERS; j++) {
        for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
            if (_distance_valid[i][j]) {
                if (!sector_found || (_distance[i][j] < _distance[sector][stack])) {
                    sector = i;
                    stack = j;
                    sector_found = true;
                }
            }
        }
    }

    if (sector_found) {
        angle_deg = _angle[sector][stack];
        distance = _distance[sector][stack];
    }
    return sector_found;
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
    if (object_number < PROXIMITY_NUM_SECTORS && _distance_valid[object_number][PROXIMITY_MIDDLE_LAYER]) {
        angle_deg = _angle[object_number][PROXIMITY_MIDDLE_LAYER];
        distance = _distance[object_number][PROXIMITY_MIDDLE_LAYER];
        return true;
    }
    return false;
}
