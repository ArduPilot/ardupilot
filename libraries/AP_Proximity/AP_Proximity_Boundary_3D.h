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

#pragma once

#define PROXIMITY_NUM_SECTORS         8       // number of sectors
#define PROXIMITY_NUM_LAYERS          5       // num of stacks in a sector
#define PROXIMITY_MIDDLE_LAYER        2       // middle stack 
#define PROXIMITY_PITCH_WIDTH_DEG     30      // width between each stack in degrees
#define PROXIMITY_SECTOR_WIDTH_DEG    45.0f   // width of sectors in degrees
#define PROXIMITY_BOUNDARY_DIST_MIN   0.6f    // minimum distance for a boundary point.  This ensures the object avoidance code doesn't think we are outside the boundary.
#define PROXIMITY_BOUNDARY_DIST_DEFAULT 100   // if we have no data for a sector, boundary is placed 100m out
#define DISTANCE_MAX               999999.0f  // arbritary "large" distance 

class AP_Proximity_Boundary_3D 
{ 
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Boundary_3D();

    // This class is used to store the stack and sector as a single packet to access and modify the 3-D boundary 
    class Boundary_Location
    {   
    public:
        // constructor when both stack and sector are passed
        Boundary_Location(uint8_t Sector, uint8_t Stack) { sector = Sector; stack = Stack; }
        // constructor defaults to "middle(horizontal) layer" if only sector is passed
        Boundary_Location(uint8_t Sector) { sector = Sector; stack = PROXIMITY_MIDDLE_LAYER; }
        
        uint8_t stack; // vertical "steps" on the 3D Boundary 
        uint8_t sector; // horizontal "steps" on the 3D Boundary
    }; 
    
    // returns Boundary_Location object consisting of appropriate stack and sector
    // corresponding to the yaw and pitch.
    // Pitch defaults to zero if only yaw is passed to this method
    // Yaw is the horizontal body-frame angle the detected object makes with the vehicle
    // Pitch is the vertical body-frame angle the detected object makes with the vehicle 
    Boundary_Location get_sector(float yaw, float pitch = 0.0f);

    // Set the actual body-frame angle(yaw), pitch, and distance of the detected object.
    // This method will also mark the sector and stack to be "valid"
    // This distance can then be used for Obstacle Avoidance
    void set_attributes(const Boundary_Location& bnd_loc, float angle, float pitch, float distance);
    
    // Set the actual body-frame angle(yaw), pitch, and distance of the detected object.
    // This method will also mark the sector and stack to be "valid", 
    // This distance can then be used for Obstacle Avoidance
    // Assume detected obstacle is horizontal (zero pitch), if no pitch is passed 
    void set_attributes(const Boundary_Location& bnd_loc, float angle, float distance) { set_attributes(bnd_loc, angle, 0.0f, distance); }

    // update boundary points used for object avoidance based on a single sector and pitch distance changing
    //   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
    //   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
    void update_boundary(const Boundary_Location& bnd_loc);

    // Reset this location, specified by Boundary_Location object, back to default
    // i.e Distance is marked as not-valid, and set to a large number.
    void reset_sector(const Boundary_Location& bnd_loc);
    // Reset all horizontal sectors
    void reset_all_horizontal_sectors();
    // Reset all stacks and sectors
    void reset_all_sectors_and_stacks();

    // Get values given the stack and sector as a Boundary_Location object
    float get_angle(const Boundary_Location& bnd_loc) const { return _angle[bnd_loc.sector][bnd_loc.stack]; }
    float get_pitch(const Boundary_Location& bnd_loc) const { return _pitch[bnd_loc.sector][bnd_loc.stack]; }
    float get_distance(const Boundary_Location& bnd_loc) const { return _distance[bnd_loc.sector][bnd_loc.stack]; }
    bool check_distance_valid(const Boundary_Location& bnd_loc) const { return _distance_valid[bnd_loc.sector][bnd_loc.stack]; }
   
    // Get the total number of obstacles 
    // This method iterates through the entire 3-D boundary and checks which layer has atleast one valid distance
    uint8_t get_obstacle_count();

    // WARNING: This requires get_obstacle_count() to be called before calling this method
    // Appropriate stack and sector are found from the passed obstacle_num
    // This function then draws a line between this sector, and sector + 1 at the given stack 
    // Then returns the closest point on this line from vehicle, in body-frame. 
    void get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_boundary) const;

    // WARNING: This requires get_obstacle_count() to be called before calling this method
    // Appropriate stack and sector are found from the passed obstacle_num
    // This function then draws a line between this sector, and sector + 1 at the given stack  
    // Then returns the closest point on this line from the segment that was passed, in body-frame.
    // Used by GPS based Simple Avoidance  - for "brake mode" 
    float distance_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const;

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_horizontal_object_count() const;
    bool get_horizontal_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const;

    // sectors
    const uint16_t _sector_middle_deg[PROXIMITY_NUM_SECTORS] {0, 45, 90, 135, 180, 225, 270, 315};    // middle angle of each sector
    // layers
    const int16_t _pitch_middle_deg[PROXIMITY_NUM_LAYERS] {-60, -30, 0, 30, 60};

private:
    // initialise the boundary and sector_edge_vector array used for object avoidance
    void init_boundary();

    // Converts obstacle_num passed from avoidance library into appropriate stack and sector
    // This is packed into a Boundary Location object and returned 
    Boundary_Location convert_obstacle_num_to_boundary_loc(uint8_t obstacle_num) const;

    Vector3f _sector_edge_vector[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];
    Vector3f _boundary_points[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];
    
    // sensor data
    float _angle[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];            // angle to closest object within each sector and stack
    float _pitch[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];            // pitch to the closest object within each sector and stack
    float _distance[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];         // distance to closest object within each sector and stack
    bool _distance_valid[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_LAYERS];    // true if a valid distance received for each sector and stack
    bool _active_layer[PROXIMITY_NUM_LAYERS];                             // layers which have atleast one valid distance are marked true
};

typedef AP_Proximity_Boundary_3D::Boundary_Location boundary_location;
