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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"

#define PROXIMITY_SECTORS_MAX   12  // maximum number of sectors
#define PROXIMITY_BOUNDARY_DIST_MIN 0.6f    // minimum distance for a boundary point.  This ensures the object avoidance code doesn't think we are outside the boundary.
#define PROXIMITY_BOUNDARY_DIST_DEFAULT 100 // if we have no data for a sector, boundary is placed 100m out

class AP_Proximity_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Proximity_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    // get maximum and minimum distances (in meters) of sensor
    virtual float distance_max() const = 0;
    virtual float distance_min() const = 0;

    // get distance upwards in meters. returns true on success
    virtual bool get_upward_distance(float &distance) const { return false; }

    // handle mavlink DISTANCE_SENSOR messages
    virtual void handle_msg(mavlink_message_t *msg) {}

    // get distance in meters in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance
    bool get_horizontal_distance(float angle_deg, float &distance) const;

    // get boundary points around vehicle for use by avoidance
    //   returns nullptr and sets num_points to zero if no boundary can be returned
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const;

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_object_count() const;
    bool get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const;

    // get distances in 8 directions. used for sending distances to ground station
    bool get_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const;

protected:

    // set status and update valid_count
    void set_status(AP_Proximity::Proximity_Status status);

    // find which sector a given angle falls into
    bool convert_angle_to_sector(float angle_degrees, uint8_t &sector) const;

    // initialise the boundary and sector_edge_vector array used for object avoidance
    //   should be called if the sector_middle_deg or _setor_width_deg arrays are changed
    void init_boundary();

    // update boundary points used for object avoidance based on a single sector's distance changing
    //   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
    //   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
    void update_boundary_for_sector(uint8_t sector);

    // get ignore area info
    uint8_t get_ignore_area_count() const;
    bool get_ignore_area(uint8_t index, uint16_t &angle_deg, uint8_t &width_deg) const;
    bool get_next_ignore_start_or_end(uint8_t start_or_end, int16_t start_angle, int16_t &ignore_start) const;

    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state

    // sectors
    uint8_t _num_sectors = 8;
    uint16_t _sector_middle_deg[PROXIMITY_SECTORS_MAX] = {0, 45, 90, 135, 180, 225, 270, 315, 0, 0, 0, 0};  // middle angle of each sector
    uint8_t _sector_width_deg[PROXIMITY_SECTORS_MAX] = {45, 45, 45, 45, 45, 45, 45, 45, 0, 0, 0, 0};        // width (in degrees) of each sector

    // sensor data
    float _angle[PROXIMITY_SECTORS_MAX];            // angle to closest object within each sector
    float _distance[PROXIMITY_SECTORS_MAX];         // distance to closest object within each sector
    bool _distance_valid[PROXIMITY_SECTORS_MAX];    // true if a valid distance received for each sector

    // fence boundary
    Vector2f _sector_edge_vector[PROXIMITY_SECTORS_MAX];    // vector for right-edge of each sector, used to speed up calculation of boundary
    Vector2f _boundary_point[PROXIMITY_SECTORS_MAX];        // bounding polygon around the vehicle calculated conservatively for object avoidance
};
