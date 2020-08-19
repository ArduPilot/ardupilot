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
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/Location.h>

class AP_Beacon_Backend;

#define AP_BEACON_MAX_BEACONS 4
#define AP_BEACON_TIMEOUT_MS 300
#define AP_BEACON_MINIMUM_FENCE_BEACONS 3

class AP_Beacon
{
public:
    friend class AP_Beacon_Backend;

    AP_Beacon(AP_SerialManager &_serial_manager);

    // get singleton instance
    static AP_Beacon *get_singleton() { return _singleton; }

    // external position backend types (used by _TYPE parameter)
    enum AP_BeaconType {
        AP_BeaconType_None   = 0,
        AP_BeaconType_Pozyx  = 1,
        AP_BeaconType_Marvelmind = 2,
        AP_BeaconType_SITL   = 10
    };

    // The AP_BeaconState structure is filled in by the backend driver
    struct BeaconState {
        uint16_t id;            // unique id of beacon
        bool     healthy;       // true if beacon is healthy
        float    distance;      // distance from vehicle to beacon (in meters)
        uint32_t distance_update_ms;    // system time of last update from this beacon
        Vector3f position;      // location of beacon as an offset from origin in NED in meters
    };

    // initialise any available position estimators
    void init(void);

    // return true if beacon feature is enabled
    bool enabled(void);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy(void);

    // update state of all beacons
    void update(void);

    // return origin of position estimate system in lat/lon
    bool get_origin(Location &origin_loc) const;

    // return vehicle position in NED from position estimate system's origin in meters
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const;

    // return the number of beacons
    uint8_t count() const;

    // methods to return beacon specific information

    // return all beacon data
    bool get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const;

    // return individual beacon's id
    uint8_t beacon_id(uint8_t beacon_instance) const;

    // return beacon health
    bool beacon_healthy(uint8_t beacon_instance) const;

    // return distance to beacon in meters
    float beacon_distance(uint8_t beacon_instance) const;

    // return NED position of beacon in meters relative to the beacon systems origin
    Vector3f beacon_position(uint8_t beacon_instance) const;

    // return last update time from beacon in milliseconds
    uint32_t beacon_last_update_ms(uint8_t beacon_instance) const;

    // update fence boundary array
    void update_boundary_points();

    // return fence boundary array
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_Beacon *_singleton;

    // check if device is ready
    bool device_ready(void) const;

    // find next boundary point from an array of boundary points given the current index into that array
    // returns true if a next point can be found
    //   current_index should be an index into the boundary_pts array
    //   start_angle is an angle (in radians), the search will sweep clockwise from this angle
    //   the index of the next point is returned in the next_index argument
    //   the angle to the next point is returned in the next_angle argument
    static bool get_next_boundary_point(const Vector2f* boundary, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle);

    // parameters
    AP_Int8 _type;
    AP_Float origin_lat;
    AP_Float origin_lon;
    AP_Float origin_alt;
    AP_Int16 orient_yaw;

    // external references
    AP_Beacon_Backend *_driver;
    AP_SerialManager &serial_manager;

    // last known position
    Vector3f veh_pos_ned;
    float veh_pos_accuracy;
    uint32_t veh_pos_update_ms;

    // individual beacon data
    uint8_t num_beacons = 0;
    BeaconState beacon_state[AP_BEACON_MAX_BEACONS];

    // fence boundary
    Vector2f boundary[AP_BEACON_MAX_BEACONS+1]; // array of boundary points (used for fence)
    uint8_t boundary_num_points;                // number of points in boundary
    uint8_t boundary_num_beacons;               // total number of beacon points consumed while building boundary
};

namespace AP {
    AP_Beacon *beacon();
};
