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

class AP_Beacon_Backend;

#define AP_BEACON_MAX_BEACONS 4
#define AP_BEACON_TIMEOUT_MS 300

class AP_Beacon
{
public:
    friend class AP_Beacon_Backend;

    AP_Beacon(AP_SerialManager &_serial_manager);

    // external position backend types (used by _TYPE parameter)
    enum AP_BeaconType {
        AP_BeaconType_None   = 0,
        AP_BeaconType_Pozyx  = 1,
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

    // return origin of position estimate system
    bool get_origin(Location &origin_loc) const;

    // return vehicle position in NED from position estimate system's origin
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

    // return NED position of beacon relative to the beacon systems origin
    Vector3f beacon_position(uint8_t beacon_instance) const;

    // return last update time from beacon
    uint32_t beacon_last_update_ms(uint8_t beacon_instance) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // check if device is ready
    bool device_ready(void) const;

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
};
