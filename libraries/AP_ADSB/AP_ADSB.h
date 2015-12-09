/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_ADSB_H
#define AP_ADSB_H
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
/*
    ADS-B RF based collision avoidance module
    https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast

  Tom Pittenger, November 2015
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#define VEHICLE_THREAT_RADIUS_M         1000
#define VEHICLE_LIST_LENGTH             25      // # of ADS-B vehicles to remember at any given time
#define VEHICLE_TIMEOUT_MS              10000   // if no updates in this time, drop it from the list

class AP_ADSB
{
public:
    enum ADSB_BEHAVIOR {
        ADSB_BEHAVIOR_NONE = 0,
        ADSB_BEHAVIOR_LOITER = 1,
        ADSB_BEHAVIOR_LOITER_AND_DESCEND = 2,
        ADSB_BEHAVIOR_GUIDED = 3
    };

    enum ADSB_THREAT_LEVEL {
        ADSB_THREAT_LOW = 0,
        ADSB_THREAT_HIGH = 1
    };

    struct adsb_vehicle_t {
        mavlink_adsb_vehicle_t info; // the whole mavlink struct with all the juicy details. sizeof() == 38
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
        ADSB_THREAT_LEVEL threat_level;   // basic threat level
    };


    // Constructor
    AP_ADSB(AP_AHRS &ahrs) :
        _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update(void);

    // add or update vehicle_list from inbound mavlink msg
    void update_vehicle(const mavlink_message_t* msg);

    bool get_possible_threat()  { return _enabled && _another_vehicle_within_radius; }

    ADSB_BEHAVIOR get_behavior()  { return (ADSB_BEHAVIOR)(_behavior.get()); }
    bool get_is_evading_threat()  { return _enabled && _is_evading_threat; }
    void set_is_evading_threat(bool is_evading) { if (_enabled) { _is_evading_threat = is_evading; } }
    uint16_t get_vehicle_count() { return _vehicle_count; }

private:

    // initialize _vehicle_list
    void init();

    // free _vehicle_list
    void deinit();

    // compares current vector against vehicle_list to detect threats
    void perform_threat_detection(void);

    // extract a location out of a vehicle item
    Location get_location(const adsb_vehicle_t &vehicle) const;

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    bool find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const;

    // remove a vehicle from the list
    void delete_vehicle(uint16_t index);

    void set_vehicle(uint16_t index, const adsb_vehicle_t &vehicle);

    // reference to AHRS, so we can ask for our position,
    // heading and speed
    const AP_AHRS &_ahrs;

    AP_Int8     _enabled;
    AP_Int8     _behavior;
    adsb_vehicle_t *_vehicle_list;
    uint16_t    _vehicle_count = 0;
    bool        _another_vehicle_within_radius = false;
    bool        _is_evading_threat = false;

    // index of and distance to vehicle with lowest threat
    uint16_t    _lowest_threat_index = 0;
    float       _lowest_threat_distance = 0;

    // index of and distance to vehicle with highest threat
    uint16_t    _highest_threat_index = 0;
    float       _highest_threat_distance = 0;
};
#endif // AP_ADSB_H
