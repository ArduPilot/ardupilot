/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
    AP_ADSB.cpp

    ADS-B RF based collision avoidance module
    https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
*/

#include <AP_HAL/AP_HAL.h>
#include "AP_ADSB.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ADSB::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable ADS-B
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE",     0, AP_ADSB, _enabled,    0),

    // @Param: BEHAVIOR
    // @DisplayName: Collision Avoidance Behavior
    // @Description: Collision Avoidance Behavior selector
    // @Values: 0:None,1:Loiter,2:LoiterAndDescend
    // @User: Advanced
    AP_GROUPINFO("BEHAVIOR",   1, AP_ADSB, _behavior, ADSB_BEHAVIOR_NONE),

    AP_GROUPEND
};

/*
 * Initialize variables and allocate memory for array
 */
void AP_ADSB::init(void)
{
    if (_vehicle_list == NULL) {
        _vehicle_list = new adsb_vehicle_t[VEHICLE_LIST_LENGTH];

        if (_vehicle_list == NULL) {
            // dynamic RAM allocation of _vehicle_list[] failed, disable gracefully
            hal.console->printf("Unable to initialize ADS-B vehicle list\n");
            _enabled.set(0);
        }
    }
    _vehicle_count = 0;
    _furthest_vehicle_index = 0;
    _furthest_vehicle_distance = 0;
    _another_vehicle_within_radius = false;
    _is_evading_threat = false;
}

/*
 * de-initialize and free up some memory
 */
void AP_ADSB::deinit(void)
{
    if (_vehicle_list != NULL) {
        delete [] _vehicle_list;
        _vehicle_list = NULL;
    }
    _vehicle_count = 0;
}

/*
 * periodic update to handle vehicle timeouts and trigger collision detection
 */
void AP_ADSB::update(void)
{
    if (!_enabled) {
        if (_vehicle_list != NULL) {
            deinit();
        }
        // nothing to do
        return;
    } else if (_vehicle_list == NULL)  {
        init();
        return;
    }

    uint16_t index = 0;
    while (index < _vehicle_count) {
        // check list and drop stale vehicles. When disabled, the list will get flushed
        if (AP_HAL::millis() - _vehicle_list[index].last_update_ms > VEHICLE_TIMEOUT_MS) {
             // don't increment index, we want to check this same index again because the contents changed
            // also, if we're disabled then clear the list
            delete_vehicle(index);
        } else {
            index++;
        }
    }

    perform_threat_detection();
}

/*
 * calculate threat vectors
 */
void AP_ADSB::perform_threat_detection(void)
{
    Location my_loc;
    if (_vehicle_count == 0 ||
        _ahrs.get_position(my_loc) == false) {
        // nothing to do or current location is unknown so we can't calculate any collisions
        _another_vehicle_within_radius = false;
        return;
    }

    bool no_threat_within_radius = true;
    for (uint16_t index = 0; index < _vehicle_count; index++) {
        // TODO: perform more advanced threat detection
        Location vehicle_loc = get_location(_vehicle_list[index]);

        // if within radius, set flag and enforce a double radius to clear flag
        float threat_distance = get_distance(vehicle_loc, my_loc);
        if (threat_distance <= 2*VEHICLE_THREAT_RADIUS_M) {
            no_threat_within_radius = false;
            if (threat_distance <= VEHICLE_THREAT_RADIUS_M) {
                _another_vehicle_within_radius = true;
            }
        } // if get
    } // for

    if (no_threat_within_radius) {
        _another_vehicle_within_radius = false;
    }

}

/*
 * Convert/Extract a Location from a vehicle
 */
Location AP_ADSB::get_location(const adsb_vehicle_t &vehicle) const
{
    Location loc {};
    loc.alt = vehicle.info.altitude * 100;
    loc.lat = vehicle.info.lat * 1e7;
    loc.lng = vehicle.info.lon * 1e7;
    loc.flags.relative_alt = false;
    return loc;
}

/*
 *  delete a vehicle by copying last vehicle to
 *  current index then decrementing count
 */
void AP_ADSB::delete_vehicle(uint16_t index)
{
    if (index < _vehicle_count) {
        if (index != _vehicle_count-1) {
            _vehicle_list[index] = _vehicle_list[_vehicle_count-1];
        }
        memset(&_vehicle_list[_vehicle_count-1], 0, sizeof(adsb_vehicle_t));
        _vehicle_count--;
    }
}

/*
 * Search _vehicle_list for the given vehicle. A match
 * depends on ICAO_address. Returns true if match found
 * and index is populated. otherwise, return false.
 */
bool AP_ADSB::find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const
{
    for (uint16_t i = 0; i < _vehicle_count; i++) {
        if (_vehicle_list[i].info.ICAO_address == vehicle.info.ICAO_address) {
            *index = i;
            return true;
        }
    }
    return false;
}

/*
 * Update the vehicle list. If the vehicle is already in the
 * list then it will update it, otherwise it will be added.
 */
void AP_ADSB::update_vehicle(const mavlink_message_t* packet)
{
    if (_vehicle_list == NULL) {
        // We are only null when disabled. Updating is inhibited.
        return;
    }

    uint16_t index;
    adsb_vehicle_t vehicle {};

    mavlink_msg_adsb_vehicle_decode(packet, &vehicle.info);

    if (find_index(vehicle, &index)) {
        // found, update it
        set_vehicle(index, vehicle);
    } else if (_vehicle_count < VEHICLE_LIST_LENGTH-1) {
        // not found, add it if there's room
        set_vehicle(_vehicle_count, vehicle);
        _vehicle_count++;
    } else {
        // TODO: buffer is full, delete the vehicle that is furthest away since it is probably not a useful threat to monitor
    }
}

/*
 * Copy a vehicle's data into the list
 */
void AP_ADSB::set_vehicle(uint16_t index, const adsb_vehicle_t &vehicle)
{
    if (index < VEHICLE_LIST_LENGTH) {
        _vehicle_list[index] = vehicle;
        _vehicle_list[index].last_update_ms = AP_HAL::millis();
    }
}

