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
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ADSB::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable ADSB
    // @Description: Enable ADS-B
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",     0, AP_ADSB, _enabled,    0),

    // @Param: BEHAVIOR
    // @DisplayName: ADSB based Collision Avoidance Behavior
    // @Description: ADSB based Collision Avoidance Behavior selector
    // @Values: 0:None,1:Loiter,2:LoiterAndDescend
    // @User: Advanced
    AP_GROUPINFO("BEHAVIOR",   1, AP_ADSB, avoid_state.behavior, ADSB_BEHAVIOR_NONE),

    // @Param: LIST_MAX
    // @DisplayName: ADSB vehicle list size
    // @Description: ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("LIST_MAX",   2, AP_ADSB, in_state.list_size_param, ADSB_VEHICLE_LIST_SIZE_DEFAULT),


    AP_GROUPEND
};

/*
 * Initialize variables and allocate memory for array
 */
void AP_ADSB::init(void)
{
    // in_state
    in_state.vehicle_count = 0;
    if (in_state.vehicle_list == nullptr) {
        if (in_state.list_size_param != constrain_int16(in_state.list_size_param, 1, ADSB_VEHICLE_LIST_SIZE_MAX)) {
            in_state.list_size_param.set_and_notify(ADSB_VEHICLE_LIST_SIZE_DEFAULT);
            in_state.list_size_param.save();
        }
        in_state.list_size = in_state.list_size_param;
        in_state.vehicle_list = new adsb_vehicle_t[in_state.list_size];

        if (in_state.vehicle_list == nullptr) {
            // dynamic RAM allocation of _vehicle_list[] failed, disable gracefully
            hal.console->printf("Unable to initialize ADS-B vehicle list\n");
            _enabled.set_and_notify(0);
        }
    }

    // avoid_state
    avoid_state.lowest_threat_distance = 0;
    avoid_state.highest_threat_distance = 0;
    avoid_state.another_vehicle_within_radius = false;
    avoid_state.is_evading_threat = false;
}

/*
 * de-initialize and free up some memory
 */
void AP_ADSB::deinit(void)
{
    in_state.vehicle_count = 0;
    if (in_state.vehicle_list != nullptr) {
        delete [] in_state.vehicle_list;
        in_state.vehicle_list = nullptr;
    }
}

/*
 * periodic update to handle vehicle timeouts and trigger collision detection
 */
void AP_ADSB::update(void)
{
    // update _my_loc
    if (!_ahrs.get_position(_my_loc)) {
        _my_loc.zero();
    }

    if (!_enabled) {
        if (in_state.vehicle_list != nullptr) {
            deinit();
        }
        // nothing to do
        return;
    } else if (in_state.vehicle_list == nullptr)  {
        init();
        return;
    } else if (in_state.list_size != in_state.list_size_param) {
        deinit();
        return;
    }

    uint32_t now = AP_HAL::millis();

    // check current list for vehicles that time out
    uint16_t index = 0;
    while (index < in_state.vehicle_count) {
        // check list and drop stale vehicles. When disabled, the list will get flushed
        if (now - in_state.vehicle_list[index].last_update_ms > VEHICLE_TIMEOUT_MS) {
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
    if (in_state.vehicle_count == 0 || _my_loc.is_zero()) {
        // nothing to do or current location is unknown so we can't calculate any collisions
        avoid_state.another_vehicle_within_radius = false;
        avoid_state.lowest_threat_distance = 0; // 0 means invalid
        avoid_state.highest_threat_distance = 0; // 0 means invalid
        return;
    }

    // TODO: compute lowest_threat using the 3D flight vector with respect to
    // time-to-collision and probability of collision instead of furthest 2D distance

    // TODO: compute highest_threat using the 3D flight vector with respect to
    // time-to-collision and probability of collision instead of closest 2D distance

    float min_distance = 0;
    float max_distance = 0;
    uint16_t min_distance_index = 0;
    uint16_t max_distance_index = 0;

    for (uint16_t index = 0; index < in_state.vehicle_count; index++) {
        float distance = _my_loc.get_distance(get_location(in_state.vehicle_list[index]));
        if (min_distance > distance || index == 0) {
            min_distance = distance;
            min_distance_index = index;
        }
        if (max_distance < distance || index == 0) {
            max_distance = distance;
            max_distance_index = index;
        }

        if (distance <= VEHICLE_THREAT_RADIUS_M) {
            in_state.vehicle_list[index].threat_level = ADSB_THREAT_HIGH;
        } else {
            in_state.vehicle_list[index].threat_level = ADSB_THREAT_LOW;
        }
    } // for index

    avoid_state.highest_threat_index = min_distance_index;
    avoid_state.highest_threat_distance = min_distance;

    avoid_state.lowest_threat_index = max_distance_index;
    avoid_state.lowest_threat_distance = max_distance;

    // if within radius, set flag and enforce a double radius to clear flag
    if (is_zero(avoid_state.highest_threat_distance) ||  // 0 means invalid
            avoid_state.highest_threat_distance > 2*VEHICLE_THREAT_RADIUS_M) {
        avoid_state.another_vehicle_within_radius = false;
    } else if (avoid_state.highest_threat_distance <= VEHICLE_THREAT_RADIUS_M) {
        avoid_state.another_vehicle_within_radius = true;
    }
}

/*
 * Convert/Extract a Location from a vehicle
 */
Location AP_ADSB::get_location(const adsb_vehicle_t &vehicle) const
{
    Location loc {};
    loc.alt = vehicle.info.altitude * 0.1f; // convert mm to cm.
    loc.lat = vehicle.info.lat;
    loc.lng = vehicle.info.lon;
    loc.flags.relative_alt = false;
    return loc;
}

/*
 *  delete a vehicle by copying last vehicle to
 *  current index then decrementing count
 */
void AP_ADSB::delete_vehicle(uint16_t index)
{
    if (index < in_state.vehicle_count) {
        // if the vehicle is the lowest/highest threat, invalidate it
        if (index == avoid_state.lowest_threat_index) {
            avoid_state.lowest_threat_distance = 0;
        }
        if (index == avoid_state.highest_threat_index) {
            avoid_state.highest_threat_distance = 0;
        }

        if (index != (in_state.vehicle_count-1)) {
            in_state.vehicle_list[index] = in_state.vehicle_list[in_state.vehicle_count-1];
        }
        // TODO: is memset needed? When we decrement the index we essentially forget about it
        memset(&in_state.vehicle_list[in_state.vehicle_count-1], 0, sizeof(adsb_vehicle_t));
        in_state.vehicle_count--;
    }
}

/*
 * Search _vehicle_list for the given vehicle. A match
 * depends on ICAO_address. Returns true if match found
 * and index is populated. otherwise, return false.
 */
bool AP_ADSB::find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const
{
    for (uint16_t i = 0; i < in_state.vehicle_count; i++) {
        if (in_state.vehicle_list[i].info.ICAO_address == vehicle.info.ICAO_address) {
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
    if (in_state.vehicle_list == nullptr) {
        // We are only null when disabled. Updating is inhibited.
        return;
    }

    uint16_t index;
    adsb_vehicle_t vehicle {};
    mavlink_msg_adsb_vehicle_decode(packet, &vehicle.info);

    if (find_index(vehicle, &index)) {

        // found, update it
        set_vehicle(index, vehicle);

    } else if (in_state.vehicle_count < in_state.list_size) {

        // not found and there's room, add it to the end of the list
        set_vehicle(in_state.vehicle_count, vehicle);
        in_state.vehicle_count++;

    } else {

        // buffer is full, replace the vehicle with lowest threat as long as it's not further away
        if (!is_zero(avoid_state.lowest_threat_distance) && !_my_loc.is_zero()) { // nonzero means it is valid

            float distance = _my_loc.get_distance(get_location(vehicle));
            if (distance < avoid_state.lowest_threat_distance) { // is closer than the furthest

                 // overwrite the lowest_threat/furthest
                index = avoid_state.lowest_threat_index;
                set_vehicle(index, vehicle);

                // this is now invalid because the vehicle was overwritten, need
                // to run perform_threat_detection() to determine new one because
                // we aren't keeping track of the second-furthest vehicle.
                avoid_state.lowest_threat_distance = 0;

                // is it the nearest? Then it's the highest threat. That's an easy check
                // that we don't need to run perform_threat_detection() to determine
                if (avoid_state.highest_threat_distance > distance) {
                    avoid_state.highest_threat_distance = distance;
                    avoid_state.highest_threat_index = index;
                }
            } // if distance

        } // if !zero
    } // if buffer full
}

/*
 * Copy a vehicle's data into the list
 */
void AP_ADSB::set_vehicle(uint16_t index, const adsb_vehicle_t &vehicle)
{
    if (index < in_state.list_size) {
        in_state.vehicle_list[index] = vehicle;
        in_state.vehicle_list[index].last_update_ms = AP_HAL::millis();
    }
}

void AP_ADSB::send_adsb_vehicle(mavlink_channel_t chan)
{
    if (in_state.vehicle_list == nullptr || in_state.vehicle_count == 0) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    if (in_state.send_index[chan] >= in_state.vehicle_count) {
        // we've finished a list
        if (now - in_state.send_start_ms[chan] < 1000) {
            // too soon to start a new one
            return;
        } else {
            // start new list
            in_state.send_start_ms[chan] = now;
            in_state.send_index[chan] = 0;
        }
    }

    if (in_state.send_index[chan] < in_state.vehicle_count) {
        mavlink_adsb_vehicle_t vehicle = in_state.vehicle_list[in_state.send_index[chan]].info;
        in_state.send_index[chan]++;

        mavlink_msg_adsb_vehicle_send(chan,
            vehicle.ICAO_address,
            vehicle.lat,
            vehicle.lon,
            vehicle.altitude_type,
            vehicle.altitude,
            vehicle.heading,
            vehicle.hor_velocity,
            vehicle.ver_velocity,
            vehicle.callsign,
            vehicle.emitter_type,
            vehicle.tslc,
            vehicle.flags,
            vehicle.squawk);
    }
}

