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
#include <stdio.h>  // for sprintf
#include <limits.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#define VEHICLE_THREAT_RADIUS_M         1000
#define VEHICLE_TIMEOUT_MS              5000   // if no updates in this time, drop it from the list
#define ADSB_VEHICLE_LIST_SIZE_DEFAULT  25
#define ADSB_VEHICLE_LIST_SIZE_MAX      100
#define ADSB_CHAN_TIMEOUT_MS            15000

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define ADSB_LIST_RADIUS_DEFAULT        10000 // in meters
#else // APM_BUILD_TYPE(APM_BUILD_ArduCopter), Rover, Boat
    #define ADSB_LIST_RADIUS_DEFAULT        2000 // in meters
#endif

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


    // @Param: LIST_RADIUS
    // @DisplayName: ADSB vehicle list radius filter
    // @Description: ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations.
    // @Range: 1 100000
    // @User: Advanced
    AP_GROUPINFO("LIST_RADIUS",   3, AP_ADSB, in_state.list_radius, ADSB_LIST_RADIUS_DEFAULT),

    // @Param: ICAO_ID
    // @DisplayName: ICAO_ID vehicle identifaction number
    // @Description: ICAO_ID unique vehicle identifaction number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
    // @Range: -1 16777215
    // @User: Advanced
    AP_GROUPINFO("ICAO_ID",   4, AP_ADSB, out_state.cfg.ICAO_id_param, 0),

    // @Param: EMIT_TYPE
    // @DisplayName: Emitter type
    // @Description: ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).
    // @Values: 0:NoInfo,1:Light,2:Small,3:Large,4:HighVortexlarge,5:Heavy,6:HighlyManuv,7:Rotocraft,8:RESERVED,9:Glider,10:LightAir,11:Parachute,12:UltraLight,13:RESERVED,14:UAV,15:Space,16:RESERVED,17:EmergencySurface,18:ServiceSurface,19:PointObstacle
    // @User: Advanced
    AP_GROUPINFO("EMIT_TYPE",   5, AP_ADSB, out_state.cfg.emitterType, ADSB_EMITTER_TYPE_UAV),

    // @Param: LEN_WIDTH
    // @DisplayName: Aircraft length and width
    // @Description: Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.
	// @Values: 0:NO_DATA,1:L15W23,2:L25W28P5,3:L25W34,4:L35W33,5:L35W38,6:L45W39P5,7:L45W45,8:L55W45,9:L55W52,10:L65W59P5,11:L65W67,12:L75W72P5,13:L75W80,14:L85W80,15:L85W90
    // @User: Advanced
    AP_GROUPINFO("LEN_WIDTH",   6, AP_ADSB, out_state.cfg.lengthWidth, UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M),

    // @Param: OFFSET_LAT
    // @DisplayName: GPS antenna lateral offset
    // @Description: GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.
	// @Values: 0:NoData,1:Left2m,2:Left4m,3:Left6m,4:Center,5:Right2m,6:Right4m,7:Right6m
    // @User: Advanced
    AP_GROUPINFO("OFFSET_LAT",   7, AP_ADSB, out_state.cfg.gpsLatOffset, UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M),

    // @Param: OFFSET_LON
    // @DisplayName: GPS antenna longitudinal offset
    // @Description: GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
    // @Values: 0:NO_DATA,1:AppliedBySensor
    // @User: Advanced
    AP_GROUPINFO("OFFSET_LON",   8, AP_ADSB, out_state.cfg.gpsLonOffset, UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR),

    // @Param: RF_SELECT
    // @DisplayName: Transceiver RF selection
    // @Description: Transceiver RF selection for Rx enable and/or Tx enable.
    // @Values: 0:Disabled,1:Rx-Only,2:Tx-Only,3:Rx and Tx Enabled
    // @User: Advanced
    AP_GROUPINFO("RF_SELECT",   9, AP_ADSB, out_state.cfg.rfSelect, UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED | UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED),



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

    // out_state
    set_callsign("PING1234", false);
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

// -----------------------
    if (_my_loc.is_zero()) {
        // if we don't have a GPS lock then there's nothing else to do
        return;
    }
// -----------------------



    perform_threat_detection();

    // ensure it's positive 24bit but allow -1
    if (out_state.cfg.ICAO_id_param <= -1 || out_state.cfg.ICAO_id_param > 0x00FFFFFF) {
        // icao param of -1 means static information is not sent, transceiver is assumed pre-programmed.
        // reset timer constantly so it never reaches 10s so it never sends
        out_state.last_config_ms = now;

    } else if (out_state.cfg.ICAO_id == 0 ||
        out_state.cfg.ICAO_id_param_prev != out_state.cfg.ICAO_id_param) {

        // if param changed then regenerate. This allows the param to be changed back to zero to trigger a re-generate
        if (out_state.cfg.ICAO_id_param == 0) {
            out_state.cfg.ICAO_id = genICAO(_my_loc);
        } else {
            out_state.cfg.ICAO_id = out_state.cfg.ICAO_id_param;
        }
        out_state.cfg.ICAO_id_param_prev = out_state.cfg.ICAO_id_param;
        set_callsign("PING", true);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "ADSB: Using ICAO_id %d and Callsign %s", out_state.cfg.ICAO_id, out_state.cfg.callsign);
        out_state.last_config_ms = 0; // send now
    }


    // send static configuration data to transceiver, every 10s
    if (out_state.chan >= 0 && out_state.chan < MAVLINK_COMM_NUM_BUFFERS) {
        if (now - out_state.chan_last_ms > ADSB_CHAN_TIMEOUT_MS) {
            // haven't gotten a heartbeat health status packet in a while, assume hardware failure
            // TODO: reset out_state.chan
            out_state.chan = -1;
        } else {
            mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + out_state.chan);
            if (now - out_state.last_config_ms >= 5000 && HAVE_PAYLOAD_SPACE(chan, UAVIONIX_ADSB_OUT_CFG)) {
                out_state.last_config_ms = now;
                send_configure(chan);
            } // last_config_ms

            // send dynamic data to transceiver at 5Hz
            if (now - out_state.last_report_ms >= 200 && HAVE_PAYLOAD_SPACE(chan, UAVIONIX_ADSB_OUT_DYNAMIC)) {
                out_state.last_report_ms = now;
                send_dynamic_out(chan);
            } // last_report_ms
        } // chan_last_ms
    }
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
Location_Class AP_ADSB::get_location(const adsb_vehicle_t &vehicle) const
{
    Location_Class loc = Location_Class(
        vehicle.info.lat,
        vehicle.info.lon,
        vehicle.info.altitude * 0.1f,
        Location_Class::ALT_FRAME_ABSOLUTE);

    return loc;
}

/*
 *  delete a vehicle by copying last vehicle to
 *  current index then decrementing count
 */
void AP_ADSB::delete_vehicle(const uint16_t index)
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

    uint16_t index = in_state.list_size + 1; // initialize with invalid index
    adsb_vehicle_t vehicle {};
    mavlink_msg_adsb_vehicle_decode(packet, &vehicle.info);
    Location_Class vehicle_loc = Location_Class(AP_ADSB::get_location(vehicle));
    bool my_loc_is_zero = _my_loc.is_zero();
    float my_loc_distance_to_vehicle = _my_loc.get_distance(vehicle_loc);
    bool out_of_range = in_state.list_radius > 0 && !my_loc_is_zero && my_loc_distance_to_vehicle > in_state.list_radius;
    bool is_tracked_in_list = find_index(vehicle, &index);

    if (vehicle_loc.is_zero() || out_of_range) {

        // vehicle is out of range or invalid lat/lng. If we're tracking it, delete from list. Otherwise ignore it.
        if (is_tracked_in_list) {
            delete_vehicle(index);
        }
        return;

    } else if (is_tracked_in_list) {

        // found, update it
        set_vehicle(index, vehicle);

    } else if (in_state.vehicle_count < in_state.list_size) {

        // not found and there's room, add it to the end of the list
        set_vehicle(in_state.vehicle_count, vehicle);
        in_state.vehicle_count++;

    } else if (!my_loc_is_zero &&
            !is_zero(avoid_state.lowest_threat_distance) &&
            my_loc_distance_to_vehicle < avoid_state.lowest_threat_distance) { // is closer than the furthest

        // buffer is full, replace the vehicle with lowest threat as long as it's not further away
         // overwrite the lowest_threat/furthest
        index = avoid_state.lowest_threat_index;
        set_vehicle(index, vehicle);

        // this is now invalid because the vehicle was overwritten, need
        // to run perform_threat_detection() to determine new one because
        // we aren't keeping track of the second-furthest vehicle.
        avoid_state.lowest_threat_distance = 0;

        // is it the nearest? Then it's the highest threat. That's an easy check
        // that we don't need to run perform_threat_detection() to determine
        if (avoid_state.highest_threat_distance > my_loc_distance_to_vehicle) {
            avoid_state.highest_threat_distance = my_loc_distance_to_vehicle;
            avoid_state.highest_threat_index = index;
        }
    } // if buffer full

    vehicle.last_update_ms = AP_HAL::millis() - (vehicle.info.tslc * 1000);
    push_sample(vehicle); // note that set_vehicle modifies vehicle
}

/*
 * Copy a vehicle's data into the list
 */
void AP_ADSB::set_vehicle(const uint16_t index, const adsb_vehicle_t &vehicle)
{
    if (index < in_state.list_size) {
        in_state.vehicle_list[index] = vehicle;
    }
}

void AP_ADSB::send_adsb_vehicle(const mavlink_channel_t chan)
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


void AP_ADSB::send_dynamic_out(const mavlink_channel_t chan)
{
    // --------------
    // Knowns
    AP_GPS gps = _ahrs.get_gps();
    Vector3f gps_velocity = gps.velocity();

    int32_t latitude = _my_loc.lat;
    int32_t longitude = _my_loc.lng;
    int32_t altGNSS = _my_loc.alt*0.1f; // convert cm to mm
    int16_t velVert = gps_velocity.z * 1E2; // convert m/s to cm/s
    int16_t nsVog = gps_velocity.x * 1E2; // convert m/s to cm/s
    int16_t ewVog = gps_velocity.y * 1E2; // convert m/s to cm/s
    uint8_t fixType = gps.status(); // this lines up perfectly with our enum
    uint8_t emStatus = 0; // TODO: implement this ENUM. no emergency = 0
    uint8_t numSats = gps.num_sats();
    uint16_t squawk = 1200; // Mode A code (typically 1200 [0x04B0] for VFR)

    uint32_t accHoriz = UINT_MAX;
    float accHoriz_f;
    if (gps.horizontal_accuracy(accHoriz_f)) {
        accHoriz = accHoriz_f * 1E3; // convert m to mm
    }

    uint16_t accVert = USHRT_MAX;
    float accVert_f;
    if (gps.vertical_accuracy(accVert_f)) {
        accVert = accVert_f * 1E2; // convert m to cm
    }

    uint16_t accVel = USHRT_MAX;
    float accVel_f;
    if (gps.speed_accuracy(accVel_f)) {
        accVel = accVel_f * 1E3; // convert m/s to mm/s
    }

    uint16_t state = 0;
    if (_is_in_auto_mode) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
    }
    if (!out_state.is_flying) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
    }



    // --------------
    // Not Sure
    uint32_t utcTime = UINT_MAX; //    uint32_t utcTime,
    // TODO: confirm this sets utcTime correctly
    const uint64_t gps_time = gps.time_epoch_usec();
    utcTime = gps_time / 1000000ULL;



    // --------------
    // Unknowns
    // TODO: implement http://www.srh.noaa.gov/images/epz/wxcalc/pressureAltitude.pdf
    int32_t altPres = INT_MAX; //_ahrs.get_baro().get_altitude() relative to home, not MSL



    mavlink_msg_uavionix_adsb_out_dynamic_send(
            chan,
            utcTime,
            latitude,
            longitude,
            altGNSS,
            fixType,
            numSats,
            altPres,
            accHoriz,
            accVert,
            accVel,
            velVert,
            nsVog,
            ewVog,
            emStatus,
            state,
            squawk);
}

void AP_ADSB::send_configure(const mavlink_channel_t chan)
{
    mavlink_msg_uavionix_adsb_out_cfg_send(
            chan,
            (uint32_t)out_state.cfg.ICAO_id,
            out_state.cfg.callsign,
            (uint8_t)out_state.cfg.emitterType,
            (uint8_t)out_state.cfg.lengthWidth,
            (uint8_t)out_state.cfg.gpsLatOffset,
            (uint8_t)out_state.cfg.gpsLonOffset,
            out_state.cfg.stall_speed_cm,
            (uint8_t)out_state.cfg.rfSelect);
}

/*
 * this is a message from the transceiver reporting it's health. Using this packet
 * we determine which channel is on so we don't have to send out_state to all channels
 */

void AP_ADSB::transceiver_report(const mavlink_channel_t chan, const mavlink_message_t* msg)
{
    mavlink_uavionix_adsb_transceiver_health_report_t packet {};
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(msg, &packet);

    if (out_state.chan != chan) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_DEBUG, "ADSB: Found transceiver on channel %d", chan);
    }

    out_state.chan_last_ms = AP_HAL::millis();
    out_state.chan = chan;
    out_state.status = (UAVIONIX_ADSB_RF_HEALTH)packet.rfHealth;
}

/*
 @brief Generates pseudorandom ICAO from gps time, lat, and lon.
 Reference: DO282B, 2.2.4.5.1.3.2

 Note gps.time is the number of seconds since UTC midnight
*/
uint32_t AP_ADSB::genICAO(const Location_Class &loc)
{
    // gps_time is not seconds since UTC midnight, but it is an equivalent random number
    // TODO: use UTC time instead of GPS time
    AP_GPS gps = _ahrs.get_gps();
    const uint64_t gps_time = gps.time_epoch_usec();

    uint32_t timeSum = 0;
    uint32_t M3 = 4096 * (loc.lat & 0x00000FFF) + (loc.lng & 0x00000FFF);

    for (uint8_t i=0; i<24; i++) {
        timeSum += (((gps_time & 0x00FFFFFF)>> i) & 0x00000001);
    }
    return( (timeSum ^ M3) & 0x00FFFFFF);
}

// assign a string to out_state.cfg.callsign but ensure it's null terminated
void AP_ADSB::set_callsign(const char* str, const bool append_icao)
{
    bool zero_char_pad = false;

    // clean slate
    memset(out_state.cfg.callsign, 0, sizeof(out_state.cfg.callsign));

    // copy str to cfg.callsign but we can't use strncpy because we need
    // to restrict values to only 'A' - 'Z' and '0' - '9' and pad
    for (uint8_t i=0; i<sizeof(out_state.cfg.callsign)-1; i++) {
        if (!str[i] || zero_char_pad) {
            // finish early. Either pad the rest with zero char or null terminate and call it a day
            if ((append_icao && i<4) || zero_char_pad) {
                out_state.cfg.callsign[i] = '0';
                zero_char_pad = true;
            } else {
                // already null terminated via memset so just stop
                break;
            }

        } else if (('A' <= str[i] && str[i] <= 'Z') ||
                   ('0' <= str[i] && str[i] <= '9')) {
            // valid as-is
            // spaces are also allowed but are handled in the last else
            out_state.cfg.callsign[i] = str[i];

        } else if ('a' <= str[i] && str[i] <= 'z') {
            // toupper()
            out_state.cfg.callsign[i] = str[i] - ('a' - 'A');

        } else if (i == 0) {
            // invalid, pad to char zero because first index can't be space
            out_state.cfg.callsign[i] = '0';

        } else {
            // invalid, pad with space
            out_state.cfg.callsign[i] = ' ';
        }
    } // for i

    if (append_icao) {
        char str_icao[5];
        sprintf(str_icao, "%04X", out_state.cfg.ICAO_id % 0x10000);
        out_state.cfg.callsign[4] = str_icao[0];
        out_state.cfg.callsign[5] = str_icao[1];
        out_state.cfg.callsign[6] = str_icao[2];
        out_state.cfg.callsign[7] = str_icao[3];
    }

    out_state.cfg.callsign[sizeof(out_state.cfg.callsign)-1] = 0; // always null terminate just to be sure
}


void AP_ADSB::push_sample(adsb_vehicle_t &vehicle)
{
    samples.push_back(vehicle);
}

bool AP_ADSB::next_sample(adsb_vehicle_t &vehicle)
{
    return samples.pop_front(vehicle);
}
