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
#include <AP_Logger/AP_Logger.h>


#define VEHICLE_TIMEOUT_MS              5000   // if no updates in this time, drop it from the list
#define ADSB_VEHICLE_LIST_SIZE_DEFAULT  25
#define ADSB_VEHICLE_LIST_SIZE_MAX      100
#define ADSB_CHAN_TIMEOUT_MS            15000
#define ADSB_SQUAWK_OCTAL_DEFAULT       1200

#define ADSB_BITBASK_RF_CAPABILITIES_UAT_IN         (1 << 0)
#define ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN      (1 << 1)

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
    AP_GROUPINFO_FLAGS("ENABLE",     0, AP_ADSB, _enabled,    0, AP_PARAM_FLAG_ENABLE),

    // index 1 is reserved - was BEHAVIOR

    // @Param: LIST_MAX
    // @DisplayName: ADSB vehicle list size
    // @Description: ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("LIST_MAX",   2, AP_ADSB, in_state.list_size_param, ADSB_VEHICLE_LIST_SIZE_DEFAULT),


    // @Param: LIST_RADIUS
    // @DisplayName: ADSB vehicle list radius filter
    // @Description: ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations. A value of 0 will disable this filter.
    // @Range: 0 100000
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("LIST_RADIUS",   3, AP_ADSB, in_state.list_radius, ADSB_LIST_RADIUS_DEFAULT),

    // @Param: ICAO_ID
    // @DisplayName: ICAO_ID vehicle identification number
    // @Description: ICAO_ID unique vehicle identification number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
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
    // @Description: Transceiver RF selection for Rx enable and/or Tx enable. This only effects devices that can Tx and Rx. Rx-only devices override this to always be Rx-only.
    // @Values: 0:Disabled,1:Rx-Only,2:Tx-Only,3:Rx and Tx Enabled
    // @User: Advanced
    AP_GROUPINFO("RF_SELECT",   9, AP_ADSB, out_state.cfg.rfSelect, UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED),

    // @Param: SQUAWK
    // @DisplayName: Squawk code
    // @Description: VFR squawk (Mode 3/A) code is a pre-programmed default code when the pilot is flying VFR and not in contact with ATC. In the USA, the VFR squawk code is octal 1200 (hex 0x280, decimal 640) and in most parts of Europe the VFR squawk code is octal 7000. If an invalid octal number is set then it will be reset to 1200.
    // @Range: 0 7777
    // @Units: octal
    // @User: Advanced
    AP_GROUPINFO("SQUAWK",  10, AP_ADSB, out_state.cfg.squawk_octal_param, ADSB_SQUAWK_OCTAL_DEFAULT),

    // @Param: RF_CAPABLE
    // @DisplayName: RF capabilities
    // @Description: Describes your hardware RF In/Out capabilities.
    // @Values: 0:Unknown,1:Rx UAT only,3:Rx UAT and 1090ES,7:Rx&Tx UAT and 1090ES
    // @Bitmask: 0:UAT_in,1:1090ES_in,2:UAT_out,3:1090ES_out
    // @User: Advanced
    AP_GROUPINFO("RF_CAPABLE",  11, AP_ADSB, out_state.cfg.rf_capable, 0),

    // @Param: LIST_ALT
    // @DisplayName: ADSB vehicle list altitude filter
    // @Description: ADSB vehicle list altitude filter. Vehicles detected above this altitude will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations. A value of 0 will disable this filter.
    // @Range: 0 32767
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("LIST_ALT",   12, AP_ADSB, in_state.list_altitude, 0),

    // @Param: ICAO_SPECL
    // @DisplayName: ICAO_ID of special vehicle
    // @Description: ICAO_ID of special vehicle that ignores ADSB_LIST_RADIUS and ADSB_LIST_ALT. The vehicle is always tracked. Use 0 to disable.
    // @User: Advanced
    AP_GROUPINFO("ICAO_SPECL",  13, AP_ADSB, _special_ICAO_target, 0),

    // @Param: LOG
    // @DisplayName: ADS-B logging
    // @Description: 0: no logging, 1: log only special ID, 2:log all
    // @Values: 0:no logging,1:log only special ID,2:log all
    // @User: Advanced
    AP_GROUPINFO("LOG",  14, AP_ADSB, _log, 1),

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

    furthest_vehicle_distance = 0;
    furthest_vehicle_index = 0;

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
    if (!AP::ahrs().get_position(_my_loc)) {
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

    const uint32_t now = AP_HAL::millis();

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

    if (_my_loc.is_zero()) {
        // if we don't have a GPS lock then there's nothing else to do
        return;
    }

    if (out_state.chan < 0) {
        // if there's no transceiver detected then do not set ICAO and do not service the transceiver
        return;
    }


    if (out_state.cfg.squawk_octal_param != out_state.cfg.squawk_octal) {
        // param changed, check that it's a valid octal
        if (!is_valid_octal(out_state.cfg.squawk_octal_param)) {
            // invalid, reset it to default
            out_state.cfg.squawk_octal_param = ADSB_SQUAWK_OCTAL_DEFAULT;
        }
        out_state.cfg.squawk_octal = (uint16_t)out_state.cfg.squawk_octal_param;
    }

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
        gcs().send_text(MAV_SEVERITY_INFO, "ADSB: Using ICAO_id %d and Callsign %s", out_state.cfg.ICAO_id, out_state.cfg.callsign);
        out_state.last_config_ms = 0; // send now
    }


    // send static configuration data to transceiver, every 5s
    if (out_state.chan_last_ms > 0 && now - out_state.chan_last_ms > ADSB_CHAN_TIMEOUT_MS) {
        // haven't gotten a heartbeat health status packet in a while, assume hardware failure
        // TODO: reset out_state.chan
        out_state.chan = -1;
        gcs().send_text(MAV_SEVERITY_ERROR, "ADSB: Transceiver heartbeat timed out");
    } else if (out_state.chan < MAVLINK_COMM_NUM_BUFFERS) {
        const mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + out_state.chan);
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

/*
 * determine index and distance of furthest vehicle. This is
 * used to bump it off when a new closer aircraft is detected
 */
void AP_ADSB::determine_furthest_aircraft(void)
{
    float max_distance = 0;
    uint16_t max_distance_index = 0;

    for (uint16_t index = 0; index < in_state.vehicle_count; index++) {
        if (is_special_vehicle(in_state.vehicle_list[index].info.ICAO_address)) {
            continue;
        }
        const float distance = _my_loc.get_distance(get_location(in_state.vehicle_list[index]));
        if (max_distance < distance || index == 0) {
            max_distance = distance;
            max_distance_index = index;
        }
    } // for index

    furthest_vehicle_index = max_distance_index;
    furthest_vehicle_distance = max_distance;
}

/*
 * Convert/Extract a Location from a vehicle
 */
Location AP_ADSB::get_location(const adsb_vehicle_t &vehicle) const
{
    const Location loc = Location(
        vehicle.info.lat,
        vehicle.info.lon,
        vehicle.info.altitude * 0.1f,
        Location::AltFrame::ABSOLUTE);

    return loc;
}

/*
 *  delete a vehicle by copying last vehicle to
 *  current index then decrementing count
 */
void AP_ADSB::delete_vehicle(const uint16_t index)
{
    if (index >= in_state.vehicle_count) {
        // index out of range
        return;
    }

    // if the vehicle is the furthest, invalidate it. It has been bumped
    if (index == furthest_vehicle_index && furthest_vehicle_distance > 0) {
        furthest_vehicle_distance = 0;
        furthest_vehicle_index = 0;
    }
    if (index != (in_state.vehicle_count-1)) {
        in_state.vehicle_list[index] = in_state.vehicle_list[in_state.vehicle_count-1];
    }
    // TODO: is memset needed? When we decrement the index we essentially forget about it
    memset(&in_state.vehicle_list[in_state.vehicle_count-1], 0, sizeof(adsb_vehicle_t));
    in_state.vehicle_count--;
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
void AP_ADSB::handle_vehicle(const mavlink_message_t* packet)
{
    if (in_state.vehicle_list == nullptr) {
        // We are only null when disabled. Updating is inhibited.
        return;
    }

    uint16_t index = in_state.list_size + 1; // initialize with invalid index
    adsb_vehicle_t vehicle {};
    mavlink_msg_adsb_vehicle_decode(packet, &vehicle.info);
    const Location vehicle_loc = AP_ADSB::get_location(vehicle);
    const bool my_loc_is_zero = _my_loc.is_zero();
    const float my_loc_distance_to_vehicle = _my_loc.get_distance(vehicle_loc);
    const bool is_special = is_special_vehicle(vehicle.info.ICAO_address);
    const bool out_of_range = in_state.list_radius > 0 && !my_loc_is_zero && my_loc_distance_to_vehicle > in_state.list_radius && !is_special;
    const bool out_of_range_alt = in_state.list_altitude > 0 && !my_loc_is_zero && abs(vehicle_loc.alt - _my_loc.alt) > in_state.list_altitude*100 && !is_special;
    const bool is_tracked_in_list = find_index(vehicle, &index);
    const uint32_t now = AP_HAL::millis();

    // note the last time the receiver got a packet from the aircraft
    vehicle.last_update_ms = now - (vehicle.info.tslc * 1000);

    const uint16_t required_flags_position = ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE;
    const bool detected_ourself = (out_state.cfg.ICAO_id != 0) && ((uint32_t)out_state.cfg.ICAO_id == vehicle.info.ICAO_address);

    if (vehicle_loc.is_zero() ||
            out_of_range ||
            out_of_range_alt ||
            detected_ourself ||
            (vehicle.info.ICAO_address > 0x00FFFFFF) || // ICAO address is 24bits, so ignore higher values.
            !(vehicle.info.flags & required_flags_position) ||
            now - vehicle.last_update_ms > VEHICLE_TIMEOUT_MS) {

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

    } else {
        // buffer is full. if new vehicle is closer than furthest, replace furthest with new

        if (my_loc_is_zero) {
            // nothing else to do
            furthest_vehicle_distance = 0;
            furthest_vehicle_index = 0;

        } else {
            if (furthest_vehicle_distance <= 0) {
                // ensure this is populated
                determine_furthest_aircraft();
            }

            if (my_loc_distance_to_vehicle < furthest_vehicle_distance) { // is closer than the furthest
                // replace with the furthest vehicle
                set_vehicle(furthest_vehicle_index, vehicle);

                // furthest_vehicle_index is now invalid because the vehicle was overwritten, need
                // to run determine_furthest_aircraft() to determine a new one next time
                furthest_vehicle_distance = 0;
                furthest_vehicle_index = 0;
            }
        }
    } // if buffer full

    const uint16_t required_flags_avoidance =
            ADSB_FLAGS_VALID_COORDS |
            ADSB_FLAGS_VALID_ALTITUDE |
            ADSB_FLAGS_VALID_HEADING |
            ADSB_FLAGS_VALID_VELOCITY;

    if (vehicle.info.flags & required_flags_avoidance) {
        push_sample(vehicle); // note that set_vehicle modifies vehicle
    }
}

/*
 * Copy a vehicle's data into the list
 */
void AP_ADSB::set_vehicle(const uint16_t index, const adsb_vehicle_t &vehicle)
{
    if (index >= in_state.list_size) {
        // out of range
        return;
    }
    in_state.vehicle_list[index] = vehicle;

    write_log(vehicle);
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
    const AP_GPS &gps = AP::gps();
    const Vector3f &gps_velocity = gps.velocity();

    const int32_t latitude = _my_loc.lat;
    const int32_t longitude = _my_loc.lng;
    const int32_t altGNSS = _my_loc.alt * 10; // convert cm to mm
    const int16_t velVert = gps_velocity.z * 1E2; // convert m/s to cm/s
    const int16_t nsVog = gps_velocity.x * 1E2; // convert m/s to cm/s
    const int16_t ewVog = gps_velocity.y * 1E2; // convert m/s to cm/s
    const uint8_t fixType = gps.status(); // this lines up perfectly with our enum
    const uint8_t emStatus = 0; // TODO: implement this ENUM. no emergency = 0
    const uint8_t numSats = gps.num_sats();
    const uint16_t squawk = out_state.cfg.squawk_octal;

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
    if (out_state._is_in_auto_mode) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
    }
    if (!out_state.is_flying) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
    }

    // TODO: confirm this sets utcTime correctly
    const uint64_t gps_time = gps.time_epoch_usec();
    const uint32_t utcTime = gps_time / 1000000ULL;

    const AP_Baro &baro = AP::baro();
    int32_t altPres = INT_MAX;
    if (baro.healthy()) {
        // Altitude difference between sea level pressure and current pressure. Result in millimeters
        altPres = baro.get_altitude_difference(SSL_AIR_PRESSURE, baro.get_pressure()) * 1E3; // convert m to mm;
    }



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


/*
 * To expand functionality in their HW, uAvionix has extended a few of the unused MAVLink bits to pack in more new features
 * This function will override the MSB byte of the 24bit ICAO address. To ensure an invalid >24bit ICAO is never broadcasted,
 * this function is used to create the encoded verison without ever writing to the actual ICAO number. It's created on-demand
 */
uint32_t AP_ADSB::get_encoded_icao(void)
{
    // utilize the upper unused 8bits of the icao with special flags.
    // This encoding is required for uAvionix devices that break the MAVLink spec.

    // ensure the user assignable icao is 24 bits
    uint32_t encoded_icao = (uint32_t)out_state.cfg.ICAO_id & 0x00FFFFFF;

    encoded_icao &= ~0x20000000;    // useGnssAltitude should always be FALSE
    encoded_icao |=  0x10000000;    // csidLogic       should always be TRUE

    //SIL/SDA are special fields that should be set to 0 with only expert user adjustment
    encoded_icao &= ~0x03000000;    // SDA should always be FALSE
    encoded_icao &= ~0x0C000000;    // SIL should always be FALSE

    return encoded_icao;
}

/*
 * To expand functionality in their HW, uAvionix has extended a few of the unused MAVLink bits to pack in more new features
 * This function will override the usually-null ending char of the callsign. It always encodes the last byte [8], even if
 * the callsign string is less than 9 chars and there are other zero-padded nulls.
 */
uint8_t AP_ADSB::get_encoded_callsign_null_char()
{
//  Encoding of the 8bit null char
//  (LSB) - knots
//  bit.1 - knots
//  bit.2 - knots
//  bit.3 - (unused)
//  bit.4 - flag - ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN
//  bit.5 - flag - ADSB_BITBASK_RF_CAPABILITIES_UAT_IN
//  bit.6 - flag - 0 = callsign is treated as callsign, 1 = callsign is treated as flightPlanID/Squawk
//  (MSB) - (unused)

    uint8_t encoded_null = 0;

    if (out_state.cfg.maxAircraftSpeed_knots <= 0) {
        // not set or unknown. no bits set
    } else if (out_state.cfg.maxAircraftSpeed_knots <= 75) {
        encoded_null |= 0x01;
    } else if (out_state.cfg.maxAircraftSpeed_knots <= 150) {
        encoded_null |= 0x02;
    } else if (out_state.cfg.maxAircraftSpeed_knots <= 300) {
        encoded_null |= 0x03;
    } else if (out_state.cfg.maxAircraftSpeed_knots <= 600) {
        encoded_null |= 0x04;
    } else if (out_state.cfg.maxAircraftSpeed_knots <= 1200) {
        encoded_null |= 0x05;
    } else {
        encoded_null |= 0x06;
    }


    if (out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN) {
        encoded_null |= 0x10;
    }
    if (out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_UAT_IN) {
        encoded_null |= 0x20;
    }


    /*
    If the user has an 8 digit flightPlanID assigned from a filed flight plan, this should be assigned to FlightPlanID, (assigned by remote app)
    else if the user has an assigned squawk code from ATC this should be converted from 4 digit octal to 4 character alpha string and assigned to FlightPlanID,
    else if a tail number is known it should be set to the tail number of the aircraft, (assigned by remote app)
    else it should be left blank (all 0's)
     */

    // using the above logic, we must always assign the squawk. once we get configured
    // externally then get_encoded_callsign_null_char() stops getting called
    snprintf(out_state.cfg.callsign, 5, "%04d", unsigned(out_state.cfg.squawk_octal) & 0x1FFF);
    memset(&out_state.cfg.callsign[4], 0, 5); // clear remaining 5 chars
    encoded_null |= 0x40;

    return encoded_null;
}

/*
 * handle incoming packet UAVIONIX_ADSB_OUT_CFG.
 * This allows a GCS to send cfg info through the autopilot to the ADSB hardware.
 * This is done indirectly by reading and storing the packet and then another mechanism sends it out periodically
 */
void AP_ADSB::handle_out_cfg(const mavlink_message_t* msg)
{
    mavlink_uavionix_adsb_out_cfg_t packet {};
    mavlink_msg_uavionix_adsb_out_cfg_decode(msg, &packet);

    out_state.cfg.was_set_externally = true;

    out_state.cfg.ICAO_id = packet.ICAO;
    out_state.cfg.ICAO_id_param = out_state.cfg.ICAO_id_param_prev = packet.ICAO & 0x00FFFFFFFF;

    // May contain a non-null value at the end so accept it as-is with memcpy instead of strcpy
    memcpy(out_state.cfg.callsign, packet.callsign, sizeof(out_state.cfg.callsign));

    out_state.cfg.emitterType = packet.emitterType;
    out_state.cfg.lengthWidth = packet.aircraftSize;
    out_state.cfg.gpsLatOffset = packet.gpsOffsetLat;
    out_state.cfg.gpsLonOffset = packet.gpsOffsetLon;
    out_state.cfg.rfSelect = packet.rfSelect;
    out_state.cfg.stall_speed_cm = packet.stallSpeed;

    // guard against string with non-null end char
    const char c = out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1];
    out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "ADSB: Using ICAO_id %d and Callsign %s", out_state.cfg.ICAO_id, out_state.cfg.callsign);
    out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = c;

    // send now
    out_state.last_config_ms = 0;
}

/*
 * populate and send MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG
 */
void AP_ADSB::send_configure(const mavlink_channel_t chan)
{
    // MAVLink spec says the 9 byte callsign field is 8 byte string with 9th byte as null.
    // Here we temporarily set some flags in that null char to signify the callsign
    // may be a flightplanID instead
    int8_t callsign[sizeof(out_state.cfg.callsign)];
    uint32_t icao;

    memcpy(callsign, out_state.cfg.callsign, sizeof(out_state.cfg.callsign));

    if (out_state.cfg.was_set_externally) {
        // take values as-is
        icao = out_state.cfg.ICAO_id;
    } else {
        callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = (int8_t)get_encoded_callsign_null_char();
        icao = get_encoded_icao();
    }

    mavlink_msg_uavionix_adsb_out_cfg_send(
            chan,
            icao,
            (const char*)callsign,
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

void AP_ADSB::handle_transceiver_report(const mavlink_channel_t chan, const mavlink_message_t* msg)
{
    mavlink_uavionix_adsb_transceiver_health_report_t packet {};
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(msg, &packet);

    if (out_state.chan != chan) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "ADSB: Found transceiver on channel %d", chan);
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
uint32_t AP_ADSB::genICAO(const Location &loc)
{
    // gps_time is not seconds since UTC midnight, but it is an equivalent random number
    // TODO: use UTC time instead of GPS time
    const AP_GPS &gps = AP::gps();
    const uint64_t gps_time = gps.time_epoch_usec();

    uint32_t timeSum = 0;
    const uint32_t M3 = 4096 * (loc.lat & 0x00000FFF) + (loc.lng & 0x00000FFF);

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
        snprintf(&out_state.cfg.callsign[4], 5, "%04X", unsigned(out_state.cfg.ICAO_id % 0x10000));
    }
}


void AP_ADSB::push_sample(adsb_vehicle_t &vehicle)
{
    samples.push_back(vehicle);
}

bool AP_ADSB::next_sample(adsb_vehicle_t &vehicle)
{
    return samples.pop_front(vehicle);
}

void AP_ADSB::handle_message(const mavlink_channel_t chan, const mavlink_message_t* msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_ADSB_VEHICLE:
        handle_vehicle(msg);
        break;
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        handle_transceiver_report(chan, msg);
        break;

    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
        handle_out_cfg(msg);
        break;

    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
        // unhandled, this is an outbound packet only
    default:
        break;
    }

}

// If that ICAO is found in the database then return true with a fully populated vehicle
bool AP_ADSB::get_vehicle_by_ICAO(const uint32_t icao, adsb_vehicle_t &vehicle) const
{
    adsb_vehicle_t temp_vehicle;
    temp_vehicle.info.ICAO_address = icao;

    uint16_t i;
    if (find_index(temp_vehicle, &i)) {
        // vehicle is tracked in list.
        // we must memcpy it because the database may reorganize itself and we don't
        // want the reference to magically start pointing at a different vehicle
        memcpy(&vehicle, &in_state.vehicle_list[i], sizeof(adsb_vehicle_t));
        return true;
    }
    return false;
}

/*
 * Write vehicle to log
 */
void AP_ADSB::write_log(const adsb_vehicle_t &vehicle)
{
    switch (_log) {
        case logging::SPECIAL_ONLY:
            if (!is_special_vehicle(vehicle.info.ICAO_address)) {
                return;
            }
        case logging::ALL:
            break;

        case logging::NONE:
        default:
            return;
    }

    struct log_ADSB pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ADSB_MSG),
        time_us       : AP_HAL::micros64(),
        ICAO_address  : vehicle.info.ICAO_address,
        lat           : vehicle.info.lat,
        lng           : vehicle.info.lon,
        alt           : vehicle.info.altitude,
        heading       : vehicle.info.heading,
        hor_velocity  : vehicle.info.hor_velocity,
        ver_velocity  : vehicle.info.ver_velocity,
        squawk        : vehicle.info.squawk,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
