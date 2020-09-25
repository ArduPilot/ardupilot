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

#include "AP_ADSB.h"
#if HAL_ADSB_ENABLED
#include "AP_ADSB_Sagetech.h"
#include "AP_ADSB_MAVLink.h"
#include <stdio.h>  // for sprintf
#include <limits.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>

#ifndef ADSB_SAGETECH_ENABLED
    #define ADSB_SAGETECH_ENABLED !HAL_MINIMIZE_FEATURES
#endif


//#define ADSB_STATIC_CALLSIGN            "APM1234"  // 8 ASCII chars of a Callsign. This can be overwritten by MAVLink msg UAVIONIX_ADSB_OUT_CFG

#define VEHICLE_TIMEOUT_MS              5000    // if no updates in this time, drop it from the list
#define ADSB_VEHICLE_LIST_SIZE_DEFAULT  25
#define ADSB_VEHICLE_LIST_SIZE_MAX      100     // This should be hw/ram dependent
#define ADSB_SQUAWK_OCTAL_DEFAULT       1200    // This is standard VFR in the USA

extern const AP_HAL::HAL& hal;

AP_ADSB *AP_ADSB::_singleton;

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
    AP_GROUPINFO("LIST_RADIUS",   3, AP_ADSB, in_state.list_radius, 0),

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
    // @Description: Transceiver RF selection for Rx enable and/or Tx enable. This only effects devices that can Tx and/or Rx. Rx-only devices override this to always be Rx-only.
    // @Bitmask: 0:Rx,1:Tx,2:Ident
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

// constructor
AP_ADSB::AP_ADSB()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ADSB must be singleton");
    }
#endif
    _singleton = this;

    // out_state not set by params
#ifdef ADSB_STATIC_CALLSIGN
    memcpy(&out_state.cfg.callsign, ADSB_STATIC_CALLSIGN, 8);
#else
    memset(&out_state.cfg.callsign, 0, 8);
#endif
}

/*
 * Initialize variables and allocate memory for new driver
 */
void AP_ADSB::hw_init(void)
{
    if (backend != nullptr) {
        // already initialized
        return;
    }

#if ADSB_SAGETECH_ENABLED
    if (AP_ADSB_Sagetech::detect()) {
        backend = new AP_ADSB_Sagetech(*this);
    } else
#endif
    {
        backend = new AP_ADSB_MAVLink(*this);
    }

    if (backend != nullptr) {
        // disable RF outputs on init
        out_state.cfg.rfSelect.set_and_save_and_notify(out_state.cfg.rfSelect & 0x01);
        backend->init();

    } else {
        // disabled ADSB, init failed. If we don't do this it will try to re-init forever
        _enabled.set_and_notify(false);
        gcs().send_text(MAV_SEVERITY_INFO, "%sInit failed", GcsHeader);
    }
}


void AP_ADSB::list_init(void)
{
    furthest_vehicle_distance = 0;
    furthest_vehicle_index = 0;
    in_state.vehicle_count = 0;

    if (in_state.vehicle_list == nullptr) {
        if (in_state.list_size_param != constrain_int16(in_state.list_size_param, 1, ADSB_VEHICLE_LIST_SIZE_MAX)) {
            in_state.list_size_param.set_and_notify(ADSB_VEHICLE_LIST_SIZE_DEFAULT);
            in_state.list_size_param.save();
        }
        in_state.list_size = in_state.list_size_param;
        in_state.vehicle_list = new adsb_vehicle_t[in_state.list_size];
    }


    if (in_state.vehicle_list == nullptr) {
        // dynamic RAM allocation of _vehicle_list[] failed, disable gracefully
        hal.console->printf("Unable to initialize ADS-B vehicle list\n");
        _enabled.set_and_notify(0);
        in_state.list_size = 0;
    }
}

/*
 * de-initialize and free up some memory
 */
void AP_ADSB::list_deinit(void)
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
    if (!_enabled) {
        if (in_state.vehicle_list != nullptr) {
            // we've been enabled before, disable the list
            list_deinit();
        }
        // nothing to do
        return;

    } else if (in_state.vehicle_list == nullptr || in_state.list_size != in_state.list_size_param) {
        // list size param changed or is not initialized, reinit the list
        list_deinit();
        list_init();
        return;
    }


    const uint32_t now = AP_HAL::millis();

    // update _my_loc
    if (!AP::ahrs().get_position(_my_loc)) {
        _my_loc.zero();
    }

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

    if (out_state.cfg.squawk_octal_param != out_state.cfg.squawk_octal) {
        // param changed, check that it's a valid octal
        if (!is_valid_callsign(out_state.cfg.squawk_octal_param)) {
            // invalid, reset it to default
            out_state.cfg.squawk_octal_param.set_and_notify(ADSB_SQUAWK_OCTAL_DEFAULT);
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
#ifndef ADSB_STATIC_CALLSIGN
        set_callsign("APM ", true);
#endif
        //gcs().send_text(MAV_SEVERITY_INFO, "%sUsing ICAO_id %d and Callsign %s", GcsHeader, (int)out_state.cfg.ICAO_id, out_state.cfg.callsign);
        out_state.last_config_ms = 0; // send now
    }

    if (backend == nullptr) {
        hw_init();
    } else {
        backend->update();
    }

}


bool AP_ADSB::is_valid_callsign(uint16_t octal)
{
    // treat "octal" as decimal and test if any decimal digit is > 7
    if (octal > 7777) {
        return false;
    }

    while (octal != 0) {
        if (octal % 10 > 7) {
            return false;
        }
        octal /= 10;
    }

    return true;
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
void AP_ADSB::handle_adsb_vehicle(const adsb_vehicle_t &vehicle)
{
    if (in_state.vehicle_list == nullptr) {
        // We are only null when disabled. Updating is inhibited.
        return;
    }

    uint16_t index = in_state.list_size + 1; // initialize with invalid index
    const Location vehicle_loc = AP_ADSB::get_location(vehicle);
    const bool my_loc_is_zero = _my_loc.is_zero();
    const float my_loc_distance_to_vehicle = _my_loc.get_distance(vehicle_loc);
    const bool is_special = is_special_vehicle(vehicle.info.ICAO_address);
    const bool out_of_range = in_state.list_radius > 0 && !my_loc_is_zero && my_loc_distance_to_vehicle > in_state.list_radius && !is_special;
    const bool out_of_range_alt = in_state.list_altitude > 0 && !my_loc_is_zero && abs(vehicle_loc.alt - _my_loc.alt) > in_state.list_altitude*100 && !is_special;
    const bool is_tracked_in_list = find_index(vehicle, &index);
    const uint32_t now = AP_HAL::millis();

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
 * Update the vehicle list. If the vehicle is already in the
 * list then it will update it, otherwise it will be added.
 */
void AP_ADSB::handle_vehicle(const mavlink_message_t &packet)
{
    adsb_vehicle_t vehicle {};
    const uint32_t now = AP_HAL::millis();
    mavlink_msg_adsb_vehicle_decode(&packet, &vehicle.info);
    vehicle.last_update_ms = now - (vehicle.info.tslc * 1000);
    handle_adsb_vehicle(vehicle);
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

    uint16_t flags = vehicle.info.flags;

    if (flags == 0) {
        // if no flags set, assume everything
        flags = ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN |
                ADSB_FLAGS_VALID_SQUAWK |
                ADSB_FLAGS_VERTICAL_VELOCITY_VALID |
                ADSB_FLAGS_BARO_VALID;
                // all flags set except ADSB_FLAGS_SOURCE_UAT and ADSB_FLAGS_SIMULATED
    }

    in_state.vehicle_list[index].last_update_ms = vehicle.last_update_ms;

    // use |= so we allow partial sets and those don't override previous valid settings
    in_state.vehicle_list[index].info.flags |= flags;

    in_state.vehicle_list[index].info.ICAO_address = vehicle.info.ICAO_address;
    in_state.vehicle_list[index].info.emitter_type = vehicle.info.emitter_type;
    in_state.vehicle_list[index].info.tslc = vehicle.info.tslc;

    if (flags & ADSB_FLAGS_VALID_COORDS) {
        in_state.vehicle_list[index].info.lat = vehicle.info.lat;
        in_state.vehicle_list[index].info.lon = vehicle.info.lon;
    }
    if (flags & ADSB_FLAGS_VALID_ALTITUDE) {
        in_state.vehicle_list[index].info.altitude_type = vehicle.info.altitude_type;
        in_state.vehicle_list[index].info.altitude = vehicle.info.altitude;
    }
    if (flags & ADSB_FLAGS_VALID_HEADING) {
        in_state.vehicle_list[index].info.heading = vehicle.info.heading;
    }
    if (flags & ADSB_FLAGS_VALID_VELOCITY) {
        in_state.vehicle_list[index].info.hor_velocity = vehicle.info.hor_velocity;
    }
    if (flags & ADSB_FLAGS_VALID_SQUAWK) {
        in_state.vehicle_list[index].info.squawk = vehicle.info.squawk;
    }
    if (flags & ADSB_FLAGS_SIMULATED) {
        // do we care about this? Maybe have a mode where we
    }
    if (flags & ADSB_FLAGS_VERTICAL_VELOCITY_VALID) {
        in_state.vehicle_list[index].info.ver_velocity = vehicle.info.ver_velocity;
    }
    if (flags & ADSB_FLAGS_VALID_CALLSIGN) {
        memcpy(in_state.vehicle_list[index].info.callsign, vehicle.info.callsign, sizeof(vehicle.info.callsign));
    }
    if (flags & ADSB_FLAGS_BARO_VALID) {
        // what do we do with this? Does it invalid ADSB_FLAGS_VALID_ALTITUDE with altitude_type == ADSB_ALTITUDE_TYPE_PRESSURE_QNH(0)?
    }
    if (flags & ADSB_FLAGS_SOURCE_UAT) {
        // don't care
    }

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
    if (out_state.cfg.was_set_externally) {
        return;
    }
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


void AP_ADSB::push_sample(const adsb_vehicle_t &vehicle)
{
    samples.push(vehicle);
}

bool AP_ADSB::next_sample(adsb_vehicle_t &vehicle)
{
    return samples.pop(vehicle);
}

void AP_ADSB::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_ADSB_VEHICLE:
        handle_vehicle(msg);
        break;
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
        // unhandled, this is an outbound packet only
        break;

    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: {
            mavlink_uavionix_adsb_out_cfg_t packet {};
            mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet);
            handle_out_cfg(packet);
        }
        break;

    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
    default:
        if (backend != nullptr) {
            backend->handle_msg(chan, msg);
        }
        break;
    }
}


/*
 * handle incoming packet UAVIONIX_ADSB_OUT_CFG.
 * This allows a GCS to send cfg info through the autopilot to the ADSB hardware.
 * This is done indirectly by reading and storing the packet and then another mechanism sends it out periodically
 */
void AP_ADSB::handle_out_cfg(const mavlink_uavionix_adsb_out_cfg_t &packet)
{
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
    gcs().send_text(MAV_SEVERITY_INFO, "%sUsing ICAO_id %d and Callsign %s", GcsHeader, (int)out_state.cfg.ICAO_id, out_state.cfg.callsign);
    out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = c;

    // send now
    out_state.last_config_ms = 0;
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
            break;

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

AP_ADSB *AP::ADSB()
{
    return AP_ADSB::get_singleton();
}

#endif // HAL_ADSB_ENABLED


