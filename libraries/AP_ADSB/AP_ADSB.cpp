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

#include "AP_ADSB_config.h"

#if HAL_ADSB_ENABLED

#include "AP_ADSB.h"

#include "AP_ADSB_uAvionix_MAVLink.h"
#include "AP_ADSB_uAvionix_UCP.h"
#include "AP_ADSB_Sagetech.h"
#include "AP_ADSB_Sagetech_MXS.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RTC/AP_RTC.h>

#define VEHICLE_TIMEOUT_MS              5000   // if no updates in this time, drop it from the list
#define ADSB_SQUAWK_OCTAL_DEFAULT       1200

#ifndef ADSB_VEHICLE_LIST_SIZE_DEFAULT
    #define ADSB_VEHICLE_LIST_SIZE_DEFAULT  25
#endif

#ifndef ADSB_LIST_RADIUS_DEFAULT
    #if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        #define ADSB_LIST_RADIUS_DEFAULT        10000 // in meters
    #else
        #define ADSB_LIST_RADIUS_DEFAULT        2000 // in meters
    #endif
#endif

#ifndef AP_ADSB_TYPE_DEFAULT
#define AP_ADSB_TYPE_DEFAULT 0
#endif

extern const AP_HAL::HAL& hal;

AP_ADSB *AP_ADSB::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_ADSB::var_info[] = {
    // @Param: TYPE
    // @DisplayName: ADSB Type
    // @Description: Type of ADS-B hardware for ADSB-in and ADSB-out configuration and operation. If any type is selected then MAVLink based ADSB-in messages will always be enabled
    // @Values: 0:Disabled,1:uAvionix-MAVLink-InOut,2:Sagetech,3:uAvionix-UCP,4:Sagetech MX Series,5:uAvionix-MAVLink-In
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE",     0, AP_ADSB, _type[0],    AP_ADSB_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // index 1 is reserved - was BEHAVIOR

    // @Param: LIST_MAX
    // @DisplayName: ADSB vehicle list size
    // @Description: ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
    // @Range: 1 100
    // @User: Advanced
    // @RebootRequired: True
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
    // @Description: ICAO_ID unique vehicle identification number of this aircraft. This is an integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
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
    // @Description: GPS antenna lateral offset. This describes the physical location offset from center of the GPS antenna on the aircraft.
	// @Values: 0:NoData,1:Left2m,2:Left4m,3:Left6m,4:Center,5:Right2m,6:Right4m,7:Right6m
    // @User: Advanced
    AP_GROUPINFO("OFFSET_LAT",   7, AP_ADSB, out_state.cfg.gpsOffsetLat, UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M),

    // @Param: OFFSET_LON
    // @DisplayName: GPS antenna longitudinal offset
    // @Description: GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
    // @Values: 0:NO_DATA,1:AppliedBySensor
    // @User: Advanced
    AP_GROUPINFO("OFFSET_LON",   8, AP_ADSB, out_state.cfg.gpsOffsetLon, UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR),

    // @Param: RF_SELECT
    // @DisplayName: Transceiver RF selection
    // @Description: Transceiver RF selection for Rx enable and/or Tx enable. This only effects devices that can Tx and/or Rx. Rx-only devices should override this to always be Rx-only.
    // @Bitmask: 0:Rx,1:Tx
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

    // @Param: OPTIONS
    // @DisplayName: ADS-B Options
    // @Description: Options for emergency failsafe codes and device capabilities
    // @Bitmask: 0:Ping200X Send GPS,1:Squawk 7400 on RC failsafe,2:Squawk 7400 on GCS failsafe,3:Sagetech MXS use External Config
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  15, AP_ADSB, _options, 0),

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

#ifdef ADSB_STATIC_CALLSIGN
    strncpy(out_state.cfg.callsign, ADSB_STATIC_CALLSIGN, sizeof(out_state.cfg.callsign));
#endif
}

/*
 * Initialize variables and allocate memory for array
 */
void AP_ADSB::init(void)
{
    if (in_state.vehicle_list == nullptr) {
        // sanity check param
        in_state.list_size_param.set(constrain_int16(in_state.list_size_param, 1, INT16_MAX));

        in_state.vehicle_list = NEW_NOTHROW adsb_vehicle_t[in_state.list_size_param];

        if (in_state.vehicle_list == nullptr) {
            // dynamic RAM allocation of in_state.vehicle_list[] failed
            _init_failed = true; // this keeps us from constantly trying to init forever in main update
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADSB: Unable to initialize ADSB vehicle list");
            return;
        }
        in_state.list_size_allocated = in_state.list_size_param;
    }

    if (detected_num_instances == 0) {
        for (uint8_t i=0; i<ADSB_MAX_INSTANCES; i++) {
            detect_instance(i);
            if (_backend[i] == nullptr) {
                continue;
            }
            if (!_backend[i]->init()) {
                delete _backend[i];
                _backend[i] = nullptr;
                continue;
            }
            // success
            detected_num_instances = i+1;
        }
    }

    if (detected_num_instances == 0) {
        _init_failed = true;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ADSB: Unable to initialize ADSB driver");
    }
}

bool AP_ADSB::check_startup()
{
    if (_init_failed) {
        return false;
    }

    bool all_backends_disabled = true;
    for (uint8_t instance=0; instance<ADSB_MAX_INSTANCES; instance++) {
        if (_type[instance] > 0) {
            all_backends_disabled = false;
            break;
        }
    }

    if (all_backends_disabled) {
        // nothing to do
        return false;
    }
    if (in_state.vehicle_list == nullptr)  {
        init();
    }
    return in_state.vehicle_list != nullptr;
}


//  detect if an instance of an ADSB sensor is connected.
void AP_ADSB::detect_instance(uint8_t instance)
{
    switch (get_type(instance)) {
    case Type::None:
        return;

    case Type::uAvionix_MAVLink_In:
    case Type::uAvionix_MAVLink_InOut:
#if HAL_ADSB_UAVIONIX_MAVLINK_ENABLED
        if (AP_ADSB_uAvionix_MAVLink::detect()) {
            _backend[instance] = NEW_NOTHROW AP_ADSB_uAvionix_MAVLink(*this, instance);
        }
#endif
        break;

    case Type::uAvionix_UCP:
#if HAL_ADSB_UCP_ENABLED
        if (AP_ADSB_uAvionix_UCP::detect()) {
            _backend[instance] = NEW_NOTHROW AP_ADSB_uAvionix_UCP(*this, instance);
        }
#endif
        break;

    case Type::Sagetech:
#if HAL_ADSB_SAGETECH_ENABLED
        if (AP_ADSB_Sagetech::detect()) {
            _backend[instance] = NEW_NOTHROW AP_ADSB_Sagetech(*this, instance);
        }
#endif
        break;

    case Type::Sagetech_MXS:
#if HAL_ADSB_SAGETECH_MXS_ENABLED
        if (AP_ADSB_Sagetech_MXS::detect()) {
            _backend[instance] = NEW_NOTHROW AP_ADSB_Sagetech_MXS(*this, instance);
        }
#endif
        break;
    }

}

// get instance type from instance
AP_ADSB::Type AP_ADSB::get_type(uint8_t instance) const
{
    if (instance < ADSB_MAX_INSTANCES) {
        return (Type)(_type[instance].get());
    }
    return Type::None;
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

#if AP_GPS_ENABLED && AP_AHRS_ENABLED && AP_BARO_ENABLED
/*
 * periodic update to handle vehicle timeouts and trigger collision detection
 */
void AP_ADSB::update(void)
{
    Loc loc{};
    if (!AP::ahrs().get_location(loc)) {
        loc.zero();
    }

    const AP_GPS &gps = AP::gps();

    loc.fix_type = (AP_GPS_FixType)gps.status();
    loc.epoch_us = gps.time_epoch_usec();
#if AP_RTC_ENABLED
    loc.have_epoch_from_rtc_us = AP::rtc().get_utc_usec(loc.epoch_from_rtc_us);
#endif

    loc.satellites = gps.num_sats();

    loc.horizontal_pos_accuracy_is_valid = gps.horizontal_accuracy(loc.horizontal_pos_accuracy);
    loc.vertical_pos_accuracy_is_valid = gps.vertical_accuracy(loc.vertical_pos_accuracy);
    loc.horizontal_vel_accuracy_is_valid = gps.speed_accuracy(loc.horizontal_vel_accuracy);


    loc.vel_ned = gps.velocity();

    loc.vertRateD_is_valid = AP::ahrs().get_vert_pos_rate_D(loc.vertRateD);

    const auto &baro = AP::baro();
    loc.baro_is_healthy = baro.healthy();

    // Altitude difference between sea level pressure and current
    // pressure (in metres)
    if (loc.baro_is_healthy) {
        loc.baro_alt_press_diff_sea_level = baro.get_altitude_difference(SSL_AIR_PRESSURE, baro.get_pressure());
    }

    update(loc);
}
#endif  // AP_GPS_ENABLED && AP_AHRS_ENABLED

void AP_ADSB::update(const AP_ADSB::Loc &loc)
{
    if (!check_startup()) {
        return;
    }

    _my_loc = loc;

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

    if (out_state.cfg.squawk_octal_param != out_state.cfg.squawk_octal) {
        // param changed, check that it's a valid octal
        if (!is_valid_callsign(out_state.cfg.squawk_octal_param)) {
            // invalid, reset it to default
            out_state.cfg.squawk_octal_param.set(ADSB_SQUAWK_OCTAL_DEFAULT);
        }
        out_state.cfg.squawk_octal = (uint16_t)out_state.cfg.squawk_octal_param;
    }

    // ensure it's positive 24bit but allow -1
    if (out_state.cfg.ICAO_id_param <= -1 || out_state.cfg.ICAO_id_param > 0x00FFFFFF) {
        // icao param of -1 means static information is not sent, transceiver is assumed pre-programmed.
        // reset timer constantly so it never reaches 10s so it never sends
        out_state.last_config_ms = now;

    } else if ((out_state.cfg.rfSelect & UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED) &&
                (out_state.cfg.ICAO_id == 0 || out_state.cfg.ICAO_id_param_prev != out_state.cfg.ICAO_id_param)) {

        // if param changed then regenerate. This allows the param to be changed back to zero to trigger a re-generate
        if (out_state.cfg.ICAO_id_param == 0) {
            out_state.cfg.ICAO_id = genICAO(_my_loc);
        } else {
            out_state.cfg.ICAO_id = out_state.cfg.ICAO_id_param;
        }
        out_state.cfg.ICAO_id_param_prev = out_state.cfg.ICAO_id_param;

#ifndef ADSB_STATIC_CALLSIGN
        if (!out_state.cfg.was_set_externally) {
            set_callsign("ARDU", true);
        }
#endif
        out_state.last_config_ms = 0; // send now
    }

    for (uint8_t i=0; i<detected_num_instances; i++) {
        if (_backend[i] != nullptr && _type[i].get() != (int8_t)Type::None) {
            _backend[i]->update();
        }
    }

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

    in_state.furthest_vehicle_index = max_distance_index;
    in_state.furthest_vehicle_distance = max_distance;
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
    if (index == in_state.furthest_vehicle_index && in_state.furthest_vehicle_distance > 0) {
        in_state.furthest_vehicle_distance = 0;
        in_state.furthest_vehicle_index = 0;
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
    if (!check_startup()) {
        return;
    }

    uint16_t index = in_state.list_size_allocated + 1; // initialize with invalid index
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

    } else if (in_state.vehicle_count < in_state.list_size_allocated) {

        // not found and there's room, add it to the end of the list
        set_vehicle(in_state.vehicle_count, vehicle);
        in_state.vehicle_count++;

    } else {
        // buffer is full. if new vehicle is closer than furthest, replace furthest with new

        if (my_loc_is_zero) {
            // nothing else to do
            in_state.furthest_vehicle_distance = 0;
            in_state.furthest_vehicle_index = 0;

        } else {
            if (in_state.furthest_vehicle_distance <= 0) {
                // ensure this is populated
                determine_furthest_aircraft();
            }

            if (my_loc_distance_to_vehicle < in_state.furthest_vehicle_distance) { // is closer than the furthest
                // replace with the furthest vehicle
                set_vehicle(in_state.furthest_vehicle_index, vehicle);

                // in_state.furthest_vehicle_index is now invalid because the vehicle was overwritten, need
                // to run determine_furthest_aircraft() to determine a new one next time
                in_state.furthest_vehicle_distance = 0;
                in_state.furthest_vehicle_index = 0;
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
    if (index >= in_state.list_size_allocated) {
        // out of range
        return;
    }
    in_state.vehicle_list[index] = vehicle;

#if HAL_LOGGING_ENABLED
    write_log(vehicle);
#endif
}

void AP_ADSB::send_adsb_vehicle(const mavlink_channel_t chan)
{
    if (!check_startup() || in_state.vehicle_count == 0) {
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
 * handle incoming packet UAVIONIX_ADSB_OUT_CFG.
 * This allows a GCS to send cfg info through the autopilot to the ADSB hardware.
 * This is done indirectly by reading and storing the packet and then another mechanism sends it out periodically
 */
void AP_ADSB::handle_out_cfg(const mavlink_uavionix_adsb_out_cfg_t &packet)
{
    out_state.cfg.was_set_externally = true;

    out_state.cfg.ICAO_id = packet.ICAO;
    out_state.cfg.ICAO_id_param.set(out_state.cfg.ICAO_id_param_prev = packet.ICAO & 0x00FFFFFFFF);

    // May contain a non-null value at the end so accept it as-is with memcpy instead of strcpy
    memcpy(out_state.cfg.callsign, packet.callsign, sizeof(out_state.cfg.callsign));

    out_state.cfg.emitterType.set(packet.emitterType);
    out_state.cfg.lengthWidth.set(packet.aircraftSize);
    out_state.cfg.gpsOffsetLat.set(packet.gpsOffsetLat);
    out_state.cfg.gpsOffsetLon.set(packet.gpsOffsetLon);
    out_state.cfg.rfSelect.set(packet.rfSelect);
    out_state.cfg.stall_speed_cm = packet.stallSpeed;

    // guard against string with non-null end char
    char tmp_callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN+1] {};
    memcpy(tmp_callsign, out_state.cfg.callsign, MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADSB: Using ICAO_id %d and Callsign %s", (int)out_state.cfg.ICAO_id, tmp_callsign);

    // send now
    out_state.last_config_ms = 0;
}

/*
 * handle incoming packet UAVIONIX_ADSB_OUT_CONTROL
 * allows a GCS to set the contents of the control message sent by ardupilot to the transponder
 */
void AP_ADSB::handle_out_control(const mavlink_uavionix_adsb_out_control_t &packet)
{
    out_state.ctrl.baroCrossChecked = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_EXTERNAL_BARO_CROSSCHECKED;
    out_state.ctrl.airGroundState = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_ON_GROUND;
    out_state.ctrl.identActive = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_IDENT_BUTTON_ACTIVE;
    out_state.ctrl.modeAEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_A_ENABLED;
    out_state.ctrl.modeCEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_C_ENABLED;
    out_state.ctrl.modeSEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_S_ENABLED;
    out_state.ctrl.es1090TxEnabled = packet.state & UAVIONIX_ADSB_OUT_CONTROL_STATE::UAVIONIX_ADSB_OUT_CONTROL_STATE_1090ES_TX_ENABLED;
    out_state.ctrl.externalBaroAltitude_mm = packet.baroAltMSL;
    out_state.ctrl.squawkCode = packet.squawk;
    out_state.ctrl.emergencyState = packet.emergencyStatus;
    memcpy(out_state.ctrl.callsign, packet.flight_id, sizeof(out_state.ctrl.callsign));
    out_state.ctrl.x_bit = packet.x_bit;
}

/*
 * this is a message from the transceiver reporting it's health. Using this packet
 * we determine which channel is on so we don't have to send out_state to all channels
 */
void AP_ADSB::handle_transceiver_report(const mavlink_channel_t chan, const mavlink_uavionix_adsb_transceiver_health_report_t &packet)
{
    // Don't ingest a transciever report, as it's used to trigger sending out control packets
    if (_type[0] == (int8_t)(AP_ADSB::Type::uAvionix_MAVLink_In)) {
        return;
    }
    // we need to refactor the code to use AP_SerialManager and ask the user to tell us which serial port is rx vs tx
    static_assert(ADSB_MAX_INSTANCES == 1, "Transceiver control is not correctly handled with more then one receiver");

    if (out_state.chan != chan) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ADSB: Found transceiver on channel %d", chan);
    }

    out_state.chan_last_ms = AP_HAL::millis();
    out_state.chan = chan;
    out_state.status = (UAVIONIX_ADSB_RF_HEALTH)packet.rfHealth;
}

/*
 * send a periodic report of the ADSB out status
 */
void AP_ADSB::send_adsb_out_status(const mavlink_channel_t chan) const
{
    for (uint8_t i=0; i < ADSB_MAX_INSTANCES; i++) {
        if (_type[i] == (int8_t)(AP_ADSB::Type::uAvionix_UCP) || _type[i] == (int8_t)(AP_ADSB::Type::Sagetech_MXS)) {
            mavlink_msg_uavionix_adsb_out_status_send_struct(chan, &out_state.tx_status);
            return;
        }
    }
}

/*
 @brief Generates pseudorandom ICAO from gps time, lat, and lon.
 Reference: DO282B, 2.2.4.5.1.3.2
*/
uint32_t AP_ADSB::genICAO(const Location &loc) const
{
    // gps_time is used as a pseudo-random number instead of seconds since UTC midnight
    // TODO: use UTC time instead of GPS time
    const AP_ADSB::Loc &gps { _my_loc };
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
        hal.util->snprintf(&out_state.cfg.callsign[4], 5, "%04X", unsigned(out_state.cfg.ICAO_id % 0x10000));
    }
}


void AP_ADSB::push_sample(const adsb_vehicle_t &vehicle)
{
    _samples.push(vehicle);
}

bool AP_ADSB::next_sample(adsb_vehicle_t &vehicle)
{
    return _samples.pop(vehicle);
}

void AP_ADSB::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ADSB_VEHICLE: {
            adsb_vehicle_t vehicle {};
            mavlink_msg_adsb_vehicle_decode(&msg, &vehicle.info);
            vehicle.last_update_ms = AP_HAL::millis() - uint32_t(vehicle.info.tslc * 1000U);
            handle_adsb_vehicle(vehicle);
            break;
        }

        case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT: {
            mavlink_uavionix_adsb_transceiver_health_report_t packet {};
            mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&msg, &packet);
            handle_transceiver_report(chan, packet);
            break;
        }

        case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG: {
            mavlink_uavionix_adsb_out_cfg_t packet {};
            mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet);
            handle_out_cfg(packet);
            break;
        }

        case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
            // unhandled, this is an outbound packet only
            break;

        case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL: {
            mavlink_uavionix_adsb_out_control_t packet {};
            mavlink_msg_uavionix_adsb_out_control_decode(&msg, &packet);
            handle_out_control(packet);
            break;
        }
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

#if HAL_LOGGING_ENABLED
/*
 * Write vehicle to log
 */
void AP_ADSB::write_log(const adsb_vehicle_t &vehicle) const
{
    switch ((Logging)_log) {
        case Logging::SPECIAL_ONLY:
            if (!is_special_vehicle(vehicle.info.ICAO_address)) {
                return;
            }
            break;

        case Logging::ALL:
            break;

        case Logging::NONE:
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
#endif // HAL_LOGGING_ENABLED

/**
* @brief Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value stored on a GCS as a string field in different format, but then transmitted over mavlink as a float which is always a decimal.
* baseIn: base of input number
* inputNumber: value currently in base "baseIn" to be converted to base "baseOut"
*
* Example: convert ADSB squawk octal "1200" stored in memory as 0x0280 to 0x04B0
*          uint16_t squawk_decimal = convertMathBase(8, squawk_octal);
*/
uint32_t AP_ADSB::convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber)
{
    // Our only sensible input bases are 16 and 8
    if (baseIn != 8 && baseIn != 16) {
        return inputNumber;
    }
    uint32_t outputNumber = 0;
    for (uint8_t i=0; i < 10; i++) {
        outputNumber += (inputNumber % 10) * powf(baseIn, i);
        inputNumber /= 10;
        if (inputNumber == 0) break;
    }
    return outputNumber;
}

// methods for embedded class Location
bool AP_ADSB::Loc::speed_accuracy(float &sacc) const
{
    if (!horizontal_vel_accuracy_is_valid) {
        return false;
    }
    sacc = horizontal_vel_accuracy;
    return true;
}

bool AP_ADSB::Loc::horizontal_accuracy(float &hacc) const
{
    if (!horizontal_pos_accuracy_is_valid) {
        return false;
    }
    hacc = horizontal_pos_accuracy;
    return true;
}

bool AP_ADSB::Loc::vertical_accuracy(float &vacc) const
{
    if (!vertical_pos_accuracy_is_valid) {
        return false;
    }
    vacc = vertical_pos_accuracy;
    return true;
}

AP_ADSB *AP::ADSB()
{
    return AP_ADSB::get_singleton();
}
#endif // HAL_ADSB_ENABLED
