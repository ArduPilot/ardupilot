#pragma once

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
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Buffer/AP_Buffer.h>

class AP_ADSB {
public:
    AP_ADSB()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_ADSB(const AP_ADSB &other) = delete;
    AP_ADSB &operator=(const AP_ADSB&) = delete;

    struct adsb_vehicle_t {
        mavlink_adsb_vehicle_t info; // the whole mavlink struct with all the juicy details. sizeof() == 38
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
    };

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update(void);

    uint16_t get_vehicle_count() { return in_state.vehicle_count; }

    // send ADSB_VEHICLE mavlink message, usually as a StreamRate
    void send_adsb_vehicle(mavlink_channel_t chan);

    void set_stall_speed_cm(const uint16_t stall_speed_cm) { out_state.cfg.stall_speed_cm = stall_speed_cm; }
    void set_max_speed(int16_t max_speed) {
        if (!out_state.cfg.was_set_externally) {
            // convert m/s to knots
            out_state.cfg.maxAircraftSpeed_knots = (float)max_speed * 1.94384f;
        }
    }

    void set_is_auto_mode(const bool is_in_auto_mode) { out_state._is_in_auto_mode = is_in_auto_mode; }
    void set_is_flying(const bool is_flying) { out_state.is_flying = is_flying; }

    UAVIONIX_ADSB_RF_HEALTH get_transceiver_status(void) { return out_state.status; }

    // extract a location out of a vehicle item
    Location get_location(const adsb_vehicle_t &vehicle) const;

    bool enabled() const {
        return _enabled;
    }
    bool next_sample(adsb_vehicle_t &obstacle);

    // mavlink message handler
    void handle_message(const mavlink_channel_t chan, const mavlink_message_t* msg);

    // when true, a vehicle with that ICAO was found in database and the vehicle is populated.
    bool get_vehicle_by_ICAO(const uint32_t icao, adsb_vehicle_t &vehicle) const;

    uint32_t get_special_ICAO_target() const { return (uint32_t)_special_ICAO_target; };
    void set_special_ICAO_target(const uint32_t new_icao_target) { _special_ICAO_target = (int32_t)new_icao_target; };
    bool is_special_vehicle(uint32_t icao) const { return _special_ICAO_target != 0 && (_special_ICAO_target == (int32_t)icao); }


private:
    // initialize _vehicle_list
    void init();

    // free _vehicle_list
    void deinit();

    // compares current vector against vehicle_list to detect threats
    void determine_furthest_aircraft(void);

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    bool find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const;

    // remove a vehicle from the list
    void delete_vehicle(const uint16_t index);

    void set_vehicle(const uint16_t index, const adsb_vehicle_t &vehicle);

    // Generates pseudorandom ICAO from gps time, lat, and lon
    uint32_t genICAO(const Location &loc);

    // set callsign: 8char string (plus null termination) then optionally append last 4 digits of icao
    void set_callsign(const char* str, const bool append_icao);

    // send static and dynamic data to ADSB transceiver
    void send_configure(const mavlink_channel_t chan);
    void send_dynamic_out(const mavlink_channel_t chan);

    // special helpers for uAvionix workarounds
    uint32_t get_encoded_icao(void);
    uint8_t get_encoded_callsign_null_char(void);

    // add or update vehicle_list from inbound mavlink msg
    void handle_vehicle(const mavlink_message_t* msg);

    // handle ADS-B transceiver report for ping2020
    void handle_transceiver_report(mavlink_channel_t chan, const mavlink_message_t* msg);

    void handle_out_cfg(const mavlink_message_t* msg);

    AP_Int8     _enabled;

    Location  _my_loc;


    // ADSB-IN state. Maintains list of external vehicles
    struct {
        // list management
        AP_Int16    list_size_param;
        uint16_t    list_size = 1; // start with tiny list, then change to param-defined size. This ensures it doesn't fail on start
        adsb_vehicle_t *vehicle_list = nullptr;
        uint16_t    vehicle_count;
        AP_Int32    list_radius;
        AP_Int16    list_altitude;

        // streamrate stuff
        uint32_t    send_start_ms[MAVLINK_COMM_NUM_BUFFERS];
        uint16_t    send_index[MAVLINK_COMM_NUM_BUFFERS];
    } in_state;


    // ADSB-OUT state. Maintains export data
    struct {
        uint32_t    last_config_ms; // send once every 10s
        uint32_t    last_report_ms; // send at 5Hz
        int8_t      chan = -1; // channel that contains an ADS-b Transceiver. -1 means transceiver is not detected
        uint32_t    chan_last_ms;
        UAVIONIX_ADSB_RF_HEALTH status;     // transceiver status
        bool        is_flying;
        bool        _is_in_auto_mode;

        // ADSB-OUT configuration
        struct {
            int32_t     ICAO_id;
            AP_Int32    ICAO_id_param;
            int32_t     ICAO_id_param_prev = -1; // assume we never send
            char        callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN]; //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only).
            AP_Int8     emitterType;
            AP_Int8     lengthWidth;  // Aircraft length and width encoding (table 2-35 of DO-282B)
            AP_Int8     gpsLatOffset;
            AP_Int8     gpsLonOffset;
            uint16_t    stall_speed_cm;
            AP_Int8     rfSelect;
            AP_Int16    squawk_octal_param;
            uint16_t    squawk_octal;
            float       maxAircraftSpeed_knots;
            AP_Int8     rf_capable;
            bool        was_set_externally;
        } cfg;

    } out_state;


    // index of and distance to furthest vehicle in list
    uint16_t    furthest_vehicle_index;
    float       furthest_vehicle_distance;


    // special ICAO of interest that ignored filters when != 0
    AP_Int32 _special_ICAO_target;

    static const uint8_t max_samples = 30;
    AP_Buffer<adsb_vehicle_t, max_samples> samples;

    void push_sample(adsb_vehicle_t &vehicle);

};
