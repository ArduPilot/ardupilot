/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
    AP_ADSB(const AP_AHRS &ahrs) :
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

    // handle ADS-B transceiver report
    void transceiver_report(mavlink_channel_t chan, const mavlink_message_t* msg);

    bool get_possible_threat()  { return _enabled && avoid_state.another_vehicle_within_radius; }

    ADSB_BEHAVIOR get_behavior()  { return (ADSB_BEHAVIOR)(avoid_state.behavior.get()); }
    bool get_is_evading_threat()  { return _enabled && avoid_state.is_evading_threat; }
    void set_is_evading_threat(const bool is_evading) { if (_enabled) { avoid_state.is_evading_threat = is_evading; } }
    uint16_t get_vehicle_count() { return in_state.vehicle_count; }

    // send ADSB_VEHICLE mavlink message, usually as a StreamRate
    void send_adsb_vehicle(mavlink_channel_t chan);

    void set_stall_speed_cm(const uint16_t stall_speed_cm) { out_state.cfg.stall_speed_cm = stall_speed_cm; }

    void set_is_auto_mode(const bool is_in_auto_mode) { _is_in_auto_mode = is_in_auto_mode; }
    void set_is_flying(const bool is_flying) { out_state.is_flying = is_flying; }

    UAVIONIX_ADSB_RF_HEALTH get_transceiver_status(void) { return out_state.status; }

    // extract a location out of a vehicle item
    Location_Class get_location(const adsb_vehicle_t &vehicle) const;

    bool enabled() const {
        return _enabled;
    }
    bool next_sample(adsb_vehicle_t &obstacle);

private:

    // initialize _vehicle_list
    void init();

    // free _vehicle_list
    void deinit();

    // compares current vector against vehicle_list to detect threats
    void perform_threat_detection(void);

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    bool find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const;

    // remove a vehicle from the list
    void delete_vehicle(const uint16_t index);

    void set_vehicle(const uint16_t index, const adsb_vehicle_t &vehicle);

    // Generates pseudorandom ICAO from gps time, lat, and lon
    uint32_t genICAO(const Location_Class &loc);

    // set callsign: 8char string (plus null termination) then optionally append last 4 digits of icao
    void set_callsign(const char* str, const bool append_icao);

    // send static and dynamic data to ADSB transceiver
    void send_configure(const mavlink_channel_t chan);
    void send_dynamic_out(const mavlink_channel_t chan);


    // reference to AHRS, so we can ask for our position,
    // heading and speed
    const AP_AHRS &_ahrs;

    AP_Int8     _enabled;

    Location_Class  _my_loc;

    bool _is_in_auto_mode;

    // ADSB-IN state. Maintains list of external vehicles
    struct {
        // list management
        AP_Int16    list_size_param;
        uint16_t    list_size = 1; // start with tiny list, then change to param-defined size. This ensures it doesn't fail on start
        adsb_vehicle_t *vehicle_list = nullptr;
        uint16_t    vehicle_count;
        AP_Int32    list_radius;

        // streamrate stuff
        uint32_t    send_start_ms[MAVLINK_COMM_NUM_BUFFERS];
        uint16_t    send_index[MAVLINK_COMM_NUM_BUFFERS];
    } in_state;


    // ADSB-OUT state. Maintains export data
    struct {
        uint32_t    last_config_ms; // send once every 10s
        uint32_t    last_report_ms; // send at 5Hz
        int8_t      chan = -1; // channel that contains an ADS-b Transceiver. -1 means broadcast to all
        uint32_t    chan_last_ms;
        UAVIONIX_ADSB_RF_HEALTH status;     // transceiver status
        bool        is_flying;

        // ADSB-OUT configuration
        struct {
            int32_t     ICAO_id;
            AP_Int32    ICAO_id_param;
            int32_t     ICAO_id_param_prev = -1; // assume we never send
            char        callsign[9]; //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only).
            AP_Int8     emitterType;
            AP_Int8     lengthWidth;  // Aircraft length and width encoding (table 2-35 of DO-282B)
            AP_Int8     gpsLatOffset;
            AP_Int8     gpsLonOffset;
            uint16_t    stall_speed_cm;
            AP_Int8     rfSelect;
        } cfg;

    } out_state;


    // Avoidance state
    struct {
        AP_Int8     behavior;

        bool        another_vehicle_within_radius;
        bool        is_evading_threat;

        // index of and distance to vehicle with lowest threat
        uint16_t    lowest_threat_index;
        float       lowest_threat_distance;

        // index of and distance to vehicle with highest threat
        uint16_t    highest_threat_index;
        float       highest_threat_distance;
    } avoid_state;

    static const uint8_t max_samples = 30;
    AP_Buffer<adsb_vehicle_t, max_samples> samples;

    void push_sample(adsb_vehicle_t &vehicle);

};
