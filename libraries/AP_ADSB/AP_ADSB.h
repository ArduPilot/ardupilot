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

#include "AP_ADSB_config.h"

#if HAL_ADSB_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_GPS/AP_GPS_FixType.h>

#define ADSB_MAX_INSTANCES             1   // Maximum number of ADSB sensor instances available on this platform

#define ADSB_BITBASK_RF_CAPABILITIES_UAT_IN         (1 << 0)
#define ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN      (1 << 1)
#define ADSB_BITBASK_RF_CAPABILITIES_UAT_OUT        (1 << 2)
#define ADSB_BITBASK_RF_CAPABILITIES_1090ES_OUT     (1 << 3)

class AP_ADSB_Backend;

class AP_ADSB {
public:
    friend class AP_ADSB_Backend;
    friend class AP_ADSB_uAvionix_MAVLink;
    friend class AP_ADSB_uAvionix_UCP;
    friend class AP_ADSB_Sagetech;
    friend class AP_ADSB_Sagetech_MXS;

    // constructor
    AP_ADSB();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ADSB);

    // get singleton instance
    static AP_ADSB *get_singleton(void) {
        return _singleton;
    }

    // ADSB driver types
    enum class Type {
        None                = 0,
        uAvionix_MAVLink    = 1,
        Sagetech            = 2,
        uAvionix_UCP        = 3,
        Sagetech_MXS        = 4,
    };

    struct adsb_vehicle_t {
        mavlink_adsb_vehicle_t info; // the whole mavlink struct with all the juicy details. sizeof() == 38
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
    };

    // enum for adsb optional features
    enum class AdsbOption {
        Ping200X_Send_GPS               = (1<<0),
        Squawk_7400_FS_RC               = (1<<1),
        Squawk_7400_FS_GCS              = (1<<2),
        SagteTech_MXS_External_Config   = (1<<3),
        Mode3_Only                      = (1<<4),
    };

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update(void);

    // a structure holding *this vehicle's* position-related information:
    enum class AltType {
        Barometric = 0,  // we use a specific model for this?
        WGS84 = 1,
    };
    struct Loc : Location {
        AltType loc_alt_type;  // more information on altitude in base class

        AP_GPS_FixType fix_type;
        uint64_t epoch_us;  // microseconds since 1970-01-01
        uint64_t epoch_from_rtc_us;  // microseconds since 1970-01-01
        bool have_epoch_from_rtc_us;
        uint8_t satellites;

        float horizontal_pos_accuracy;
        bool horizontal_pos_accuracy_is_valid;

        float vertical_pos_accuracy;
        bool vertical_pos_accuracy_is_valid;

        float horizontal_vel_accuracy;
        bool horizontal_vel_accuracy_is_valid;

        Vector3f vel_ned;

        float vertRateD;  // m/s down
        bool vertRateD_is_valid;

        // methods to make us look much like the AP::gps() singleton:
        AP_GPS_FixType status() const { return fix_type; }
        const Vector3f &velocity() const {
            return vel_ned;
        }
        uint64_t time_epoch_usec() const { return epoch_us; }

        bool speed_accuracy(float &sacc) const;
        bool horizontal_accuracy(float &hacc) const;
        bool vertical_accuracy(float &vacc) const;

        uint8_t num_sats() const { return satellites; }

        // methods to make us look like the AP::ahrs() singleton:
        const Vector2f &groundspeed_vector() const { return vel_ned.xy(); }
        bool get_vert_pos_rate_D(float &velocity) const {
            velocity = vertRateD;
            return vertRateD_is_valid;
        }

        // data from a pressure sensor:
        bool baro_is_healthy;
        float baro_alt_press_diff_sea_level;

    } _my_loc;

    // periodic task that maintains vehicle_list
    void update(const Loc &loc);

    // send ADSB_VEHICLE mavlink message, usually as a StreamRate
    void send_adsb_vehicle(mavlink_channel_t chan);

    bool set_stall_speed_cm(const uint16_t stall_speed_cm) {
        if (out_state.cfg.was_set_externally) {
            return false;
        }
        out_state.cfg.stall_speed_cm = stall_speed_cm;
        return true;
    }

    bool set_max_speed(int16_t max_speed) {
        if (out_state.cfg.was_set_externally) {
            return false;
        }
        // convert m/s to knots
        out_state.cfg.maxAircraftSpeed_knots = (float)max_speed * M_PER_SEC_TO_KNOTS;
        return true;
    }

    void set_is_auto_mode(const bool is_in_auto_mode) { out_state.is_in_auto_mode = is_in_auto_mode; }
    void set_is_flying(const bool is_flying) { out_state.is_flying = is_flying; }

    // extract a location out of a vehicle item
    Location get_location(const adsb_vehicle_t &vehicle) const;

    // ADSB is considered enabled if there are any configured backends
    bool enabled() const {
        for (uint8_t instance=0; instance<detected_num_instances; instance++) {
            if (_type[instance] > 0) {
                return true;
            }
        }
        return false;
    }

    bool init_failed() const {
        return _init_failed;
    }

    bool healthy() {
        return check_startup();
    }

    bool next_sample(adsb_vehicle_t &obstacle);

    // handle a adsb_vehicle_t from an external source
    void handle_adsb_vehicle(const adsb_vehicle_t &vehicle);

    // mavlink message handler
    void handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg);

    void send_adsb_out_status(const mavlink_channel_t chan) const;

    // when true, a vehicle with that ICAO was found in database and the vehicle is populated.
    bool get_vehicle_by_ICAO(const uint32_t icao, adsb_vehicle_t &vehicle) const;

    uint32_t get_special_ICAO_target() const { return (uint32_t)_special_ICAO_target; };
    void set_special_ICAO_target(const uint32_t new_icao_target) { _special_ICAO_target.set((int32_t)new_icao_target); };
    bool is_special_vehicle(uint32_t icao) const { return _special_ICAO_target != 0 && (_special_ICAO_target == (int32_t)icao); }

    // confirm a value is a valid callsign
    static bool is_valid_callsign(uint16_t octal) WARN_IF_UNUSED;

    static uint8_t convert_maxknots_to_enum(const float maxAircraftSpeed_knots);

    // Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value
    // stored on a GCS as a string field in different format, but then transmitted
    // over mavlink as a float which is always a decimal.
    static uint32_t convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber);

    // Trigger a Mode 3/A transponder IDENT. This should only be done when requested to do so by an Air Traffic Controller.
    // See wikipedia for IDENT explanation https://en.wikipedia.org/wiki/Transponder_(aeronautics)
    bool ident_start();

    AP_ADSB::Type get_type(uint8_t instance) const;

private:
    static AP_ADSB *_singleton;

    // initialize vehicle_list
    void init();

    // check to see if we are initialized (and possibly do initialization)
    bool check_startup();

    // compares current vector against vehicle_list to detect threats
    void determine_furthest_aircraft(void);

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    bool find_index(const adsb_vehicle_t &vehicle, uint16_t *index) const;

    // remove a vehicle from the list
    void delete_vehicle(const uint16_t index);

    void set_vehicle(const uint16_t index, const adsb_vehicle_t &vehicle);

    // Generates pseudorandom ICAO from gps time, lat, and lon
    uint32_t genICAO(const Location &loc) const;

    // set callsign: 8char string (plus null termination) then optionally append last 4 digits of icao
    void set_callsign(const char* str, const bool append_icao);

    // configure ADSB-out transceivers
    void handle_out_cfg(const mavlink_uavionix_adsb_out_cfg_t &packet);

    // control ADSB-out transcievers
    void handle_out_control(const mavlink_uavionix_adsb_out_control_t &packet);

    // mavlink handler
    void handle_transceiver_report(const mavlink_channel_t chan, const mavlink_uavionix_adsb_transceiver_health_report_t &packet);

    void detect_instance(uint8_t instance);

    AP_Int8 _type[ADSB_MAX_INSTANCES];

    bool _init_failed;

    // ADSB-IN state. Maintains list of external vehicles
    struct {
        // list management
        AP_Int16    list_size_param;
        uint16_t    list_size_allocated;
        adsb_vehicle_t *vehicle_list;
        uint16_t    vehicle_count;
        AP_Int32    list_radius;
        AP_Int16    list_altitude;

        // index of and distance to furthest vehicle in list
        uint16_t    furthest_vehicle_index;
        float       furthest_vehicle_distance;

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
        bool        is_in_auto_mode;

        // ADSB-OUT configuration
        struct {
            int32_t     ICAO_id;
            AP_Int32    ICAO_id_param;
            int32_t     ICAO_id_param_prev = -1; // assume we never send
            char        callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN]; //Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only).
            AP_Int8     emitterType;
            AP_Int8     lengthWidth;  // Aircraft length and width encoding (table 2-35 of DO-282B)
            AP_Int8     gpsOffsetLat;
            AP_Int8     gpsOffsetLon;
            uint16_t    stall_speed_cm;
            AP_Int8     rfSelect;
            AP_Int16    squawk_octal_param;
            uint16_t    squawk_octal;
            float       maxAircraftSpeed_knots;
            AP_Int8     rf_capable;
            bool        was_set_externally;
        } cfg;

        struct {
            bool                          baroCrossChecked;
            uint8_t                       airGroundState;
            bool                          identActive;
            bool                          modeAEnabled;
            bool                          modeCEnabled;
            bool                          modeSEnabled;
            bool                          es1090TxEnabled;
            int32_t                       externalBaroAltitude_mm;
            uint16_t                      squawkCode;
            uint8_t                       emergencyState;
            uint8_t                       callsign[8];
            bool                          x_bit;
        } ctrl;

        mavlink_uavionix_adsb_out_status_t tx_status;
    } out_state;

    uint8_t detected_num_instances;

    // special ICAO of interest that ignored filters when != 0
    AP_Int32 _special_ICAO_target;

    AP_Int32 _options;

    static const uint8_t _max_samples = 30;
    ObjectBuffer<adsb_vehicle_t> _samples{_max_samples};

    void push_sample(const adsb_vehicle_t &vehicle);

    // logging
    void write_log(const adsb_vehicle_t &vehicle) const;
    enum class Logging {
        NONE            = 0,
        SPECIAL_ONLY    = 1,
        ALL             = 2
    };
    AP_Enum<Logging> _log;

    // reference to backend
    AP_ADSB_Backend *_backend[ADSB_MAX_INSTANCES];
};

namespace AP {
    AP_ADSB *ADSB();
};

#endif // HAL_ADSB_ENABLED
