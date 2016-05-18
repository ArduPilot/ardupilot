// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "GPS_detect_state.h"
#include <AP_SerialManager/AP_SerialManager.h>

/**
   maximum number of GPS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#define GPS_MAX_INSTANCES 2
#define GPS_RTK_INJECT_TO_ALL 127

class DataFlash_Class;
class AP_GPS_Backend;

/// @class AP_GPS
/// GPS driver main class
class AP_GPS
{
public:
    // constructor
	AP_GPS() {
		AP_Param::setup_object_defaults(this, var_info);
    }

    /// Startup initialisation.
    void init(DataFlash_Class *dataflash, const AP_SerialManager& serial_manager);

    /// Update GPS state based on possible bytes received from the module.
    /// This routine must be called periodically (typically at 10Hz or
    /// more) to process incoming data.
    void update(void);

    // GPS driver types
    enum GPS_Type {
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_AUTO  = 1,
        GPS_TYPE_UBLOX = 2,
        GPS_TYPE_MTK   = 3,
        GPS_TYPE_MTK19 = 4,
        GPS_TYPE_NMEA  = 5,
        GPS_TYPE_SIRF  = 6,
        GPS_TYPE_HIL   = 7,
        GPS_TYPE_SBP   = 8,
        GPS_TYPE_PX4   = 9,
        GPS_TYPE_SBF   = 10,
		GPS_TYPE_GSOF  = 11,
		GPS_TYPE_QURT  = 12,
        GPS_TYPE_ERB = 13,
    };

    /// GPS status codes
    enum GPS_Status {
        NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
    };

    // GPS navigation engine settings. Not all GPS receivers support
    // this
    enum GPS_Engine_Setting {
        GPS_ENGINE_NONE        = -1,
        GPS_ENGINE_PORTABLE    = 0,
        GPS_ENGINE_STATIONARY  = 2,
        GPS_ENGINE_PEDESTRIAN  = 3,
        GPS_ENGINE_AUTOMOTIVE  = 4,
        GPS_ENGINE_SEA         = 5,
        GPS_ENGINE_AIRBORNE_1G = 6,
        GPS_ENGINE_AIRBORNE_2G = 7,
        GPS_ENGINE_AIRBORNE_4G = 8
    };

   enum GPS_Config {
       GPS_ALL_CONFIGURED = 255
   };

    /*
      The GPS_State structure is filled in by the backend driver as it
      parses each message from the GPS.
     */
    struct GPS_State {
        uint8_t instance; // the instance number of this GPS

        // all the following fields must all be filled by the backend driver
        GPS_Status status;                  ///< driver fix status
        uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
        uint16_t time_week;                 ///< GPS week number
        Location location;                  ///< last fix location
        float ground_speed;                 ///< ground speed in m/sec
        float ground_course;                ///< ground course in degrees
        uint16_t hdop;                      ///< horizontal dilution of precision in cm
        uint16_t vdop;                      ///< vertical dilution of precision in cm
        uint8_t num_sats;                   ///< Number of visible satellites        
        Vector3f velocity;                  ///< 3D velocitiy in m/s, in NED format
        float speed_accuracy;
        float horizontal_accuracy;
        float vertical_accuracy;
        bool have_vertical_velocity:1;      ///< does this GPS give vertical velocity?
        bool have_speed_accuracy:1;
        bool have_horizontal_accuracy:1;
        bool have_vertical_accuracy:1;
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
    };

    // Accessor functions

    // return number of active GPS sensors. Note that if the first GPS
    // is not present but the 2nd is then we return 2
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    uint8_t primary_sensor(void) const {
        return primary_instance;
    }

    /// Query GPS status
    GPS_Status status(uint8_t instance) const {
        return state[instance].status;
    }
    GPS_Status status(void) const {
        return status(primary_instance);
    }

    // Query the highest status this GPS supports
    GPS_Status highest_supported_status(uint8_t instance) const;
    GPS_Status highest_supported_status(void) const;

    // location of last fix
    const Location &location(uint8_t instance) const {
        return state[instance].location;
    }
    const Location &location() const {
        return location(primary_instance);
    }

    bool speed_accuracy(uint8_t instance, float &sacc) const {
        if(state[instance].have_speed_accuracy) {
            sacc = state[instance].speed_accuracy;
            return true;
        }
        return false;
    }

    bool speed_accuracy(float &sacc) const {
        return speed_accuracy(primary_instance, sacc);
    }

    bool horizontal_accuracy(uint8_t instance, float &hacc) const {
        if(state[instance].have_horizontal_accuracy) {
            hacc = state[instance].horizontal_accuracy;
            return true;
        }
        return false;
    }

    bool horizontal_accuracy(float &hacc) const {
        return horizontal_accuracy(primary_instance, hacc);
    }

    bool vertical_accuracy(uint8_t instance, float &vacc) const {
        if(state[instance].have_vertical_accuracy) {
            vacc = state[instance].vertical_accuracy;
            return true;
        }
        return false;
    }

    bool vertical_accuracy(float &vacc) const {
        return vertical_accuracy(primary_instance, vacc);
    }

    // 3D velocity in NED format
    const Vector3f &velocity(uint8_t instance) const {
        return state[instance].velocity;
    }
    const Vector3f &velocity() const {
        return velocity(primary_instance);
    }

    // ground speed in m/s
    float ground_speed(uint8_t instance) const {
        return state[instance].ground_speed;
    }
    float ground_speed() const {
        return ground_speed(primary_instance);
    }

    // ground speed in cm/s
    uint32_t ground_speed_cm(void) {
        return ground_speed() * 100;
    }

    // ground course in centidegrees
    float ground_course(uint8_t instance) const {
        return state[instance].ground_course;
    }
    float ground_course() const {
        return ground_course(primary_instance);
    }
    int32_t ground_course_cd(uint8_t instance) const {
        return ground_course(instance) * 100;
    }
    int32_t ground_course_cd() const {
        return ground_course_cd(primary_instance);
    }

    // number of locked satellites
    uint8_t num_sats(uint8_t instance) const {
        return state[instance].num_sats;
    }
    uint8_t num_sats() const {
        return num_sats(primary_instance);
    }

    // GPS time of week in milliseconds
    uint32_t time_week_ms(uint8_t instance) const {
        return state[instance].time_week_ms;
    }
    uint32_t time_week_ms() const {
        return time_week_ms(primary_instance);
    }

    // GPS week
    uint16_t time_week(uint8_t instance) const {
        return state[instance].time_week;
    }
    uint16_t time_week() const {
        return time_week(primary_instance);
    }

    // horizontal dilution of precision
    uint16_t get_hdop(uint8_t instance) const {
        return state[instance].hdop;
    }
    uint16_t get_hdop() const {
        return get_hdop(primary_instance);
    }

    // vertical dilution of precision
    uint16_t get_vdop(uint8_t instance) const {
        return state[instance].vdop;
    }
    uint16_t get_vdop() const {
        return get_vdop(primary_instance);
    }

    // the time we got our last fix in system milliseconds. This is
    // used when calculating how far we might have moved since that fix
    uint32_t last_fix_time_ms(uint8_t instance) const {
        return timing[instance].last_fix_time_ms;
    }
    uint32_t last_fix_time_ms(void) const {
        return last_fix_time_ms(primary_instance);
    }

    // the time we last processed a message in milliseconds. This is
    // used to indicate that we have new GPS data to process
    uint32_t last_message_time_ms(uint8_t instance) const {
        return timing[instance].last_message_time_ms;
    }
    uint32_t last_message_time_ms(void) const {
        return last_message_time_ms(primary_instance);
    }

    // convert GPS week and millis to unix epoch in ms
    static uint64_t time_epoch_convert(uint16_t gps_week, uint32_t gps_ms);
    
    // return last fix time since the 1/1/1970 in microseconds
    uint64_t time_epoch_usec(uint8_t instance);
    uint64_t time_epoch_usec(void) { 
        return time_epoch_usec(primary_instance); 
    }

	// return true if the GPS supports vertical velocity values
    bool have_vertical_velocity(uint8_t instance) const { 
        return state[instance].have_vertical_velocity; 
    }
    bool have_vertical_velocity(void) const { 
        return have_vertical_velocity(primary_instance);
    }

    // the expected lag (in seconds) in the position and velocity readings from the gps
    float get_lag() const { return 0.2f; }

    // set position for HIL
    void setHIL(uint8_t instance, GPS_Status status, uint64_t time_epoch_ms, 
                const Location &location, const Vector3f &velocity, uint8_t num_sats,
                uint16_t hdop);

    // set accuracy for HIL
    void setHIL_Accuracy(uint8_t instance, float vdop, float hacc, float vacc, float sacc, bool _have_vertical_velocity, uint32_t sample_ms);
    
    static const struct AP_Param::GroupInfo var_info[];

    // dataflash for logging, if available
    DataFlash_Class *_DataFlash;

    // configuration parameters
    AP_Int8 _type[GPS_MAX_INSTANCES];
    AP_Int8 _navfilter;
    AP_Int8 _auto_switch;
    AP_Int8 _min_dgps;
    AP_Int16 _sbp_logmask;
    AP_Int8 _inject_to;
    uint32_t _last_instance_swap_ms;
    AP_Int8 _sbas_mode;
    AP_Int8 _min_elevation;
    AP_Int8 _raw_data;
    AP_Int8 _gnss_mode[2];
    AP_Int8 _save_config;
    AP_Int8 _auto_config;
    
    // handle sending of initialisation strings to the GPS
    void send_blob_start(uint8_t instance, const char *_blob, uint16_t size);
    void send_blob_update(uint8_t instance);

    // lock out a GPS port, allowing another application to use the port
    void lock_port(uint8_t instance, bool locked);

    //Inject a packet of raw binary to a GPS
    void inject_data(uint8_t *data, uint8_t len);
    void inject_data(uint8_t instance, uint8_t *data, uint8_t len);

    //MAVLink Status Sending
    void send_mavlink_gps_raw(mavlink_channel_t chan);
    void send_mavlink_gps2_raw(mavlink_channel_t chan);

    void send_mavlink_gps_rtk(mavlink_channel_t chan);
    void send_mavlink_gps2_rtk(mavlink_channel_t chan);

    // Returns the index of the first unconfigured GPS (returns GPS_ALL_CONFIGURED if all instances report as being configured)
    uint8_t first_unconfigured_gps(void) const;
    void broadcast_first_configuration_failure_reason(void) const;

private:
    struct GPS_timing {
        // the time we got our last fix in system milliseconds
        uint32_t last_fix_time_ms;

        // the time we got our last fix in system milliseconds
        uint32_t last_message_time_ms;
    };
    GPS_timing timing[GPS_MAX_INSTANCES];
    GPS_State state[GPS_MAX_INSTANCES];
    AP_GPS_Backend *drivers[GPS_MAX_INSTANCES];
    AP_HAL::UARTDriver *_port[GPS_MAX_INSTANCES];

    /// primary GPS instance
    uint8_t primary_instance:2;

    /// number of GPS instances present
    uint8_t num_instances:2;

    // which ports are locked
    uint8_t locked_ports:2;

    // state of auto-detection process, per instance
    struct detect_state {
        uint32_t detect_started_ms;
        uint32_t last_baud_change_ms;
        uint8_t last_baud;
        struct UBLOX_detect_state ublox_detect_state;
        struct MTK_detect_state mtk_detect_state;
        struct MTK19_detect_state mtk19_detect_state;
        struct SIRF_detect_state sirf_detect_state;
        struct NMEA_detect_state nmea_detect_state;
        struct SBP_detect_state sbp_detect_state;
        struct ERB_detect_state erb_detect_state;
    } detect_state[GPS_MAX_INSTANCES];

    struct {
        const char *blob;
        uint16_t remaining;
    } initblob_state[GPS_MAX_INSTANCES];

    static const uint32_t  _baudrates[];
    static const char _initialisation_blob[];
    static const char _initialisation_raw_blob[];

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);
    void _broadcast_gps_type(const char *type, uint8_t instance, int8_t baud_index);
};

#define GPS_BAUD_TIME_MS 1200
