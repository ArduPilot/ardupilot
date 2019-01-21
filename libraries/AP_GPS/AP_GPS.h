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
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "GPS_detect_state.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RTC/AP_RTC.h>

/**
   maximum number of GPS instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#define GPS_MAX_RECEIVERS 2 // maximum number of physical GPS sensors allowed - does not include virtual GPS created by blending receiver data
#define GPS_MAX_INSTANCES  (GPS_MAX_RECEIVERS + 1) // maximum number of GPS instances including the 'virtual' GPS created by blending receiver data
#define GPS_BLENDED_INSTANCE GPS_MAX_RECEIVERS  // the virtual blended GPS is always the highest instance (2)
#define GPS_RTK_INJECT_TO_ALL 127
#define GPS_MAX_RATE_MS 200 // maximum value of rate_ms (i.e. slowest update rate) is 5hz or 200ms
#define GPS_UNKNOWN_DOP UINT16_MAX // set unknown DOP's to maximum value, which is also correct for MAVLink
#define GPS_WORST_LAG_SEC 0.22f // worst lag value any GPS driver is expected to return, expressed in seconds
#define GPS_MAX_DELTA_MS 245 // 200 ms (5Hz) + 45 ms buffer

// the number of GPS leap seconds
#define GPS_LEAPSECONDS_MILLIS 18000ULL

#define UNIX_OFFSET_MSEC (17000ULL * 86400ULL + 52ULL * 10ULL * AP_MSEC_PER_WEEK - GPS_LEAPSECONDS_MILLIS)

class AP_GPS_Backend;

/// @class AP_GPS
/// GPS driver main class
class AP_GPS
{
    friend class AP_GPS_ERB;
    friend class AP_GPS_GSOF;
    friend class AP_GPS_MAV;
    friend class AP_GPS_MTK;
    friend class AP_GPS_MTK19;
    friend class AP_GPS_NMEA;
    friend class AP_GPS_NOVA;
    friend class AP_GPS_PX4;
    friend class AP_GPS_SBF;
    friend class AP_GPS_SBP;
    friend class AP_GPS_SBP2;
    friend class AP_GPS_SIRF;
    friend class AP_GPS_UBLOX;
    friend class AP_GPS_Backend;

public:
    AP_GPS();

    /* Do not allow copies */
    AP_GPS(const AP_GPS &other) = delete;
    AP_GPS &operator=(const AP_GPS&) = delete;

    static AP_GPS &gps() {
        return *_singleton;
    }

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
        GPS_TYPE_UAVCAN = 9,
        GPS_TYPE_SBF   = 10,
        GPS_TYPE_GSOF  = 11,
        GPS_TYPE_ERB = 13,
        GPS_TYPE_MAV = 14,
        GPS_TYPE_NOVA = 15
    };

    /// GPS status codes
    enum GPS_Status {
        NO_GPS = GPS_FIX_TYPE_NO_GPS,                     ///< No GPS connected/detected
        NO_FIX = GPS_FIX_TYPE_NO_FIX,                     ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = GPS_FIX_TYPE_2D_FIX,              ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = GPS_FIX_TYPE_3D_FIX,              ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = GPS_FIX_TYPE_DGPS,           ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK_FLOAT = GPS_FIX_TYPE_RTK_FLOAT, ///< Receiving valid messages and 3D RTK Float
        GPS_OK_FIX_3D_RTK_FIXED = GPS_FIX_TYPE_RTK_FIXED, ///< Receiving valid messages and 3D RTK Fixed
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
        Vector3f velocity;                  ///< 3D velocity in m/s, in NED format
        float speed_accuracy;               ///< 3D velocity RMS accuracy estimate in m/s
        float horizontal_accuracy;          ///< horizontal RMS accuracy estimate in m
        float vertical_accuracy;            ///< vertical RMS accuracy estimate in m
        bool have_vertical_velocity;      ///< does GPS give vertical velocity? Set to true only once available.
        bool have_speed_accuracy;         ///< does GPS give speed accuracy? Set to true only once available.
        bool have_horizontal_accuracy;    ///< does GPS give horizontal position accuracy? Set to true only once available.
        bool have_vertical_accuracy;      ///< does GPS give vertical position accuracy? Set to true only once available.
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
        uint32_t uart_timestamp_ms;         ///< optional timestamp from set_uart_timestamp()

        // all the following fields must only all be filled by RTK capable backend drivers
        uint32_t rtk_time_week_ms;         ///< GPS Time of Week of last baseline in milliseconds
        uint16_t rtk_week_number;          ///< GPS Week Number of last baseline
        uint32_t rtk_age_ms;               ///< GPS age of last baseline correction in milliseconds  (0 when no corrections, 0xFFFFFFFF indicates overflow)
        uint8_t  rtk_num_sats;             ///< Current number of satellites used for RTK calculation
        uint8_t  rtk_baseline_coords_type; ///< Coordinate system of baseline. 0 == ECEF, 1 == NED
        int32_t  rtk_baseline_x_mm;        ///< Current baseline in ECEF x or NED north component in mm
        int32_t  rtk_baseline_y_mm;        ///< Current baseline in ECEF y or NED east component in mm
        int32_t  rtk_baseline_z_mm;        ///< Current baseline in ECEF z or NED down component in mm
        uint32_t rtk_accuracy;             ///< Current estimate of 3D baseline accuracy (receiver dependent, typical 0 to 9999)
        int32_t  rtk_iar_num_hypotheses;   ///< Current number of integer ambiguity hypotheses
    };

    /// Startup initialisation.
    void init(const AP_SerialManager& serial_manager);

    /// Update GPS state based on possible bytes received from the module.
    /// This routine must be called periodically (typically at 10Hz or
    /// more) to process incoming data.
    void update(void);

    // Pass mavlink data to message handlers (for MAV type)
    void handle_msg(const mavlink_message_t *msg);

    // Accessor functions

    // return number of active GPS sensors. Note that if the first GPS
    // is not present but the 2nd is then we return 2. Note that a blended
    // GPS solution is treated as an additional sensor.
    uint8_t num_sensors(void) const;

    // Return the index of the primary sensor which is the index of the sensor contributing to
    // the output. A blended solution is available as an additional instance
    uint8_t primary_sensor(void) const {
        return primary_instance;
    }

    /// Query GPS status
    GPS_Status status(uint8_t instance) const {
        if (_force_disable_gps && state[instance].status > NO_FIX) {
            return NO_FIX;
        }
        return state[instance].status;
    }
    GPS_Status status(void) const {
        return status(primary_instance);
    }

    // Query the highest status this GPS supports (always reports GPS_OK_FIX_3D for the blended GPS)
    GPS_Status highest_supported_status(uint8_t instance) const;

    // location of last fix
    const Location &location(uint8_t instance) const {
        return state[instance].location;
    }
    const Location &location() const {
        return location(primary_instance);
    }

    // report speed accuracy
    bool speed_accuracy(uint8_t instance, float &sacc) const;
    bool speed_accuracy(float &sacc) const {
        return speed_accuracy(primary_instance, sacc);
    }

    bool horizontal_accuracy(uint8_t instance, float &hacc) const;
    bool horizontal_accuracy(float &hacc) const {
        return horizontal_accuracy(primary_instance, hacc);
    }

    bool vertical_accuracy(uint8_t instance, float &vacc) const;
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

    // ground course in degrees
    float ground_course(uint8_t instance) const {
        return state[instance].ground_course;
    }
    float ground_course() const {
        return ground_course(primary_instance);
    }
    // ground course in centi-degrees
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

    // system time delta between the last two reported positions
    uint16_t last_message_delta_time_ms(uint8_t instance) const {
        return timing[instance].delta_time_ms;
    }
    uint16_t last_message_delta_time_ms(void) const {
        return last_message_delta_time_ms(primary_instance);
    }

    // return true if the GPS supports vertical velocity values
    bool have_vertical_velocity(uint8_t instance) const {
        return state[instance].have_vertical_velocity;
    }
    bool have_vertical_velocity(void) const {
        return have_vertical_velocity(primary_instance);
    }

    // return number of satellites used for RTK calculation
    uint8_t rtk_num_sats(uint8_t instance) const {
        return state[instance].rtk_num_sats;
    }
    uint8_t rtk_num_sats(void) const {
        return rtk_num_sats(primary_instance);
    }

    // return age of last baseline correction in milliseconds
    uint32_t rtk_age_ms(uint8_t instance) const {
        return state[instance].rtk_age_ms;
    }
    uint32_t rtk_age_ms(void) const {
        return rtk_age_ms(primary_instance);
    }

    // the expected lag (in seconds) in the position and velocity readings from the gps
    // return true if the GPS hardware configuration is known or the lag parameter has been set manually
    bool get_lag(uint8_t instance, float &lag_sec) const;
    bool get_lag(float &lag_sec) const {
        return get_lag(primary_instance, lag_sec);
    }

    // return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
    const Vector3f &get_antenna_offset(uint8_t instance) const;

    // set position for HIL
    void setHIL(uint8_t instance, GPS_Status status, uint64_t time_epoch_ms,
                const Location &location, const Vector3f &velocity, uint8_t num_sats,
                uint16_t hdop);

    // set accuracy for HIL
    void setHIL_Accuracy(uint8_t instance, float vdop, float hacc, float vacc, float sacc, bool _have_vertical_velocity, uint32_t sample_ms);

    // lock out a GPS port, allowing another application to use the port
    void lock_port(uint8_t instance, bool locked);

    //MAVLink Status Sending
    void send_mavlink_gps_raw(mavlink_channel_t chan);
    void send_mavlink_gps2_raw(mavlink_channel_t chan);

    void send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst);

    // Returns the index of the first unconfigured GPS (returns GPS_ALL_CONFIGURED if all instances report as being configured)
    uint8_t first_unconfigured_gps(void) const;
    void broadcast_first_configuration_failure_reason(void) const;

    // return true if all GPS instances have finished configuration
    bool all_configured(void) const {
        return first_unconfigured_gps() == GPS_ALL_CONFIGURED;
    }

    // pre-arm check that all GPSs are close to each other.  farthest distance between GPSs (in meters) is returned
    bool all_consistent(float &distance) const;

    // pre-arm check of GPS blending.  False if blending is unhealthy, True if healthy or blending is not being used
    bool blend_health_check() const;

    // handle sending of initialisation strings to the GPS - only used by backends
    void send_blob_start(uint8_t instance, const char *_blob, uint16_t size);
    void send_blob_update(uint8_t instance);

    // return last fix time since the 1/1/1970 in microseconds
    uint64_t time_epoch_usec(uint8_t instance) const;
    uint64_t time_epoch_usec(void) const {
        return time_epoch_usec(primary_instance);
    }

    // convert GPS week and millis to unix epoch in ms
    static uint64_t time_epoch_convert(uint16_t gps_week, uint32_t gps_ms);

    static const struct AP_Param::GroupInfo var_info[];

    void Write_AP_Logger_Log_Startup_messages();

    // indicate which bit in LOG_BITMASK indicates gps logging enabled
    void set_log_gps_bit(uint32_t bit) { _log_gps_bit = bit; }

    // report if the gps is healthy (this is defined as existing, an update at a rate greater than 4Hz,
    // as well as any driver specific behaviour)
    bool is_healthy(uint8_t instance) const;
    bool is_healthy(void) const { return is_healthy(primary_instance); }

    // returns true if all GPS instances have passed all final arming checks/state changes
    bool prepare_for_arming(void);

    // used to disable GPS for GPS failure testing in flight
    void force_disable(bool disable) {
        _force_disable_gps = disable;
    }

protected:

    // configuration parameters
    AP_Int8 _type[GPS_MAX_RECEIVERS];
    AP_Int8 _navfilter;
    AP_Int8 _auto_switch;
    AP_Int8 _min_dgps;
    AP_Int16 _sbp_logmask;
    AP_Int8 _inject_to;
    uint32_t _last_instance_swap_ms;
    AP_Int8 _sbas_mode;
    AP_Int8 _min_elevation;
    AP_Int8 _raw_data;
    AP_Int8 _gnss_mode[GPS_MAX_RECEIVERS];
    AP_Int16 _rate_ms[GPS_MAX_RECEIVERS];   // this parameter should always be accessed using get_rate_ms()
    AP_Int8 _save_config;
    AP_Int8 _auto_config;
    AP_Vector3f _antenna_offset[GPS_MAX_RECEIVERS];
    AP_Int16 _delay_ms[GPS_MAX_RECEIVERS];
    AP_Int8 _blend_mask;
    AP_Float _blend_tc;

    uint32_t _log_gps_bit = -1;

private:
    static AP_GPS *_singleton;

    // returns the desired gps update rate in milliseconds
    // this does not provide any guarantee that the GPS is updating at the requested
    // rate it is simply a helper for use in the backends for determining what rate
    // they should be configuring the GPS to run at
    uint16_t get_rate_ms(uint8_t instance) const;

    struct GPS_timing {
        // the time we got our last fix in system milliseconds
        uint32_t last_fix_time_ms;

        // the time we got our last message in system milliseconds
        uint32_t last_message_time_ms;

        // delta time between the last pair of GPS updates in system milliseconds
        uint16_t delta_time_ms;
    };
    // Note allowance for an additional instance to contain blended data
    GPS_timing timing[GPS_MAX_RECEIVERS+1];
    GPS_State state[GPS_MAX_RECEIVERS+1];
    AP_GPS_Backend *drivers[GPS_MAX_RECEIVERS];
    AP_HAL::UARTDriver *_port[GPS_MAX_RECEIVERS];

    /// primary GPS instance
    uint8_t primary_instance;

    /// number of GPS instances present
    uint8_t num_instances;

    // which ports are locked
    uint8_t locked_ports;

    // state of auto-detection process, per instance
    struct detect_state {
        uint32_t last_baud_change_ms;
        uint8_t current_baud;
        bool auto_detected_baud;
        struct UBLOX_detect_state ublox_detect_state;
        struct MTK_detect_state mtk_detect_state;
        struct MTK19_detect_state mtk19_detect_state;
        struct SIRF_detect_state sirf_detect_state;
        struct NMEA_detect_state nmea_detect_state;
        struct SBP_detect_state sbp_detect_state;
        struct SBP2_detect_state sbp2_detect_state;
        struct ERB_detect_state erb_detect_state;
    } detect_state[GPS_MAX_RECEIVERS];

    struct {
        const char *blob;
        uint16_t remaining;
    } initblob_state[GPS_MAX_RECEIVERS];

    static const uint32_t  _baudrates[];
    static const char _initialisation_blob[];
    static const char _initialisation_raw_blob[];

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);

    /*
      buffer for re-assembling RTCM data for GPS injection.
      The 8 bit flags field in GPS_RTCM_DATA is interpreted as:
              1 bit for "is fragmented"
              2 bits for fragment number
              5 bits for sequence number

      The rtcm_buffer is allocated on first use. Once a block of data
      is successfully reassembled it is injected into all active GPS
      backends. This assumes we don't want more than 4*180=720 bytes
      in a RTCM data block
     */
    struct rtcm_buffer {
        uint8_t fragments_received;
        uint8_t sequence;
        uint8_t fragment_count;
        uint16_t total_length;
        uint8_t buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4];
    } *rtcm_buffer;

    // re-assemble GPS_RTCM_DATA message
    void handle_gps_rtcm_data(const mavlink_message_t *msg);
    void handle_gps_inject(const mavlink_message_t *msg);

    //Inject a packet of raw binary to a GPS
    void inject_data(uint8_t *data, uint16_t len);
    void inject_data(uint8_t instance, uint8_t *data, uint16_t len);


    // GPS blending and switching
    Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS]; // Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
    float _hgt_offset_cm[GPS_MAX_RECEIVERS]; // Filtered height offset from GPS instance relative to blended solution in _output_state.location (cm)
    Vector3f _blended_antenna_offset; // blended antenna offset
    float _blended_lag_sec = 0.001f * GPS_MAX_RATE_MS; // blended receiver lag in seconds
    float _blend_weights[GPS_MAX_RECEIVERS]; // blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
    uint32_t _last_time_updated[GPS_MAX_RECEIVERS]; // the last value of state.last_gps_time_ms read for that GPS instance - used to detect new data.
    float _omega_lpf; // cutoff frequency in rad/sec of LPF applied to position offsets
    bool _output_is_blended; // true when a blended GPS solution being output
    uint8_t _blend_health_counter;  // 0 = perfectly health, 100 = very unhealthy

    // calculate the blend weight.  Returns true if blend could be calculated, false if not
    bool calc_blend_weights(void);

    // calculate the blended state
    void calc_blended_state(void);

    bool should_df_log() const;

    // Auto configure types
    enum GPS_AUTO_CONFIG {
        GPS_AUTO_CONFIG_DISABLE = 0,
        GPS_AUTO_CONFIG_ENABLE  = 1
    };

    // used for flight testing with GPS loss
    bool _force_disable_gps;
};

namespace AP {
    AP_GPS &gps();
};
