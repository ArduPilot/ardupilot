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

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include "GPS_detect_state.h"
#include <AP_Math/AP_Math.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <SITL/SIM_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define GPS_UNKNOWN_DOP UINT16_MAX // set unknown DOP's to maximum value, which is also correct for MAVLink

// the number of GPS leap seconds - copied into SIM_GPS.cpp
#define GPS_LEAPSECONDS_MILLIS 18000ULL

#define UNIX_OFFSET_MSEC (17000ULL * 86400ULL + 52ULL * 10ULL * AP_MSEC_PER_WEEK - GPS_LEAPSECONDS_MILLIS)

#ifndef GPS_MOVING_BASELINE
#define GPS_MOVING_BASELINE GPS_MAX_RECEIVERS>1
#endif

#if GPS_MOVING_BASELINE
#include "MovingBase.h"
#endif // GPS_MOVING_BASELINE

class AP_GPS_Backend;
class RTCM3_Parser;

/// @class AP_GPS
/// GPS driver main class
class AP_GPS
{
    friend class AP_GPS_ERB;
    friend class AP_GPS_GSOF;
    friend class AP_GPS_MAV;
    friend class AP_GPS_MSP;
    friend class AP_GPS_ExternalAHRS;
    friend class AP_GPS_NMEA;
    friend class AP_GPS_NOVA;
    friend class AP_GPS_PX4;
    friend class AP_GPS_SBF;
    friend class AP_GPS_SBP;
    friend class AP_GPS_SBP2;
    friend class AP_GPS_SIRF;
    friend class AP_GPS_UBLOX;
    friend class AP_GPS_Backend;
    friend class AP_GPS_DroneCAN;

public:
    AP_GPS();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_GPS);

    static AP_GPS *get_singleton() {
        return _singleton;
    }

    // allow threads to lock against GPS update
    HAL_Semaphore &get_semaphore(void) {
        return rsem;
    }
    
    // GPS driver types
    enum GPS_Type {
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_AUTO  = 1,
        GPS_TYPE_UBLOX = 2,
        // GPS_TYPE_MTK   = 3,  // driver removed
        // GPS_TYPE_MTK19 = 4,  // driver removed
        GPS_TYPE_NMEA  = 5,
        GPS_TYPE_SIRF  = 6,
        GPS_TYPE_HIL   = 7,
        GPS_TYPE_SBP   = 8,
        GPS_TYPE_UAVCAN = 9,
        GPS_TYPE_SBF   = 10,
        GPS_TYPE_GSOF  = 11,
        GPS_TYPE_ERB = 13,
        GPS_TYPE_MAV = 14,
        GPS_TYPE_NOVA = 15,
        GPS_TYPE_HEMI = 16, // hemisphere NMEA
        GPS_TYPE_UBLOX_RTK_BASE = 17,
        GPS_TYPE_UBLOX_RTK_ROVER = 18,
        GPS_TYPE_MSP = 19,
        GPS_TYPE_ALLYSTAR = 20, // AllyStar NMEA
        GPS_TYPE_EXTERNAL_AHRS = 21,
        GPS_TYPE_UAVCAN_RTK_BASE = 22,
        GPS_TYPE_UAVCAN_RTK_ROVER = 23,
        GPS_TYPE_UNICORE_NMEA = 24,
        GPS_TYPE_UNICORE_MOVINGBASE_NMEA = 25,
        GPS_TYPE_SBF_DUAL_ANTENNA = 26,
#if HAL_SIM_GPS_ENABLED
        GPS_TYPE_SITL = 100,
#endif
    };

    // convenience methods for working out what general type an instance is:
    bool is_rtk_base(uint8_t instance) const;
    bool is_rtk_rover(uint8_t instance) const;

    // params for an instance:
    class Params {
    public:
        // Constructor
        Params(void);

        AP_Enum<GPS_Type> type;
        AP_Int8 gnss_mode;
        AP_Int16 rate_ms;   // this parameter should always be accessed using get_rate_ms()
        AP_Vector3f antenna_offset;
        AP_Int16 delay_ms;
        AP_Int8  com_port;
#if HAL_ENABLE_DRONECAN_DRIVERS
        AP_Int32 node_id;
        AP_Int32 override_node_id;
#endif
#if GPS_MOVING_BASELINE
        MovingBase mb_params;
#endif // GPS_MOVING_BASELINE

        static const struct AP_Param::GroupInfo var_info[];
    };

    /// GPS status codes.  These are kept aligned with MAVLink by
    /// static_assert in AP_GPS.cpp
    enum GPS_Status {
        NO_GPS = 0,                  ///< No GPS connected/detected
        NO_FIX = 1,                  ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,           ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,           ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4,      ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK_FLOAT = 5, ///< Receiving valid messages and 3D RTK Float
        GPS_OK_FIX_3D_RTK_FIXED = 6, ///< Receiving valid messages and 3D RTK Fixed
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

    // role for auto-config
    enum GPS_Role {
        GPS_ROLE_NORMAL,
        GPS_ROLE_MB_BASE,
        GPS_ROLE_MB_ROVER,
    };

    // GPS Covariance Types matching ROS2 sensor_msgs/msg/NavSatFix
    enum class CovarianceType : uint8_t {
        UNKNOWN = 0,  ///< The GPS does not support any accuracy metrics
        APPROXIMATED = 1,  ///< The accuracy is approximated through metrics such as HDOP/VDOP
        DIAGONAL_KNOWN = 2, ///< The diagonal (east, north, up) components of covariance are reported by the GPS
        KNOWN = 3, ///< The full covariance array is reported by the GPS
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
        float ground_speed;                 ///< ground speed in m/s
        float ground_course;                ///< ground course in degrees, wrapped 0-360
        float gps_yaw;                      ///< GPS derived yaw information, if available (degrees)
        uint32_t gps_yaw_time_ms;           ///< timestamp of last GPS yaw reading
        bool  gps_yaw_configured;           ///< GPS is configured to provide yaw
        uint16_t hdop;                      ///< horizontal dilution of precision in cm
        uint16_t vdop;                      ///< vertical dilution of precision in cm
        uint8_t num_sats;                   ///< Number of visible satellites
        Vector3f velocity;                  ///< 3D velocity in m/s, in NED format
        float speed_accuracy;               ///< 3D velocity RMS accuracy estimate in m/s
        float horizontal_accuracy;          ///< horizontal RMS accuracy estimate in m
        float vertical_accuracy;            ///< vertical RMS accuracy estimate in m
        float gps_yaw_accuracy;           ///< heading accuracy of the GPS in degrees
        bool have_vertical_velocity;      ///< does GPS give vertical velocity? Set to true only once available.
        bool have_speed_accuracy;         ///< does GPS give speed accuracy? Set to true only once available.
        bool have_horizontal_accuracy;    ///< does GPS give horizontal position accuracy? Set to true only once available.
        bool have_vertical_accuracy;      ///< does GPS give vertical position accuracy? Set to true only once available.
        bool have_gps_yaw;                ///< does GPS give yaw? Set to true only once available.
        bool have_gps_yaw_accuracy;       ///< does the GPS give a heading accuracy estimate? Set to true only once available
        float undulation;                   //<height that WGS84 is above AMSL at the current location
        bool have_undulation;               ///<do we have a value for the undulation
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
        bool announced_detection;           ///< true once we have announced GPS has been seen to the user
        uint64_t last_corrected_gps_time_us;///< the system time we got the last corrected GPS timestamp, microseconds
        bool corrected_timestamp_updated;  ///< true if the corrected timestamp has been updated
        uint32_t lagged_sample_count;       ///< number of samples with 50ms more lag than expected

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
        
        // UBX Relative Position and Heading message information
        float relPosHeading;               ///< Reported Heading in degrees
        float relPosLength;                ///< Reported Position horizontal distance in meters
        float relPosD;                     ///< Reported Vertical distance in meters
        float accHeading;                  ///< Reported Heading Accuracy in degrees
        uint32_t relposheading_ts;        ///< True if new data has been received since last time it was false
    };

    /// Startup initialisation.
    void init();

    // ethod for APPPeriph to set the default type for the first GPS instance:
    void set_default_type_for_gps1(uint8_t default_type) {
        params[0].type.set_default(default_type);
    }

    /// Update GPS state based on possible bytes received from the module.
    /// This routine must be called periodically (typically at 10Hz or
    /// more) to process incoming data.
    void update(void);

    // Pass mavlink data to message handlers (for MAV type)
    void handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg);
#if HAL_MSP_GPS_ENABLED
    void handle_msp(const MSP::msp_gps_data_message_t &pkt);
#endif
#if HAL_EXTERNAL_AHRS_ENABLED
    // Retrieve the first instance ID that is configured as type GPS_TYPE_EXTERNAL_AHRS.
    // Can be used by external AHRS systems that only report one GPS to get the instance ID.
    // Returns true if an instance was found, false otherwise.
    bool get_first_external_instance(uint8_t& instance) const WARN_IF_UNUSED;
    void handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt, const uint8_t instance);
#endif

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

    // return a single human-presentable character representing the
    // fix type.  For space-constrained human-readable displays
    char status_onechar(void) const {
        switch (status()) {
        case AP_GPS::NO_GPS:
            return ' ';
        case AP_GPS::NO_FIX:
            return '-';
        case AP_GPS::GPS_OK_FIX_2D:
            return '2';
        case AP_GPS::GPS_OK_FIX_3D:
            return '3';
        case AP_GPS::GPS_OK_FIX_3D_DGPS:
            return '4';
        case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
            return '5';
        case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
            return '6';
        }
        // should never reach here; compiler flags guarantees this.
        return '?';
    }

    // Query the highest status this GPS supports (always reports GPS_OK_FIX_3D for the blended GPS)
    GPS_Status highest_supported_status(uint8_t instance) const WARN_IF_UNUSED;

    // location of last fix
    const Location &location(uint8_t instance) const {
        return state[instance].location;
    }
    const Location &location() const {
        return location(primary_instance);
    }

    // get the difference between WGS84 and AMSL. A positive value means
    // the AMSL height is higher than WGS84 ellipsoid height
    bool get_undulation(uint8_t instance, float &undulation) const;

    // get the difference between WGS84 and AMSL. A positive value means
    // the AMSL height is higher than WGS84 ellipsoid height
    bool get_undulation(float &undulation) const {
        return get_undulation(primary_instance, undulation);
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

    CovarianceType position_covariance(const uint8_t instance, Matrix3f& cov) const WARN_IF_UNUSED;

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
    uint32_t ground_speed_cm(void) const {
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

    // yaw in degrees if available
    bool gps_yaw_deg(uint8_t instance, float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const;
    bool gps_yaw_deg(float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const {
        return gps_yaw_deg(primary_instance, yaw_deg, accuracy_deg, time_ms);
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

    // return true if the GPS currently has yaw available
    bool have_gps_yaw(uint8_t instance) const {
        return !_force_disable_gps_yaw && state[instance].have_gps_yaw;
    }
    bool have_gps_yaw(void) const {
        return have_gps_yaw(primary_instance);
    }

    // return true if the GPS is configured to provide yaw. This will
    // be true if we expect the GPS to provide yaw, even if it
    // currently is not able to provide yaw
    bool have_gps_yaw_configured(uint8_t instance) const {
        return state[instance].gps_yaw_configured;
    }
    
    // the expected lag (in seconds) in the position and velocity readings from the gps
    // return true if the GPS hardware configuration is known or the lag parameter has been set manually
    bool get_lag(uint8_t instance, float &lag_sec) const;
    bool get_lag(float &lag_sec) const {
        return get_lag(primary_instance, lag_sec);
    }

    // return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
    const Vector3f &get_antenna_offset(uint8_t instance) const;

    // lock out a GPS port, allowing another application to use the port
    void lock_port(uint8_t instance, bool locked);

    //MAVLink Status Sending
    void send_mavlink_gps_raw(mavlink_channel_t chan);
    void send_mavlink_gps2_raw(mavlink_channel_t chan);

    void send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst);

    // Returns true if there is an unconfigured GPS, and provides the instance number of the first non configured GPS
    bool first_unconfigured_gps(uint8_t &instance) const WARN_IF_UNUSED;
    void broadcast_first_configuration_failure_reason(void) const;

    // pre-arm check that all GPSs are close to each other.  farthest distance between GPSs (in meters) is returned
    bool all_consistent(float &distance) const;

    // pre-arm check of GPS blending.  False if blending is unhealthy, True if healthy or blending is not being used
    bool blend_health_check() const;

    // handle sending of initialisation strings to the GPS - only used by backends
    void send_blob_start(uint8_t instance);
    void send_blob_start(uint8_t instance, const char *_blob, uint16_t size);
    void send_blob_update(uint8_t instance);

    // return last fix time since the 1/1/1970 in microseconds
    uint64_t time_epoch_usec(uint8_t instance) const;
    uint64_t time_epoch_usec(void) const {
        return time_epoch_usec(primary_instance);
    }

    uint64_t last_message_epoch_usec(uint8_t instance) const;
    uint64_t last_message_epoch_usec() const {
        return last_message_epoch_usec(primary_instance);
    }

    // convert GPS week and millis to unix epoch in ms
    static uint64_t istate_time_to_epoch_ms(uint16_t gps_week, uint32_t gps_ms);

    static const struct AP_Param::GroupInfo var_info[];

#if HAL_LOGGING_ENABLED
    void Write_AP_Logger_Log_Startup_messages();
#endif

    // indicate which bit in LOG_BITMASK indicates gps logging enabled
    void set_log_gps_bit(uint32_t bit) { _log_gps_bit = bit; }

    // report if the gps is healthy (this is defined as existing, an update at a rate greater than 4Hz,
    // as well as any driver specific behaviour)
    bool is_healthy(uint8_t instance) const;
    bool is_healthy(void) const { return is_healthy(primary_instance); }

    // returns true if all GPS instances have passed all final arming checks/state changes
    bool prepare_for_arming(void);

    // returns true if all GPS backend drivers haven't seen any failure
    // this is for backends to be able to spout pre arm error messages
    bool backends_healthy(char failure_msg[], uint16_t failure_msg_len);

    // returns false if any GPS drivers are not performing their logging appropriately
    bool logging_failed(void) const;

    bool logging_present(void) const { return _raw_data != 0; }
    bool logging_enabled(void) const { return _raw_data != 0; }

    // used to disable GPS for GPS failure testing in flight
    void force_disable(bool disable) {
        _force_disable_gps = disable;
    }

    // used to disable GPS yaw for GPS failure testing in flight
    void set_force_disable_yaw(bool disable) {
        _force_disable_gps_yaw = disable;
    }

    // handle possibly fragmented RTCM injection data
    void handle_gps_rtcm_fragment(uint8_t flags, const uint8_t *data, uint8_t len);

    // get configured type by instance
    GPS_Type get_type(uint8_t instance) const {
        return instance>=ARRAY_SIZE(params) ? GPS_Type::GPS_TYPE_NONE : params[instance].type;
    }

    // get iTOW, if supported, zero otherwie
    uint32_t get_itow(uint8_t instance) const;

    bool get_error_codes(uint8_t instance, uint32_t &error_codes) const;
    bool get_error_codes(uint32_t &error_codes) const { return get_error_codes(primary_instance, error_codes); }

    enum class SBAS_Mode : int8_t {
        Disabled = 0,
        Enabled = 1,
        DoNotChange = 2,
    };

#if GPS_MOVING_BASELINE
    // methods used by UAVCAN GPS driver and AP_Periph for moving baseline
    void inject_MBL_data(uint8_t* data, uint16_t length);
    bool get_RelPosHeading(uint32_t &timestamp, float &relPosHeading, float &relPosLength, float &relPosD, float &accHeading) WARN_IF_UNUSED;
    bool get_RTCMV3(const uint8_t *&bytes, uint16_t &len);
    void clear_RTCMV3();
#endif // GPS_MOVING_BASELINE

#if !AP_GPS_BLENDED_ENABLED
    uint8_t get_auto_switch_type() const { return _auto_switch; }
#endif

protected:

    // configuration parameters
    Params params[GPS_MAX_RECEIVERS];
    AP_Int8 _navfilter;
    AP_Int8 _auto_switch;
    AP_Int16 _sbp_logmask;
    AP_Int8 _inject_to;
    uint32_t _last_instance_swap_ms;
    AP_Enum<SBAS_Mode> _sbas_mode;
    AP_Int8 _min_elevation;
    AP_Int8 _raw_data;
    AP_Int8 _save_config;
    AP_Int8 _auto_config;
    AP_Int8 _blend_mask;
    AP_Int16 _driver_options;
    AP_Int8 _primary;

    uint32_t _log_gps_bit = -1;

    enum DriverOptions : int16_t {
        UBX_MBUseUart2    = (1U << 0U),
        SBF_UseBaseForYaw = (1U << 1U),
        UBX_Use115200     = (1U << 2U),
        UAVCAN_MBUseDedicatedBus  = (1 << 3U),
        HeightEllipsoid   = (1U << 4),
        GPSL5HealthOverride = (1U << 5),
        AlwaysRTCMDecode = (1U << 6),
        DisableRTCMDecode = (1U << 7),
    };

    // check if an option is set
    bool option_set(const DriverOptions option) const {
        return (uint8_t(_driver_options.get()) & uint8_t(option)) != 0;
    }

private:
    static AP_GPS *_singleton;
    HAL_Semaphore rsem;

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

        // count of delayed frames
        uint8_t delayed_count;

        // the average time delta
        float average_delta_ms;
    };
    // Note allowance for an additional instance to contain blended data
    GPS_timing timing[GPS_MAX_INSTANCES];
    GPS_State state[GPS_MAX_INSTANCES];
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
        uint32_t probe_baud;
        bool auto_detected_baud;
#if AP_GPS_UBLOX_ENABLED
        struct UBLOX_detect_state ublox_detect_state;
#endif
#if AP_GPS_SIRF_ENABLED
        struct SIRF_detect_state sirf_detect_state;
#endif
#if AP_GPS_NMEA_ENABLED
        struct NMEA_detect_state nmea_detect_state;
#endif
#if AP_GPS_SBP_ENABLED
        struct SBP_detect_state sbp_detect_state;
#endif
#if AP_GPS_SBP2_ENABLED
        struct SBP2_detect_state sbp2_detect_state;
#endif
#if AP_GPS_ERB_ENABLED
        struct ERB_detect_state erb_detect_state;
#endif
    } detect_state[GPS_MAX_RECEIVERS];

    struct {
        const char *blob;
        uint16_t remaining;
    } initblob_state[GPS_MAX_RECEIVERS];

    static const uint32_t  _baudrates[];
    static const char _initialisation_blob[];
    static const char _initialisation_raw_blob[];

    void detect_instance(uint8_t instance);
    // run detection step for one GPS instance. If this finds a GPS then it
    // will return it - otherwise nullptr
    AP_GPS_Backend *_detect_instance(uint8_t instance);

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

    struct {
        uint16_t fragments_used;
        uint16_t fragments_discarded;
    } rtcm_stats;

    // re-assemble GPS_RTCM_DATA message
    void handle_gps_rtcm_data(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_gps_inject(const mavlink_message_t &msg);

    //Inject a packet of raw binary to a GPS
    void inject_data(const uint8_t *data, uint16_t len);
    void inject_data(uint8_t instance, const uint8_t *data, uint16_t len);

#if AP_GPS_BLENDED_ENABLED
    // GPS blending and switching
    Vector3f _blended_antenna_offset; // blended antenna offset
    float _blended_lag_sec; // blended receiver lag in seconds
    float _blend_weights[GPS_MAX_RECEIVERS]; // blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
    bool _output_is_blended; // true when a blended GPS solution being output
    uint8_t _blend_health_counter;  // 0 = perfectly health, 100 = very unhealthy

    // calculate the blend weight.  Returns true if blend could be calculated, false if not
    bool calc_blend_weights(void);

    // calculate the blended state
    void calc_blended_state(void);
#endif

    bool should_log() const;

    bool needs_uart(GPS_Type type) const;

#if GPS_MAX_RECEIVERS > 1
    /// Update primary instance
    void update_primary(void);
#endif

    // helper function for mavlink gps yaw
    uint16_t gps_yaw_cdeg(uint8_t instance) const;

    // Auto configure types
    enum GPS_AUTO_CONFIG {
        GPS_AUTO_CONFIG_DISABLE = 0,
        GPS_AUTO_CONFIG_ENABLE_SERIAL_ONLY  = 1,
        GPS_AUTO_CONFIG_ENABLE_ALL = 2,
    };

    enum class GPSAutoSwitch {
        NONE        = 0,
        USE_BEST    = 1,
        BLEND       = 2,
        //USE_SECOND  = 3, deprecated for new primary param
        USE_PRIMARY_IF_3D_FIX = 4,
    };

    // used for flight testing with GPS loss
    bool _force_disable_gps;

    // used for flight testing with GPS yaw loss
    bool _force_disable_gps_yaw;

    // logging support
    void Write_GPS(uint8_t instance);

#if AP_GPS_RTCM_DECODE_ENABLED
    /*
      per mavlink channel RTCM decoder, enabled with RTCM decode
       option in GPS_DRV_OPTIONS
    */
    struct {
        RTCM3_Parser *parsers[MAVLINK_COMM_NUM_BUFFERS];
        uint32_t sent_crc[32];
        uint8_t sent_idx;
        uint16_t seen_mav_channels;
    } rtcm;
    bool parse_rtcm_injection(mavlink_channel_t chan, const mavlink_gps_rtcm_data_t &pkt);
#endif

    void convert_parameters();
};

namespace AP {
    AP_GPS &gps();
};

#endif  // AP_GPS_ENABLED
