#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "GCS.h"

#include <AP_Airspeed/AP_Airspeed_config.h>
#include <AP_AIS/AP_AIS_config.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_Compass/AP_Compass_config.h>
#include <AP_GPS/AP_GPS_config.h>
#include <AP_Landing/AP_Landing_config.h>
#include <AP_Mount/AP_Mount_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>
#include <AP_RPM/AP_RPM_config.h>
#include <AP_Terrain/AP_Terrain_config.h>
#include <RC_Channel/RC_Channel_config.h>

const struct AP_Param::GroupInfo *GCS::_chan_var_info[MAVLINK_COMM_NUM_BUFFERS];

// tradition has different vehicles use different default stream rates.  This may not be the case forever, but for now we maintain this behaviour:

#ifndef AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS 2
#define AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT 2
#define AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN 2
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL 0
#define AP_MAV_DEFAULT_STREAM_RATE_POSITION 3
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA1 10
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA2 10
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA3 3
#define AP_MAV_DEFAULT_STREAM_RATE_PARAMS 0
#define AP_MAV_DEFAULT_STREAM_RATE_ADSB 0
#elif APM_BUILD_TYPE(APM_BUILD_AntennaTracker) || APM_BUILD_TYPE(APM_BUILD_Rover)
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT 1
#define AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN 1
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL 1
#define AP_MAV_DEFAULT_STREAM_RATE_POSITION 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA1 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA2 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA3 1
#define AP_MAV_DEFAULT_STREAM_RATE_PARAMS 10
#define AP_MAV_DEFAULT_STREAM_RATE_ADSB 0
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT 1
#define AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN 1
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL 1
#define AP_MAV_DEFAULT_STREAM_RATE_POSITION 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA1 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA2 1
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA3 1
#define AP_MAV_DEFAULT_STREAM_RATE_PARAMS 10
#define AP_MAV_DEFAULT_STREAM_RATE_ADSB 5
#elif APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_Blimp) || APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_AP_Periph)
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT 0
#define AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN 0
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL 0
#define AP_MAV_DEFAULT_STREAM_RATE_POSITION 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA1 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA2 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA3 0
#define AP_MAV_DEFAULT_STREAM_RATE_PARAMS 0
#define AP_MAV_DEFAULT_STREAM_RATE_ADSB 0
#elif APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT 0
#define AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN 0
#define AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL 0
#define AP_MAV_DEFAULT_STREAM_RATE_POSITION 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA1 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA2 0
#define AP_MAV_DEFAULT_STREAM_RATE_EXTRA3 0
#define AP_MAV_DEFAULT_STREAM_RATE_PARAMS 0
#define AP_MAV_DEFAULT_STREAM_RATE_ADSB 0
#else
#error Need to set streamrates
#endif  // APM_BUILD_TYPE
#endif  // AP_MAV_DEFAULT_STREAM_RATE

// this must be ordered identically to GCS_MAVLINK::streams!
static const uint8_t default_rates[] {
    AP_MAV_DEFAULT_STREAM_RATE_RAW_SENS,
    AP_MAV_DEFAULT_STREAM_RATE_EXT_STAT,
    AP_MAV_DEFAULT_STREAM_RATE_RC_CHAN,
    AP_MAV_DEFAULT_STREAM_RATE_RAW_CTRL,
    AP_MAV_DEFAULT_STREAM_RATE_POSITION,
    AP_MAV_DEFAULT_STREAM_RATE_EXTRA1,
    AP_MAV_DEFAULT_STREAM_RATE_EXTRA2,
    AP_MAV_DEFAULT_STREAM_RATE_EXTRA3,
    AP_MAV_DEFAULT_STREAM_RATE_PARAMS,
    AP_MAV_DEFAULT_STREAM_RATE_ADSB,
};
static_assert(ARRAY_SIZE(default_rates) == GCS_MAVLINK::NUM_STREAMS, "enough default rates for all streams");

#define DRATE(x) (float(default_rates[x]))

/*
  default stream rates to 1Hz
 */
const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    // @Param: _RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and AIRSPEED
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RAW_SENS", 1, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_RAW_SENSORS], DRATE(GCS_MAVLINK::STREAM_RAW_SENSORS)),

    // @Param: _EXT_STAT
    // @DisplayName: Extended status stream rate
    // @Description: MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXT_STAT", 2, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_EXTENDED_STATUS], DRATE(GCS_MAVLINK::STREAM_EXTENDED_STATUS)),

    // @Param: _RC_CHAN
    // @DisplayName: RC Channel stream rate
    // @Description: MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RC_CHAN",  3, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_RC_CHANNELS], DRATE(2)),

    // @Param: _RAW_CTRL
    // @DisplayName: Raw Control stream rate
    // @Description: MAVLink Raw Control stream rate of SERVO_OUT
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_RAW_CTRL", 4, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_RAW_CONTROLLER], DRATE(GCS_MAVLINK::STREAM_RAW_CONTROLLER)),

    // @Param: _POSITION
    // @DisplayName: Position stream rate
    // @Description: MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_POSITION", 5, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_POSITION], DRATE(GCS_MAVLINK::STREAM_POSITION)),

    // @Param: _EXTRA1
    // @DisplayName: Extra data type 1 stream rate
    // @Description: MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2, RPM, AOA_SSA, LANDING,ESC_TELEMETRY,EFI_STATUS, and PID_TUNING
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA1",   6, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_EXTRA1], DRATE(GCS_MAVLINK::STREAM_EXTRA1)),

    // @Param: _EXTRA2
    // @DisplayName: Extra data type 2 stream rate
    // @Description: MAVLink Stream rate of VFR_HUD
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA2",   7, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_EXTRA2], DRATE(GCS_MAVLINK::STREAM_EXTRA2)),

    // @Param: _EXTRA3
    // @DisplayName: Extra data type 3 stream rate
    // @Description: MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, TERRAIN_REPORT, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, and BATTERY_STATUS
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_EXTRA3",   8, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_EXTRA3], DRATE(GCS_MAVLINK::STREAM_EXTRA3)),

    // @Param: _PARAMS
    // @DisplayName: Parameter stream rate
    // @Description: MAVLink Stream rate of PARAM_VALUE
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_PARAMS",   9, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_PARAMS], DRATE(GCS_MAVLINK::STREAM_PARAMS)),

    // @Param: _ADSB
    // @DisplayName: ADSB stream rate
    // @Description: MAVLink ADSB stream rate
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("_ADSB",   10, GCS_MAVLINK, streamRates[GCS_MAVLINK::STREAM_ADSB], DRATE(GCS_MAVLINK::STREAM_ADSB)),

    // ------------
    // IMPORTANT: Add new stream rates *before* the _OPTIONS parameter.
    // ------------

    // @Param: _OPTIONS
    // @DisplayName: Bitmask for configuring this telemetry channel
    // @Description: Bitmask for configuring this telemetry channel. For having effect on all channels, set the relevant mask in all MAVx_OPTIONS parameters. Keep in mind that part of the flags may require a reboot to take action.
    // @RebootRequired: True
    // @User: Standard
    // @Bitmask: 0:Accept unsigned MAVLink2 messages, 1:Don't forward mavlink to/from, 2:Ignore Streamrate, 3:forward mavlink packets that don't pass CRC
    AP_GROUPINFO("_OPTIONS",   20, GCS_MAVLINK, options, 0),

    // PARAMETER_CONVERSION - Added: May-2025 for ArduPilot-4.7
    // Hidden param used as a flag for param conversion
    // This allows one time conversion while allowing user to flash between versions with and without converted params
    AP_GROUPINFO_FLAGS("_OPTIONSCNV",   21, GCS_MAVLINK, options_were_converted, 0, AP_PARAM_FLAG_HIDDEN),

    AP_GROUPEND
};
#undef DRATE

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
#if AP_AIRSPEED_ENABLED
    MSG_AIRSPEED,
#endif
};

static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
#if HAL_WITH_MCU_MONITORING
    MSG_MCU_STATUS,
#endif
    MSG_MEMINFO,
#if !APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
    MSG_CURRENT_WAYPOINT,
#endif
#if AP_GPS_GPS_RAW_INT_SENDING_ENABLED
    MSG_GPS_RAW,
#endif
#if AP_GPS_GPS_RTK_SENDING_ENABLED
    MSG_GPS_RTK,
#endif
#if AP_GPS_GPS2_RAW_SENDING_ENABLED
    MSG_GPS2_RAW,
#endif
#if AP_GPS_GPS2_RTK_SENDING_ENABLED
    MSG_GPS2_RTK,
#endif
    MSG_NAV_CONTROLLER_OUTPUT,
#if AP_FENCE_ENABLED
    MSG_FENCE_STATUS,
#endif
    MSG_POSITION_TARGET_GLOBAL_INT,
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
    MSG_NAMED_FLOAT
#endif  // APM_BUILD_TYPE(APM_BUILD_ArduSub)
};

static const ap_message STREAM_POSITION_msgs[] = {
#if AP_AHRS_ENABLED
    MSG_LOCATION,
    MSG_LOCAL_POSITION
#endif  // AP_AHRS_ENABLED
};

static const ap_message STREAM_RAW_CONTROLLER_msgs[] = {
#if APM_BUILD_TYPE(APM_BUILD_Rover)
    MSG_SERVO_OUT,
#endif  // #if APM_BUILD_TYPE(APM_BUILD_Rover)
};

static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
#if AP_RC_CHANNEL_ENABLED
    MSG_RC_CHANNELS,
#if AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
#endif
#endif  // AP_RC_CHANNEL_ENABLED
};

static const ap_message STREAM_EXTRA1_msgs[] = {
#if AP_AHRS_ENABLED
    MSG_ATTITUDE,
#endif  // AP_AHRS_ENABLED
#if AP_SIM_ENABLED
    MSG_SIMSTATE,
#endif
#if AP_AHRS_ENABLED
    MSG_AHRS2,
#endif  // AP_AHRS_ENABLED
#if AP_RPM_ENABLED
    MSG_RPM,
#endif
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    MSG_AOA_SSA,
#endif
    MSG_PID_TUNING,
#if HAL_LANDING_DEEPSTALL_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    MSG_LANDING,
#endif  // HAL_LANDING_DEEPSTALL_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#if HAL_WITH_ESC_TELEM
    MSG_ESC_TELEMETRY,
#endif
#if APM_BUILD_TYPE(APM_BUILD_Rover)
    MSG_WHEEL_DISTANCE,
#endif  // APM_BUILD_TYPE(APM_BUILD_Rover)
#if HAL_GENERATOR_ENABLED
    MSG_GENERATOR_STATUS,
#endif
#if AP_WINCH_ENABLED
    MSG_WINCH_STATUS,
#endif
#if HAL_EFI_ENABLED
    MSG_EFI_STATUS,
#endif
#if AP_AIRSPEED_HYGROMETER_ENABLE
    MSG_HYGROMETER,
#endif
};

static const ap_message STREAM_EXTRA2_msgs[] = {
#if !APM_BUILD_TYPE(APM_BUILD_AntennaTracker) && AP_AHRS_ENABLED

    MSG_VFR_HUD
#endif
};

static const ap_message STREAM_EXTRA3_msgs[] = {
#if AP_AHRS_ENABLED
    MSG_AHRS,
#endif  // AP_AHRS_ENABLED
#if AP_MAVLINK_MSG_WIND_ENABLED
    MSG_WIND,
#endif  // AP_MAVLINK_MSG_WIND_ENABLED
#if AP_RANGEFINDER_ENABLED && APM_BUILD_TYPE(APM_BUILD_Rover)
    MSG_WATER_DEPTH,
#endif  // AP_RANGEFINDER_ENABLED && APM_BUILD_TYPE(APM_BUILD_Rover)
    MSG_DISTANCE_SENSOR,
    MSG_SYSTEM_TIME,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN_REPORT,
    MSG_TERRAIN_REQUEST,
#endif
#if AP_BATTERY_ENABLED
    MSG_BATTERY_STATUS,
#endif
#if HAL_MOUNT_ENABLED
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,
#endif
#if AP_OPTICALFLOW_ENABLED
    MSG_OPTICAL_FLOW,
#endif
#if COMPASS_CAL_ENABLED
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
#endif
#if AP_AHRS_ENABLED
    MSG_EKF_STATUS_REPORT,
#endif  // AP_AHRS_ENABLED
#if !APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
    MSG_VIBRATION,
#endif
};

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM,
    MSG_AVAILABLE_MODES
};

static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE,
#if AP_AIS_ENABLED
    MSG_AIS_VESSEL,
#endif
};

// convenience macros for defining which ap_message ids are in which streams:
#define MAV_STREAM_ENTRY(stream_name)           \
    {                                           \
        GCS_MAVLINK::stream_name,               \
        stream_name ## _msgs,                   \
        ARRAY_SIZE(stream_name ## _msgs)        \
    }
#define MAV_STREAM_TERMINATOR { (streams)0, nullptr, 0 }

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RAW_CONTROLLER),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

#endif  // HAL_GCS_ENABLED
