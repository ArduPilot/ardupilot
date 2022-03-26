#pragma once

#include <AP_Common/AP_Common.h>
#include "RC_Channel.h"

// Global parameter class.
//
class Parameters
{
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t        k_format_version = 1;

    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.  This number is used by
    // AP_Param to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them. When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration. Place these groups in numerical order
    // at the end of the enumeration.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //          AP_Param load/save code depends on the values here to identify
    //          variables saved in EEPROM.
    //
    //
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_ins,                            // libraries/AP_InertialSensor variables
        k_param_NavEKF2,
        k_param_g2, // 2nd block of parameters
        k_param_NavEKF3,
        k_param_can_mgr,
        k_param_osd,

        // simulation
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // BoardConfig object
        k_param_BoardConfig,

        // GPS object
        k_param_gps,

        // Parachute object
        k_param_parachute,

        // Landing gear object
        k_param_landinggear,

        // Input Management object
        k_param_input_manager,

        // Misc
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_poshold_brake_rate,
        k_param_poshold_brake_angle_max,
        k_param_pilot_accel_z,
        k_param_fs_ekf_thresh,
        k_param_terrain,
        k_param_throttle_deadzone,
        k_param_log_bitmask,
        k_param_throttle_filt,
        k_param_throttle_behavior,
        k_param_pilot_takeoff_alt,

        // AP_ADSB Library
        k_param_adsb,
        k_param_notify,

        //PID Controllers
        k_param_pid_vel_xy = 32,
        k_param_pid_vel_z,
        k_param_pid_vel_yaw,
        k_param_pid_pos_xy,
        k_param_pid_pos_z,
        k_param_pid_pos_yaw,

        //Position & Velocity controller params
        k_param_max_vel_xy = 50,
        k_param_max_vel_z,
        k_param_max_vel_yaw,
        k_param_max_pos_xy,
        k_param_max_pos_z,
        k_param_max_pos_yaw,
        k_param_simple_mode,
        k_param_dis_mask,

        //
        // 90: misc2
        //
        k_param_motors = 90,
        k_param_disarm_delay,
        k_param_fs_crash_check,
        k_param_throw_motor_start,
        k_param_rtl_alt_type,
        k_param_avoid,
        k_param_avoidance_adsb,

        // 97: RSSI
        k_param_rssi = 97,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial_manager,
        k_param_gcs3,
        k_param_gcs_pid_mask,    // 126
        k_param_gcs4,
        k_param_gcs5,
        k_param_gcs6,

        //
        // 135 : reserved for Solo until features merged with master
        //
        k_param_rtl_speed_cms = 135,
        k_param_fs_batt_curr_rtl,
        k_param_rtl_cone_slope, // 137

        //
        // 140: Sensor parameters
        //
        k_param_compass,
        k_param_frame_type, //unused
        k_param_ahrs, // AHRS group // 159

        //
        // 160: Navigation parameters
        //
        k_param_rtl_altitude = 160,
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,

        //
        // 170: Radio settings
        //
        k_param_failsafe_throttle = 170,
        k_param_failsafe_throttle_value,
        k_param_radio_tuning, // unused
        k_param_rc_speed = 192,
        k_param_failsafe_gcs,
        k_param_rcmap, // 199

        //
        // 200: flight modes
        //
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_flight_mode_chan,
        k_param_initial_mode,

        //
        // 220: Misc
        //
        k_param_fs_ekf_action = 220,
        k_param_arming,

        k_param_logger = 253, // 253 - Logging Group

        k_param_vehicle = 257, // vehicle common block of parameters

        // the k_param_* space is 9-bits in size
    };

    AP_Int16        format_version;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int8         telem_delay;

    AP_Float        throttle_filt;
    AP_Int16        throttle_behavior;
    AP_Float        pilot_takeoff_alt;

    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    // Throttle
    //
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        throttle_deadzone;

    // Flight modes
    //
    AP_Int8         flight_mode1;
    AP_Int8         flight_mode2;
    AP_Int8         flight_mode3;
    AP_Int8         flight_mode4;
    AP_Int8         flight_mode5;
    AP_Int8         flight_mode6;
    AP_Int8         flight_mode_chan;
    AP_Int8         initial_mode;

    // Misc
    //
    AP_Int32        log_bitmask;
    AP_Int8         disarm_delay;

    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

    AP_Float        max_vel_xy;
    AP_Float        max_vel_z;
    AP_Float        max_vel_yaw;
    AP_Float        max_pos_xy;
    AP_Float        max_pos_z;
    AP_Float        max_pos_yaw;

    AP_Int8         simple_mode;
    AP_Int16        dis_mask;

    AP_Int8         rtl_alt_type;

    AP_Int16        rc_speed; // speed of fast RC Channels in Hz

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters()
    {
    }
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2
{
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // altitude at which nav control can start in takeoff
    AP_Float wp_navalt_min;

#if STATS_ENABLED == ENABLED
    // vehicle statistics
    AP_Stats stats;
#endif

    // whether to enforce acceptance of packets only from sysid_my_gcs
    AP_Int8 sysid_enforce;

    // developer options
    AP_Int32 dev_options;

    // acro exponent parameters
    AP_Float acro_y_expo;

    // frame class
    AP_Int8 frame_class;

    // RC input channels
    RC_Channels_Blimp rc_channels;

    // control over servo output ranges
    SRV_Channels servo_channels;

    // Additional pilot velocity items
    AP_Int16    pilot_speed_dn;

    // Land alt final stage
    AP_Int16 land_alt_low;


#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif // AP_SCRIPTING_ENABLED

    // vibration failsafe enable/disable
    AP_Int8 fs_vibe_enabled;

    // Failsafe options bitmask #36
    AP_Int32 fs_options;

    AP_Float fs_gcs_timeout;
};

extern const AP_Param::Info        var_info[];
