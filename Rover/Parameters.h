#pragma once

#include <AP_Common/AP_Common.h>

#include "RC_Channel.h"
#include <AC_Avoidance/AC_Avoid.h>
#include "AC_Sprayer/AC_Sprayer.h"
#include <AP_AIS/AP_AIS.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Follow/AP_Follow.h>
#include "AP_Gripper/AP_Gripper.h"
#include <AP_Proximity/AP_Proximity.h>
#include "AP_Rally.h"
#include <AP_SmartRTL/AP_SmartRTL.h>
#include <AP_Stats/AP_Stats.h>
#include "AP_Torqeedo/AP_Torqeedo.h"
#include <AP_WindVane/AP_WindVane.h>

// Global parameter class.
//
class Parameters {
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    //
    // The increment will prevent old parameters from being used incorrectly
    // by newer code.
    //
    static const uint16_t k_format_version = 16;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type, // unused
        k_param_can_mgr,

        // Misc
        //
        k_param_log_bitmask_old = 10,  // unused
        k_param_num_resets_old,         // unused
        k_param_reset_switch_chan,
        k_param_initial_mode,
        k_param_scheduler,
        k_param_relay,
        k_param_BoardConfig,
        k_param_pivot_turn_angle_old,   // unused
        k_param_rc_13_old,  // unused
        k_param_rc_14_old,  // unused

        // IO pins
        k_param_rssi_pin = 20,  // unused, replaced by rssi_ library parameters
        k_param_battery_volt_pin,
        k_param_battery_curr_pin,

        k_param_precland = 24,

        // braking
        k_param_braking_percent_old = 30,   // unused
        k_param_braking_speederr_old,       // unused

        // misc2
        k_param_log_bitmask = 40,
        k_param_gps,
        k_param_serial0_baud,   // deprecated, can be deleted
        k_param_serial1_baud,   // deprecated, can be deleted
        k_param_serial2_baud,   // deprecated, can be deleted

        // 97: RSSI
        k_param_rssi = 97,
        k_param_rpm_sensor,     // rpm sensor 98
        
        // 100: Arming parameters
        k_param_arming = 100,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,         // stream rates for uartA
        k_param_gcs1,               // stream rates for uartC
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial0_baud_old,   // unused
        k_param_serial1_baud_old,   // unused
        k_param_telem_delay,
        k_param_skip_gyro_cal,      // unused
        k_param_gcs2,               // stream rates for uartD
        k_param_serial2_baud_old,   // unused
        k_param_serial2_protocol,   // deprecated, can be deleted
        k_param_serial_manager,     // serial manager library
        k_param_cli_enabled_old,    // unused
        k_param_gcs3,
        k_param_gcs_pid_mask,
        k_param_gcs4,
        k_param_gcs5,
        k_param_gcs6,

        //
        // 130: Sensor parameters
        //
        k_param_compass_enabled_deprecated = 130,
        k_param_steering_learn,     // unused
        k_param_NavEKF,             // deprecated - remove
        k_param_mission,            // mission library
        k_param_NavEKF2_old,        // deprecated - remove
        k_param_NavEKF2,
        k_param_g2,                 // 2nd block of parameters
        k_param_NavEKF3,

        // 140: battery controls
        k_param_battery_monitoring = 140,   // deprecated, can be deleted
        k_param_volt_div_ratio,             // deprecated, can be deleted
        k_param_curr_amp_per_volt,          // deprecated, can be deleted
        k_param_input_voltage,              // deprecated, can be deleted
        k_param_pack_capacity,              // deprecated, can be deleted
        k_param_battery,

        //
        // 150: Navigation parameters
        //
        k_param_crosstrack_gain = 150,  // unused
        k_param_crosstrack_entry_angle, // unused
        k_param_speed_cruise,
        k_param_speed_turn_gain,    // unused
        k_param_speed_turn_dist,    // unused
        k_param_ch7_option,         // unused
        k_param_auto_trigger_pin,
        k_param_auto_kickstart,
        k_param_turn_circle,  // unused
        k_param_turn_max_g_old, // unused

        //
        // 160: Radio settings
        //
        k_param_rc_1_old = 160, // unused
        k_param_rc_2_old,       // unused
        k_param_rc_3_old,       // unused
        k_param_rc_4_old,       // unused
        k_param_rc_5_old,       // unused
        k_param_rc_6_old,       // unused
        k_param_rc_7_old,       // unused
        k_param_rc_8_old,       // unused

        // throttle control
        k_param_throttle_min_old = 170, // unused
        k_param_throttle_max_old,       // unused
        k_param_throttle_cruise,
        k_param_throttle_slewrate_old,  // unused
        k_param_throttle_reduction,     // unused
        k_param_pilot_steer_type,
        k_param_skid_steer_out_old, // unused

        // failsafe control
        k_param_fs_action = 180,
        k_param_fs_timeout,
        k_param_fs_throttle_enabled,
        k_param_fs_throttle_value,
        k_param_fs_gcs_enabled,
        k_param_fs_crash_check,
        k_param_fs_ekf_action,
        k_param_fs_ekf_thresh,  // 187

        // obstacle control
        k_param_sonar_enabled = 190,  // deprecated, can be removed
        k_param_sonar_old,            // unused
        k_param_rangefinder_trigger_cm, // unused
        k_param_rangefinder_turn_angle, // unused
        k_param_rangefinder_turn_time,  // unused
        k_param_sonar2_old,           // unused
        k_param_rangefinder_debounce, // unused
        k_param_rangefinder,          // rangefinder object

        //
        // 210: driving modes
        //
        k_param_mode_channel = 210,
        k_param_mode1,
        k_param_mode2,
        k_param_mode3,
        k_param_mode4,
        k_param_mode5,
        k_param_mode6,
        k_param_aux_channel_old,

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,    // unused
        k_param_command_index,          // unused
        k_param_waypoint_radius_old,    // unused
        k_param_waypoint_overshoot_old, // unused

        //
        // camera control
        //
        k_param_camera,
        k_param_camera_mount,
        k_param_camera_mount2,          // unused

        //
        // 230: PID Controllers
        k_param_pidNavSteer = 230,
        k_param_pidServoSteer,  // unused
        k_param_pidSpeedThrottle_old,   // unused

        // high RC channels
        k_param_rc_9_old = 235, // unused
        k_param_rc_10_old,      // unused
        k_param_rc_11_old,      // unused
        k_param_rc_12_old,      // unusedS

        // other objects
        k_param_sitl = 240,
        k_param_ahrs,
        k_param_ins,
        k_param_compass,
        k_param_rcmap,
        k_param_L1_controller,          // unused
        k_param_steerController_old,    // unused
        k_param_barometer,
        k_param_notify,
        k_param_button,
        k_param_osd,

        k_param_logger = 253,  // Logging Group

        // 254,255: reserved

        k_param_vehicle = 257, // vehicle common block of parameters
        };

    AP_Int16    format_version;

    // Misc
    //
    AP_Int32    log_bitmask;
    AP_Int8     reset_switch_chan;
    AP_Int8     initial_mode;

    // Telemetry control
    //
    AP_Int16    sysid_this_mav;
    AP_Int16    sysid_my_gcs;
    AP_Int8     telem_delay;

    // navigation parameters
    //
    AP_Float    speed_cruise;
    AP_Int8     ch7_option;
    AP_Int8     auto_trigger_pin;
    AP_Float    auto_kickstart;
    AP_Int16    gcs_pid_mask;

    // Throttle
    //
    AP_Int8     throttle_cruise;
    AP_Int8     pilot_steer_type;

    // failsafe control
    AP_Int8     fs_action;
    AP_Float    fs_timeout;
    AP_Int8     fs_throttle_enabled;
    AP_Int16    fs_throttle_value;
    AP_Int8     fs_gcs_enabled;
    AP_Int8     fs_crash_check;
    AP_Int8     fs_ekf_action;
    AP_Float    fs_ekf_thresh;

    // driving modes
    //
    AP_Int8     mode_channel;
    AP_Int8     mode1;
    AP_Int8     mode2;
    AP_Int8     mode3;
    AP_Int8     mode4;
    AP_Int8     mode5;
    AP_Int8     mode6;

    Parameters() {}
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

#if STATS_ENABLED == ENABLED
    // vehicle statistics
    AP_Stats stats;
#endif

    // whether to enforce acceptance of packets only from sysid_my_gcs
    AP_Int8 sysid_enforce;

    // RC input channels
    RC_Channels_Rover rc_channels;

    // control over servo output ranges
    SRV_Channels servo_channels;

#if ADVANCED_FAILSAFE == ENABLED
    // advanced failsafe library
    AP_AdvancedFailsafe_Rover afs;
#endif

    AP_Beacon beacon;

    // Motor library
    AP_MotorsUGV motors;

    // wheel encoders
    AP_WheelEncoder wheel_encoder;
    AP_WheelRateControl wheel_rate_control;

    // steering and throttle controller
    AR_AttitudeControl attitude_control;

    // turn radius of vehicle (only used in steering mode)
    AP_Float turn_radius;

    // acro mode turn rate maximum
    AP_Float acro_turn_rate;

    // Safe RTL library
    AP_SmartRTL smart_rtl;

    // default speed for rtl
    AP_Float rtl_speed;

    // frame class for vehicle
    AP_Int8 frame_class;

#if HAL_PROXIMITY_ENABLED
    // proximity library
    AP_Proximity proximity;
#endif

#if MODE_DOCK_ENABLED == ENABLED
    // we need a pointer to the mode for the G2 table
    class ModeDock *mode_dock_ptr;
#endif

    // avoidance library
    AC_Avoid avoid;

    // pitch angle at 100% throttle
    AP_Float bal_pitch_max;

    // pitch/roll angle for crash check
    AP_Int8 crash_angle;

    // follow mode library
    AP_Follow follow;

    // frame type for vehicle (used for vectored motor vehicles and custom motor configs)
    AP_Int8 frame_type;

    // loiter type
    AP_Int8 loit_type;
    AP_Float loit_radius;

#if HAL_SPRAYER_ENABLED
    // Sprayer
    AC_Sprayer sprayer;
#endif

#if AP_GRIPPER_ENABLED
    AP_Gripper gripper;
#endif

    // Rally point library
    AP_Rally_Rover rally;

    // Simple mode types
    AP_Int8 simple_type;

    // windvane
    AP_WindVane windvane;

    // mission behave
    AP_Int8 mis_done_behave;

    // balance both pitch trim
    AP_Float bal_pitch_trim;

    // stick mixing for auto modes
    AP_Int8     stick_mixing;

#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif // AP_SCRIPTING_ENABLED

    // waypoint navigation
    AR_WPNav_OA wp_nav;

    // Sailboat functions
    Sailboat sailboat;

    // object avoidance path planning
    AP_OAPathPlanner oa;

    // maximum speed for vehicle
    AP_Float speed_max;

    // gain for speed of correction in loiter
    AP_Float loiter_speed_gain;

    // FS options
    AP_Int32 fs_options;

#if HAL_TORQEEDO_ENABLED
    // torqeedo motor driver
    AP_Torqeedo torqeedo;
#endif

    // position controller
    AR_PosControl pos_control;

    // guided options bitmask
    AP_Int32 guided_options;

    // Rover options
    AP_Int32 manual_options;
};

extern const AP_Param::Info var_info[];
