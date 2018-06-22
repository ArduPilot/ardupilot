#pragma once

#include <AP_Common/AP_Common.h>

#include <AP_Gripper/AP_Gripper.h>

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
    static const uint16_t        k_format_version = 1;

    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.   This number is used by
    // AP_Param to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them.    When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration.    Place these groups in numerical order
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
        k_param_software_type, // unusued

        k_param_g2, // 2nd block of parameters

        k_param_sitl, // Simulation

        // Telemetry
        k_param_gcs0 = 10,
        k_param_gcs1,
        k_param_gcs2,
        k_param_gcs3,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,

        // Hardware/Software configuration
        k_param_BoardConfig = 20, // Board configuration (PX4/Linux/etc)
        k_param_scheduler, // Scheduler (for debugging/perf_info)
        k_param_DataFlash, // DataFlash Logging
        k_param_serial_manager, // Serial ports, AP_SerialManager
        k_param_notify, // Notify Library, AP_Notify
        k_param_arming = 26, // Arming checks
        k_param_BoardConfig_CAN,

        // Sensor objects
        k_param_ins = 30, // AP_InertialSensor
        k_param_compass, // Compass
        k_param_barometer, // Barometer/Depth Sensor
        k_param_battery, // AP_BattMonitor
        k_param_leak_detector, // Leak Detector
        k_param_rangefinder, // Rangefinder
        k_param_gps, // GPS
        k_param_optflow, // Optical Flow


        // Navigation libraries
        k_param_ahrs = 50, // AHRS
        k_param_NavEKF, // Extended Kalman Filter Inertial Navigation             // remove
        k_param_NavEKF2, // EKF2
        k_param_attitude_control, // Attitude Control
        k_param_pos_control, // Position Control
        k_param_wp_nav, // Waypoint navigation
        k_param_mission, // Mission library
        k_param_fence, // Fence Library
        k_param_terrain, // Terrain database
        k_param_rally, // Disabled
        k_param_circle_nav, // Disabled
        k_param_avoid, // Relies on proximity and fence
        k_param_NavEKF3,
        k_param_loiter_nav,


        // Other external hardware interfaces
        k_param_motors = 65, // Motors
        k_param_relay, // Relay
        k_param_camera, // Camera
        k_param_camera_mount, // Camera gimbal


        // RC_Channel settings (deprecated)
        k_param_rc_1_old = 75,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_9_old,
        k_param_rc_10_old,
        k_param_rc_11_old,
        k_param_rc_12_old,
        k_param_rc_13_old,
        k_param_rc_14_old,

        // Joystick gain parameters
        k_param_gain_default,
        k_param_maxGain,
        k_param_minGain,
        k_param_numGainSettings,
        k_param_cam_tilt_step, // deprecated
        k_param_lights_step, // deprecated

        // Joystick button mapping parameters
        k_param_jbtn_0 = 95,
        k_param_jbtn_1,
        k_param_jbtn_2,
        k_param_jbtn_3,
        k_param_jbtn_4,
        k_param_jbtn_5,
        k_param_jbtn_6,
        k_param_jbtn_7,
        k_param_jbtn_8,
        k_param_jbtn_9,
        k_param_jbtn_10,
        k_param_jbtn_11,
        k_param_jbtn_12,
        k_param_jbtn_13,
        k_param_jbtn_14,
        k_param_jbtn_15,


        // PID Controllers
        k_param_p_pos_xy = 126, // deprecated
        k_param_p_alt_hold, // deprecated
        k_param_pi_vel_xy, // deprecated
        k_param_p_vel_z, // deprecated
        k_param_pid_accel_z, // deprecated


        // Failsafes
        k_param_failsafe_gcs = 140,
        k_param_failsafe_leak, // leak failsafe behavior
        k_param_failsafe_pressure, // internal pressure failsafe behavior
        k_param_failsafe_pressure_max, // maximum internal pressure in pascal before failsafe is triggered
        k_param_failsafe_temperature, // internal temperature failsafe behavior
        k_param_failsafe_temperature_max, // maximum internal temperature in degrees C before failsafe is triggered
        k_param_failsafe_terrain, // terrain failsafe behavior
        k_param_fs_ekf_thresh,
        k_param_fs_ekf_action,
        k_param_fs_crash_check,
        k_param_failsafe_battery_enabled, // unused - moved to AP_BattMonitor
        k_param_fs_batt_mah,              // unused - moved to AP_BattMonitor
        k_param_fs_batt_voltage,          // unused - moved to AP_BattMonitor
        k_param_failsafe_pilot_input,
        k_param_failsafe_pilot_input_timeout,


        // Misc Sub settings
        k_param_log_bitmask = 165,
        k_param_angle_max = 167,
        k_param_rangefinder_gain,
        k_param_wp_yaw_behavior = 170,
        k_param_xtrack_angle_limit, // Angle limit for crosstrack correction in Auto modes (degrees)
        k_param_pilot_speed_up,     // renamed from k_param_pilot_velocity_z_max
        k_param_pilot_accel_z,
        k_param_compass_enabled,
        k_param_surface_depth,
        k_param_rc_speed, // Main output pwm frequency
        k_param_gcs_pid_mask = 178,
        k_param_throttle_filt,
        k_param_throttle_deadzone, // Used in auto-throttle modes
        k_param_terrain_follow = 182,
        k_param_rc_feel_rp,
        k_param_throttle_gain,
        k_param_cam_tilt_center, // deprecated
        k_param_frame_configuration,

        // Acro Mode parameters
        k_param_acro_yaw_p = 220, // Used in all modes for get_pilot_desired_yaw_rate
        k_param_acro_trainer,
        k_param_acro_expo,
        k_param_acro_rp_p,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,

        // RPM Sensor
        k_param_rpm_sensor = 232, // Disabled

        // RC_Mapper Library
        k_param_rcmap, // Disabled

        k_param_cam_slew_limit = 237, // deprecated
        k_param_lights_steps,
        k_param_pilot_speed_dn,

    };

    AP_Int16        format_version;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;

    AP_Float        throttle_filt;

#if RANGEFINDER_ENABLED == ENABLED
    AP_Float        rangefinder_gain;
#endif

    AP_Int8         failsafe_leak;              // leak detection failsafe behavior
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int8         failsafe_pressure;
    AP_Int8         failsafe_temperature;
    AP_Int32        failsafe_pressure_max;
    AP_Int8         failsafe_temperature_max;
    AP_Int8         failsafe_terrain;
    AP_Int8         failsafe_pilot_input;       // pilot input failsafe behavior
    AP_Float        failsafe_pilot_input_timeout;

    AP_Int8         xtrack_angle_limit;

    AP_Int8         compass_enabled;

    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    // Waypoints
    //
    AP_Int16        pilot_speed_up;        // maximum vertical ascending velocity the pilot may request
    AP_Int16        pilot_speed_dn;        // maximum vertical descending velocity the pilot may request
    AP_Int16        pilot_accel_z;               // vertical acceleration the pilot may request

    // Throttle
    //
    AP_Int16        throttle_deadzone;

    // Misc
    //
    AP_Int32        log_bitmask;

    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    AP_Int8         terrain_follow;
#endif

    AP_Int16        rc_speed; // speed of fast RC Channels in Hz

    AP_Float        gain_default;
    AP_Float        maxGain;
    AP_Float        minGain;
    AP_Int8         numGainSettings;
    AP_Float        throttle_gain;

    AP_Int16        lights_steps;

    // Joystick button parameters
    JSButton        jbtn_0;
    JSButton        jbtn_1;
    JSButton        jbtn_2;
    JSButton        jbtn_3;
    JSButton        jbtn_4;
    JSButton        jbtn_5;
    JSButton        jbtn_6;
    JSButton        jbtn_7;
    JSButton        jbtn_8;
    JSButton        jbtn_9;
    JSButton        jbtn_10;
    JSButton        jbtn_11;
    JSButton        jbtn_12;
    JSButton        jbtn_13;
    JSButton        jbtn_14;
    JSButton        jbtn_15;

    // Acro parameters
    AP_Float        acro_rp_p;
    AP_Float        acro_yaw_p;
    AP_Float        acro_balance_roll;
    AP_Float        acro_balance_pitch;
    AP_Int8         acro_trainer;
    AP_Float        acro_expo;

    AP_Float                surface_depth;
    AP_Int8                 frame_configuration;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters()
    {
    }
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
*/
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

#if GRIPPER_ENABLED
    AP_Gripper gripper;
#endif

#if PROXIMITY_ENABLED == ENABLED
    // proximity (aka object avoidance) library
    AP_Proximity proximity;
#endif

    // RC input channels
    RC_Channels rc_channels;

    // control over servo output ranges
    SRV_Channels servo_channels;
};

extern const AP_Param::Info        var_info[];

