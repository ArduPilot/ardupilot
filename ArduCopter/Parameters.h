// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <AP_Common.h>

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
    static const uint16_t        k_format_version = 120;

    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus
    // ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t        k_software_type = 10;          // 0 for APM
                                                                // trunk

    // Parameter identities.
    //
    // The enumeration defined here is used to ensure that every parameter
    // or parameter group has a unique ID number.	This number is used by
    // AP_Var to store and locate parameters in EEPROM.
    //
    // Note that entries without a number are assigned the next number after
    // the entry preceding them.	When adding new entries, ensure that they
    // don't overlap.
    //
    // Try to group related variables together, and assign them a set
    // range in the enumeration.	Place these groups in numerical order
    // at the end of the enumeration.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //			AP_Var load/save code depends on the values here to identify
    //			variables saved in EEPROM.
    //
    //
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_ins,

        // simulation
        k_param_sitl = 10,

        // Misc
        //
        k_param_log_bitmask = 20,
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                           // THOR The memory
                                                        // location for the
                                                        // Yaw Rate 1 = fast,
                                                        // 2 = med, 3 = slow

        // 65: AP_Limits Library
        k_param_limits = 65,
        k_param_gpslock_limit,
        k_param_geofence_limit,
        k_param_altitude_limit,

        //
        // 80: Heli
        //
        k_param_heli_servo_1 = 80,
        k_param_heli_servo_2,
        k_param_heli_servo_3,
        k_param_heli_servo_4,

        //
        // 90: Motors
        //
        k_param_motors = 90,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs3,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial3_baud,
        k_param_telem_delay,

        //
        // 140: Sensor parameters
        //
        k_param_imu = 140, // sensor calibration
        k_param_battery_monitoring,
        k_param_volt_div_ratio,
        k_param_curr_amp_per_volt,
        k_param_input_voltage,
        k_param_pack_capacity,
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled,
        k_param_frame_orientation,
        k_param_optflow_enabled,
        k_param_low_voltage,
        k_param_ch7_option,
        k_param_auto_slew_rate,
        k_param_sonar_type,
        k_param_super_simple = 155,
        k_param_axis_enabled = 157,
        k_param_copter_leds_mode,
        k_param_ahrs, // AHRS group

        //
        // 160: Navigation parameters
        //
        k_param_RTL_altitude = 160,
        k_param_crosstrack_gain,
        k_param_auto_land_timeout,
        k_param_rtl_approach_alt,
        k_param_tilt_comp, //164


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,
        k_param_camera_mount2,

        //
        // Batery monitoring parameters
        //
        k_param_battery_volt_pin = 168,
        k_param_battery_curr_pin,   // 169

        //
        // 170: Radio settings
        //
        k_param_rc_1 = 170,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_10,
        k_param_rc_11,
        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_action,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed = 192,

        //
        // 200: flight modes
        //
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_simple_modes,

        //
        // 210: Waypoint data
        //
        k_param_waypoint_mode = 210, // remove
        k_param_command_total,
        k_param_command_index,
        k_param_command_nav_index,
        k_param_waypoint_radius,
        k_param_loiter_radius,
        k_param_waypoint_speed_max,

        //
        // 220: PI/D Controllers
        //
        k_param_stabilize_d_schedule = 219,
        k_param_stabilize_d = 220,
        k_param_acro_p,
        k_param_axis_lock_p,
        k_param_pid_rate_roll,
        k_param_pid_rate_pitch,
        k_param_pid_rate_yaw,
        k_param_pi_stabilize_roll,
        k_param_pi_stabilize_pitch,
        k_param_pi_stabilize_yaw,
        k_param_pi_loiter_lat,
        k_param_pi_loiter_lon,
        k_param_pid_loiter_rate_lat,
        k_param_pid_loiter_rate_lon,
        k_param_pid_nav_lat,
        k_param_pid_nav_lon,
        k_param_pi_alt_hold,
        k_param_pid_throttle,
        k_param_pid_optflow_roll,
        k_param_pid_optflow_pitch,

        // 254,255: reserved
    };

    AP_Int16        format_version;
    AP_Int8         software_type;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int8         serial3_baud;
    AP_Int8         telem_delay;

    AP_Int16        RTL_altitude;
    AP_Int8         sonar_enabled;
    AP_Int8         sonar_type;       // 0 = XL, 1 = LV,
                                      // 2 = XLL (XL with 10m range)
                                      // 3 = HRLV
    AP_Int8         battery_monitoring;         // 0=disabled, 3=voltage only,
                                                // 4=voltage and current
    AP_Float        volt_div_ratio;
    AP_Float        curr_amp_per_volt;
    AP_Float        input_voltage;
    AP_Int16        pack_capacity;              // Battery pack capacity less
                                                // reserve
    AP_Int8         compass_enabled;
    AP_Int8         optflow_enabled;
    AP_Float        low_voltage;
    AP_Int8         super_simple;
    AP_Int16        rtl_approach_alt;
    AP_Int8         tilt_comp;
    AP_Int8         axis_enabled;
    AP_Int8         copter_leds_mode;           // Operating mode of LED
                                                // lighting system

    AP_Int8         battery_volt_pin;
    AP_Int8         battery_curr_pin;

    // Waypoints
    //
    AP_Int8         waypoint_mode;
    AP_Int8         command_total;
    AP_Int8         command_index;
    AP_Int8         command_nav_index;
    AP_Int16        waypoint_radius;
    AP_Int16        loiter_radius;
    AP_Int16        waypoint_speed_max;
    AP_Float        crosstrack_gain;
    AP_Int32        auto_land_timeout;


    // Throttle
    //
    AP_Int16        throttle_min;
    AP_Int16        throttle_max;
    AP_Int8         throttle_fs_enabled;
    AP_Int8         throttle_fs_action;
    AP_Int16        throttle_fs_value;
    AP_Int16        throttle_cruise;

    // Flight modes
    //
    AP_Int8         flight_mode1;
    AP_Int8         flight_mode2;
    AP_Int8         flight_mode3;
    AP_Int8         flight_mode4;
    AP_Int8         flight_mode5;
    AP_Int8         flight_mode6;
    AP_Int8         simple_modes;

    // Misc
    //
    AP_Int16        log_bitmask;
    AP_Int16        log_last_filenumber;        // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
    AP_Int8         toy_yaw_rate;                               // THOR The
                                                                // Yaw Rate 1
                                                                // = fast, 2 =
                                                                // med, 3 =
                                                                // slow
    AP_Int8         esc_calibrate;
    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
    AP_Int8         frame_orientation;
    AP_Int8         ch7_option;
    AP_Int16        auto_slew_rate;

#if FRAME_CONFIG ==     HELI_FRAME
    // Heli
    RC_Channel        heli_servo_1, heli_servo_2, heli_servo_3, heli_servo_4;   //
                                                                                // servos
                                                                                // for
                                                                                // swash
                                                                                // plate
                                                                                // and
                                                                                // tail
#endif

    // Camera
#if CAMERA == ENABLED
    AP_Camera        camera;
#endif

    // RC channels
    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel_aux          rc_5;
    RC_Channel_aux          rc_6;
    RC_Channel_aux          rc_7;
    RC_Channel_aux          rc_8;

#if MOUNT == ENABLED
    RC_Channel_aux          rc_10;
    RC_Channel_aux          rc_11;
#endif
    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

    AP_Float                stabilize_d;
    AP_Float                stabilize_d_schedule;

    // PI/D controllers
    AP_Float                acro_p;
    AP_Float                axis_lock_p;

    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PID                  pid_loiter_rate_lat;
    AC_PID                  pid_loiter_rate_lon;
    AC_PID                  pid_nav_lat;
    AC_PID                  pid_nav_lon;

    AC_PID                  pid_throttle;
    AC_PID                  pid_optflow_roll;
    AC_PID                  pid_optflow_pitch;

    APM_PI                  pi_loiter_lat;
    APM_PI                  pi_loiter_lon;
    APM_PI                  pi_stabilize_roll;
    APM_PI                  pi_stabilize_pitch;
    APM_PI                  pi_stabilize_yaw;
    APM_PI                  pi_alt_hold;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters() :

#if FRAME_CONFIG ==     HELI_FRAME
        heli_servo_1        (CH_1),
        heli_servo_2        (CH_2),
        heli_servo_3        (CH_3),
        heli_servo_4        (CH_4),
#endif

        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8),
#if MOUNT == ENABLED
        rc_10               (CH_10),
        rc_11               (CH_11),
#endif

        // PID controller	initial P	        initial I		    initial D
        //          initial imax
        //-----------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,           RATE_ROLL_I,            RATE_ROLL_D,            RATE_ROLL_IMAX * 100),
        pid_rate_pitch          (RATE_PITCH_P,          RATE_PITCH_I,           RATE_PITCH_D,           RATE_PITCH_IMAX * 100),
        pid_rate_yaw            (RATE_YAW_P,            RATE_YAW_I,             RATE_YAW_D,             RATE_YAW_IMAX * 100),

        pid_loiter_rate_lat     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX * 100),
        pid_loiter_rate_lon     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX * 100),

        pid_nav_lat             (NAV_P,                 NAV_I,                  NAV_D,                  NAV_IMAX * 100),
        pid_nav_lon             (NAV_P,                 NAV_I,                  NAV_D,                  NAV_IMAX * 100),

        pid_throttle            (THROTTLE_P,            THROTTLE_I,             THROTTLE_D,             THROTTLE_IMAX),
        pid_optflow_roll        (OPTFLOW_ROLL_P,        OPTFLOW_ROLL_I,         OPTFLOW_ROLL_D,         OPTFLOW_IMAX * 100),
        pid_optflow_pitch       (OPTFLOW_PITCH_P,       OPTFLOW_PITCH_I,        OPTFLOW_PITCH_D,        OPTFLOW_IMAX * 100),

        // PI controller	initial P			initial I			initial
        // imax
        //----------------------------------------------------------------------
        pi_loiter_lat           (LOITER_P,              LOITER_I,               LOITER_IMAX * 100),
        pi_loiter_lon           (LOITER_P,              LOITER_I,               LOITER_IMAX * 100),

        pi_stabilize_roll       (STABILIZE_ROLL_P,      STABILIZE_ROLL_I,       STABILIZE_ROLL_IMAX * 100),
        pi_stabilize_pitch      (STABILIZE_PITCH_P,     STABILIZE_PITCH_I,      STABILIZE_PITCH_IMAX * 100),
        pi_stabilize_yaw        (STABILIZE_YAW_P,       STABILIZE_YAW_I,        STABILIZE_YAW_IMAX * 100),

        pi_alt_hold             (ALT_HOLD_P,            ALT_HOLD_I,             ALT_HOLD_IMAX)
    {
    }
};

extern const AP_Param::Info        var_info[];

#endif // PARAMETERS_H

