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
    // AP_Param to store and locate parameters in EEPROM.
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
    //			AP_Param load/save code depends on the values here to identify
    //			variables saved in EEPROM.
    //
    //
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables

        // simulation
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // relay object
        k_param_relay,

        // EPM object
        k_param_epm,

        // BoardConfig object
        k_param_BoardConfig,

        // Misc
        //
        k_param_log_bitmask = 20,
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                   // deprecated - remove
        k_param_crosstrack_min_distance,	// deprecated - remove with next eeprom number change
        k_param_rssi_pin,
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_velocity_z_max,
        k_param_circle_rate,
        k_param_sonar_gain,
        k_param_ch8_option,
        k_param_arming_check,
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,
        k_param_angle_rate_max,
        k_param_rssi_range,             // 39

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence,
        k_param_gps_glitch,             // 70

        //
        // 75: Singlecopter
        //
        k_param_single_servo_1 = 75,
        k_param_single_servo_2,
        k_param_single_servo_3,
        k_param_single_servo_4, // 78

        //
        // 80: Heli
        //
        k_param_heli_servo_1 = 80,
        k_param_heli_servo_2,
        k_param_heli_servo_3,
        k_param_heli_servo_4,
        k_param_heli_pitch_ff,
        k_param_heli_roll_ff,
        k_param_heli_yaw_ff,
        k_param_heli_stab_col_min,
        k_param_heli_stab_col_max,  // 88

        //
        // 90: Motors
        //
        k_param_motors = 90,

        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100,
        k_param_wp_nav = 101,

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud,
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial2_baud,

        //
        // 140: Sensor parameters
        //
        k_param_imu = 140, // deprecated - can be deleted
        k_param_battery_monitoring = 141,   // deprecated - can be deleted
        k_param_volt_div_ratio, // deprecated - can be deleted
        k_param_curr_amp_per_volt,  // deprecated - can be deleted
        k_param_input_voltage,  // deprecated - can be deleted
        k_param_pack_capacity,  // deprecated - can be deleted
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled,
        k_param_frame_orientation,
        k_param_optflow_enabled,
        k_param_fs_batt_voltage,
        k_param_ch7_option,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_sonar_type,
        k_param_super_simple = 155,
        k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
        k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
        k_param_ahrs, // AHRS group // 159

        //
        // 160: Navigation parameters
        //
        k_param_rtl_altitude = 160,
        k_param_crosstrack_gain,	// deprecated - remove with next eeprom number change
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,
        k_param_tilt_comp, 	//164	deprecated - remove with next eeprom number change


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,
        k_param_camera_mount2,

        //
        // Batery monitoring parameters
        //
        k_param_battery_volt_pin = 168, // deprecated - can be deleted
        k_param_battery_curr_pin,   // 169 deprecated - can be deleted

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
        k_param_failsafe_throttle,
        k_param_throttle_fs_action,     // remove
        k_param_failsafe_throttle_value,
        k_param_throttle_cruise,
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed = 192,
        k_param_failsafe_battery_enabled,
        k_param_throttle_mid,
        k_param_failsafe_gps_enabled,
        k_param_rc_9,
        k_param_rc_12,
        k_param_failsafe_gcs,           // 198
        k_param_rcmap,

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
        k_param_command_nav_index,   // remove
        k_param_waypoint_radius,     // remove
        k_param_circle_radius,
        k_param_waypoint_speed_max,  // remove
        k_param_land_speed,
        k_param_auto_velocity_z_min, // remove
        k_param_auto_velocity_z_max, // remove - 219

        //
        // 220: PI/D Controllers
        //
        k_param_acro_rp_p = 221,
        k_param_axis_lock_p,    // remove
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
        k_param_pid_nav_lat,        // 233 - remove
        k_param_pid_nav_lon,        // 234 - remove
        k_param_pi_alt_hold,
        k_param_pid_throttle_rate,
        k_param_pid_optflow_roll,
        k_param_pid_optflow_pitch,
        k_param_acro_balance_roll_old,  // 239 - remove
        k_param_acro_balance_pitch_old, // 240 - remove
        k_param_pid_throttle_accel,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p, // 244

        // 254,255: reserved
    };

    AP_Int16        format_version;
    AP_Int8         software_type;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int8         serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    AP_Int8         serial2_baud;
#endif
    AP_Int8         telem_delay;

    AP_Int16        rtl_altitude;
    AP_Int8         sonar_enabled;
    AP_Int8         sonar_type;       // 0 = XL, 1 = LV,
                                      // 2 = XLL (XL with 10m range)
                                      // 3 = HRLV
    AP_Float        sonar_gain;

    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

    AP_Int8         failsafe_gps_enabled;       // gps failsafe enabled
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         compass_enabled;
    AP_Int8         optflow_enabled;
    AP_Int8         super_simple;
    AP_Int16        rtl_alt_final;

    AP_Int8         rssi_pin;
    AP_Float        rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int16        angle_max;                  // maximum lean angle of the copter in centi-degrees
    AP_Int32        angle_rate_max;             // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    
    // Waypoints
    //
    AP_Int8         command_total;
    AP_Int8         command_index;
    AP_Int16        circle_radius;
    AP_Float        circle_rate;                // Circle mode's turn rate in deg/s.  positive to rotate clockwise, negative for counter clockwise
    AP_Int32        rtl_loiter_time;
    AP_Int16        land_speed;
    AP_Int16        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request


    // Throttle
    //
    AP_Int16        throttle_min;
    AP_Int16        throttle_max;
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        throttle_cruise;
    AP_Int16        throttle_mid;

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
    AP_Int8         esc_calibrate;
    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
    AP_Int8         frame_orientation;
    AP_Int8         ch7_option;
    AP_Int8         ch8_option;
    AP_Int8         arming_check;

#if FRAME_CONFIG ==     HELI_FRAME
    // Heli
    RC_Channel      heli_servo_1, heli_servo_2, heli_servo_3, heli_servo_4;     // servos for swash plate and tail
    AP_Float        heli_pitch_ff;												// pitch rate feed-forward
    AP_Float        heli_roll_ff;												// roll rate feed-forward
    AP_Float        heli_yaw_ff;												// yaw rate feed-forward
    AP_Int16        heli_stab_col_min;                                          // min collective while pilot directly controls collective in stabilize mode
    AP_Int16        heli_stab_col_max;                                          // min collective while pilot directly controls collective in stabilize mode
#endif
#if FRAME_CONFIG ==     SINGLE_FRAME
    // Single
    RC_Channel      single_servo_1, single_servo_2, single_servo_3, single_servo_4;     // servos for four flaps
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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux          rc_9;
#endif
    RC_Channel_aux          rc_10;
    RC_Channel_aux          rc_11;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    RC_Channel_aux          rc_12;
#endif

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

    // Acro parameters
    AP_Float                acro_rp_p;
    AP_Float                acro_yaw_p;
    AP_Float                acro_balance_roll;
    AP_Float                acro_balance_pitch;
    AP_Int8                 acro_trainer;

    // PI/D controllers
    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PID                  pid_loiter_rate_lat;
    AC_PID                  pid_loiter_rate_lon;

    AC_PID                  pid_throttle_rate;
    AC_PID                  pid_throttle_accel;
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
#if FRAME_CONFIG ==     SINGLE_FRAME
        single_servo_1        (CH_1),
        single_servo_2        (CH_2),
        single_servo_3        (CH_3),
        single_servo_4        (CH_4),
#endif

        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_9                (CH_9),
#endif
        rc_10               (CH_10),
        rc_11               (CH_11),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        rc_12               (CH_12),
#endif

        // PID controller	initial P	        initial I		    initial D
        //          initial imax
        //-----------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,           RATE_ROLL_I,            RATE_ROLL_D,            RATE_ROLL_IMAX),
        pid_rate_pitch          (RATE_PITCH_P,          RATE_PITCH_I,           RATE_PITCH_D,           RATE_PITCH_IMAX),
        pid_rate_yaw            (RATE_YAW_P,            RATE_YAW_I,             RATE_YAW_D,             RATE_YAW_IMAX),

        pid_loiter_rate_lat     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),
        pid_loiter_rate_lon     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),

        pid_throttle_rate       (THROTTLE_RATE_P,       THROTTLE_RATE_I,        THROTTLE_RATE_D,        THROTTLE_RATE_IMAX),
        pid_throttle_accel      (THROTTLE_ACCEL_P,      THROTTLE_ACCEL_I,       THROTTLE_ACCEL_D,       THROTTLE_ACCEL_IMAX),
        pid_optflow_roll        (OPTFLOW_ROLL_P,        OPTFLOW_ROLL_I,         OPTFLOW_ROLL_D,         OPTFLOW_IMAX),
        pid_optflow_pitch       (OPTFLOW_PITCH_P,       OPTFLOW_PITCH_I,        OPTFLOW_PITCH_D,        OPTFLOW_IMAX),

        // PI controller	initial P			initial I			initial
        // imax
        //----------------------------------------------------------------------
        pi_loiter_lat           (LOITER_P,              LOITER_I,               LOITER_IMAX),
        pi_loiter_lon           (LOITER_P,              LOITER_I,               LOITER_IMAX),

        pi_stabilize_roll       (STABILIZE_ROLL_P,      STABILIZE_ROLL_I,       STABILIZE_ROLL_IMAX),
        pi_stabilize_pitch      (STABILIZE_PITCH_P,     STABILIZE_PITCH_I,      STABILIZE_PITCH_IMAX),
        pi_stabilize_yaw        (STABILIZE_YAW_P,       STABILIZE_YAW_I,        STABILIZE_YAW_IMAX),

        pi_alt_hold             (ALT_HOLD_P,            ALT_HOLD_I,             ALT_HOLD_IMAX)
    {
    }
};

extern const AP_Param::Info        var_info[];

#endif // PARAMETERS_H

