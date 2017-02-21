// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

#include <AP_Common/AP_Common.h>

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
        k_param_NavEKF2,

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

        // GPS object
        k_param_gps,

        // Parachute object
        k_param_parachute,

        // Landing gear object
        k_param_landinggear,    // 18

        // Input Management object
        k_param_input_manager,  // 19

        // Misc
        //
        k_param_log_bitmask_old = 20,           // Deprecated
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                   // deprecated - remove
        k_param_crosstrack_min_distance,	// deprecated - remove with next eeprom number change
        k_param_rssi_pin,                   // unused, replaced by rssi_ library parameters
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_velocity_z_max,
        k_param_circle_rate,                // deprecated - remove
        k_param_sonar_gain,
        k_param_ch8_option,
        k_param_arming_check,
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,
        k_param_angle_rate_max,         // remove
        k_param_rssi_range,             // unused, replaced by rssi_ library parameters
        k_param_rc_feel_rp,
        k_param_NavEKF,                 // Extended Kalman Filter Inertial Navigation Group
        k_param_mission,                // mission library
        k_param_rc_13,
        k_param_rc_14,
        k_param_rally,
        k_param_poshold_brake_rate,
        k_param_poshold_brake_angle_max,
        k_param_pilot_accel_z,
        k_param_serial0_baud,           // deprecated - remove
        k_param_serial1_baud,           // deprecated - remove
        k_param_serial2_baud,           // deprecated - remove
        k_param_land_repositioning,
        k_param_sonar, // sonar object
        k_param_fs_ekf_thresh,
        k_param_terrain,
        k_param_acro_expo,
        k_param_throttle_deadzone,
        k_param_optflow,
        k_param_dcmcheck_thresh,        // deprecated - remove
        k_param_log_bitmask,
        k_param_cli_enabled,
        k_param_throttle_filt,
        k_param_throttle_behavior,
        k_param_pilot_takeoff_alt, // 64

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence,
        k_param_gps_glitch,             // deprecated
        k_param_baro_glitch,            // 71 - deprecated

        // AP_ADSB Library
        k_param_adsb,                   // 72
		k_param_notify, 				// 73

        // 74: precision landing object
        k_param_precland = 74,

        //
        // 75: Singlecopter, CoaxCopter
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
        k_param_heli_pitch_ff,      // remove
        k_param_heli_roll_ff,       // remove
        k_param_heli_yaw_ff,        // remove
        k_param_heli_stab_col_min,  // remove
        k_param_heli_stab_col_max,  // remove
        k_param_heli_servo_rsc,     // 89 = full!

        //
        // 90: misc2
        //
        k_param_motors = 90,
        k_param_disarm_delay,
        k_param_fs_crash_check,
		k_param_throw_motor_start,

        // 97: RSSI
        k_param_rssi = 97,
                
        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100, // deprecated
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,     // 104

        // 110: Telemetry control
        //
        k_param_gcs0 = 110,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_serial1_baud_old, // deprecated
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial2_baud_old, // deprecated
        k_param_serial2_protocol, // deprecated
        k_param_serial_manager,
        k_param_ch9_option,
        k_param_ch10_option,
        k_param_ch11_option,
        k_param_ch12_option,
        k_param_takeoff_trigger_dz,
        k_param_gcs3,
        k_param_gcs_pid_mask,    // 126

        //
        // 135 : reserved for Solo until features merged with master
        //
        k_param_rtl_speed_cms = 135,
        k_param_fs_batt_curr_rtl, // 136

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
        k_param_sonar_enabled_old, // deprecated
        k_param_frame_orientation,
        k_param_optflow_enabled,    // deprecated
        k_param_fs_batt_voltage,
        k_param_ch7_option,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_sonar_type_old,     // deprecated
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
        k_param_camera_mount2,      // deprecated

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
        k_param_throttle_max,           // remove
        k_param_failsafe_throttle,
        k_param_throttle_fs_action,     // remove
        k_param_failsafe_throttle_value,
        k_param_throttle_trim,          // remove
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed = 192,
        k_param_failsafe_battery_enabled,
        k_param_throttle_mid,
        k_param_failsafe_gps_enabled,   // remove
        k_param_rc_9,
        k_param_rc_12,
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
        k_param_simple_modes,

        //
        // 210: Waypoint data
        //
        k_param_waypoint_mode = 210, // remove
        k_param_command_total,       // remove
        k_param_command_index,       // remove
        k_param_command_nav_index,   // remove
        k_param_waypoint_radius,     // remove
        k_param_circle_radius,       // remove
        k_param_waypoint_speed_max,  // remove
        k_param_land_speed,
        k_param_auto_velocity_z_min, // remove
        k_param_auto_velocity_z_max, // remove - 219
        k_param_land_speed_high,

        //
        // 220: PI/D Controllers
        //
        k_param_acro_rp_p = 221,
        k_param_axis_lock_p,    // remove
        k_param_pid_rate_roll,
        k_param_pid_rate_pitch,
        k_param_pid_rate_yaw,
        k_param_p_stabilize_roll,
        k_param_p_stabilize_pitch,
        k_param_p_stabilize_yaw,
        k_param_p_pos_xy,
        k_param_p_loiter_lon,       // remove
        k_param_pid_loiter_rate_lat,    // remove
        k_param_pid_loiter_rate_lon,    // remove
        k_param_pid_nav_lat,        // remove
        k_param_pid_nav_lon,        // remove
        k_param_p_alt_hold,
        k_param_p_vel_z,
        k_param_pid_optflow_roll,       // remove
        k_param_pid_optflow_pitch,      // remove
        k_param_acro_balance_roll_old,  // remove
        k_param_acro_balance_pitch_old, // remove
        k_param_pid_accel_z,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p,
        k_param_autotune_axis_bitmask,
        k_param_autotune_aggressiveness,
        k_param_pi_vel_xy,
        k_param_fs_ekf_action,
        k_param_rtl_climb_min,
        k_param_rpm_sensor,
        k_param_autotune_min_d, // 251
        k_param_pi_precland,    // 252
        k_param_DataFlash = 253, // 253 - Logging Group

        // 254,255: reserved

		//Sub-specific parameters
		k_param_surface_depth = 256,

		// Joystick button mapping parameters
		k_param_jbtn_0 = 261,
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
		k_param_jbtn_15, // 276
    };

    AP_Int16        format_version;
    AP_Int8         software_type;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int8         telem_delay;
#if CLI_ENABLED == ENABLED
    AP_Int8         cli_enabled;
#endif

    AP_Float        throttle_filt;
    AP_Int16        throttle_behavior;
    AP_Int16        takeoff_trigger_dz;
    AP_Float        pilot_takeoff_alt;

    AP_Int16        rtl_altitude;
    AP_Int16        rtl_speed_cms;
    AP_Float        sonar_gain;

    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         compass_enabled;
    AP_Int8         super_simple;
    AP_Int16        rtl_alt_final;
    AP_Int16        rtl_climb_min;              // rtl minimum climb in cm

    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp

    AP_Int16        poshold_brake_rate;         // PosHold flight mode's rotation rate during braking in deg/sec
    AP_Int16        poshold_brake_angle_max;    // PosHold flight mode's max lean angle during braking in centi-degrees
    
    // Waypoints
    //
    AP_Int32        rtl_loiter_time;
    AP_Int16        land_speed;
    AP_Int16        land_speed_high;
    AP_Int16        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
    AP_Int16        pilot_accel_z;               // vertical acceleration the pilot may request

    // Throttle
    //
    AP_Int16        throttle_min;
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        throttle_mid;
    AP_Int16        throttle_deadzone;

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
    AP_Int32        log_bitmask;
    AP_Int8         esc_calibrate;
    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
    AP_Int8         frame_orientation;
    AP_Int8         ch7_option;
    AP_Int8         ch8_option;
    AP_Int8         ch9_option;
    AP_Int8         ch10_option;
    AP_Int8         ch11_option;
    AP_Int8         ch12_option;
    AP_Int8         arming_check;
    AP_Int8         disarm_delay;

    AP_Int8         land_repositioning;
    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

    AP_Int8         throw_motor_start;

#if FRAME_CONFIG ==     SINGLE_FRAME
    // Single
    RC_Channel      single_servo_1, single_servo_2, single_servo_3, single_servo_4;     // servos for four flaps
#endif

#if FRAME_CONFIG ==     COAX_FRAME
    // Coax copter flaps
    RC_Channel      single_servo_1, single_servo_2; // servos for two flaps
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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    RC_Channel_aux          rc_9;
#endif
    RC_Channel_aux          rc_10;
    RC_Channel_aux          rc_11;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    RC_Channel_aux          rc_12;
    RC_Channel_aux          rc_13;
    RC_Channel_aux          rc_14;
#endif

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

    // Joystick button parameters
    JSButton 				jbtn_0;
    JSButton 				jbtn_1;
    JSButton 				jbtn_2;
    JSButton 				jbtn_3;
    JSButton 				jbtn_4;
    JSButton 				jbtn_5;
    JSButton 				jbtn_6;
    JSButton 				jbtn_7;
    JSButton 				jbtn_8;
    JSButton 				jbtn_9;
    JSButton 				jbtn_10;
    JSButton 				jbtn_11;
    JSButton 				jbtn_12;
    JSButton 				jbtn_13;
    JSButton 				jbtn_14;
    JSButton 				jbtn_15;

    // Acro parameters
    AP_Float                acro_rp_p;
    AP_Float                acro_yaw_p;
    AP_Float                acro_balance_roll;
    AP_Float                acro_balance_pitch;
    AP_Int8                 acro_trainer;
    AP_Float                acro_expo;

    // PI/D controllers
    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PI_2D                pi_vel_xy;

    AC_P                    p_vel_z;
    AC_PID                  pid_accel_z;

#if PRECISION_LANDING == ENABLED
    AC_PI_2D                pi_precland;
#endif

    AC_P                    p_pos_xy;
    AC_P                    p_stabilize_roll;
    AC_P                    p_stabilize_pitch;
    AC_P                    p_stabilize_yaw;
    AC_P                    p_alt_hold;

    // Autotune
    AP_Int8                 autotune_axis_bitmask;
    AP_Float                autotune_aggressiveness;
    AP_Float                autotune_min_d;

    AP_Float				surface_depth;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters() :

#if FRAME_CONFIG ==     SINGLE_FRAME
        single_servo_1        (CH_1),
        single_servo_2        (CH_2),
        single_servo_3        (CH_3),
        single_servo_4        (CH_4),
#endif

#if FRAME_CONFIG ==     COAX_FRAME
        single_servo_1        (CH_1),
        single_servo_2        (CH_2),
#endif

        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        rc_9                (CH_9),
#endif
        rc_10               (CH_10),
        rc_11               (CH_11),
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        rc_12               (CH_12),
        rc_13               (CH_13),
        rc_14               (CH_14),
#endif

        // PID controller	    initial P	      initial I         initial D       initial imax        initial filt hz     pid rate
        //---------------------------------------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,     RATE_ROLL_I,      RATE_ROLL_D,    RATE_ROLL_IMAX,     RATE_ROLL_FILT_HZ,  MAIN_LOOP_SECONDS),
        pid_rate_pitch          (RATE_PITCH_P,    RATE_PITCH_I,     RATE_PITCH_D,   RATE_PITCH_IMAX,    RATE_PITCH_FILT_HZ, MAIN_LOOP_SECONDS),
        pid_rate_yaw            (RATE_YAW_P,      RATE_YAW_I,       RATE_YAW_D,     RATE_YAW_IMAX,      RATE_YAW_FILT_HZ,   MAIN_LOOP_SECONDS),

        pi_vel_xy               (VEL_XY_P,        VEL_XY_I,                         VEL_XY_IMAX,        VEL_XY_FILT_HZ,     WPNAV_LOITER_UPDATE_TIME),

        p_vel_z                 (VEL_Z_P),
        pid_accel_z             (ACCEL_Z_P,       ACCEL_Z_I,        ACCEL_Z_D,      ACCEL_Z_IMAX,       ACCEL_Z_FILT_HZ,    MAIN_LOOP_SECONDS),

#if PRECISION_LANDING == ENABLED
        pi_precland             (PRECLAND_P,      PRECLAND_I,                       PRECLAND_IMAX,      VEL_XY_FILT_HZ,     PRECLAND_UPDATE_TIME),
#endif

        // P controller	        initial P
        //----------------------------------------------------------------------
        p_pos_xy                (POS_XY_P),

        p_stabilize_roll        (STABILIZE_ROLL_P),
        p_stabilize_pitch       (STABILIZE_PITCH_P),
        p_stabilize_yaw         (STABILIZE_YAW_P),

        p_alt_hold              (ALT_HOLD_P)
    {
    }
};

extern const AP_Param::Info        var_info[];

