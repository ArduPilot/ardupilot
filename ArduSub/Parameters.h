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
    static const uint16_t        k_format_version = 1;

    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus
    // ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t        k_software_type = 40;          // 0 for APM
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
        k_param_cli_enabled, // Old (deprecated) command line interface


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
        k_param_NavEKF, // Extended Kalman Filter Inertial Navigation
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


		// Other external hardware interfaces
        k_param_motors = 65, // Motors
        k_param_relay, // Relay
        k_param_camera, // Camera
        k_param_camera_mount, // Camera gimbal


		// RC_Channel settings
        k_param_rc_1 = 75,
        k_param_rc_2,
        k_param_rc_3,
        k_param_rc_4,
        k_param_rc_5,
        k_param_rc_6,
        k_param_rc_7,
        k_param_rc_8,
        k_param_rc_9,
        k_param_rc_10,
        k_param_rc_11,
        k_param_rc_12,
        k_param_rc_13,
        k_param_rc_14,

		// Joystick gain parameters
		k_param_gain_default,
		k_param_maxGain,
		k_param_minGain,
		k_param_numGainSettings,
		k_param_cam_tilt_step,
		k_param_lights_step,
		
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


		// Flight mode selection
        k_param_flight_mode1 = 120,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,


		// PID Controllers
        k_param_p_pos_xy,
        k_param_p_alt_hold,
        k_param_pi_vel_xy,
        k_param_p_vel_z,
        k_param_pid_accel_z,
		k_param_pid_crosstrack_control, // Experimental
		k_param_pid_heading_control, // Experimental


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
        k_param_failsafe_battery_enabled,
        k_param_fs_batt_mah,
        k_param_fs_batt_voltage,


        // Misc Sub settings
        k_param_log_bitmask = 165,
        k_param_arming_check,
        k_param_angle_max,
		k_param_rangefinder_gain,
        k_param_gps_hdop_good,
        k_param_wp_yaw_behavior,
		k_param_xtrack_angle_limit, // Angle limit for crosstrack correction in Auto modes (degrees)
        k_param_pilot_velocity_z_max,
        k_param_pilot_accel_z,
        k_param_compass_enabled,
		k_param_surface_depth,
        k_param_rc_speed, // Main output pwm frequency
        k_param_esc_calibrate, // Boot-time ESC calibration behavior
        k_param_gcs_pid_mask,
        k_param_throttle_filt,
        k_param_throttle_deadzone, // Used in auto-throttle modes
        k_param_disarm_delay,
		k_param_terrain_follow,
        k_param_rc_feel_rp,
		k_param_throttle_gain,
		k_param_cam_tilt_center,
		k_param_frame_configuration,

		// Acro Mode parameters
        k_param_acro_yaw_p = 220, // Used in all modes for get_pilot_desired_yaw_rate
        k_param_acro_trainer,
        k_param_acro_expo,
        k_param_acro_rp_p,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,


		// AUX switch options
        k_param_ch7_option, // Disabled
        k_param_ch8_option, // Disabled
        k_param_ch9_option, // Disabled
        k_param_ch10_option, // Disabled
        k_param_ch11_option, // Disabled
        k_param_ch12_option, // Disabled

		// RPM Sensor
        k_param_rpm_sensor, // Disabled

		// RC_Mapper Library
        k_param_rcmap, // Disabled

		// CH6 Tuning
        k_param_radio_tuning, // Disabled
        k_param_radio_tuning_high, // Disabled
        k_param_radio_tuning_low, // Disabled

		// Autotune parameters
        k_param_autotune_axis_bitmask, // Disabled
        k_param_autotune_aggressiveness, // Disabled
        k_param_autotune_min_d, // Disabled
    };

    AP_Int16        format_version;
    AP_Int8         software_type;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
#if CLI_ENABLED == ENABLED
    AP_Int8         cli_enabled;
#endif

    AP_Float        throttle_filt;

    AP_Float        rangefinder_gain;

    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

    AP_Int8			failsafe_leak;				// leak detection failsafe behavior
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int8			failsafe_pressure;
    AP_Int8			failsafe_temperature;
    AP_Int32		failsafe_pressure_max;
    AP_Int8			failsafe_temperature_max;
    AP_Int8			failsafe_terrain;

    AP_Int8			xtrack_angle_limit;

    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         compass_enabled;

    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp
    
    // Waypoints
    //
    AP_Int16        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
    AP_Int16        pilot_accel_z;               // vertical acceleration the pilot may request

    // Throttle
    //
    AP_Int16        throttle_deadzone;

    // Flight modes
    //
    AP_Int8         flight_mode1;
    AP_Int8         flight_mode2;
    AP_Int8         flight_mode3;
    AP_Int8         flight_mode4;
    AP_Int8         flight_mode5;
    AP_Int8         flight_mode6;

    // Misc
    //
    AP_Int32        log_bitmask;
    AP_Int8         esc_calibrate;
#if CH6_TUNE_ENABLED == ENABLED
    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
#endif
    AP_Int8         ch7_option;
    AP_Int8         ch8_option;
    AP_Int8         ch9_option;
    AP_Int8         ch10_option;
    AP_Int8         ch11_option;
    AP_Int8         ch12_option;
    AP_Int8         arming_check;
    AP_Int8         disarm_delay;

    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

    AP_Int8         terrain_follow;

    // RC channels
    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel_aux          rc_5;
    RC_Channel_aux          rc_6;
    RC_Channel_aux          rc_7;
    RC_Channel_aux          rc_8;
    RC_Channel_aux          rc_9;
    RC_Channel_aux          rc_10;
    RC_Channel_aux          rc_11;
    RC_Channel_aux          rc_12;
    RC_Channel_aux          rc_13;
    RC_Channel_aux          rc_14;

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

	AP_Float				gain_default;
	AP_Float				maxGain;
	AP_Float				minGain;
	AP_Int8					numGainSettings;
	AP_Float				throttle_gain;
	AP_Int16				cam_tilt_center;

	AP_Int16					cam_tilt_step;
	AP_Int16					lights_step;

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
    AC_PI_2D                pi_vel_xy;

    AC_P                    p_vel_z;
    AC_PID                  pid_accel_z;

    AC_P                    p_pos_xy;
    AC_P                    p_alt_hold;



#if TRANSECT_ENABLED == ENABLED
	AC_PID pid_crosstrack_control;
	AC_PID pid_heading_control;
#endif


    // Autotune
#if AUTOTUNE_ENABLED == ENABLED
    AP_Int8                 autotune_axis_bitmask;
    AP_Float                autotune_aggressiveness;
    AP_Float                autotune_min_d;
#endif

    AP_Float				surface_depth;
    AP_Int8					frame_configuration;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters() :

        rc_1                (CH_1),
        rc_2                (CH_2),
        rc_3                (CH_3),
        rc_4                (CH_4),
        rc_5                (CH_5),
        rc_6                (CH_6),
        rc_7                (CH_7),
        rc_8                (CH_8),
        rc_9                (CH_9),
        rc_10               (CH_10),
        rc_11               (CH_11),
        rc_12               (CH_12),
        rc_13               (CH_13),
        rc_14               (CH_14),

        // PID controller	    initial P	      initial I         initial D       initial imax        initial filt hz     pid rate
        //---------------------------------------------------------------------------------------------------------------------------------
        pi_vel_xy               (VEL_XY_P,        VEL_XY_I,                         VEL_XY_IMAX,        VEL_XY_FILT_HZ,     WPNAV_LOITER_UPDATE_TIME),

        p_vel_z                 (VEL_Z_P),
        pid_accel_z             (ACCEL_Z_P,       ACCEL_Z_I,        ACCEL_Z_D,      ACCEL_Z_IMAX,       ACCEL_Z_FILT_HZ,    MAIN_LOOP_SECONDS),

        // P controller	        initial P
        //----------------------------------------------------------------------
        p_pos_xy                (POS_XY_P),

        p_alt_hold              (ALT_HOLD_P)

#if TRANSECT_ENABLED == ENABLED
		,
		pid_crosstrack_control  (XTRACK_P,        XTRACK_I,         XTRACK_D,       XTRACK_IMAX,        XTRACK_FILT_HZ,      XTRACK_DT),
		pid_heading_control     (HEAD_P,		  HEAD_I,           HEAD_D,         HEAD_IMAX,          HEAD_FILT_HZ,       HEAD_DT)
#endif

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

    // altitude at which nav control can start in takeoff
    AP_Float wp_navalt_min;

#if GRIPPER_ENABLED
    AP_Gripper gripper;
#endif

#if PROXIMITY_ENABLED == ENABLED
    // proximity (aka object avoidance) library
    AP_Proximity proximity;
#endif

};

extern const AP_Param::Info        var_info[];

