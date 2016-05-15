// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    static const uint16_t        k_format_version = 4;

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
        k_param_ins,                            // libraries/AP_InertialSensor variables

        // simulation
        k_param_sitl = 5,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // BoardConfig object
        k_param_BoardConfig,

        // GPS object
        k_param_gps = 10,

        k_param_rssi_pin,
        k_param_pilot_velocity_z_max,
        k_param_arming_check,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,
        k_param_rssi_range,
        k_param_rc_feel_rp,
        k_param_NavEKF,                 // Extended Kalman Filter Inertial Navigation Group
        k_param_mission,                // mission library
        k_param_rc_13,
        k_param_rc_14,
        k_param_pilot_accel_z,
        k_param_serial0_baud,
        k_param_serial1_baud,
        k_param_serial2_baud,
        //k_param_land_repositioning,
        k_param_ekfcheck_thresh,
        k_param_acro_expo,
        k_param_throttle_deadzone,
        k_param_dcmcheck_thresh,        // 41
        k_param_log_bitmask,

        // 50: AP_Limits Library
        k_param_gps_glitch = 45,
        k_param_baro_glitch, //53

        //
        // 55: Motors
        //
        k_param_motors = 50,

        //
        // 56: Inertial Nav
        //
        k_param_inertial_nav = 55,
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,     // 59

        // 62: Telemetry control
        //
        k_param_gcs0 = 60,
        k_param_gcs1,
        k_param_sysid_this_mav,
        k_param_sysid_my_gcs,
        k_param_telem_delay,
        k_param_gcs2,
        k_param_serial2_protocol, //67

        //
        // 70: Sensor parameters
        //
        k_param_compass_enabled = 70,
        k_param_compass,
        k_param_frame_orientation,
        k_param_fs_batt_voltage,
        k_param_ahrs, // AHRS group // 77

        //
        // 80: Navigation parameters
        //
        k_param_rtl_altitude = 80,


        //
        // Camera and mount parameters
        //
        k_param_camera = 84,
        k_param_camera_mount = 85,

        //
        // 86: Radio settings
        //
        k_param_rc_1 = 86,
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
        k_param_failsafe_throttle_value,
        k_param_throttle_cruise,
        k_param_esc_calibrate,
        k_param_rc_speed,
        k_param_failsafe_battery_enabled,
        k_param_throttle_mid,
        k_param_failsafe_gps_enabled,
        k_param_rc_9,
        k_param_rc_12,
        k_param_failsafe_gcs,

        //
        // 115: flight modes
        //
        k_param_flight_mode1 = 115,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,

        //
        // 125: Waypoint data
        //
        k_param_land_speed = 125,

        //
        // 130: PI/D Controllers
        //
        k_param_acro_rp_p = 126,
        k_param_pid_rate_roll,
        k_param_pid_rate_pitch,
        k_param_pid_rate_yaw,
        k_param_p_stabilize_roll,
        k_param_p_stabilize_pitch,
        k_param_p_stabilize_yaw,
        k_param_p_loiter_pos,
        k_param_pid_loiter_rate_lat,
        k_param_pid_loiter_rate_lon,
        k_param_p_alt_hold,
        k_param_p_throttle_rate,
        k_param_pid_throttle_accel,
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p,

        /***BEGIN PLANE PARAMS***/
		// Misc
		//
		k_param_hil_servos = 145,
		k_param_att_controller,
		k_param_stab_pitch_down,
		k_param_wingless,

		//BEV keys
		k_param_key_pid = 150,
		k_param_key_value,

		// 160: Fly-by-wire control
		//
		k_param_airspeed_min = 160,
		k_param_airspeed_max,
		k_param_FBWB_min_altitude_cm,  // 0=disabled, minimum value for altitude in cm (for first time try 30 meters = 3000 cm)
		k_param_flybywire_climb_rate,

		//
		// 170: elevons
		//
		k_param_rc_elevon_left = 170,
		k_param_rc_elevon_right,

		//
		// 210: Navigation parameters
		//
		k_param_roll_limit_cd = 175,
		k_param_pitch_limit_max_cd,
		k_param_pitch_limit_min_cd,
		k_param_throttle_slewrate,
		k_param_plane_throttle_cruise,

		//
		// 230: Waypoint data
		//
		k_param_plane_loiter_radius = 190,

		// other objects
		k_param_rollController = 200,
		k_param_pitchController,
		k_param_yawController,
		k_param_L1_controller,
		k_param_SpdHgt_Controller,

		//BEV our objects
		k_param_transitionState = 210,
        k_param_stabilize_allow,

        // 254,255: reserved
    };

    AP_Int16        format_version;
    AP_Int8         software_type;

    // Telemetry control
    //
    AP_Int16        sysid_this_mav;
    AP_Int16        sysid_my_gcs;
    AP_Int16        serial0_baud;
    AP_Int16        serial1_baud;
#if MAVLINK_COMM_NUM_BUFFERS > 2
    AP_Int16        serial2_baud;
    AP_Int8         serial2_protocol;
#endif
    AP_Int8         telem_delay;

    AP_Int16        rtl_altitude;

    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
    AP_Float        fs_batt_voltage;            // battery voltage below which failsafe will be triggered
    AP_Float        fs_batt_mah;                // battery capacity (in mah) below which failsafe will be triggered

    AP_Int8         failsafe_gps_enabled;       // gps failsafe enabled
    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         compass_enabled;

    AP_Int8         rssi_pin;
    AP_Float        rssi_range;                 // allows to set max voltage for rssi pin such as 5.0, 3.3 etc. 
    AP_Int8         rc_feel_rp;                 // controls vehicle response to user input with 0 being extremely soft and 100 begin extremely crisp
    
    // Waypoints
    //
    AP_Int16        land_speed;
    AP_Int16        pilot_velocity_z_max;        // maximum vertical velocity the pilot may request
    AP_Int16        pilot_accel_z;               // vertical acceleration the pilot may request

    // Throttle
    //
    AP_Int16        throttle_min;
    AP_Int16        throttle_max;
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;
    AP_Int16        throttle_cruise;
    AP_Int16        throttle_mid;
    AP_Int16        throttle_deadzone;

    //BEV keys
    AP_Int32 key_pid;
    AP_Int32 key_value;

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
    AP_Int8         frame_orientation;
    AP_Int8         arming_check;

    //AP_Int8         land_repositioning;
    AP_Float        ekfcheck_thresh;
    AP_Float        dcmcheck_thresh;

    // RC channels
    RC_Channel              rc_1;
    RC_Channel              rc_2;
    RC_Channel              rc_3;
    RC_Channel              rc_4;
    RC_Channel              rc_5;
    RC_Channel              rc_6;
    RC_Channel          rc_7;
    RC_Channel          rc_8; //BEV changed from RC_Channel_aux
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    RC_Channel              rc_9; //BEV changed from RC_Channel_aux
#endif
    RC_Channel              rc_10; //BEV changed from RC_Channel_aux
    RC_Channel              rc_11; //BEV changed from RC_Channel_aux
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    RC_Channel          rc_12;  //BEV changed from RC_Channel_aux
    RC_Channel          rc_13;  //BEV changed from RC_Channel_aux
    RC_Channel          rc_14;  //BEV changed from RC_Channel_aux
#endif
    //BEV creating rc channel objects for the left and right elevons.
    //They are linked to ch 1 and 2 because they output on those pins.
    //They do not read in from the radio, and are fundamentally different
    //from the above rc_1 and 2 (roll and pitch inputs)
    RC_Channel          rc_elevon_left;
    RC_Channel          rc_elevon_right;

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

    // Acro parameters
    AP_Float                acro_rp_p;
    AP_Float                acro_yaw_p;
    AP_Float                acro_balance_roll;
    AP_Float                acro_balance_pitch;
    AP_Float                acro_expo;

    AC_PID                  pid_rate_roll;
    AC_PID                  pid_rate_pitch;
    AC_PID                  pid_rate_yaw;
    AC_PID                  pid_loiter_rate_lat;
    AC_PID                  pid_loiter_rate_lon;

    AC_P                    p_throttle_rate;
    AC_PID                  pid_throttle_accel;

    AC_P                    p_loiter_pos;
    AC_P                    p_stabilize_roll;
    AC_P                    p_stabilize_pitch;
    AC_P                    p_stabilize_yaw;
    AC_P                    p_alt_hold;

    //BEV begin plane parameter additions
    AP_Int8 wingless;

    // Waypoints
    //
    AP_Int16 plane_loiter_radius;

    // Fly-by-wire
    //
    AP_Int16 flybywire_climb_rate;

    // Navigational maneuvering limits
    //
    AP_Int16 roll_limit_cd;

    AP_Int8 stabilize_allow;

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

        //BEV creating rc channel objects for the left and right elevons.
        //They are linked to ch 1 and 2 because they output on those pins.
        //They do not read in from the radio, and are fundamentally different
        //from the above rc_1 and 2 (roll and pitch inputs)
        rc_elevon_left      (CH_1),
        rc_elevon_right     (CH_2),

        // PID controller	    initial P	            initial I		        initial D               initial imax
        //----------------------------------------------------------------------------------------------------------
        pid_rate_roll           (RATE_ROLL_P,           RATE_ROLL_I,            RATE_ROLL_D,            RATE_ROLL_IMAX),
        pid_rate_pitch          (RATE_PITCH_P,          RATE_PITCH_I,           RATE_PITCH_D,           RATE_PITCH_IMAX),
        pid_rate_yaw            (RATE_YAW_P,            RATE_YAW_I,             RATE_YAW_D,             RATE_YAW_IMAX),

        pid_loiter_rate_lat     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),
        pid_loiter_rate_lon     (LOITER_RATE_P,         LOITER_RATE_I,          LOITER_RATE_D,          LOITER_RATE_IMAX),

        p_throttle_rate         (THROTTLE_RATE_P),
        pid_throttle_accel      (THROTTLE_ACCEL_P,      THROTTLE_ACCEL_I,       THROTTLE_ACCEL_D,       THROTTLE_ACCEL_IMAX),

        // P controller	        initial P
        //----------------------------------------------------------------------
        p_loiter_pos            (LOITER_POS_P),

        p_stabilize_roll        (STABILIZE_ROLL_P),
        p_stabilize_pitch       (STABILIZE_PITCH_P),
        p_stabilize_yaw         (STABILIZE_YAW_P),

        p_alt_hold              (ALT_HOLD_P)
    {
    }
};

extern const AP_Param::Info        var_info[];

#endif // PARAMETERS_H

