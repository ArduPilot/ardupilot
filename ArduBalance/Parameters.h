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
    static const uint16_t k_format_version = 200;

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
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables

        // simulation
        k_param_sitl = 10,

        // Misc
        //
        k_param_log_bitmask = 20,
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_crosstrack_min_distance,
        k_param_wheel_encoder_speed,
        k_param_rssi_pin,

        // 65: AP_Limits Library
        k_param_limits = 65,
        k_param_gpslock_limit,
        k_param_geofence_limit,
        k_param_altitude_limit,

        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100,

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
        k_param_imu = 140, // deprecated - can be deleted
        k_param_battery_monitoring = 141,
        k_param_volt_div_ratio,
        k_param_curr_amp_per_volt,
        k_param_input_voltage,
        k_param_pack_capacity,
        k_param_compass_enabled,
        k_param_compass,
        k_param_sonar_enabled,
        k_param_optflow_enabled,
        k_param_low_voltage,
        k_param_ch7_option,
        k_param_sonar_type,
        k_param_copter_leds_mode,
        k_param_ahrs, // AHRS group

        //
        // 160: Navigation parameters
        //
        k_param_crosstrack_gain = 160,
        k_param_fbw_speed,
        k_param_throttle,
        k_param_dead_zone,

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
        k_param_failsafe_throttle,
        k_param_failsafe_throttle_value,
        k_param_radio_tuning,
        k_param_radio_tuning_high,
        k_param_radio_tuning_low,
        k_param_rc_speed,
        k_param_failsafe_battery_enabled,

        //
        // 200: flight modes
        //
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,

        //
        // 210: Waypoint data
        //
        k_param_waypoint_mode = 210, // remove
        k_param_command_total,
        k_param_command_index,
        k_param_command_nav_index,   // remove
        k_param_waypoint_radius,
        k_param_circle_radius,
        k_param_waypoint_speed_max,

        //
        // 220: PI/D Controllers
        //
        k_param_p_vel = 220,
        k_param_pid_balance,
        k_param_pid_yaw,
        k_param_pid_wheel_left_mixer,
        k_param_pid_wheel_right_mixer,
        k_param_pid_nav,
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

    AP_Int8         sonar_enabled;
    AP_Int8         sonar_type;                 // 0 = XL, 1 = LV,
                                                // 2 = XLL (XL with 10m range)
                                                // 3 = HRLV

    AP_Int8         battery_monitoring;         // 0=disabled, 3=voltage only,
                                                // 4=voltage and current
    AP_Float        volt_div_ratio;
    AP_Float        curr_amp_per_volt;
    AP_Float        input_voltage;
    AP_Int16        pack_capacity;              // Battery pack capacity less reserve
    AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled

    AP_Int8         compass_enabled;
    AP_Int8         optflow_enabled;
    AP_Float        low_voltage;

    AP_Int16        fbw_speed;
    AP_Float        throttle;
    AP_Int16        dead_zone;

    AP_Int8         copter_leds_mode;           // Operating mode of LED
                                                // lighting system

    AP_Int8         battery_volt_pin;
    AP_Int8         battery_curr_pin;
    AP_Int8         rssi_pin;

    // Waypoints
    //
    AP_Int8         command_total;
    AP_Int8         command_index;
    AP_Int16        waypoint_radius;
    AP_Int16        circle_radius;
    AP_Int16        waypoint_speed_max;
    AP_Float        crosstrack_gain;
    AP_Int16 		crosstrack_min_distance;
    AP_Int16 		wheel_encoder_speed;

    // Throttle
    //
    AP_Int8         failsafe_throttle;
    AP_Int16        failsafe_throttle_value;

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
    AP_Int16        log_bitmask;

    AP_Int8         radio_tuning;
    AP_Int16        radio_tuning_high;
    AP_Int16        radio_tuning_low;
    AP_Int8         ch7_option;

    // Camera
#if CAMERA == ENABLED
//    AP_Camera        camera;
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


    AP_Float                p_vel;

    // PI/D controllers
    AC_PID                  pid_balance;
    AC_PID                  pid_yaw;
    AC_PID                  pid_wheel_left_mixer;
    AC_PID                  pid_wheel_right_mixer;
    AC_PID                  pid_nav;


    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters() :
    rc_1                    (CH_1),
    rc_2                    (CH_2),
    rc_3                    (CH_3),
    rc_4                    (CH_4),
    rc_5                    (CH_5),
    rc_6                    (CH_6),
    rc_7                    (CH_7),
    rc_8                    (CH_8),
#if MOUNT == ENABLED
    rc_10                   (CH_10),
    rc_11                   (CH_11),
#endif

    // 220
    // PID controller       initial P           initial I       initial D           initial imax
    //-----------------------------------------------------------------------------------------------------
    pid_balance             (BALANCE_P,     BALANCE_I,          BALANCE_D,          BALANCE_IMAX    * 100),
    pid_yaw                 (YAW_P,         YAW_I,              YAW_D,              YAW_IMAX        * 100),
    pid_wheel_left_mixer    (WHEEL_P,       WHEEL_I,            WHEEL_D,            WHEEL_IMAX      * 100),
    pid_wheel_right_mixer   (WHEEL_P,       WHEEL_I,            WHEEL_D,            WHEEL_IMAX      * 100),
    pid_nav                 (NAV_P,         NAV_I,              NAV_D,              NAV_IMAX        * 100)
    {
    }
};

extern const AP_Param::Info        var_info[];

#endif // PARAMETERS_H

