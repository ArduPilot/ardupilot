#pragma once

#define AP_PARAM_VEHICLE_NAME copter

#include <AP_Common/AP_Common.h>
#include "RC_Channel.h"
#include <AP_Proximity/AP_Proximity.h>

#if MODE_FOLLOW_ENABLED
 # include <AP_Follow/AP_Follow.h>
#endif
#if WEATHERVANE_ENABLED
 #include <AC_AttitudeControl/AC_WeatherVane.h>
#endif

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
        k_param_software_type, // deprecated
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables
        k_param_NavEKF2_old, // deprecated - remove
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

        // relay object
        k_param_relay,

        // (old) EPM object
        k_param_epm_unused,

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
        k_param_crosstrack_min_distance,    // deprecated - remove with next eeprom number change
        k_param_rssi_pin,                   // unused, replaced by rssi_ library parameters
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_speed_up,         // renamed from k_param_pilot_velocity_z_max
        k_param_circle_rate,            // deprecated - remove
        k_param_rangefinder_gain,       // deprecated - remove
        k_param_ch8_option_old,         // deprecated
        k_param_arming_check_old,       // deprecated - remove
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,            // unused - moved to AP_BattMonitor
        k_param_angle_rate_max,         // remove
        k_param_rssi_range,             // unused, replaced by rssi_ library parameters
        k_param_rc_feel_rp,             // deprecated
        k_param_NavEKF,                 // deprecated - remove
        k_param_mission,                // mission library
        k_param_rc_13_old,
        k_param_rc_14_old,
        k_param_rally,
        k_param_poshold_brake_rate,
        k_param_poshold_brake_angle_max,
        k_param_pilot_accel_z,
        k_param_serial0_baud,           // deprecated - remove
        k_param_serial1_baud,           // deprecated - remove
        k_param_serial2_baud,           // deprecated - remove
        k_param_land_repositioning,
        k_param_rangefinder, // rangefinder object
        k_param_fs_ekf_thresh,
        k_param_terrain,
        k_param_acro_rp_expo,           // deprecated - remove
        k_param_throttle_deadzone,
        k_param_optflow,
        k_param_dcmcheck_thresh,        // deprecated - remove
        k_param_log_bitmask,
        k_param_cli_enabled_old,        // deprecated - remove
        k_param_throttle_filt,
        k_param_throttle_behavior,
        k_param_pilot_takeoff_alt, // 64

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence_old,              // only used for conversion
        k_param_gps_glitch,             // deprecated
        k_param_baro_glitch,            // 71 - deprecated

        // AP_ADSB Library
        k_param_adsb,                   // 72
        k_param_notify,                 // 73

        // 74: precision landing object
        k_param_precland = 74,

        //
        // 75: Singlecopter, CoaxCopter
        //
        k_param_single_servo_1 = 75,    // remove
        k_param_single_servo_2,         // remove
        k_param_single_servo_3,         // remove
        k_param_single_servo_4,         // 78 - remove

        //
        // 80: Heli
        //
        k_param_heli_servo_1 = 80,  // remove
        k_param_heli_servo_2,       // remove
        k_param_heli_servo_3,       // remove
        k_param_heli_servo_4,       // remove
        k_param_heli_pitch_ff,      // remove
        k_param_heli_roll_ff,       // remove
        k_param_heli_yaw_ff,        // remove
        k_param_heli_stab_col_min,  // remove
        k_param_heli_stab_col_max,  // remove
        k_param_heli_servo_rsc,     // 89 = full! - remove

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
                
        //
        // 100: Inertial Nav
        //
        k_param_inertial_nav = 100, // deprecated
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,
        k_param_loiter_nav,     // 105
        k_param_custom_control,

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
        k_param_serial_manager_old,
        k_param_ch9_option_old,
        k_param_ch10_option_old,
        k_param_ch11_option_old,
        k_param_ch12_option_old,
        k_param_takeoff_trigger_dz_old,
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
        k_param_imu = 140, // deprecated - can be deleted
        k_param_battery_monitoring = 141,   // deprecated - can be deleted
        k_param_volt_div_ratio, // deprecated - can be deleted
        k_param_curr_amp_per_volt,  // deprecated - can be deleted
        k_param_input_voltage,  // deprecated - can be deleted
        k_param_pack_capacity,  // deprecated - can be deleted
        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_rangefinder_enabled_old, // deprecated
        k_param_frame_type,
        k_param_optflow_enabled,    // deprecated
        k_param_fs_batt_voltage,    // unused - moved to AP_BattMonitor
        k_param_ch7_option_old,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_rangefinder_type_old,     // deprecated
        k_param_super_simple = 155,
        k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
        k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
        k_param_ahrs, // AHRS group // 159

        //
        // 160: Navigation parameters
        //
        k_param_rtl_altitude = 160,
        k_param_crosstrack_gain,    // deprecated - remove with next eeprom number change
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,
        k_param_tilt_comp, // 164 deprecated - remove with next eeprom number change


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,
        k_param_camera_mount2,      // deprecated

        //
        // Battery monitoring parameters
        //
        k_param_battery_volt_pin = 168, // deprecated - can be deleted
        k_param_battery_curr_pin,   // 169 deprecated - can be deleted

        //
        // 170: Radio settings
        //
        k_param_rc_1_old = 170,
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_10_old,
        k_param_rc_11_old,
        k_param_throttle_min,           // remove
        k_param_throttle_max,           // remove
        k_param_failsafe_throttle,
        k_param_throttle_fs_action,     // remove
        k_param_failsafe_throttle_value,
        k_param_throttle_trim,          // remove
        k_param_esc_calibrate,
        k_param_radio_tuning,
        k_param_radio_tuning_high_old,   // unused
        k_param_radio_tuning_low_old,    // unused
        k_param_rc_speed = 192,
        k_param_failsafe_battery_enabled, // unused - moved to AP_BattMonitor
        k_param_throttle_mid,           // remove
        k_param_failsafe_gps_enabled,   // remove
        k_param_rc_9_old,
        k_param_rc_12_old,
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
        k_param_flight_mode_chan,
        k_param_initial_mode,

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
        k_param_acro_rp_p = 221,    // remove
        k_param_axis_lock_p,        // remove
        k_param_pid_rate_roll,      // remove
        k_param_pid_rate_pitch,     // remove
        k_param_pid_rate_yaw,       // remove
        k_param_p_stabilize_roll,   // remove
        k_param_p_stabilize_pitch,  // remove
        k_param_p_stabilize_yaw,    // remove
        k_param_p_pos_xy,           // remove
        k_param_p_loiter_lon,       // remove
        k_param_pid_loiter_rate_lat,    // remove
        k_param_pid_loiter_rate_lon,    // remove
        k_param_pid_nav_lat,        // remove
        k_param_pid_nav_lon,        // remove
        k_param_p_alt_hold,             // remove
        k_param_p_vel_z,                // remove
        k_param_pid_optflow_roll,       // remove
        k_param_pid_optflow_pitch,      // remove
        k_param_acro_balance_roll_old,  // remove
        k_param_acro_balance_pitch_old, // remove
        k_param_pid_accel_z,            // remove
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p,             // remove
        k_param_autotune_axis_bitmask, // remove
        k_param_autotune_aggressiveness, // remove
        k_param_pi_vel_xy,              // remove
        k_param_fs_ekf_action,
        k_param_rtl_climb_min,
        k_param_rpm_sensor,
        k_param_autotune_min_d, // remove
        k_param_arming, // 252  - AP_Arming
        k_param_logger = 253, // 253 - Logging Group

        // 254,255: reserved

        k_param_vehicle = 257, // vehicle common block of parameters
        k_param_throw_altitude_min,
        k_param_throw_altitude_max,

        // the k_param_* space is 9-bits in size
        // 511: reserved
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

#if MODE_RTL_ENABLED
    AP_Int32        rtl_altitude;
    AP_Int16        rtl_speed_cms;
    AP_Float        rtl_cone_slope;
    AP_Int16        rtl_alt_final;
    AP_Int16        rtl_climb_min;              // rtl minimum climb in cm
    AP_Int32        rtl_loiter_time;
    AP_Enum<ModeRTL::RTLAltType> rtl_alt_type;
#endif

    AP_Int8         failsafe_gcs;               // ground station failsafe behavior
    AP_Int16        gps_hdop_good;              // GPS Hdop value at or below this value represent a good position

    AP_Int8         super_simple;

    AP_Int8         wp_yaw_behavior;            // controls how the autopilot controls yaw during missions

#if MODE_POSHOLD_ENABLED
    AP_Int16        poshold_brake_rate;         // PosHold flight mode's rotation rate during braking in deg/sec
    AP_Int16        poshold_brake_angle_max;    // PosHold flight mode's max lean angle during braking in centi-degrees
#endif

    // Waypoints
    //
    AP_Int16        land_speed;
    AP_Int16        land_speed_high;
    AP_Int16        pilot_speed_up;    // maximum vertical ascending velocity the pilot may request
    AP_Int16        pilot_accel_z;               // vertical acceleration the pilot may request

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
    AP_Int8         simple_modes;
    AP_Int8         flight_mode_chan;
    AP_Int8         initial_mode;

    // Misc
    //
    AP_Int32        log_bitmask;
    AP_Int8         esc_calibrate;
    AP_Int8         radio_tuning;
    AP_Int8         frame_type;
    AP_Int8         disarm_delay;

    AP_Int8         land_repositioning;
    AP_Int8         fs_ekf_action;
    AP_Int8         fs_crash_check;
    AP_Float        fs_ekf_thresh;
    AP_Int16        gcs_pid_mask;

#if MODE_THROW_ENABLED
    AP_Enum<ModeThrow::PreThrowMotorState>         throw_motor_start;
    AP_Int16         throw_altitude_min; // minimum altitude in m above which a throw can be detected
    AP_Int16         throw_altitude_max; // maximum altitude in m below which a throw can be detected
#endif

    AP_Int16                rc_speed; // speed of fast RC Channels in Hz

#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    // Acro parameters
    AP_Float                acro_balance_roll;
    AP_Float                acro_balance_pitch;
#endif

#if MODE_ACRO_ENABLED
    // Acro parameters
    AP_Int8                 acro_trainer;
#endif

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
    static const struct AP_Param::GroupInfo var_info2[];

    // altitude at which nav control can start in takeoff
    AP_Float wp_navalt_min;

    // button checking
#if HAL_BUTTON_ENABLED
    AP_Button *button_ptr;
#endif

#if MODE_THROW_ENABLED
    // Throw mode parameters
    AP_Int8 throw_nextmode;
    AP_Enum<ModeThrow::ThrowType> throw_type;
#endif

    // ground effect compensation enable/disable
    AP_Int8 gndeffect_comp_enabled;

#if AP_TEMPCALIBRATION_ENABLED
    // temperature calibration handling
    AP_TempCalibration temp_calibration;
#endif

#if AP_BEACON_ENABLED
    // beacon (non-GPS positioning) library
    AP_Beacon beacon;
#endif

#if HAL_PROXIMITY_ENABLED
    // proximity (aka object avoidance) library
    AP_Proximity proximity;
#endif

    // whether to enforce acceptance of packets only from sysid_my_gcs
    AP_Int8 sysid_enforce;
    
#if ADVANCED_FAILSAFE
    // advanced failsafe library
    AP_AdvancedFailsafe_Copter afs;
#endif

    // developer options
    AP_Int32 dev_options;

#if MODE_ACRO_ENABLED
    AP_Float acro_thr_mid;
#endif

    // frame class
    AP_Int8 frame_class;

    // RC input channels
    RC_Channels_Copter rc_channels;
    
    // control over servo output ranges
    SRV_Channels servo_channels;

#if MODE_SMARTRTL_ENABLED
    // Safe RTL library
    AP_SmartRTL smart_rtl;
#endif

    // wheel encoder and winch
#if AP_WINCH_ENABLED
    AP_Winch winch;
#endif

    // Additional pilot velocity items
    AP_Int16    pilot_speed_dn;

    // Land alt final stage
    AP_Int16 land_alt_low;

#if TOY_MODE_ENABLED
    ToyMode toy_mode;
#endif

#if MODE_FLOWHOLD_ENABLED
    // we need a pointer to the mode for the G2 table
    void *mode_flowhold_ptr;
#endif

#if MODE_FOLLOW_ENABLED
    // follow
    AP_Follow follow;
#endif

#if USER_PARAMS_ENABLED
    // User custom parameters
    UserParameters user_parameters;
#endif

#if AUTOTUNE_ENABLED
    // we need a pointer to autotune for the G2 table
    void *autotune_ptr;
#endif

    AP_Float tuning_min;
    AP_Float tuning_max;

#if AP_OAPATHPLANNER_ENABLED
    // object avoidance path planning
    AP_OAPathPlanner oa;
#endif

#if MODE_SYSTEMID_ENABLED
    // we need a pointer to the mode for the G2 table
    void *mode_systemid_ptr;
#endif

    // vibration failsafe enable/disable
    AP_Int8 fs_vibe_enabled;

    // Failsafe options bitmask #36
    AP_Int32 fs_options;

#if MODE_AUTOROTATE_ENABLED
    // Autonmous autorotation
    AC_Autorotation arot;
#endif

#if MODE_ZIGZAG_ENABLED
    // we need a pointer to the mode for the G2 table
    void *mode_zigzag_ptr;
#endif

    // command model parameters
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    AC_CommandModel command_model_acro_rp;
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    AC_CommandModel command_model_acro_y;
#endif

    AC_CommandModel command_model_pilot;

#if MODE_ACRO_ENABLED
    AP_Int8 acro_options;
#endif

#if MODE_AUTO_ENABLED
    AP_Int32 auto_options;
#endif

#if MODE_GUIDED_ENABLED
    AP_Int32 guided_options;
#endif

    AP_Float fs_gcs_timeout;

#if MODE_RTL_ENABLED
    AP_Int32 rtl_options;
#endif

    AP_Int32 flight_options;

#if AP_RANGEFINDER_ENABLED
    AP_Float rangefinder_filt;
#endif

#if MODE_GUIDED_ENABLED
    AP_Float guided_timeout;
#endif

    AP_Int8                 surftrak_mode;
    AP_Int8                 failsafe_dr_enable;
    AP_Int16                failsafe_dr_timeout;
    AP_Float                surftrak_tc;

    // ramp time of throttle during take-off
    AP_Float takeoff_throttle_slew_time;
    AP_Float takeoff_throttle_max;
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    AP_Int16 takeoff_rpm_min;
    AP_Int16 takeoff_rpm_max;
#endif

    // EKF variance filter cutoff
    AP_Float fs_ekf_filt_hz;

#if WEATHERVANE_ENABLED
    AC_WeatherVane weathervane;
#endif

    // payload place parameters
    AP_Float pldp_thrust_placed_fraction;
    AP_Float pldp_range_finder_maximum_m;
    AP_Float pldp_delay_s;
    AP_Float pldp_descent_speed_ms;
};

extern const AP_Param::Info        var_info[];
