/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
/*
  This is the main Blimp class
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

// Application dependencies
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>          // ArduPilot Mega Flash Memory Library
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
// #include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
// #include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>
// #include <AP_Mission/AP_Mission.h>     // Mission command library
// #include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
// #include <AC_AttitudeControl/AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
// #include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
// #include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <Filter/Filter.h>             // Filter library
#include <AP_Airspeed/AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
// #include <AC_WPNav/AC_WPNav.h>           // Blimp waypoint navigation library
// #include <AC_WPNav/AC_Loiter.h>
// #include <AC_WPNav/AC_Circle.h>          // circle navigation library
// #include <AP_Declination/AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
// #include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
// #include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
// #include <AC_InputManager/AC_InputManager_Heli.h>   // Heli specific pilot input handling library
#include <AP_Arming/AP_Arming.h>
// #include <AP_SmartRTL/AP_SmartRTL.h>
// #include <AP_TempCalibration/AP_TempCalibration.h>
// #include <AC_AutoTune/AC_AutoTune.h>
// #include <AP_Parachute/AP_Parachute.h>
// #include <AC_Sprayer/AC_Sprayer.h>
// #include <AP_ADSB/AP_ADSB.h>
#include <AP_Scripting/AP_Scripting.h>

// Configuration
#include "defines.h"
#include "config.h"

#include "Fins.h"

// #define MOTOR_CLASS Fins

#include "RC_Channel.h"         // RC Channel Library

#include "GCS_Mavlink.h"
#include "GCS_Blimp.h"
// #include "AP_Rally.h"           // Rally point library
#include "AP_Arming.h"


#include <AP_Mount/AP_Mount.h>

// Local modules

#include "Parameters.h"


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#include "mode.h"

class Blimp : public AP_Vehicle
{
public:
    friend class GCS_MAVLINK_Blimp;
    friend class GCS_Blimp;
    friend class Parameters;
    friend class ParametersG2;

    friend class AP_Arming_Blimp;
    friend class RC_Channel_Blimp;
    friend class RC_Channels_Blimp;

    friend class Mode;
    friend class ModeManual;
    friend class ModeLand;

    friend class Fins;

    Blimp(void);

private:

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // primary input control channels
    RC_Channel *channel_right;
    RC_Channel *channel_front;
    RC_Channel *channel_down;
    RC_Channel *channel_yaw;

    AP_Logger logger;

    // flight modes convenience array
    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Arming/Disarming management class
    AP_Arming_Blimp arming;

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms;
    int8_t ekf_primary_core;

    // vibration check
    struct {
        bool high_vibes;    // true while high vibration are detected
        uint32_t start_ms;  // system time high vibration were last detected
        uint32_t clear_ms;  // system time high vibrations stopped
    } vibration_check;

    // GCS selection
    GCS_Blimp _gcs; // avoid using this; use gcs()
    GCS_Blimp &gcs()
    {
        return _gcs;
    }

    // Documentation of Globals:
    typedef union {
        struct {
            uint8_t unused1                 : 1; // 0
            uint8_t pre_arm_rc_check        : 1; // 3       // true if rc input pre-arm checks have been completed successfully
            uint8_t pre_arm_check           : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t auto_armed              : 1; // 5       // stops auto missions from beginning until throttle is raised
            uint8_t logging_started         : 1; // 6       // true if logging has started
            uint8_t land_complete           : 1; // 7       // true if we have detected a landing
            uint8_t new_radio_frame         : 1; // 8       // Set true if we have new PWM data to act on from the Radio
            uint8_t usb_connected_unused    : 1; // 9       // UNUSED
            uint8_t rc_receiver_present     : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
            uint8_t compass_mot             : 1; // 11      // true if we are currently performing compassmot calibration
            uint8_t motor_test              : 1; // 12      // true if we are currently performing the motors test
            uint8_t initialised             : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t land_complete_maybe     : 1; // 14      // true if we may have landed (less strict version of land_complete)
            uint8_t throttle_zero           : 1; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
            uint8_t system_time_set_unused  : 1; // 16      // true if the system time has been set from the GPS
            uint8_t gps_glitching           : 1; // 17      // true if GPS glitching is affecting navigation accuracy
            uint8_t using_interlock         : 1; // 20      // aux switch motor interlock function is in use
            uint8_t land_repo_active        : 1; // 21      // true if the pilot is overriding the landing position
            uint8_t motor_interlock_switch  : 1; // 22      // true if pilot is requesting motor interlock enable
            uint8_t in_arming_delay         : 1; // 23      // true while we are armed but waiting to spin motors
            uint8_t initialised_params      : 1; // 24      // true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
            uint8_t unused3                 : 1; // 25      // was compass_init_location; true when the compass's initial location has been set
            uint8_t unused2                 : 1; // 26      // aux switch rc_override is allowed
            uint8_t unused4                 : 1; // 27      // was "we armed using a arming switch"
        };
        uint32_t value;
    } ap_t;

    ap_t ap; //MIR Set of general variables

    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t must be uint32_t");

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    Mode::Number control_mode;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;
    Mode::Number prev_control_mode;

    RCMapper rcmap;

    // intertial nav alt when we armed
    float arming_altitude_m;

    // Failsafe
    struct {
        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint8_t radio               : 1; // A status flag for the radio failsafe
        uint8_t gcs                 : 1; // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // true if ekf failsafe has occurred
    } failsafe;

    bool any_failsafe_triggered() const
    {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf;
    }

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
        uint8_t primary_gps : 2;    // primary gps index
    } sensor_health;

    // Motor Output
    Fins *motors;

    int32_t _home_bearing;
    uint32_t _home_distance;

    // Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
    int32_t initial_armed_bearing;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                       FUNCTOR_BIND_MEMBER(&Blimp::handle_battery_failsafe, void, const char*, const int8_t),
                       _failsafe_priorities};

    // Altitude
    int32_t baro_alt;            // barometer altitude in cm above home
    LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

    // filtered pilot's throttle input used to cancel landing if throttle held high
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D Location vectors
    // Current location of the vehicle (altitude is relative to home)
    Location current_loc;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
    // AC_AttitudeControl_t *attitude_control;
    // AC_PosControl *pos_control;
    // AC_WPNav *wp_nav;
    // AC_Loiter *loiter_nav;


    // System Timers
    // --------------
    // arm_time_ms - Records when vehicle was armed. Will be Zero if we are disarmed.
    uint32_t arm_time_ms;

    // Used to exit the roll and pitch auto trim function
    uint8_t auto_trim_counter;
    bool auto_trim_started = false;


    // last valid RC input time
    uint32_t last_radio_update_ms;

    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;

    bool standby_active;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    enum Failsafe_Action {
        Failsafe_Action_None           = 0,
        Failsafe_Action_Land           = 1,
        Failsafe_Action_Terminate      = 5
    };

    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),   // 1
        GCS_CONTINUE_IF_AUTO            = (1<<1),   // 2
        RC_CONTINUE_IF_GUIDED           = (1<<2),   // 4
        CONTINUE_IF_LANDING             = (1<<3),   // 8
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),   // 16
        RELEASE_GRIPPER                 = (1<<5),   // 32
    };

    static constexpr int8_t _failsafe_priorities[] = {
        Failsafe_Action_Terminate,
        Failsafe_Action_Land,
        Failsafe_Action_None,
        -1 // the priority list must end with a sentinel of -1
    };

#define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == Failsafe_Action_Land,
                  "FAILSAFE_LAND_PRIORITY must match the entry in _failsafe_priorities");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // AP_State.cpp
    void set_auto_armed(bool b);
    void set_failsafe_radio(bool b);
    void set_failsafe_gcs(bool b);
    // void update_using_interlock();

    // Blimp.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void fast_loop() override;
    // bool start_takeoff(float alt) override;
    // bool set_target_location(const Location& target_loc) override;
    // bool set_target_velocity_NED(const Vector3f& vel_ned) override;
    // bool set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs) override;
    void rc_loop();
    void throttle_loop();
    void update_batt_compass(void);
    void fourhundred_hz_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void read_AHRS(void);
    void update_altitude();

    // Attitude.cpp
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    // void update_throttle_hover();
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle();
    // void set_accel_throttle_I_from_pilot_throttle();
    void rotate_body_frame_to_NE(float &x, float &y);
    uint16_t get_pilot_speed_dn();

    // baro_ground_effect.cpp
    // void update_ground_effect_detector(void);
    // void update_ekf_terrain_height_stable();

    // commands.cpp
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock) WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) WARN_IF_UNUSED;
    bool far_from_EKF_origin(const Location& loc);

    // compassmot.cpp
    // MAV_RESULT mavlink_compassmot(const GCS_MAVLINK &gcs_chan);

    // // crash_check.cpp
    // void crash_check();
    // void thrust_loss_check();
    // void parachute_check();
    // void parachute_release();
    // void parachute_manual_release();

    // ekf_check.cpp
    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void check_ekf_reset();
    void check_vibration();

    // events.cpp
    bool failsafe_option(FailsafeOption opt) const;
    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    void failsafe_gcs_check();
    // void failsafe_gcs_on_event(void); //MIR will probably need these two soon.
    // void failsafe_gcs_off_event(void);
    // void gpsglitch_check();
    // void set_mode_RTL_or_land_with_pause(ModeReason reason);
    bool should_disarm_on_failsafe();
    void do_failsafe_action(Failsafe_Action action, ModeReason reason);

    // failsafe.cpp
    void failsafe_enable();
    void failsafe_disable();

    // fence.cpp
    void fence_check();

    // inertia.cpp
    void read_inertia();

    // landing_detector.cpp
    void update_land_and_crash_detectors();
    void update_land_detector();
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    // void update_throttle_mix();

    // landing_gear.cpp
    void landinggear_update();

    // // standby.cpp
    // void standby_update();

    // Log.cpp
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_EKF_POS();
    void Log_Write_MotBatt();
    void Log_Write_Data(LogDataID id, int32_t value);
    void Log_Write_Data(LogDataID id, uint32_t value);
    void Log_Write_Data(LogDataID id, int16_t value);
    void Log_Write_Data(LogDataID id, uint16_t value);
    void Log_Write_Data(LogDataID id, float value);
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max);
    void Log_Sensor_Health();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out);
    void Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z);
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // mode.cpp
    bool set_mode(Mode::Number mode, ModeReason reason);
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    uint8_t get_mode() const override
    {
        return (uint8_t)control_mode;
    }
    void update_flight_mode();
    void notify_flight_mode();

    // mode_land.cpp
    void set_mode_land_with_pause(ModeReason reason);
    bool landing_with_GPS();

    // // motors.cpp
    void arm_motors_check();
    // void auto_disarm_check();
    void motors_output();
    // void lost_vehicle_check();

    // navigation.cpp
    // void run_nav_updates(void);
    // int32_t home_bearing();
    // uint32_t home_distance();

    // Parameters.cpp
    void load_parameters(void) override;
    void convert_pid_parameters(void);
    void convert_lgr_parameters(void);
    void convert_fs_options_params(void);

    // radio.cpp
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    int16_t get_throttle_mid(void);

    // sensors.cpp
    void read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok();
    bool rangefinder_up_ok();
    void rpm_update();
    void update_optical_flow(void);
    void compass_cal_update(void);
    void accel_cal_update(void);
    void init_proximity();
    void update_proximity();

    // RC_Channel.cpp
    void save_trim();
    void auto_trim();
    void auto_trim_cancel();

    // system.cpp
    void init_ardupilot() override;
    void startup_INS_ground();
    bool position_ok() const;
    bool ekf_has_absolute_position() const;
    bool ekf_has_relative_position() const;
    bool ekf_alt_ok() const;
    void update_auto_armed();
    bool should_log(uint32_t mask);
    MAV_TYPE get_frame_mav_type();
    const char* get_frame_string();
    void allocate_motors(void);

    // vehicle specific waypoint info helpers
    bool get_wp_distance_m(float &distance) const override;
    bool get_wp_bearing_deg(float &bearing) const override;
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;

    Mode *flightmode;
    ModeManual mode_manual;
    ModeLand mode_land;

    // mode.cpp
    Mode *mode_from_mode_num(const Mode::Number mode);
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

public:
    void failsafe_check();      // failsafe.cpp
};

extern Blimp blimp;

using AP_HAL::millis;
using AP_HAL::micros;
