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
  This is the main Sub class
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
#include <GCS_MAVLink/GCS.h>
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination/AP_Declination.h>     // ArduPilot Mega Declination Helper Library

// Application dependencies
#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_GPS/AP_GPS.h>             // ArduPilot GPS library
#include <AP_Logger/AP_Logger.h>          // ArduPilot Mega Flash Memory Library
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>         // Mission command library
#include <AC_AttitudeControl/AC_AttitudeControl_Sub.h> // Attitude control library
#include <AC_AttitudeControl/AC_PosControl_Sub.h>      // Position control library
#include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <Filter/Filter.h>             // Filter library
#include <AP_Relay/AP_Relay.h>           // APM relay
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav/AC_WPNav.h>           // Waypoint navigation library
#include <AC_WPNav/AC_Loiter.h>
#include <AC_WPNav/AC_Circle.h>          // circle navigation library
#include <AC_Fence/AC_Fence.h>           // Fence library
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Scheduler/PerfInfo.h>       // loop perf monitoring
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
#include <AP_Terrain/AP_Terrain.h>
#include <AP_JSButton/AP_JSButton.h>   // Joystick/gamepad button function assignment
#include <AP_LeakDetector/AP_LeakDetector.h> // Leak detector
#include <AP_TemperatureSensor/TSYS01.h>

// Local modules
#include "defines.h"
#include "config.h"
#include "GCS_Mavlink.h"
#include "RC_Channel.h"         // RC Channel Library
#include "Parameters.h"
#include "AP_Arming_Sub.h"
#include "GCS_Sub.h"

// libraries which are dependent on #defines in defines.h and/or config.h
#if OPTFLOW == ENABLED
#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#endif

#if RCMAP_ENABLED == ENABLED
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#endif

#if RPM_ENABLED == ENABLED
#include <AP_RPM/AP_RPM.h>
#endif

#if GRIPPER_ENABLED == ENABLED
#include <AP_Gripper/AP_Gripper.h>             // gripper stuff
#endif

#if PROXIMITY_ENABLED == ENABLED
#include <AP_Proximity/AP_Proximity.h>
#endif

#if AVOIDANCE_ENABLED == ENABLED
#include <AC_Avoidance/AC_Avoid.h>           // Stop at fence library
#endif

#if AC_RALLY == ENABLED
#include <AP_Rally/AP_Rally.h>           // Rally point library
#endif

#if CAMERA == ENABLED
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#endif

#ifdef ENABLE_SCRIPTING
#include <AP_Scripting/AP_Scripting.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

class Sub : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Sub;
    friend class GCS_Sub;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Sub;
    friend class RC_Channels_Sub;

    Sub(void);

private:

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;
    RC_Channel *channel_forward;
    RC_Channel *channel_lateral;

    AP_Logger logger;

    AP_LeakDetector leak_detector;

    TSYS01 celsius;

    struct {
        bool enabled:1;
        bool alt_healthy:1; // true if we can trust the altitude from the rangefinder
        int16_t alt_cm;     // tilt compensated altitude (in cm) from rangefinder
        uint32_t last_healthy_ms;
        LowPassFilterFloat alt_cm_filt; // altitude filter
    } rangefinder_state = { false, false, 0, 0 };

#if RPM_ENABLED == ENABLED
    AP_RPM rpm_sensor;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Sub::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::exit_mission, void)};

    // Optical flow sensor
#if OPTFLOW == ENABLED
    OpticalFlow optflow;
#endif

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms = 0;

    // GCS selection
    GCS_Sub _gcs; // avoid using this; use gcs()
    GCS_Sub &gcs() { return _gcs; }

    // User variables
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    // Documentation of Globals:
    union {
        struct {
            uint8_t pre_arm_check       : 1; // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t logging_started     : 1; // true if logging has started
            uint8_t compass_mot         : 1; // true if we are currently performing compassmot calibration
            uint8_t motor_test          : 1; // true if we are currently performing the motors test
            uint8_t initialised         : 1; // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t gps_base_pos_set    : 1; // true when the gps base position has been set (used for RTK gps only)
            uint8_t at_bottom           : 1; // true if we are at the bottom
            uint8_t at_surface          : 1; // true if we are at the surface
            uint8_t depth_sensor_present: 1; // true if there is a depth sensor detected at boot
            uint8_t unused1             : 1; // was compass_init_location; true when the compass's initial location has been set
        };
        uint32_t value;
    } ap;

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    control_mode_t control_mode;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;

    control_mode_t prev_control_mode;
    ModeReason prev_control_mode_reason = ModeReason::UNKNOWN;

#if RCMAP_ENABLED == ENABLED
    RCMapper rcmap;
#endif

    // Failsafe
    struct {
        uint32_t last_leak_warn_ms;      // last time a leak warning was sent to gcs
        uint32_t last_gcs_warn_ms;
        uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
        uint32_t last_pilot_input_ms; // last time we received pilot input in the form of MANUAL_CONTROL or RC_CHANNELS_OVERRIDE messages
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed
        uint32_t last_crash_warn_ms; // last time a crash warning was sent to gcs
        uint32_t last_ekf_warn_ms; // last time an ekf warning was sent to gcs

        uint8_t pilot_input          : 1; // true if pilot input failsafe is active, handles things like joystick being disconnected during operation
        uint8_t gcs                  : 1; // A status flag for the ground station failsafe
        uint8_t ekf                  : 1; // true if ekf failsafe has occurred
        uint8_t terrain              : 1; // true if the missing terrain data failsafe has occurred
        uint8_t leak                 : 1; // true if leak recently detected
        uint8_t internal_pressure    : 1; // true if internal pressure is over threshold
        uint8_t internal_temperature : 1; // true if temperature is over threshold
        uint8_t crash                : 1; // true if we are crashed
        uint8_t sensor_health        : 1; // true if at least one sensor has triggered a failsafe (currently only used for depth in depth enabled modes)
    } failsafe;

    bool any_failsafe_triggered() const {
        return (failsafe.pilot_input || battery.has_failsafed() || failsafe.gcs || failsafe.ekf || failsafe.terrain);
    }

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if any single baro is healthy
        uint8_t depth       : 1;    // true if depth sensor is healthy
        uint8_t compass     : 1;    // true if compass is healthy
    } sensor_health;

    // Baro sensor instance index of the external water pressure sensor
    uint8_t depth_sensor_idx;

    AP_Motors6DOF motors;

    // GPS variables
    // Sometimes we need to remove the scaling for distance calcs
    float scaleLongDown;

    // Auto
    AutoMode auto_mode;   // controls which auto controller is run

    // Guided
    GuidedMode guided_mode;  // controls which controller is run (pos or vel)

    // Circle
    bool circle_pilot_yaw_override; // true if pilot is overriding yaw

    // Stores initial bearing when armed
    int32_t initial_armed_bearing;

    // Throttle variables
    int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands
    uint32_t nav_delay_time_start_ms;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Sub::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    AP_Arming_Sub arming;

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;      // desired altitude in cm above the ground

    // Turn counter
    int32_t quarter_turn_count;
    uint8_t last_turn_state;

    // Input gain
    float gain;

    // Flag indicating if we are currently using input hold
    bool input_hold_engaged;

    // 3D Location vectors
    // Current location of the Sub (altitude is relative to home)
    Location current_loc;

    // Navigation Yaw control
    // auto flight mode's yaw mode
    uint8_t auto_yaw_mode;

    // Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
    Vector3f roi_WP;

    // bearing from current location to the yaw_look_at_WP
    float yaw_look_at_WP_bearing;

    float yaw_xtrack_correct_heading;

    // yaw used for YAW_LOOK_AT_HEADING yaw_mode
    int32_t yaw_look_at_heading;

    // Deg/s we should turn
    int16_t yaw_look_at_heading_slew;

    // heading when in yaw_look_ahead_bearing
    float yaw_look_ahead_bearing;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    AP_AHRS_View ahrs_view;

    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
    AC_AttitudeControl_Sub attitude_control;

    AC_PosControl_Sub pos_control;

    AC_WPNav wp_nav;
    AC_Loiter loiter_nav;
    AC_Circle circle_nav;

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera{MASK_LOG_CAMERA, current_loc};
#endif

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    AP_Mount camera_mount;
#endif

    // AC_Fence library to reduce fly-aways
#if AC_FENCE == ENABLED
    AC_Fence fence;
#endif

#if AVOIDANCE_ENABLED == ENABLED
    AC_Avoid avoid;
#endif

    // Rally library
#if AC_RALLY == ENABLED
    AP_Rally rally;
#endif

    // terrain handling
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    AP_Terrain terrain{mission};
#endif

    // used to allow attitude and depth control without a position system
    struct attitude_no_gps_struct {
        uint32_t last_message_ms;
        mavlink_set_attitude_target_t packet;
    };

    attitude_no_gps_struct set_attitude_target_no_gps {0};

    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;

    uint32_t last_pilot_heading;
    uint32_t last_pilot_yaw_input_ms;
    uint32_t fs_terrain_recover_start_ms;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    void fast_loop() override;
    void fifty_hz_loop();
    void update_batt_compass(void);
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void update_turn_counter();
    void read_AHRS(void);
    void update_altitude();
    float get_smoothing_gain();
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    void check_ekf_yaw_reset();
    float get_roi_yaw();
    float get_look_ahead_yaw();
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
    void update_poscon_alt_max();
    void rotate_body_frame_to_NE(float &x, float &y);
    void send_heartbeat(mavlink_channel_t chan);
#if RPM_ENABLED == ENABLED
    void rpm_update();
#endif
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_MotBatt();
    void Log_Write_Data(LogDataID id, int32_t value);
    void Log_Write_Data(LogDataID id, uint32_t value);
    void Log_Write_Data(LogDataID id, int16_t value);
    void Log_Write_Data(LogDataID id, uint16_t value);
    void Log_Write_Data(LogDataID id, float value);
    void Log_Sensor_Health();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Vehicle_Startup_Messages();
    void load_parameters(void) override;
    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock) WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) WARN_IF_UNUSED;
    bool far_from_EKF_origin(const Location& loc);
    void exit_mission();
    bool verify_loiter_unlimited();
    bool verify_loiter_time();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool acro_init(void);
    void acro_run();
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);
    bool althold_init(void);
    void althold_run();
    bool auto_init(void);
    void auto_run();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location& dest_loc);
    void auto_wp_run();
    void auto_spline_run();
    void auto_circle_movetoedge_start(const Location &circle_center, float radius_m);
    void auto_circle_start();
    void auto_circle_run();
    void auto_nav_guided_start();
    void auto_nav_guided_run();
    bool auto_loiter_start();
    void auto_loiter_run();
    uint8_t get_default_auto_yaw_mode(bool rtl);
    void set_auto_yaw_mode(uint8_t yaw_mode);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    void set_auto_yaw_roi(const Location &roi_location);
    float get_auto_heading(void);
    bool circle_init(void);
    void circle_run();
    bool guided_init(bool ignore_checks = false);
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location& dest_loc);
    void guided_set_velocity(const Vector3f& velocity);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    void guided_set_angle(const Quaternion &q, float climb_rate_cms);
    void guided_run();
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_limit_clear();
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    void guided_limit_init_time_and_pos();
    bool guided_limit_check();

    bool poshold_init(void);
    void poshold_run();

    bool motordetect_init();
    void motordetect_run();

    bool stabilize_init(void);
    void stabilize_run();
    bool manual_init(void);
    void manual_run();
    void failsafe_sensors_check(void);
    void failsafe_crash_check();
    void failsafe_ekf_check(void);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    void failsafe_gcs_check();
    void failsafe_pilot_input_check(void);
    void set_neutral_controls(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void mainloop_failsafe_enable();
    void mainloop_failsafe_disable();
    void fence_check();
    bool set_mode(control_mode_t mode, ModeReason reason);
    bool set_mode(const uint8_t mode, const ModeReason reason) override;
    uint8_t get_mode() const override { return (uint8_t)control_mode; }
    void update_flight_mode();
    void exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode);
    bool mode_requires_GPS(control_mode_t mode);
    bool mode_has_manual_throttle(control_mode_t mode);
    bool mode_allows_arming(control_mode_t mode, bool arming_from_gcs);
    void notify_flight_mode(control_mode_t mode);
    void read_inertia();
    void update_surface_and_bottom_detector();
    void set_surfaced(bool at_surface);
    void set_bottomed(bool at_bottom);
    void motors_output();
    Vector3f pv_location_to_vector(const Location& loc);
    float pv_alt_above_origin(float alt_above_home_cm);
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void init_joystick();
    void transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons);
    void handle_jsbutton_press(uint8_t button,bool shift=false,bool held=false);
    void handle_jsbutton_release(uint8_t button, bool shift);
    JSButton* get_button(uint8_t index);
    void default_js_buttons(void);
    void clear_input_hold();
    void read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok(void);
#if OPTFLOW == ENABLED
    void init_optflow();
#endif
    void terrain_update();
    void terrain_logging();
    void init_ardupilot() override;
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void startup_INS_ground();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    bool should_log(uint32_t mask);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);

    bool do_guided(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_surface(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);

    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_surface(const AP_Mission::Mission_Command& cmd);
    bool verify_RTL(void);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    void auto_spline_start(const Location& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location& next_destination);
    void log_init(void);
    void accel_cal_update(void);

    void failsafe_leak_check();
    void failsafe_internal_pressure_check();
    void failsafe_internal_temperature_check();

    void failsafe_terrain_act(void);
    bool auto_terrain_recover_start(void);
    void auto_terrain_recover_run(void);

    void translate_wpnav_rp(float &lateral_out, float &forward_out);
    void translate_circle_nav_rp(float &lateral_out, float &forward_out);
    void translate_pos_control_rp(float &lateral_out, float &forward_out);

    bool surface_init(void);
    void surface_run();

    uint16_t get_pilot_speed_dn();

    void convert_old_parameters(void);
    bool handle_do_motor_test(mavlink_command_long_t command);
    bool init_motor_test();
    bool verify_motor_test();

    uint32_t last_do_motor_test_fail_ms = 0;
    uint32_t last_do_motor_test_ms = 0;

    bool control_check_barometer();

    enum Failsafe_Action {
        Failsafe_Action_None    = 0,
        Failsafe_Action_Warn    = 1,
        Failsafe_Action_Disarm  = 2,
        Failsafe_Action_Surface = 3
    };

    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Disarm,
                                                      Failsafe_Action_Surface,
                                                      Failsafe_Action_Warn,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };

    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");


public:
    void mainloop_failsafe_check();
};

extern const AP_HAL::HAL& hal;
extern Sub sub;
