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
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

// Application dependencies
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_GPS/AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash/DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC/AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>         // Mission command library
#include <AP_Rally/AP_Rally.h>           // Rally point library
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PID library (2-axis)
#include <AC_PID/AC_P.h>               // P library
#include <AC_AttitudeControl/AC_AttitudeControl_Sub.h> // Attitude control library
#include <AC_AttitudeControl/AC_PosControl_Sub.h>      // Position control library
#include <RC_Channel/RC_Channel.h>         // RC Channel Library
#include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <Filter/Filter.h>             // Filter library
#include <AP_Buffer/AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay/AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed/AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav/AC_WPNav.h>           // Waypoint navigation library
#include <AC_WPNav/AC_Circle.h>          // circle navigation library
#include <AP_Declination/AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence/AC_Fence.h>           // Fence library

#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
#include <AP_JSButton/AP_JSButton.h>   // Joystick/gamepad button function assignment
#include "../libraries/AP_LeakDetector/AP_LeakDetector.h" // Leak detector
#include <AP_TemperatureSensor/TSYS01.h>
#include "defines.h"
#include "config.h"

#include "GCS_Mavlink.h"

// libraries which are dependent on #defines in defines.h and/or config.h
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

// Local modules
#include "Parameters.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

class Sub : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_Sub;
    friend class Parameters;
    friend class ParametersG2;

    Sub(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;


    // cliSerial isn't strictly necessary - it is an alias for hal.console. It may
    // be deprecated in favor of hal.console in later releases.
    AP_HAL::BetterStream* cliSerial;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;

    // AP_Notify instance
    AP_Notify notify;

    // used to detect MAVLink acks from GCS to stop compassmot
    uint8_t command_ack_counter;

    // has a log download started?
    bool in_log_download;

    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;
    RC_Channel *channel_forward;
    RC_Channel *channel_lateral;

    // Dataflash
    DataFlash_Class DataFlash;

    AP_GPS gps;

    AP_LeakDetector leak_detector;

    // flight modes convenience array
    AP_Int8 *flight_modes;

    TSYS01 celsius;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;

    RangeFinder rangefinder {serial_manager};
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

    // Inertial Navigation EKF
    NavEKF2 EKF2 {&ahrs, barometer, rangefinder};
    NavEKF3 EKF3 {&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs {ins, barometer, gps, rangefinder, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Mission library
    AP_Mission mission;

    // Optical flow sensor
#if OPTFLOW == ENABLED
    OpticalFlow optflow {ahrs};
#endif

    // gnd speed limit required to observe optical flow sensor limits
    float ekfGndSpdLimit;

    // scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
    float ekfNavVelGainScaler;

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms = 0;

    // GCS selection
    AP_SerialManager serial_manager;
    static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;

    GCS_MAVLINK_Sub gcs[MAVLINK_COMM_NUM_BUFFERS];

    // User variables
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    // Documentation of GLobals:
    union {
        struct {
            uint8_t unused1             : 1; // 0
            uint8_t simple_mode         : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
            uint8_t pre_arm_rc_check    : 1; // 3       // true if rc input pre-arm checks have been completed successfully
            uint8_t pre_arm_check       : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t auto_armed          : 1; // 5       // stops auto missions from beginning until throttle is raised
            uint8_t logging_started     : 1; // 6       // true if dataflash logging has started
            uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
            uint8_t usb_connected       : 1; // 9       // true if APM is powered from USB connection
            uint8_t rc_receiver_present : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
            uint8_t compass_mot         : 1; // 11      // true if we are currently performing compassmot calibration
            uint8_t motor_test          : 1; // 12      // true if we are currently performing the motors test
            uint8_t initialised         : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t throttle_zero       : 1; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
            uint8_t system_time_set     : 1; // 16      // true if the system time has been set from the GPS
            uint8_t gps_base_pos_set    : 1; // 17      // true when the gps base position has been set (used for RTK gps only)
            enum HomeState home_state   : 2; // 18,19   // home status (unset, set, locked)
            uint8_t using_interlock     : 1; // 20      // aux switch motor interlock function is in use
            uint8_t motor_emergency_stop: 1; // 21      // motor estop switch, shuts off motors when enabled
            uint8_t land_repo_active    : 1; // 22      // true if the pilot is overriding the landing position
            uint8_t at_bottom           : 1;            // true if we are at the bottom
            uint8_t at_surface          : 1;            // true if we are at the surface
            uint8_t depth_sensor_present: 1;            // true if we have an external baro connected
        };
        uint32_t value;
    } ap;

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    control_mode_t control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;

#if RCMAP_ENABLED == ENABLED
    RCMapper rcmap;
#endif

    // board specific config
    AP_BoardConfig BoardConfig;

    // Failsafe
    struct {
        uint8_t rc_override_active   : 1; // 0   // true if rc control are overwritten by ground station
        uint8_t manual_control       : 1; // 1   // A status flag for the radio failsafe
        uint8_t battery              : 1; // 2   // A status flag for the battery failsafe
        uint8_t gcs                  : 1; // 4   // A status flag for the ground station failsafe
        uint8_t ekf                  : 1; // 5   // true if ekf failsafe has occurred
        uint8_t terrain              : 1; // 6   // true if the missing terrain data failsafe has occurred
        uint8_t leak                 : 1; // true if leak recently detected
        uint8_t internal_pressure    : 1; // true if internal pressure is over threshold
        uint8_t internal_temperature : 1; // true if temperature is over threshold
        uint32_t last_leak_warn_ms;      // last time a leak warning was sent to gcs
        uint32_t last_gcs_warn_ms;

        uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
        uint32_t last_manual_control_ms; // last time MANUAL_CONTROL message arrived from GCS
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed
    } failsafe;

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
    } sensor_health;

    AP_Motors6DOF motors;

    // GPS variables
    // Sometimes we need to remove the scaling for distance calcs
    float scaleLongDown;

    // Location & Navigation
    int32_t wp_bearing;
    // The location of home in relation to the Sub in centi-degrees
    int32_t home_bearing;
    // distance between plane and home in cm
    int32_t home_distance;
    // distance between plane and next waypoint in cm.
    uint32_t wp_distance;
    uint8_t land_state;              // records state of land (flying to location, descending)

    // Auto
    AutoMode auto_mode;   // controls which auto controller is run

    // Guided
    GuidedMode guided_mode;  // controls which controller is run (pos or vel)

    // Circle
    bool circle_pilot_yaw_override; // true if pilot is overriding yaw

    // SIMPLE Mode
    // Used to track the orientation of the Sub for Simple mode. This value is reset at each arming
    // or in SuperSimple mode when the Sub leaves a 20m radius from home.
    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;

    // Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
    int32_t initial_armed_bearing;

    // Throttle variables
    int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    // Delay the next navigation command
    int32_t nav_delay_time_max;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start;

    // Battery Sensors
    AP_BattMonitor battery;

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;      // desired altitude in cm above the ground
    int32_t baro_alt;            // barometer altitude in cm above home
    float baro_climbrate;        // barometer climbrate in cm/s
    LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

    // Turn counter
    int32_t quarter_turn_count;
    uint8_t last_turn_state;

    // filtered pilot's throttle input used to cancel landing if throttle held high
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D Location vectors
    // Current location of the Sub (altitude is relative to home)
    Location_Class current_loc;

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

    // IMU variables
    // Integration time (in seconds) for the gyros (DCM algorithm)
    // Updated with the fast loop
    float G_Dt;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
    AC_AttitudeControl_Sub attitude_control;

    AC_PosControl_Sub pos_control;

#if AVOIDANCE_ENABLED == ENABLED
    AC_Avoid avoid;
#endif

    AC_WPNav wp_nav;
    AC_Circle circle_nav;

    // Performance monitoring
    int16_t pmTest1;

    // System Timers
    // --------------
    // Time in microseconds of main control loop
    uint32_t fast_loopTimer;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;

    // Used to exit the roll and pitch auto trim function
    uint8_t auto_trim_counter;

    // Reference to the relay object
    AP_Relay relay;

    // handle repeated servo and relay events
    AP_ServoRelayEvents ServoRelayEvents;

    // Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
    AP_Camera camera;
#endif

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    AP_Mount camera_mount;
#endif

    // AC_Fence library to reduce fly-aways
#if AC_FENCE == ENABLED
    AC_Fence    fence;
#endif

    // Rally library
#if AC_RALLY == ENABLED
    AP_Rally rally;
#endif

    // terrain handling
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    AP_Terrain terrain;
#endif

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    // true if we are out of time in our event timeslice
    bool gcs_out_of_time;

    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;

    uint32_t last_pilot_heading;
    uint32_t last_pilot_yaw_input_ms;
    uint32_t fs_terrain_recover_start_ms = 0;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    void compass_accumulate(void);
    void compass_cal_update(void);
    void barometer_accumulate(void);
    void perf_update(void);
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_mount();
    void update_trigger();
    void update_batt_compass(void);
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void update_turn_counter();
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();
    void set_home_state(enum HomeState new_home_state);
    bool home_is_set();
    void set_auto_armed(bool b);
    void set_simple_mode(uint8_t b);
    void set_failsafe_battery(bool b);
    void set_pre_arm_check(bool b);
    void set_pre_arm_rc_check(bool b);
    void update_using_interlock();
    void set_motor_emergency_stop(bool b);
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
    void gcs_send_heartbeat(void);
    void gcs_send_deferred(void);
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_limits_status(mavlink_channel_t chan);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_hwstatus(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_radio_out(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_current_waypoint(mavlink_channel_t chan);
    void send_rangefinder(mavlink_channel_t chan);
#if RPM_ENABLED == ENABLED
    void send_rpm(mavlink_channel_t chan);
    void rpm_update();
#endif
    void send_temperature(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void gcs_send_message(enum ap_message id);
    void gcs_send_mission_item_reached_message(uint16_t mission_index);
    void gcs_data_stream_send(void);
    void gcs_check_input(void);
    void gcs_send_text(MAV_SEVERITY severity, const char *str);
    void do_erase_logs(void);
#if AUTOTUNE_ENABLED == ENABLED
    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
#endif
    void Log_Write_Current();
    void Log_Write_Optflow();
    void Log_Write_Nav_Tuning();
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_MotBatt();
    void Log_Write_Event(uint8_t id);
    void Log_Write_Data(uint8_t id, int32_t value);
    void Log_Write_Data(uint8_t id, uint32_t value);
    void Log_Write_Data(uint8_t id, int16_t value);
    void Log_Write_Data(uint8_t id, uint16_t value);
    void Log_Write_Data(uint8_t id, float value);
    void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
    void Log_Write_Baro(void);
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high);
    void Log_Write_Home_And_Origin();
    void Log_Sensor_Health();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
    void start_logging() ;
    void load_parameters(void);
    void convert_pid_parameters(void);
    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location();
    bool set_home_to_current_location_and_lock();
    bool set_home_and_lock(const Location& loc);
    bool set_home(const Location& loc);
    bool far_from_EKF_origin(const Location& loc);
    void set_system_time_from_GPS();
    void exit_mission();
    bool verify_land();
    bool verify_loiter_unlimited();
    bool verify_loiter_time();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    void do_take_picture();
    void log_picture();
    uint8_t mavlink_compassmot(mavlink_channel_t chan);
    bool acro_init(bool ignore_checks);
    void acro_run();
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);
    bool althold_init(bool ignore_checks);
    void althold_run();
    bool auto_init(bool ignore_checks);
    void auto_run();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location_Class& dest_loc);
    void auto_wp_run();
    void auto_spline_run();
    void auto_land_start();
    void auto_land_start(const Vector3f& destination);
    void auto_land_run();
    void auto_circle_movetoedge_start(const Location_Class &circle_center, float radius_m);
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
#if AUTOTUNE_ENABLED == ENABLED
    bool autotune_init(bool ignore_checks);
    void autotune_stop();
    bool autotune_start(bool ignore_checks);
    void autotune_run();
    void autotune_attitude_control();
    void autotune_backup_gains_and_initialise();
    void autotune_load_orig_gains();
    void autotune_load_tuned_gains();
    void autotune_load_intra_test_gains();
    void autotune_load_twitch_gains();
    void autotune_save_tuning_gains();
    void autotune_update_gcs(uint8_t message_id);
    bool autotune_roll_enabled();
    bool autotune_pitch_enabled();
    bool autotune_yaw_enabled();
    void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max);
    void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
#endif
    bool circle_init(bool ignore_checks);
    void circle_run();
    bool guided_init(bool ignore_checks);
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location_Class& dest_loc);
    void guided_set_velocity(const Vector3f& velocity);
    void guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
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
    float get_land_descent_speed();
    bool velhold_init(bool ignore_checks);
    void velhold_run();
    bool poshold_init(bool ignore_checks);
    void poshold_run();

    bool transect_init(bool ignore_checks);
    void transect_run();
    bool stabilize_init(bool ignore_checks);
    void stabilize_run();
    bool manual_init(bool ignore_checks);
    void manual_run();
    void crash_check();
    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    bool should_disarm_on_failsafe();
    void failsafe_battery_event(void);
    void failsafe_gcs_check();
    void failsafe_manual_control_check(void);
    void set_neutral_controls(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void update_events();
    void failsafe_enable();
    void failsafe_disable();
    void fence_check();
    void fence_send_mavlink_status(mavlink_channel_t chan);
    bool set_mode(control_mode_t mode, mode_reason_t reason);
    bool gcs_set_mode(uint8_t mode) {
        return set_mode((control_mode_t)mode, MODE_REASON_GCS_COMMAND);
    }
    void update_flight_mode();
    void exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode);
    bool mode_requires_GPS(control_mode_t mode);
    bool mode_has_manual_throttle(control_mode_t mode);
    bool mode_allows_arming(control_mode_t mode, bool arming_from_gcs);
    void notify_flight_mode(control_mode_t mode);
    void check_dynamic_flight(void);
    void read_inertia();
    void read_inertial_altitude();
    void update_surface_and_bottom_detector();
    void set_surfaced(bool at_surface);
    void set_bottomed(bool at_bottom);
    void update_notify();
    void motor_test_output();
    bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc);
    uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec);
    void motor_test_stop();
    void auto_disarm_check();
    bool init_arm_motors(bool arming_from_gcs);
    void update_arming_checks(void);
    bool all_arming_checks_passing(bool arming_from_gcs);
    bool pre_arm_checks(bool display_failure);
    void pre_arm_rc_checks();
    bool pre_arm_gps_checks(bool display_failure);
    bool pre_arm_ekf_attitude_check();
    bool pre_arm_rallypoint_check();
    bool pre_arm_terrain_check(bool display_failure);
    bool arm_checks(bool display_failure, bool arming_from_gcs);
    void init_disarm_motors();
    void motors_output();
    void lost_vehicle_check();
    void run_nav_updates(void);
    void calc_position();
    void calc_distance_and_bearing();
    void calc_wp_distance();
    void calc_wp_bearing();
    void calc_home_distance_and_bearing();
    void run_autopilot();
    void perf_info_reset();
    void perf_ignore_this_loop();
    void perf_info_check_loop_time(uint32_t time_in_micros);
    uint16_t perf_info_get_num_loops();
    uint32_t perf_info_get_max_time();
    uint32_t perf_info_get_min_time();
    uint16_t perf_info_get_num_long_running();
    uint32_t perf_info_get_num_dropped();
    Vector3f pv_location_to_vector(const Location& loc);
    float pv_alt_above_origin(float alt_above_home_cm);
    float pv_alt_above_home(float alt_above_origin_cm);
    float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination);
    float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination);
    float pv_distance_to_home_cm(const Vector3f &destination);
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void init_joystick();
    void transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons);
    void handle_jsbutton_press(uint8_t button,bool shift=false,bool held=false);
    void camera_tilt_smooth();
    JSButton* get_button(uint8_t index);
    void default_js_buttons(void);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors();
    void init_barometer(bool save);
    void read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok(void);
    void init_compass();
    void init_optflow();
    void update_optical_flow(void);
    void read_battery(void);
    void gripper_update();
    void terrain_update();
    void terrain_logging();
    bool terrain_use();
    void report_batt_monitor();
    void report_frame();
    void report_radio();
    void report_ins();
    void report_flight_modes();
    void report_optflow();
    void print_radio_values();
    void print_switch(uint8_t p, uint8_t m, bool b);
    void print_accel_offsets_and_scaling(void);
    void print_gyro_offsets(void);
    void report_compass();
    void print_blanks(int16_t num);
    void print_divider(void);
    void print_enabled(bool b);
    void report_version();
    bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
    bool check_duplicate_auxsw(void);
    uint8_t read_3pos_switch(int16_t radio_in);
    void read_aux_switches();
    void init_aux_switches();
    void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
    void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
    void save_trim();
    void auto_trim();
    void init_ardupilot();
    void startup_INS_ground();
    bool calibrate_gyros();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    void update_auto_armed();
    void check_usb_mux(void);
    bool should_log(uint32_t mask);
    void print_hit_enter();
#if CH6_TUNE_ENABLED == ENABLED
    void tuning();
#endif
    void gcs_send_text_fmt(MAV_SEVERITY severity, const char *fmt, ...);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);

    bool do_guided(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
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
#if CAMERA == ENABLED
    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);
#endif
#if GRIPPER_ENABLED == ENABLED
    void do_gripper(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    void auto_spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);
    void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);
    void log_init(void);
    void run_cli(AP_HAL::UARTDriver *port);
    void init_capabilities(void);
    void dataflash_periodic(void);
    void accel_cal_update(void);

    void set_leak_status(bool status);
    void failsafe_internal_pressure_check();
    void failsafe_internal_temperature_check();

    void failsafe_terrain_act(void);
    bool auto_terrain_recover_start(void);
    void auto_terrain_recover_run(void);

    void translate_wpnav_rp(float &lateral_out, float &forward_out);

    bool surface_init(bool ignore_flags);
    void surface_run();

    void convert_old_parameters(void);

public:
    void mavlink_delay_cb();
    void failsafe_check();
    int8_t dump_log(uint8_t argc, const Menu::arg *argv);
    int8_t erase_logs(uint8_t argc, const Menu::arg *argv);
    int8_t select_logs(uint8_t argc, const Menu::arg *argv);
    bool print_log_menu(void);

    int8_t process_logs(uint8_t argc, const Menu::arg *argv);
    int8_t main_menu_help(uint8_t, const Menu::arg*);
    int8_t setup_mode(uint8_t argc, const Menu::arg *argv);
    int8_t setup_factory(uint8_t argc, const Menu::arg *argv);
    int8_t setup_set(uint8_t argc, const Menu::arg *argv);
    int8_t setup_show(uint8_t argc, const Menu::arg *argv);
    int8_t esc_calib(uint8_t argc, const Menu::arg *argv);

    int8_t test_mode(uint8_t argc, const Menu::arg *argv);
    int8_t test_baro(uint8_t argc, const Menu::arg *argv);
    int8_t test_compass(uint8_t argc, const Menu::arg *argv);
    int8_t test_ins(uint8_t argc, const Menu::arg *argv);
    int8_t test_optflow(uint8_t argc, const Menu::arg *argv);
    int8_t test_relay(uint8_t argc, const Menu::arg *argv);
    int8_t test_shell(uint8_t argc, const Menu::arg *argv);
    int8_t test_rangefinder(uint8_t argc, const Menu::arg *argv);

    int8_t reboot_board(uint8_t argc, const Menu::arg *argv);
};

#define MENU_FUNC(func) FUNCTOR_BIND(&sub, &Sub::func, int8_t, uint8_t, const Menu::arg *)

extern const AP_HAL::HAL& hal;
extern Sub sub;

using AP_HAL::millis;
using AP_HAL::micros;
