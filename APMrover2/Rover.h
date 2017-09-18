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
/*
   main Rover class, containing all vehicle specific state
*/
#pragma once

#include <cmath>
#include <stdarg.h>

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID.h>
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_ADC/AP_ADC.h>                          // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AHRS/AP_AHRS.h>                        // ArduPilot Mega DCM Library
#include <AP_Airspeed/AP_Airspeed.h>                // needed for AHRS build
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>          // Battery monitor library
#include <AP_Beacon/AP_Beacon.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Buffer/AP_Buffer.h>                    // FIFO buffer library
#include <AP_Button/AP_Button.h>
#include <AP_Camera/AP_Camera.h>                    // Camera triggering
#include <AP_Compass/AP_Compass.h>                  // ArduPilot Mega Magnetometer Library
#include <AP_Declination/AP_Declination.h>          // Compass declination library
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_GPS/AP_GPS.h>                          // ArduPilot GPS library
#include <AP_InertialSensor/AP_InertialSensor.h>    // Inertial Sensor (uncalibated IMU) Library
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_Math/AP_Math.h>                        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Menu/AP_Menu.h>
#include <AP_Mission/AP_Mission.h>                  // Mission command library
#include <AP_Mount/AP_Mount.h>                      // Camera/Antenna mount
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_Notify/AP_Notify.h>                    // Notify library
#include <AP_OpticalFlow/AP_OpticalFlow.h>          // Optical Flow library
#include <AP_Param/AP_Param.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_RangeFinder/AP_RangeFinder.h>          // Range finder library
#include <AP_RCMapper/AP_RCMapper.h>                // RC input mapping library
#include <AP_Relay/AP_Relay.h>                      // APM relay
#include <AP_RSSI/AP_RSSI.h>                        // RSSI Library
#include <AP_Scheduler/AP_Scheduler.h>              // main loop scheduler
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Stats/AP_Stats.h>                      // statistics library
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Vehicle/AP_Vehicle.h>                  // needed for AHRS build
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <DataFlash/DataFlash.h>
#include <Filter/AverageFilter.h>                   // Mode Filter from Filter library
#include <Filter/Butter.h>                          // Filter library - butterworth filter
#include <Filter/Filter.h>                          // Filter library
#include <Filter/LowPassFilter.h>
#include <Filter/ModeFilter.h>                      // Mode Filter from Filter library
#include <RC_Channel/RC_Channel.h>                  // RC Channel Library
#include <StorageManager/StorageManager.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

// Local modules
#include "AP_MotorsUGV.h"
#include "mode.h"
#include "AP_Arming.h"
// Configuration
#include "config.h"
#include "defines.h"
#if ADVANCED_FAILSAFE == ENABLED
#include "afs_rover.h"
#endif
#include "Parameters.h"
#include "GCS_Mavlink.h"
#include "GCS_Rover.h"
#include "version.h"

class Rover : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_Rover;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Rover;
#if ADVANCED_FAILSAFE == ENABLED
    friend class AP_AdvancedFailsafe_Rover;
#endif
    friend class GCS_Rover;
    friend class Mode;
    friend class ModeAuto;
    friend class ModeGuided;
    friend class ModeHold;
    friend class ModeSteering;
    friend class ModeManual;
    friend class ModeRTL;

    Rover(void);

    // HAL::Callbacks implementation.
    void setup(void) override;
    void loop(void) override;

private:

    const AP_FWVersion fwver {
        major: FW_MAJOR,
        minor: FW_MINOR,
        patch: FW_PATCH,
        fw_type: FW_TYPE,
#ifndef GIT_VERSION
        fw_string: THISFIRMWARE
#else
        fw_string: THISFIRMWARE " (" GIT_VERSION ")"
#endif
    };

    // must be the first AP_Param variable declared to ensure its
    // constructor runs before the constructors of the other AP_Param
    // variables
    AP_Param param_loader;

    // all settable parameters
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;

    // mapping between input channels
    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

#if HAL_WITH_UAVCAN
    // board specific config for CAN bus
    AP_BoardConfig_CAN BoardConfig_CAN;
#endif

    // primary control channels
    RC_Channel *channel_steer;
    RC_Channel *channel_throttle;
    RC_Channel *channel_aux;

    DataFlash_Class DataFlash;

    // sensor drivers
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    RangeFinder rangefinder { serial_manager, ROTATION_NONE };
    AP_Button button;

    // flight modes convenience array
    AP_Int8 *modes;

    // Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs {ins, barometer, gps, rangefinder, EKF2, EKF3};
#else
    AP_AHRS_DCM ahrs {ins, barometer, gps};
#endif

    // Arming/Disarming management class
    AP_Arming_Rover arming {ahrs, barometer, compass, battery};

    AP_L1_Control L1_controller;

    // selected navigation controller
    AP_Navigation *nav_controller;

    // Mission library
    AP_Mission mission;

#if AP_AHRS_NAVEKF_AVAILABLE
    OpticalFlow optflow{ahrs};
#endif

    // RSSI
    AP_RSSI rssi;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // GCS handling
    AP_SerialManager serial_manager;
    GCS_Rover _gcs;  // avoid using this; use gcs()
    GCS_Rover &gcs() { return _gcs; }

    // relay support
    AP_Relay relay;

    AP_ServoRelayEvents ServoRelayEvents;

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera;
#endif

    // The rover's current location
    struct Location current_loc;

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps solution for altitude rather than gps only.
    AP_Mount camera_mount;
#endif

    // true if initialisation has completed
    bool initialised;

    // if USB is connected
    bool usb_connected;

    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, AUTO, ...
    Mode *control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_INITIALISED;

    // Used to maintain the state of the previous control switch position
    // This is set to -1 when we need to re-read the switch
    uint8_t oldSwitchPosition;

    // These are values received from the GCS if the user is using GCS joystick
    // control and are substituted for the values coming from the RC radio
    int16_t rc_override[8];

    // A flag if GCS joystick control is in use
    bool rc_override_active;

    // Failsafe
    // A tracking variable for type of failsafe active
    // Used for failsafe based on loss of RC signal or GCS signal. See
    // FAILSAFE_EVENT_*
    struct {
        uint8_t bits;
        uint32_t rc_override_timer;
        uint32_t start_time;
        uint8_t triggered;
        uint32_t last_valid_rc_ms;
    } failsafe;

    // notification object for LEDs, buzzers etc (parameter set to false disables external leds)
    AP_Notify notify;

    // true if we have a position estimate from AHRS
    bool have_position;

    // receiver RSSI
    uint8_t receiver_rssi;

    // the time when the last HEARTBEAT message arrived from a GCS
    uint32_t last_heartbeat_ms;

    // obstacle detection information
    struct {
        // have we detected an obstacle?
        uint8_t detected_count;
        float turn_angle;
        uint16_t rangefinder1_distance_cm;
        uint16_t rangefinder2_distance_cm;

        // time when we last detected an obstacle, in milliseconds
        uint32_t detected_time_ms;
    } obstacle;

    // Ground speed
    // The amount current ground speed is below min ground speed.  meters per second
    float ground_speed;

    // CH7 auxiliary switches last known position
    aux_switch_pos aux_ch7;

    // Battery Sensors
    AP_BattMonitor battery;

#if FRSKY_TELEM_ENABLED == ENABLED
    // FrSky telemetry support
    AP_Frsky_Telem frsky_telemetry;
#endif

    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;
    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    int32_t condition_start;

    // 3D Location vectors
    // Location structure defined in AP_Common
    // The home location used for RTL.  The location is set when we first get stable GPS lock
    const struct Location &home;

    // Flag for if we have g_gps lock and have set the home location in AHRS
    enum HomeState home_is_set = HOME_UNSET;

    // true if the system time has been set from the GPS
    bool system_time_set;

    // true if the compass's initial location has been set
    bool compass_init_location;

    // IMU variables
    // The main loop execution time.  Seconds
    // This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
    float G_Dt;

    // Performance monitoring
    // Timer used to accrue data and trigger recording of the performance monitoring log message
    int32_t perf_mon_timer;
    // The maximum main loop execution time, in microseconds, recorded in the current performance monitoring interval
    uint32_t G_Dt_max;

    // System Timers
    // Time in microseconds of start of main control loop.
    uint32_t fast_loopTimer_us;
    // Number of milliseconds used in last main loop cycle
    uint32_t delta_us_fast_loop;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;

    // set if we are driving backwards
    bool in_reverse;

    // true if pivoting (set by use_pivot_steering)
    bool pivot_steering_active;

    static const AP_Scheduler::Task scheduler_tasks[];

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    static const AP_Param::Info var_info[];
    static const LogStructure log_structure[];

    // Loiter control
    uint16_t loiter_duration;       // How long we should loiter at the nav_waypoint (time in seconds)
    uint32_t loiter_start_time;     // How long have we been loitering - The start time in millis
    bool previously_reached_wp;     // set to true if we have EVER reached the waypoint

    // time that rudder/steering arming has been running
    uint32_t rudder_arm_timer;

    // Store the time the last GPS message was received.
    uint32_t last_gps_msg_ms{0};

    // last visual odometry update time
    uint32_t visual_odom_last_update_ms;

    // last wheel encoder update times
    float wheel_encoder_last_angle_rad[WHEELENCODER_MAX_INSTANCES];     // distance in radians at time of last update to EKF
    uint32_t wheel_encoder_last_update_ms[WHEELENCODER_MAX_INSTANCES];  // system time of last ping from each encoder
    uint32_t wheel_encoder_last_ekf_update_ms;                          // system time of last encoder data push to EKF
    float wheel_encoder_rpm[WHEELENCODER_MAX_INSTANCES];                // for reporting to GCS

    // True when we are doing motor test
    bool motor_test;

    ModeInitializing mode_initializing;
    ModeHold mode_hold;
    ModeManual mode_manual;
    ModeGuided mode_guided;
    ModeAuto mode_auto;
    ModeSteering mode_steering;
    ModeRTL mode_rtl;

    // cruise throttle and speed learning
    struct {
        bool learning;
        LowPassFilterFloat speed_filt = LowPassFilterFloat(2.0f);
        LowPassFilterFloat throttle_filt = LowPassFilterFloat(2.0f);
    } cruise_learn;

private:

    // APMrover2.cpp
    void stats_update();
    void ahrs_update();
    void mount_update(void);
    void update_trigger(void);
    void update_alt();
    void gcs_failsafe_check(void);
    void update_compass(void);
    void update_logging1(void);
    void update_logging2(void);
    void update_aux(void);
    void one_second_loop(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_current_mode(void);

    // capabilities.cpp
    void init_capabilities(void);

    // commands_logic.cpp
    void update_mission(void);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd, bool stay_active_at_dest);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_nav_set_yaw_speed(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_RTL();
    bool verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_set_yaw_speed();
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool verify_wait_delay();
    bool verify_within_distance();
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
#if CAMERA == ENABLED
    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);
#endif
    void do_set_reverse(const AP_Mission::Mission_Command& cmd);

    // commands.cpp
    void update_home_from_EKF();
    bool set_home_to_current_location(bool lock);
    bool set_home(const Location& loc, bool lock);
    void set_ekf_origin(const Location& loc);
    void set_system_time_from_GPS();
    void update_home();

    // compat.cpp
    void delay(uint32_t ms);

    // control_modes.cpp
    Mode *control_mode_from_num(enum mode num);
    void read_control_switch();
    uint8_t readSwitch(void);
    void reset_control_switch();
    aux_switch_pos read_aux_switch_pos();
    void init_aux_switch();
    void read_aux_switch();
    bool motor_active();

    // crash_check.cpp
    void crash_check();

    // cruise_learn.cpp
    void cruise_learn_start();
    void cruise_learn_update();
    void cruise_learn_complete();

    // events.cpp
    void update_events(void);

    // failsafe.cpp
    void failsafe_trigger(uint8_t failsafe_type, bool on);
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif

    // GCS_Mavlink.cpp
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_rangefinder(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void send_wheel_encoder(mavlink_channel_t chan);
    void gcs_data_stream_send(void);
    void gcs_update(void);
    void gcs_retry_deferred(void);

    // Log.cpp
    void Log_Write_Performance();
    void Log_Write_Steering();
    void Log_Write_Beacon();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Control_Tuning();
    void Log_Write_Nav_Tuning();
    void Log_Write_Attitude();
    void Log_Write_Rangefinder();
    void Log_Write_Current();
    void Log_Arm_Disarm();
    void Log_Write_RC(void);
    void Log_Write_Error(uint8_t sub_system, uint8_t error_code);
    void Log_Write_Baro(void);
    void Log_Write_Home_And_Origin();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_WheelEncoder();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
    void log_init(void);
    void Log_Write_Vehicle_Startup_Messages();

    // Parameters.cpp
    void load_parameters(void);

    // radio.cpp
    void set_control_channels(void);
    void init_rc_in();
    void init_rc_out();
    void rudder_arm_disarm_check();
    void read_radio();
    void control_failsafe(uint16_t pwm);
    void trim_control_surfaces();
    void trim_radio();

    // sensors.cpp
    void init_compass(void);
    void compass_accumulate(void);
    void init_barometer(bool full_calibration);
    void init_rangefinder(void);
    void init_beacon();
    void update_beacon();
    void init_visual_odom();
    void update_visual_odom();
    void update_wheel_encoder();
    void read_battery(void);
    void read_receiver_rssi(void);
    void compass_cal_update(void);
    void accel_cal_update(void);
    void read_rangefinders(void);
    void button_update(void);
    void update_sensor_status_flags(void);

    // Steering.cpp
    bool use_pivot_steering(float yaw_error_cd);
    void set_servos(void);

    // system.cpp
    void init_ardupilot();
    void startup_ground(void);
    void set_reverse(bool reverse);
    bool set_mode(Mode &new_mode, mode_reason_t reason);
    bool mavlink_set_mode(uint8_t mode);
    void startup_INS_ground(void);
    void update_notify();
    void resetPerfData(void);
    void check_usb_mux(void);
    void print_mode(AP_HAL::BetterStream *port, uint8_t mode);
    void notify_mode(enum mode new_mode);
    uint8_t check_digital_pin(uint8_t pin);
    bool should_log(uint32_t mask);
    void change_arm_state(void);
    bool arm_motors(AP_Arming::ArmingMethod method);
    bool disarm_motors(void);

    // test.cpp
    void print_hit_enter();
    void print_enabled(bool b);

public:
    void mavlink_delay_cb();
    void failsafe_check();

    void dataflash_periodic(void);
    void update_soft_armed();
    // Motor test
    void motor_test_output();
    bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc, uint8_t motor_seq, uint8_t throttle_type, int16_t throttle_value);
    uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, int16_t throttle_value, float timeout_sec);
    void motor_test_stop();
};

extern const AP_HAL::HAL& hal;
extern Rover rover;

using AP_HAL::millis;
using AP_HAL::micros;
