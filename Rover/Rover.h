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
#include <AP_AHRS/AP_AHRS.h>                        // ArduPilot Mega DCM Library
#include <AP_Airspeed/AP_Airspeed.h>                // needed for AHRS build
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>          // Battery monitor library
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Camera/AP_Camera.h>                    // Camera triggering
#include <AP_Compass/AP_Compass.h>                  // ArduPilot Mega Magnetometer Library
#include <AP_Declination/AP_Declination.h>          // Compass declination library
#include <AP_InertialSensor/AP_InertialSensor.h>    // Inertial Sensor (uncalibated IMU) Library
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_Math/AP_Math.h>                        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Mission/AP_Mission.h>                  // Mission command library
#include <AP_Mount/AP_Mount.h>                      // Camera/Antenna mount
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>          // Optical Flow library
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>          // Range finder library
#include <AP_RCMapper/AP_RCMapper.h>                // RC input mapping library
#include <AP_Scheduler/AP_Scheduler.h>              // main loop scheduler
#include <AP_Stats/AP_Stats.h>                      // statistics library
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Vehicle/AP_Vehicle.h>                  // needed for AHRS build
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AR_WPNav/AR_WPNav.h>
#include <AP_SmartRTL/AP_SmartRTL.h>
#include <AP_Logger/AP_Logger.h>
#include <Filter/AverageFilter.h>                   // Mode Filter from Filter library
#include <Filter/Butter.h>                          // Filter library - butterworth filter
#include <Filter/Filter.h>                          // Filter library
#include <Filter/LowPassFilter.h>
#include <Filter/ModeFilter.h>                      // Mode Filter from Filter library
#include <AC_Fence/AC_Fence.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include <AP_Follow/AP_Follow.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_WindVane/AP_WindVane.h>

#ifdef ENABLE_SCRIPTING
#include <AP_Scripting/AP_Scripting.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

// Local modules
#include "AP_MotorsUGV.h"
#include "mode.h"
#include "AP_Arming.h"
#include "sailboat.h"
// Configuration
#include "config.h"
#include "defines.h"
#if ADVANCED_FAILSAFE == ENABLED
#include "afs_rover.h"
#endif
#include "Parameters.h"
#include "GCS_Mavlink.h"
#include "GCS_Rover.h"
#include "AP_Rally.h"
#include "RC_Channel.h"                  // RC Channel Library

class Rover : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Rover;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Rally_Rover;
    friend class AP_Arming_Rover;
#if ADVANCED_FAILSAFE == ENABLED
    friend class AP_AdvancedFailsafe_Rover;
#endif
    friend class GCS_Rover;
    friend class Mode;
    friend class ModeAcro;
    friend class ModeAuto;
    friend class ModeGuided;
    friend class ModeHold;
    friend class ModeLoiter;
    friend class ModeSteering;
    friend class ModeManual;
    friend class ModeRTL;
    friend class ModeSmartRTL;
    friend class ModeFollow;
    friend class ModeSimple;

    friend class RC_Channel_Rover;
    friend class RC_Channels_Rover;

    friend class Sailboat;

    Rover(void);

private:

    // must be the first AP_Param variable declared to ensure its
    // constructor runs before the constructors of the other AP_Param
    // variables
    AP_Param param_loader;

    // all settable parameters
    Parameters g;
    ParametersG2 g2;

    // mapping between input channels
    RCMapper rcmap;

    // primary control channels
    RC_Channel *channel_steer;
    RC_Channel *channel_throttle;
    RC_Channel *channel_lateral;

    AP_Logger logger;

    // flight modes convenience array
    AP_Int8 *modes;
    const uint8_t num_modes = 6;

    // AP_RPM Module
    AP_RPM rpm_sensor;

    // Arming/Disarming management class
    AP_Arming_Rover arming;

    AP_L1_Control L1_controller{ahrs, nullptr};

#if AP_AHRS_NAVEKF_AVAILABLE
    OpticalFlow optflow;
#endif

#if OSD_ENABLED == ENABLED
    AP_OSD osd;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // GCS handling
    GCS_Rover _gcs;  // avoid using this; use gcs()
    GCS_Rover &gcs() { return _gcs; }

    // RC Channels:
    RC_Channels_Rover &rc() { return g2.rc_channels; }

    // The rover's current location
    struct Location current_loc;

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera{MASK_LOG_CAMERA, current_loc};
#endif

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    AP_Mount camera_mount;
#endif

    // true if initialisation has completed
    bool initialised;

    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, AUTO, ...
    Mode *control_mode;
    ModeReason control_mode_reason = ModeReason::UNKNOWN;

    // Used to maintain the state of the previous control switch position
    // This is set to -1 when we need to re-read the switch
    uint8_t oldSwitchPosition;

    // structure for holding failsafe state
    struct {
        uint8_t bits;               // bit flags of failsafes that have started (but not necessarily triggered an action)
        uint32_t start_time;        // start time of the earliest failsafe
        uint8_t triggered;          // bit flags of failsafes that have triggered an action
        uint32_t last_valid_rc_ms;  // system time of most recent RC input from pilot
        uint32_t last_heartbeat_ms; // system time of most recent heartbeat from ground station
        bool ekf;
    } failsafe;

    // true if we have a position estimate from AHRS
    bool have_position;

    // range finder last update (used for DPTH logging)
    uint32_t rangefinder_last_reading_ms;

    // Ground speed
    // The amount current ground speed is below min ground speed.  meters per second
    float ground_speed;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Rover::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    // flyforward timer
    uint32_t flyforward_start_ms;

    static const AP_Scheduler::Task scheduler_tasks[];

    static const AP_Param::Info var_info[];
    static const LogStructure log_structure[];

    // time that rudder/steering arming has been running
    uint32_t rudder_arm_timer;

    // latest wheel encoder values
    float wheel_encoder_last_distance_m[WHEELENCODER_MAX_INSTANCES];    // total distance recorded by wheel encoder (for reporting to GCS)
    bool wheel_encoder_initialised;                                     // true once arrays below have been initialised to sensors initial values
    float wheel_encoder_last_angle_rad[WHEELENCODER_MAX_INSTANCES];     // distance in radians at time of last update to EKF
    uint32_t wheel_encoder_last_reading_ms[WHEELENCODER_MAX_INSTANCES]; // system time of last ping from each encoder
    uint8_t wheel_encoder_last_index_sent;                              // index of the last wheel encoder sent to the EKF

    // True when we are doing motor test
    bool motor_test;

    ModeInitializing mode_initializing;
    ModeHold mode_hold;
    ModeManual mode_manual;
    ModeAcro mode_acro;
    ModeGuided mode_guided;
    ModeAuto mode_auto;
    ModeLoiter mode_loiter;
    ModeSteering mode_steering;
    ModeRTL mode_rtl;
    ModeSmartRTL mode_smartrtl;
    ModeFollow mode_follow;
    ModeSimple mode_simple;

    // cruise throttle and speed learning
    typedef struct {
        LowPassFilterFloat speed_filt = LowPassFilterFloat(2.0f);
        LowPassFilterFloat throttle_filt = LowPassFilterFloat(2.0f);
        uint32_t learn_start_ms;
        uint32_t log_count;
    } cruise_learn_t;
    cruise_learn_t cruise_learn;

private:

    // Rover.cpp
    bool set_target_location(const Location& target_loc) override;
    bool set_target_velocity_NED(const Vector3f& vel_ned) override;
    bool set_steering_and_throttle(float steering, float throttle) override;
    void stats_update();
    void ahrs_update();
    void gcs_failsafe_check(void);
    void update_logging1(void);
    void update_logging2(void);
    void one_second_loop(void);
    void update_current_mode(void);
    void update_mission(void);

    // balance_bot.cpp
    void balancebot_pitch_control(float &throttle);
    bool is_balancebot() const;

    // commands.cpp
    bool set_home_to_current_location(bool lock) WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) WARN_IF_UNUSED;
    void update_home();

    // compat.cpp
    void delay(uint32_t ms);

    // crash_check.cpp
    void crash_check();

    // cruise_learn.cpp
    void cruise_learn_start();
    void cruise_learn_update();
    void cruise_learn_complete();
    void log_write_cruise_learn();

    // ekf_check.cpp
    void ekf_check();
    bool ekf_over_threshold();
    bool ekf_position_ok();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);

    // failsafe.cpp
    void failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif

    // fence.cpp
    void fence_check();

    // GCS_Mavlink.cpp
    void send_servo_out(mavlink_channel_t chan);
    void send_wheel_encoder_distance(mavlink_channel_t chan);

    // Log.cpp
    void Log_Write_Attitude();
    void Log_Write_Depth();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Nav_Tuning();
    void Log_Write_Sail();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Steering();
    void Log_Write_Throttle();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
    void log_init(void);

    // mode.cpp
    Mode *mode_from_mode_num(enum Mode::Number num);

    // Parameters.cpp
    void load_parameters(void) override;

    // radio.cpp
    void set_control_channels(void) override;
    void init_rc_in();
    void rudder_arm_disarm_check();
    void read_radio();
    void radio_failsafe_check(uint16_t pwm);
    bool trim_radio();

    // sensors.cpp
    void update_compass(void);
    void compass_save(void);
    void update_wheel_encoder();
    void accel_cal_update(void);
    void read_rangefinders(void);
    void read_airspeed();
    void rpm_update(void);

    // Steering.cpp
    void set_servos(void);

    // Rover.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    // system.cpp
    void init_ardupilot() override;
    void startup_ground(void);
    void update_ahrs_flyforward();
    bool set_mode(Mode &new_mode, ModeReason reason);
    bool set_mode(const uint8_t new_mode, ModeReason reason) override;
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }
    bool mavlink_set_mode(uint8_t mode);
    void startup_INS_ground(void);
    void notify_mode(const Mode *new_mode);
    uint8_t check_digital_pin(uint8_t pin);
    bool should_log(uint32_t mask);
    bool is_boat() const;

#if OSD_ENABLED == ENABLED
    void publish_osd_info();
#endif

    enum Failsafe_Action {
        Failsafe_Action_None          = 0,
        Failsafe_Action_RTL           = 1,
        Failsafe_Action_Hold          = 2,
        Failsafe_Action_SmartRTL      = 3,
        Failsafe_Action_SmartRTL_Hold = 4,
        Failsafe_Action_Terminate     = 5
    };

    enum class Failsafe_Options : uint32_t {
        Failsafe_Option_Active_In_Hold = (1<<0)
    };

    static constexpr int8_t _failsafe_priorities[] = {
                                                       Failsafe_Action_Terminate,
                                                       Failsafe_Action_Hold,
                                                       Failsafe_Action_RTL,
                                                       Failsafe_Action_SmartRTL_Hold,
                                                       Failsafe_Action_SmartRTL,
                                                       Failsafe_Action_None,
                                                       -1 // the priority list must end with a sentinel of -1
                                                      };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");


public:
    void failsafe_check();
    // Motor test
    void motor_test_output();
    bool mavlink_motor_test_check(const GCS_MAVLINK &gcs_chan, bool check_rc, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value);
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value, float timeout_sec);
    void motor_test_stop();

    // frame type
    uint8_t get_frame_type() { return g2.frame_type.get(); }
    AP_WheelRateControl& get_wheel_rate_control() { return g2.wheel_rate_control; }

    // Simple mode
    float simple_sin_yaw;
};

extern Rover rover;

using AP_HAL::millis;
using AP_HAL::micros;
