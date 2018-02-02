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
  This is the main Copter class
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
#include <AP_Mission/AP_Mission.h>     // Mission command library
#include <AC_PID/AC_P.h>               // P library
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PI  library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)
#include <AC_PID/AC_HELI_PID.h>        // Heli specific Rate PID library
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AC_AttitudeControl/AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <RC_Channel/RC_Channel.h>         // RC Channel Library
#include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_Beacon/AP_Beacon.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <AP_RSSI/AP_RSSI.h>                   // RSSI Library
#include <Filter/Filter.h>             // Filter library
#include <AP_Buffer/AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay/AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed/AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav/AC_WPNav.h>           // ArduCopter waypoint navigation library
#include <AC_WPNav/AC_Circle.h>          // circle navigation library
#include <AP_Declination/AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence/AC_Fence.h>           // Arducopter Fence library
#include <AC_Avoidance/AC_Avoid.h>           // Arducopter stop at fence library
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Scheduler/PerfInfo.h>       // loop perf monitoring
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
#include <AP_Terrain/AP_Terrain.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_RPM/AP_RPM.h>
#include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
#include <AC_InputManager/AC_InputManager_Heli.h>   // Heli specific pilot input handling library
#include <AP_Button/AP_Button.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_SmartRTL/AP_SmartRTL.h>
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <AP_Winch/AP_Winch.h>

// Configuration
#include "defines.h"
#include "config.h"

#include "GCS_Mavlink.h"
#include "GCS_Copter.h"
#include "AP_Rally.h"           // Rally point library
#include "AP_Arming.h"

// libraries which are dependent on #defines in defines.h and/or config.h
#if SPRAYER == ENABLED
#include <AC_Sprayer/AC_Sprayer.h>         // crop sprayer library
#endif
#if GRIPPER_ENABLED == ENABLED
#include <AP_Gripper/AP_Gripper.h>             // gripper stuff
#endif
#if PARACHUTE == ENABLED
#include <AP_Parachute/AP_Parachute.h>       // Parachute release library
#endif
#if PRECISION_LANDING == ENABLED
#include <AC_PrecLand/AC_PrecLand.h>
#include <AP_IRLock/AP_IRLock.h>
#endif
#if FRSKY_TELEM_ENABLED == ENABLED
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#endif

#if ADVANCED_FAILSAFE == ENABLED
#include "afs_copter.h"
#endif

// Local modules
#include "Parameters.h"
#include "avoidance_adsb.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif


class Copter : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class AP_Rally_Copter;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Avoidance_Copter;
#if ADVANCED_FAILSAFE == ENABLED
    friend class AP_AdvancedFailsafe_Copter;
#endif
    friend class AP_Arming_Copter;

    Copter(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    static const AP_FWVersion fwver;

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::MultiCopter aparm;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;

    // AP_Notify instance
    AP_Notify notify;

    // used to detect MAVLink acks from GCS to stop compassmot
    uint8_t command_ack_counter;

    // primary input control channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;

    // Dataflash
    DataFlash_Class DataFlash;

    AP_GPS gps;

    // flight modes convenience array
    AP_Int8 *flight_modes;

    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;

    RangeFinder rangefinder{serial_manager, ROTATION_PITCH_270};
    struct {
        bool enabled:1;
        bool alt_healthy:1; // true if we can trust the altitude from the rangefinder
        int16_t alt_cm;     // tilt compensated altitude (in cm) from rangefinder
        uint32_t last_healthy_ms;
        LowPassFilterFloat alt_cm_filt; // altitude filter
        int8_t glitch_count;
    } rangefinder_state = { false, false, 0, 0 };

    AP_RPM rpm_sensor;

    // Inertial Navigation EKF
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
    AP_AHRS_NavEKF ahrs{ins, barometer, EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Mission library
    AP_Mission mission{ahrs,
            FUNCTOR_BIND_MEMBER(&Copter::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Copter::exit_mission, void)};

    bool start_command(const AP_Mission::Mission_Command& cmd) {
        return mode_auto.start_command(cmd);
    }
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) {
        return mode_auto.verify_command_callback(cmd);
    }
    void exit_mission() {
        mode_auto.exit_mission();
    }

    // Arming/Disarming mangement class
    AP_Arming_Copter arming{ahrs, barometer, compass, battery, inertial_nav, ins};

    // Optical flow sensor
#if OPTFLOW == ENABLED
    OpticalFlow optflow{ahrs};
#endif

    // gnd speed limit required to observe optical flow sensor limits
    float ekfGndSpdLimit;

    // scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
    float ekfNavVelGainScaler;

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms = 0;
    int8_t ekf_primary_core;

    AP_SerialManager serial_manager;

    // GCS selection
    GCS_Copter _gcs; // avoid using this; use gcs()
    GCS_Copter &gcs() { return _gcs; }

    // User variables
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    // Documentation of GLobals:
    typedef union {
        struct {
            uint8_t unused1                 : 1; // 0
            uint8_t simple_mode             : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
            uint8_t pre_arm_rc_check        : 1; // 3       // true if rc input pre-arm checks have been completed successfully
            uint8_t pre_arm_check           : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
            uint8_t auto_armed              : 1; // 5       // stops auto missions from beginning until throttle is raised
            uint8_t logging_started         : 1; // 6       // true if dataflash logging has started
            uint8_t land_complete           : 1; // 7       // true if we have detected a landing
            uint8_t new_radio_frame         : 1; // 8       // Set true if we have new PWM data to act on from the Radio
            uint8_t usb_connected           : 1; // 9       // true if APM is powered from USB connection
            uint8_t rc_receiver_present     : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
            uint8_t compass_mot             : 1; // 11      // true if we are currently performing compassmot calibration
            uint8_t motor_test              : 1; // 12      // true if we are currently performing the motors test
            uint8_t initialised             : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
            uint8_t land_complete_maybe     : 1; // 14      // true if we may have landed (less strict version of land_complete)
            uint8_t throttle_zero           : 1; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
            uint8_t system_time_set         : 1; // 16      // true if the system time has been set from the GPS
            uint8_t gps_glitching           : 1; // 17      // true if the gps is glitching
            enum HomeState home_state       : 2; // 18,19   // home status (unset, set, locked)
            uint8_t using_interlock         : 1; // 20      // aux switch motor interlock function is in use
            uint8_t motor_emergency_stop    : 1; // 21      // motor estop switch, shuts off motors when enabled
            uint8_t land_repo_active        : 1; // 22      // true if the pilot is overriding the landing position
            uint8_t motor_interlock_switch  : 1; // 23      // true if pilot is requesting motor interlock enable
            uint8_t in_arming_delay         : 1; // 24      // true while we are armed but waiting to spin motors
            uint8_t initialised_params      : 1; // 25      // true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
            uint8_t compass_init_location   : 1; // 26      // true when the compass's initial location has been set
            uint8_t rc_override_enable      : 1; // 27      // aux switch rc_override is allowed
        };
        uint32_t value;
    } ap_t;

    ap_t ap;

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    control_mode_t control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;

    // Structure used to detect changes in the flight mode control switch
    struct {
        int8_t debounced_switch_position;   // currently used switch position
        int8_t last_switch_position;        // switch position in previous iteration
        uint32_t last_edge_time_ms;         // system time that switch position was last changed
    } control_switch_state;

    typedef struct {
        bool running;
        float max_speed;
        float alt_delta;
        uint32_t start_ms;
    } takeoff_state_t;
    takeoff_state_t takeoff_state;

    // altitude below which we do no navigation in auto takeoff
    float auto_takeoff_no_nav_alt_cm;

    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

#if HAL_WITH_UAVCAN
    // board specific config for CAN bus
    AP_BoardConfig_CAN BoardConfig_CAN;
#endif

    // receiver RSSI
    uint8_t receiver_rssi;

    // Failsafe
    struct {
        uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
        uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
        uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
        uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // 5   // true if ekf failsafe has occurred
        uint8_t terrain             : 1; // 6   // true if the missing terrain data failsafe has occurred
        uint8_t adsb                : 1; // 7   // true if an adsb related failsafe has occurred

        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed
    } failsafe;

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
        uint8_t primary_gps;        // primary gps index
    } sensor_health;

    // Motor Output
#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#else
 #define MOTOR_CLASS AP_MotorsMulticopter
#endif

    MOTOR_CLASS *motors;
    const struct AP_Param::GroupInfo *motors_var_info;

    // GPS variables
    // Sometimes we need to remove the scaling for distance calcs
    float scaleLongDown;

    int32_t _home_bearing;
    uint32_t _home_distance;

    // SIMPLE Mode
    // Used to track the orientation of the vehicle for Simple mode. This value is reset at each arming
    // or in SuperSimple mode when the vehicle leaves a 20m radius from home.
    float simple_cos_yaw;
    float simple_sin_yaw;
    int32_t super_simple_last_bearing;
    float super_simple_cos_yaw;
    float super_simple_sin_yaw;

    // Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
    int32_t initial_armed_bearing;

    // Battery Sensors
    AP_BattMonitor battery;

#if FRSKY_TELEM_ENABLED == ENABLED
    // FrSky telemetry support
    AP_Frsky_Telem frsky_telemetry{ahrs, battery, rangefinder};
#endif

    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;   // desired altitude in cm above the ground
    int32_t baro_alt;            // barometer altitude in cm above home
    float baro_climbrate;        // barometer climbrate in cm/s
    LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

    // filtered pilot's throttle input used to cancel landing if throttle held high
    LowPassFilterFloat rc_throttle_control_in_filter;

    // loop performance monitoring:
    AP::PerfInfo perf_info;

    // 3D Location vectors
    // Current location of the vehicle (altitude is relative to home)
    Location_Class current_loc;

    // Navigation Yaw control
    // auto flight mode's yaw mode
    uint8_t auto_yaw_mode;

    // Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
    Vector3f roi_WP;

    // bearing from current location to the yaw_look_at_WP
    float yaw_look_at_WP_bearing;

    // yaw used for YAW_LOOK_AT_HEADING yaw_mode
    int32_t yaw_look_at_heading;

    // Deg/s we should turn
    int16_t yaw_look_at_heading_slew;

    // heading when in yaw_look_ahead_bearing
    float yaw_look_ahead_bearing;

    // turn rate (in cds) when auto_yaw_mode is set to AUTO_YAW_RATE
    float auto_yaw_rate_cds;

    // IMU variables
    // Integration time (in seconds) for the gyros (DCM algorithm)
    // Updated with the fast loop
    float G_Dt;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav;

    // Attitude, Position and Waypoint navigation objects
    // To-Do: move inertial nav up or other navigation variables down here
#if FRAME_CONFIG == HELI_FRAME
    #define AC_AttitudeControl_t AC_AttitudeControl_Heli
#else
    #define AC_AttitudeControl_t AC_AttitudeControl_Multi
#endif
    AC_AttitudeControl_t *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Circle *circle_nav;

    // Performance monitoring
    int16_t pmTest1;

    // System Timers
    // --------------
    // Time in microseconds of main control loop
    uint32_t fast_loopTimer;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;
    // arm_time_ms - Records when vehicle was armed. Will be Zero if we are disarmed.
    uint32_t arm_time_ms;

    // Used to exit the roll and pitch auto trim function
    uint8_t auto_trim_counter;

    // Reference to the relay object
    AP_Relay relay;

    // handle repeated servo and relay events
    AP_ServoRelayEvents ServoRelayEvents{relay};

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera{&relay, MASK_LOG_CAMERA, current_loc, ahrs};
#endif

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    AP_Mount camera_mount{ahrs, current_loc};
#endif

    // AC_Fence library to reduce fly-aways
#if AC_FENCE == ENABLED
    AC_Fence fence{ahrs};
#endif

#if AC_AVOID_ENABLED == ENABLED
    AC_Avoid avoid{ahrs, fence, g2.proximity, &g2.beacon};
#endif

    // Rally library
#if AC_RALLY == ENABLED
    AP_Rally_Copter rally{ahrs};
#endif

    // RSSI
    AP_RSSI rssi;

    // Crop Sprayer
#if SPRAYER == ENABLED
    AC_Sprayer sprayer{&inertial_nav};
#endif

    // Parachute release
#if PARACHUTE == ENABLED
    AP_Parachute parachute{relay};
#endif

    // Landing Gear Controller
    AP_LandingGear landinggear;

    // terrain handling
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    AP_Terrain terrain{ahrs, mission, rally};
#endif

    // Precision Landing
#if PRECISION_LANDING == ENABLED
    AC_PrecLand precland{ahrs, inertial_nav};
#endif

    // Pilot Input Management Library
    // Only used for Helicopter for now
#if FRAME_CONFIG == HELI_FRAME
    AC_InputManager_Heli input_manager;
#endif

    AP_ADSB adsb{ahrs};

    // avoidance of adsb enabled vehicles (normally manned vehicles)
    AP_Avoidance_Copter avoidance_adsb{ahrs, adsb};

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    // last valid RC input time
    uint32_t last_radio_update_ms;

    // last esc calibration notification update
    uint32_t esc_calibration_notify_update_ms;

#if VISUAL_ODOMETRY_ENABLED == ENABLED
    // last visual odometry update time
    uint32_t visual_odom_last_update_ms;
#endif

    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;

#if FRAME_CONFIG == HELI_FRAME
    // Mode filter to reject RC Input glitches.  Filter size is 5, and it draws the 4th element, so it can reject 3 low glitches,
    // and 1 high glitch.  This is because any "off" glitches can be highly problematic for a helicopter running an ESC
    // governor.  Even a single "off" frame can cause the rotor to slow dramatically and take a long time to restart.
    ModeFilterInt16_Size5 rotor_speed_deglitch_filter {4};

    // Tradheli flags
    typedef struct {
        uint8_t dynamic_flight          : 1;    // 0   // true if we are moving at a significant speed (used to turn on/off leaky I terms)
        uint8_t init_targets_on_arming  : 1;    // 1   // true if we have been disarmed, and need to reset rate controller targets when we arm
        uint8_t inverted_flight         : 1;    // 2   // true for inverted flight mode
    } heli_flags_t;
    heli_flags_t heli_flags;

    int16_t hover_roll_trim_scalar_slew;
#endif

    // ground effect detector
    struct {
        bool takeoff_expected;
        bool touchdown_expected;
        uint32_t takeoff_time_ms;
        float takeoff_alt_cm;
    } gndeffect_state;

    // set when we are upgrading parameters from 3.4
    bool upgrading_frame_params;
    
    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // AP_State.cpp
    void set_home_state(enum HomeState new_home_state);
    bool home_is_set();
    void set_auto_armed(bool b);
    void set_simple_mode(uint8_t b);
    void set_failsafe_radio(bool b);
    void set_failsafe_battery(bool b);
    void set_failsafe_gcs(bool b);
    void update_using_interlock();
    void set_motor_emergency_stop(bool b);

    // ArduCopter.cpp
    void perf_update(void);
    void stats_update();
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_mount();
    void update_trigger(void);
    void update_batt_compass(void);
    void fourhundred_hz_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void dataflash_periodic(void);
    void ins_periodic();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void smart_rtl_save_position();
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();

    // Attitude.cpp
    float get_smoothing_gain();
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max, float angle_limit);
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
    float get_roi_yaw();
    float get_look_ahead_yaw();
    void update_throttle_hover();
    void set_throttle_takeoff();
    float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f);
    float get_pilot_desired_climb_rate(float throttle_control);
    float get_non_takeoff_throttle();
    float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt);
    float get_avoidance_adjusted_climbrate(float target_rate);
    void set_accel_throttle_I_from_pilot_throttle();
    void rotate_body_frame_to_NE(float &x, float &y);
    uint16_t get_pilot_speed_dn();

    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);

    // baro_ground_effect.cpp
    void update_ground_effect_detector(void);

    // capabilities.cpp
    void init_capabilities(void);

    // commands.cpp
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock);
    bool set_home(const Location& loc, bool lock);
    void set_ekf_origin(const Location& loc);
    bool far_from_EKF_origin(const Location& loc);
    void set_system_time_from_GPS();

    // compassmot.cpp
    MAV_RESULT mavlink_compassmot(mavlink_channel_t chan);

    // compat.cpp
    void delay(uint32_t ms);

    // crash_check.cpp
    void crash_check();
    void parachute_check();
    void parachute_release();
    void parachute_manual_release();

    // ekf_check.cpp
    void ekf_check();
    bool ekf_check_position_problem();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void check_ekf_reset();

    // esc_calibration.cpp
    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    void esc_calibration_notify();

    // events.cpp
    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void failsafe_battery_event(void);
    void failsafe_gcs_check();
    void failsafe_gcs_off_event(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void gpsglitch_check();
    void set_mode_RTL_or_land_with_pause(mode_reason_t reason);
    void set_mode_SmartRTL_or_RTL(mode_reason_t reason);
    void set_mode_SmartRTL_or_land_with_pause(mode_reason_t reason);
    bool should_disarm_on_failsafe();
    void update_events();

    // failsafe.cpp
    void failsafe_enable();
    void failsafe_disable();
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif

    // fence.cpp
    void fence_check();
    void fence_send_mavlink_status(mavlink_channel_t chan);

    // GCS_Mavlink.cpp
    void gcs_send_heartbeat(void);
    void gcs_send_deferred(void);
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_fence_status(mavlink_channel_t chan);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_rpm(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void gcs_data_stream_send(void);
    void gcs_check_input(void);

    // heli.cpp
    void heli_init();
    void check_dynamic_flight(void);
    void update_heli_control_dynamics(void);
    void heli_update_landing_swash();
    void heli_update_rotor_speed_targets();

    // inertia.cpp
    void read_inertia();

    // landing_detector.cpp
    void update_land_and_crash_detectors();
    void update_land_detector();
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    void update_throttle_thr_mix();

    // landing_gear.cpp
    void landinggear_update();

    // leds.cpp
    void update_notify();

    // Log.cpp
    void Log_Write_Current();
    void Log_Write_Optflow();
    void Log_Write_Nav_Tuning();
    void Log_Write_Control_Tuning();
    void Log_Write_Performance();
    void Log_Write_Attitude();
    void Log_Write_EKF_POS();
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
#if FRAME_CONFIG == HELI_FRAME
    void Log_Write_Heli(void);
#endif
    void Log_Write_Precland();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Throw(ThrowModeStage stage, float velocity, float velocity_z, float accel, float ef_accel_z, bool throw_detect, bool attitude_ok, bool height_ok, bool position_ok);
    void Log_Write_Proximity();
    void Log_Write_Beacon();
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // mode.cpp
    bool set_mode(control_mode_t mode, mode_reason_t reason);
    void update_flight_mode();
    void notify_flight_mode();

    // mode_auto.cpp
    uint8_t get_default_auto_yaw_mode(bool rtl);
    void set_auto_yaw_mode(uint8_t yaw_mode);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle);
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_rate(float turn_rate_cds);
    float get_auto_heading(void);
    float get_auto_yaw_rate_cds();

    // mode_land.cpp
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horizontal_control();
    void set_mode_land_with_pause(mode_reason_t reason);
    bool landing_with_GPS();

    // motor_test.cpp
    void motor_test_output();
    bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc);
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec, uint8_t motor_count);
    void motor_test_stop();

    // motors.cpp
    void arm_motors_check();
    void auto_disarm_check();
    bool init_arm_motors(bool arming_from_gcs);
    void init_disarm_motors();
    void motors_output();
    void lost_vehicle_check();

    // navigation.cpp
    void run_nav_updates(void);
    int32_t home_bearing();
    uint32_t home_distance();

    // Parameters.cpp
    void load_parameters(void);
    void convert_pid_parameters(void);

    // position_vector.cpp
    Vector3f pv_location_to_vector(const Location& loc);
    float pv_alt_above_origin(float alt_above_home_cm);
    float pv_alt_above_home(float alt_above_origin_cm);
    float pv_distance_to_home_cm(const Vector3f &destination);

    // precision_landing.cpp
    void init_precland();
    void update_precland();

    // radio.cpp
    void default_dead_zones();
    void init_rc_in();
    void init_rc_out();
    void enable_motor_output();
    void read_radio();
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    void set_throttle_zero_flag(int16_t throttle_control);
    void radio_passthrough_to_motors();

    // sensors.cpp
    void init_barometer(bool full_calibration);
    void read_barometer(void);
    void barometer_accumulate(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok();
    void rpm_update();
    void init_compass();
    void compass_accumulate(void);
    void init_optflow();
    void update_optical_flow(void);
    void read_battery(void);
    void read_receiver_rssi(void);
    void compass_cal_update(void);
    void accel_cal_update(void);
    void gripper_update();
    void button_update();
    void init_proximity();
    void update_proximity();
    void update_sensor_status_flags(void);
    void init_beacon();
    void update_beacon();
    void init_visual_odom();
    void update_visual_odom();
    void winch_init();
    void winch_update();

    // setup.cpp
    void report_compass();
    void print_blanks(int16_t num);
    void print_divider(void);
    void print_enabled(bool b);
    void report_version();

    // switches.cpp
    void read_control_switch();
    bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check);
    bool check_duplicate_auxsw(void);
    void reset_control_switch();
    uint8_t read_3pos_switch(uint8_t chan);
    void read_aux_switches();
    void init_aux_switches();
    void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag);
    void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag);
    void save_trim();
    void auto_trim();

    // system.cpp
    void init_ardupilot();
    void startup_INS_ground();
    bool calibrate_gyros();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    void update_auto_armed();
    void check_usb_mux(void);
    bool should_log(uint32_t mask);
    void set_default_frame_class();
    uint8_t get_frame_mav_type();
    const char* get_frame_string();
    void allocate_motors(void);

    // takeoff.cpp
    bool current_mode_has_user_takeoff(bool must_navigate);
    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    void takeoff_timer_start(float alt_cm);
    void takeoff_stop();
    void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate);
    void auto_takeoff_set_start_alt(void);
    void auto_takeoff_attitude_run(float target_yaw_rate);

    // terrain.cpp
    void terrain_update();
    void terrain_logging();
    bool terrain_use();

    // tuning.cpp
    void tuning();

    // UserCode.cpp
    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();

#include "mode.h"

    Mode *flightmode;
#if FRAME_CONFIG == HELI_FRAME
    ModeAcro_Heli mode_acro{*this};
#else
    ModeAcro mode_acro{*this};
#endif
    ModeAltHold mode_althold{*this};
    ModeAuto mode_auto{*this, mission, circle_nav};
#if AUTOTUNE_ENABLED == ENABLED
    ModeAutoTune mode_autotune{*this};
#endif
    ModeBrake mode_brake{*this};
    ModeCircle mode_circle{*this, circle_nav};
    ModeDrift mode_drift{*this};
    ModeFlip mode_flip{*this};
    ModeGuided mode_guided{*this};
    ModeLand mode_land{*this};
    ModeLoiter mode_loiter{*this};
    ModePosHold mode_poshold{*this};
    ModeRTL mode_rtl{*this};
#if FRAME_CONFIG == HELI_FRAME
    ModeStabilize_Heli mode_stabilize{*this};
#else
    ModeStabilize mode_stabilize{*this};
#endif
    ModeSport mode_sport{*this};
    ModeAvoidADSB mode_avoid_adsb{*this};
    ModeThrow mode_throw{*this};
    ModeGuidedNoGPS mode_guided_nogps{*this};
    ModeSmartRTL mode_smartrtl{*this};

    // mode.cpp
    Mode *mode_from_mode_num(const uint8_t mode);
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

public:
    void mavlink_delay_cb();    // GCS_Mavlink.cpp
    void failsafe_check();      // failsafe.cpp
};

extern const AP_HAL::HAL& hal;
extern Copter copter;

using AP_HAL::millis;
using AP_HAL::micros;
