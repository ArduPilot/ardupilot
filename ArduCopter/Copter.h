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
#include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_RSSI/AP_RSSI.h>                   // RSSI Library
#include <Filter/Filter.h>             // Filter library
#include <AP_Relay/AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Airspeed/AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav/AC_WPNav.h>           // ArduCopter waypoint navigation library
#include <AC_WPNav/AC_Loiter.h>
#include <AC_WPNav/AC_Circle.h>          // circle navigation library
#include <AP_Declination/AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
#include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
#include <AC_InputManager/AC_InputManager_Heli.h>   // Heli specific pilot input handling library
#include <AP_Button/AP_Button.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_SmartRTL/AP_SmartRTL.h>
#include <AP_TempCalibration/AP_TempCalibration.h>
#include <AC_AutoTune/AC_AutoTune.h>

// Configuration
#include "defines.h"
#include "config.h"

#include "RC_Channel.h"         // RC Channel Library

#include "GCS_Mavlink.h"
#include "GCS_Copter.h"
#include "AP_Rally.h"           // Rally point library
#include "AP_Arming.h"

// libraries which are dependent on #defines in defines.h and/or config.h
#if BEACON_ENABLED == ENABLED
 #include <AP_Beacon/AP_Beacon.h>
#endif
#if AC_AVOID_ENABLED == ENABLED
 #include <AC_Avoidance/AC_Avoid.h>
#endif
#if SPRAYER_ENABLED == ENABLED
 # include <AC_Sprayer/AC_Sprayer.h>
#endif
#if GRIPPER_ENABLED == ENABLED
 # include <AP_Gripper/AP_Gripper.h>
#endif
#if PARACHUTE == ENABLED
 # include <AP_Parachute/AP_Parachute.h>
#endif
#if PRECISION_LANDING == ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
 # include <AP_IRLock/AP_IRLock.h>
#endif
#if FRSKY_TELEM_ENABLED == ENABLED
 # include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#endif
#if ADSB_ENABLED == ENABLED
 # include <AP_ADSB/AP_ADSB.h>
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
 # include <AP_Follow/AP_Follow.h>
#endif
#if AC_FENCE == ENABLED
 # include <AC_Fence/AC_Fence.h>
#endif
#if AC_TERRAIN == ENABLED
 # include <AP_Terrain/AP_Terrain.h>
#endif
#if OPTFLOW == ENABLED
 # include <AP_OpticalFlow/AP_OpticalFlow.h>
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
 # include <AP_VisualOdom/AP_VisualOdom.h>
#endif
#if RANGEFINDER_ENABLED == ENABLED
 # include <AP_RangeFinder/AP_RangeFinder.h>
#endif
#if PROXIMITY_ENABLED == ENABLED
 # include <AP_Proximity/AP_Proximity.h>
#endif
#if MOUNT == ENABLED
 #include <AP_Mount/AP_Mount.h>
#endif
#if CAMERA == ENABLED
 # include <AP_Camera/AP_Camera.h>
#endif

#if DEVO_TELEM_ENABLED == ENABLED
 #include <AP_Devo_Telem/AP_Devo_Telem.h>
#endif

#if OSD_ENABLED == ENABLED
 #include <AP_OSD/AP_OSD.h>
#endif

#if ADVANCED_FAILSAFE == ENABLED
 # include "afs_copter.h"
#endif
#if TOY_MODE_ENABLED == ENABLED
 # include "toy_mode.h"
#endif
#if WINCH_ENABLED == ENABLED
 # include <AP_WheelEncoder/AP_WheelEncoder.h>
 # include <AP_Winch/AP_Winch.h>
#endif
#if RPM_ENABLED == ENABLED
 #include <AP_RPM/AP_RPM.h>
#endif

// Local modules
#ifdef USER_PARAMS_ENABLED
#include "UserParameters.h"
#endif
#include "Parameters.h"
#if ADSB_ENABLED == ENABLED
#include "avoidance_adsb.h"
#endif

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
    friend class ToyMode;
    friend class RC_Channel_Copter;
    friend class RC_Channels_Copter;

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
    AP_Scheduler scheduler{FUNCTOR_BIND_MEMBER(&Copter::fast_loop, void)};

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
    const uint8_t num_flight_modes = 6;

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

#if RPM_ENABLED == ENABLED
    AP_RPM rpm_sensor;
#endif

    // Inertial Navigation EKF
    NavEKF2 EKF2{&ahrs, rangefinder};
    NavEKF3 EKF3{&ahrs, rangefinder};
    AP_AHRS_NavEKF ahrs{EKF2, EKF3, AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    AP_AHRS_View *ahrs_view;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Arming/Disarming mangement class
    AP_Arming_Copter arming;

    // Optical flow sensor
#if OPTFLOW == ENABLED
    OpticalFlow optflow;
#endif

    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms;
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
            uint8_t motor_emergency_stop    : 1; // 21      // motor estop switch, shuts off motors when enabled
            uint8_t land_repo_active        : 1; // 22      // true if the pilot is overriding the landing position
            uint8_t motor_interlock_switch  : 1; // 23      // true if pilot is requesting motor interlock enable
            uint8_t in_arming_delay         : 1; // 24      // true while we are armed but waiting to spin motors
            uint8_t initialised_params      : 1; // 25      // true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
            uint8_t compass_init_location   : 1; // 26      // true when the compass's initial location has been set
            uint8_t unused2                 : 1; // 27      // aux switch rc_override is allowed
            uint8_t armed_with_switch       : 1; // 28      // we armed using a arming switch
        };
        uint32_t value;
    } ap_t;

    ap_t ap;

    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t must be uint32_t");

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    control_mode_t control_mode;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;

    control_mode_t prev_control_mode;
    mode_reason_t prev_control_mode_reason = MODE_REASON_UNKNOWN;

    RCMapper rcmap;

    // intertial nav alt when we armed
    float arming_altitude_m;

    // board specific config
    AP_BoardConfig BoardConfig;

#if HAL_WITH_UAVCAN
    // board specific config for CAN bus
    AP_BoardConfig_CAN BoardConfig_CAN;
#endif

    // Failsafe
    struct {
        uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed

        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint8_t radio               : 1; // A status flag for the radio failsafe
        uint8_t gcs                 : 1; // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // true if ekf failsafe has occurred
        uint8_t terrain             : 1; // true if the missing terrain data failsafe has occurred
        uint8_t adsb                : 1; // true if an adsb related failsafe has occurred
    } failsafe;

    bool any_failsafe_triggered() const {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf || failsafe.terrain || failsafe.adsb;
    }

    // sensor health for logging
    struct {
        uint8_t baro        : 1;    // true if baro is healthy
        uint8_t compass     : 1;    // true if compass is healthy
        uint8_t primary_gps : 2;    // primary gps index
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
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Copter::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

#if FRSKY_TELEM_ENABLED == ENABLED
    // FrSky telemetry support
    AP_Frsky_Telem frsky_telemetry{ahrs, battery, rangefinder};
#endif
#if DEVO_TELEM_ENABLED == ENABLED
    AP_DEVO_Telem devo_telemetry{ahrs};
#endif

#if OSD_ENABLED == ENABLED
    AP_OSD osd;
#endif
    
    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // Altitude
    // The cm/s we are moving up or down based on filtered data - Positive = UP
    int16_t climb_rate;
    float target_rangefinder_alt;   // desired altitude in cm above the ground
    bool target_rangefinder_alt_used; // true if mode is using target_rangefinder_alt
    int32_t baro_alt;            // barometer altitude in cm above home
    LowPassFilterVector3f land_accel_ef_filter; // accelerations for land and crash detector tests

    // filtered pilot's throttle input used to cancel landing if throttle held high
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D Location vectors
    // Current location of the vehicle (altitude is relative to home)
    Location current_loc;

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
    AC_Loiter *loiter_nav;
#if MODE_CIRCLE_ENABLED == ENABLED
    AC_Circle *circle_nav;
#endif

    // System Timers
    // --------------
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
    // current_loc uses the baro/gps solution for altitude rather than gps only.
    AP_Mount camera_mount{current_loc};
#endif

    // AC_Fence library to reduce fly-aways
#if AC_FENCE == ENABLED
    AC_Fence fence{ahrs};
#endif

#if AC_AVOID_ENABLED == ENABLED
# if BEACON_ENABLED == ENABLED
    AC_Avoid avoid{ahrs, fence, g2.proximity, &g2.beacon};
# else
    AC_Avoid avoid{ahrs, fence, g2.proximity};
# endif
#endif

    // Rally library
#if AC_RALLY == ENABLED
    AP_Rally_Copter rally{ahrs};
#endif

    // RSSI
    AP_RSSI rssi;

    // Crop Sprayer
#if SPRAYER_ENABLED == ENABLED
    AC_Sprayer sprayer;
#endif

    // Parachute release
#if PARACHUTE == ENABLED
    AP_Parachute parachute{relay};
#endif

    // Landing Gear Controller
    AP_LandingGear landinggear;

    // terrain handling
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN && MODE_AUTO_ENABLED == ENABLED
    AP_Terrain terrain{mode_auto.mission};
#endif

    // Precision Landing
#if PRECISION_LANDING == ENABLED
    AC_PrecLand precland;
#endif

    // Pilot Input Management Library
    // Only used for Helicopter for now
#if FRAME_CONFIG == HELI_FRAME
    AC_InputManager_Heli input_manager;
#endif

#if ADSB_ENABLED == ENABLED
    AP_ADSB adsb;

    // avoidance of adsb enabled vehicles (normally manned vehicles)
    AP_Avoidance_Copter avoidance_adsb{ahrs, adsb};
#endif

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

    enum Failsafe_Action {
        Failsafe_Action_None           = 0,
        Failsafe_Action_Land           = 1,
        Failsafe_Action_RTL            = 2,
        Failsafe_Action_SmartRTL       = 3,
        Failsafe_Action_SmartRTL_Land  = 4,
        Failsafe_Action_Terminate      = 5
    };

    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_SmartRTL_Land,
                                                      Failsafe_Action_SmartRTL,
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
    void set_simple_mode(uint8_t b);
    void set_failsafe_radio(bool b);
    void set_failsafe_gcs(bool b);
    void update_using_interlock();
    void set_motor_emergency_stop(bool b);

    // ArduCopter.cpp
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_batt_compass(void);
    void fourhundred_hz_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void update_GPS(void);
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();

    // Attitude.cpp
    float get_pilot_desired_yaw_rate(int16_t stick_angle);
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

#if ADSB_ENABLED == ENABLED
    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);
#endif

    // baro_ground_effect.cpp
    void update_ground_effect_detector(void);

    // capabilities.cpp
    void init_capabilities(void);

    // commands.cpp
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock);
    bool set_home(const Location& loc, bool lock);
    bool far_from_EKF_origin(const Location& loc);

    // compassmot.cpp
    MAV_RESULT mavlink_compassmot(mavlink_channel_t chan);

    // crash_check.cpp
    void crash_check();
    void thrust_loss_check();
    void parachute_check();
    void parachute_release();
    void parachute_manual_release();

    // ekf_check.cpp
    void ekf_check();
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
    void handle_battery_failsafe(const char* type_str, const int8_t action);
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
    void send_fence_status(mavlink_channel_t chan);
    void send_sys_status(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_rpm(mavlink_channel_t chan);

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

    // Log.cpp
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
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, int16_t control_in, int16_t tune_low, int16_t tune_high);
    void Log_Sensor_Health();
#if FRAME_CONFIG == HELI_FRAME
    void Log_Write_Heli(void);
#endif
    void Log_Write_Precland();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // mode.cpp
    bool set_mode(control_mode_t mode, mode_reason_t reason);
    void update_flight_mode();
    void notify_flight_mode();

    // mode_land.cpp
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
    bool init_arm_motors(AP_Arming::ArmingMethod method, bool do_arming_checks=true);
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
    void convert_lgr_parameters(void);

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
    int16_t get_throttle_mid(void);

    // sensors.cpp
    void read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok();
    void rpm_update();
    void init_compass();
    void init_compass_location();
    void init_optflow();
    void update_optical_flow(void);
    void compass_cal_update(void);
    void accel_cal_update(void);
    void init_proximity();
    void update_proximity();
    void update_sensor_status_flags(void);
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
    void save_trim();
    void auto_trim();

    // system.cpp
    void init_ardupilot();
    void startup_INS_ground();
    bool position_ok();
    bool ekf_position_ok();
    bool optflow_position_ok();
    void update_auto_armed();
    bool should_log(uint32_t mask);
    void set_default_frame_class();
    MAV_TYPE get_frame_mav_type();
    const char* get_frame_string();
    void allocate_motors(void);
    bool is_tradheli() const;

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
    void userhook_auxSwitch1(uint8_t ch_flag);
    void userhook_auxSwitch2(uint8_t ch_flag);
    void userhook_auxSwitch3(uint8_t ch_flag);

#if OSD_ENABLED == ENABLED
    void publish_osd_info();
#endif

#include "mode.h"

    Mode *flightmode;
#if MODE_ACRO_ENABLED == ENABLED
#if FRAME_CONFIG == HELI_FRAME
    ModeAcro_Heli mode_acro;
#else
    ModeAcro mode_acro;
#endif
#endif
    ModeAltHold mode_althold;
#if MODE_AUTO_ENABLED == ENABLED
    ModeAuto mode_auto;
#endif
#if AUTOTUNE_ENABLED == ENABLED
    AutoTune autotune;
    ModeAutoTune mode_autotune;
#endif
#if MODE_BRAKE_ENABLED == ENABLED
    ModeBrake mode_brake;
#endif
#if MODE_CIRCLE_ENABLED == ENABLED
    ModeCircle mode_circle;
#endif
#if MODE_DRIFT_ENABLED == ENABLED
    ModeDrift mode_drift;
#endif
#if MODE_FLIP_ENABLED == ENABLED
    ModeFlip mode_flip;
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    ModeFollow mode_follow;
#endif
#if MODE_GUIDED_ENABLED == ENABLED
    ModeGuided mode_guided;
#endif
    ModeLand mode_land;
#if MODE_LOITER_ENABLED == ENABLED
    ModeLoiter mode_loiter;
#endif
#if MODE_POSHOLD_ENABLED == ENABLED
    ModePosHold mode_poshold;
#endif
#if MODE_RTL_ENABLED == ENABLED
    ModeRTL mode_rtl;
#endif
#if FRAME_CONFIG == HELI_FRAME
    ModeStabilize_Heli mode_stabilize;
#else
    ModeStabilize mode_stabilize;
#endif
#if MODE_SPORT_ENABLED == ENABLED
    ModeSport mode_sport;
#endif
#if ADSB_ENABLED == ENABLED
    ModeAvoidADSB mode_avoid_adsb;
#endif
#if MODE_THROW_ENABLED == ENABLED
    ModeThrow mode_throw;
#endif
#if MODE_GUIDED_NOGPS_ENABLED == ENABLED
    ModeGuidedNoGPS mode_guided_nogps;
#endif
#if MODE_SMARTRTL_ENABLED == ENABLED
    ModeSmartRTL mode_smartrtl;
#endif
#if !HAL_MINIMIZE_FEATURES && OPTFLOW == ENABLED
    ModeFlowHold mode_flowhold;
#endif
#if MODE_ZIGZAG_ENABLED == ENABLED
    ModeZigZag mode_zigzag;
#endif

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
