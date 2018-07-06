/*
   Lead developer: Andrew Tridgell & Tom Pittenger

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.com for details

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

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_GPS/AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro/AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass/AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Relay/AP_Relay.h>       // APM relay
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Scheduler/PerfInfo.h>                  // loop perf monitoring

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library

#include <AP_Soaring/AP_Soaring.h>
#include <AP_Notify/AP_Notify.h>      // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming/AP_Arming.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_Devo_Telem/AP_Devo_Telem.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <AP_RSSI/AP_RSSI.h>                   // RSSI Library
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Button/AP_Button.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library

#include "GCS_Mavlink.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include "tuning.h"

// Configuration
#include "config.h"

// Local modules
#include "defines.h"

#ifdef ENABLE_SCRIPTING
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel.h"     // RC Channel Library
#include "Parameters.h"
#include "avoidance_adsb.h"
#include "AP_Arming.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

/*
  a plane specific AP_AdvancedFailsafe class
 */
class AP_AdvancedFailsafe_Plane : public AP_AdvancedFailsafe
{
public:
    AP_AdvancedFailsafe_Plane(AP_Mission &_mission, const AP_GPS &_gps);

    // called to set all outputs to termination state
    void terminate_vehicle(void);
    
protected:
    // setup failsafe values for if FMU firmware stops running
    void setup_IO_failsafe(void);

    // return the AFS mapped control mode
    enum control_mode afs_mode(void);
};

/*
  main APM:Plane class
 */
class Plane : public AP_HAL::HAL::Callbacks {
public:
    friend class GCS_MAVLINK_Plane;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Plane;
    friend class QuadPlane;
    friend class QAutoTune;
    friend class AP_Tuning_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class AP_Avoidance_Plane;
    friend class GCS_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channels_Plane;

    Plane(void);

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:

    // key aircraft parameters passed to multiple libraries
    AP_Vehicle::FixedWing aparm;

    // Global parameters are all contained within the 'g' and 'g2' classes.
    Parameters g;
    ParametersG2 g2;

    // main loop scheduler
    AP_Scheduler scheduler;

    // mapping between input channels
    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

    // board specific config for CAN bus
#if HAL_WITH_UAVCAN
    AP_BoardConfig_CAN BoardConfig_CAN;
#endif

    // primary input channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;

    // notification object for LEDs, buzzers etc (parameter set to false disables external leds)
    AP_Notify notify;

    AP_Logger logger;

    // scaled roll limit based on pitch
    int32_t roll_limit_cd;
    int32_t pitch_limit_min_cd;

    // Sensors
    AP_GPS gps;

    // flight modes convenience array
    AP_Int8 *flight_modes = &g.flight_mode1;

    AP_Baro barometer;
    Compass compass;

    AP_InertialSensor ins;

    RangeFinder rangefinder{serial_manager, ROTATION_PITCH_270};

    AP_Vehicle::FixedWing::Rangefinder_State rangefinder_state;

    AP_RPM rpm_sensor;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF2 EKF2{&ahrs, rangefinder};
    NavEKF3 EKF3{&ahrs, rangefinder};
    AP_AHRS_NavEKF ahrs{EKF2, EKF3};
#else
    AP_AHRS_DCM ahrs;
#endif

    AP_TECS TECS_controller{ahrs, aparm, landing};
    AP_L1_Control L1_controller{ahrs, &TECS_controller};

    // Attitude to servo controllers
    AP_RollController rollController{ahrs, aparm};
    AP_PitchController pitchController{ahrs, aparm};
    AP_YawController yawController{ahrs, aparm};
    AP_SteerController steerController{ahrs};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    // Training mode
    bool training_manual_roll;  // user has manual roll control
    bool training_manual_pitch; // user has manual pitch control

    /*
      keep steering and rudder control separated until we update servos,
      to allow for a separate wheel servo from rudder servo
    */
    struct {
        bool ground_steering; // are we doing ground steering?
        int16_t steering; // value for nose/tail wheel
        int16_t rudder;   // value for rudder
    } steering_control;

    // should throttle be pass-thru in guided?
    bool guided_throttle_passthru;

    // are we doing calibration? This is used to allow heartbeat to
    // external failsafe boards during baro and airspeed calibration
    bool in_calibration;

    AP_SerialManager serial_manager;

    // GCS selection
    GCS_Plane _gcs; // avoid using this; use gcs()
    GCS_Plane &gcs() { return _gcs; }

    // selected navigation controller
    AP_Navigation *nav_controller = &L1_controller;

    // selected navigation controller
    AP_SpdHgtControl *SpdHgt_Controller = &TECS_controller;

    // Relay
    AP_Relay relay;

    // handle servo and relay events
    AP_ServoRelayEvents ServoRelayEvents{relay};

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera{&relay, MASK_LOG_CAMERA, current_loc, ahrs};
#endif

#if OPTFLOW == ENABLED
    // Optical flow sensor
    OpticalFlow optflow;
#endif

    // Rally Ponints
    AP_Rally rally{ahrs};

    // RSSI
    AP_RSSI rssi;

#if OSD_ENABLED == ENABLED
    AP_OSD osd;
#endif
    
    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    enum FlightMode control_mode = INITIALISING;
    mode_reason_t control_mode_reason = MODE_REASON_UNKNOWN;
    enum FlightMode previous_mode = INITIALISING;
    mode_reason_t previous_mode_reason = MODE_REASON_UNKNOWN;

    // time of last mode change
    uint32_t last_mode_change_ms;

    // Used to maintain the state of the previous control switch position
    // This is set to 254 when we need to re-read the switch
    uint8_t oldSwitchPosition = 254;

    // This is used to enable the inverted flight feature
    bool inverted_flight;

    // This is used to enable the PX4IO override for testing
    bool px4io_override_enabled;

    // Failsafe
    struct {
        // Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
        // RC receiver should be set up to output a low throttle value when signal is lost
        bool rc_failsafe:1;

        // has the saved mode for failsafe been set?
        bool saved_mode_set:1;

        // true if an adsb related failsafe has occurred
        bool adsb:1;

        // saved flight mode
        enum FlightMode saved_mode;

        // A tracking variable for type of failsafe active
        // Used for failsafe based on loss of RC signal or GCS signal
        int16_t state;

        // number of low throttle values
        uint8_t throttle_counter;

        // the time when the last HEARTBEAT message arrived from a GCS
        uint32_t last_heartbeat_ms;
        
        // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
        uint32_t short_timer_ms;
        
        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

    enum Landing_ApproachStage {
        LOITER_TO_ALT,
        ENSURE_RADIUS,
        WAIT_FOR_BREAKOUT,
        APPROACH_LINE,
        VTOL_LANDING,
    };

    // Landing
    struct {
        enum Landing_ApproachStage approach_stage;
        float approach_direction_deg;
    } vtol_approach_s;

    bool any_failsafe_triggered() {
        return failsafe.state != FAILSAFE_NONE || battery.has_failsafed() || failsafe.adsb;
    }

    // A counter used to count down valid gps fixes to allow the gps estimate to settle
    // before recording our home position (and executing a ground start if we booted with an air start)
    uint8_t ground_start_count = 5;

    // true if we have a position estimate from AHRS
    bool have_position;

    // Airspeed
    // The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
    // Also used for flap deployment criteria.  Centimeters per second.
    int32_t target_airspeed_cm;

    // The difference between current and desired airspeed.  Used in the pitch controller.  Meters per second.
    float airspeed_error;

    // An amount that the airspeed should be increased in auto modes based on the user positioning the
    // throttle stick in the top half of the range.  Centimeters per second.
    int16_t airspeed_nudge_cm;

    // Similar to airspeed_nudge, but used when no airspeed sensor.
    // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
    int16_t throttle_nudge;

    // Ground speed
    // The amount current ground speed is below min ground speed.  Centimeters per second
    int32_t groundspeed_undershoot;

    // Difference between current altitude and desired altitude.  Centimeters
    int32_t altitude_error_cm;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

#if FRSKY_TELEM_ENABLED == ENABLED
    // FrSky telemetry support
    AP_Frsky_Telem frsky_telemetry{ahrs, battery, rangefinder};
#endif
#if DEVO_TELEM_ENABLED == ENABLED
    // DEVO-M telemetry support
    AP_DEVO_Telem devo_telemetry {ahrs};
#endif

    // Variables for extended status MAVLink messages
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;
 
    // Airspeed Sensors
    AP_Airspeed airspeed;

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
    } acro_state;

    // CRUISE controller state
    struct CruiseState {
        bool locked_heading;
        int32_t locked_heading_cd;
        uint32_t lock_timer_ms;
    } cruise_state;

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
    } takeoff_state;
    
    // ground steering controller state
    struct {
        // Direction held during phases of takeoff and landing centidegrees
        // A value of -1 indicates the course has not been set/is not in use
        // this is a 0..36000 value, or -1 for disabled
        int32_t hold_course_cd;

        // locked_course and locked_course_cd are used in stabilize mode 
        // when ground steering is active, and for steering in auto-takeoff
        bool locked_course;
        float locked_course_err;
    } steer_state { -1, false, 0 };

    // flight mode specific
    struct {
        // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
        bool takeoff_complete:1;

        // are we headed to the land approach waypoint? Works for any nav type
        bool wp_is_land_approach:1;

        // should we fly inverted?
        bool inverted_flight:1;

        // should we enable cross-tracking for the next waypoint?
        bool next_wp_crosstrack:1;

        // should we use cross-tracking for this waypoint?
        bool crosstrack:1;

        // in FBWA taildragger takeoff mode
        bool fbwa_tdrag_takeoff_mode:1;

        // have we checked for an auto-land?
        bool checked_for_autoland:1;

        // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
        // are we in idle mode? used for balloon launch to stop servo
        // movement until altitude is reached
        bool idle_mode:1;

        // used to 'wiggle' servos in idle mode to prevent them freezing
        // at high altitudes
        uint8_t idle_wiggle_stage;

        // Altitude threshold to complete a takeoff command in autonomous
        // modes.  Centimeters above home
        int32_t takeoff_altitude_rel_cm;

        // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
        int16_t takeoff_pitch_cd;

        // Begin leveling out the enforced takeoff pitch angle min at this height to reduce/eliminate overshoot
        int32_t height_below_takeoff_to_level_off_cm;

        // the highest airspeed we have reached since entering AUTO. Used
        // to control ground takeoff
        float highest_airspeed;
        
        // initial pitch. Used to detect if nose is rising in a tail dragger
        int16_t initial_pitch_cd;
        
        // turn angle for next leg of mission
        float next_turn_angle {90};

        // filtered sink rate for landing
        float sink_rate;

        // time when we first pass min GPS speed on takeoff
        uint32_t takeoff_speed_time_ms;
        
        // distance to next waypoint
        float wp_distance;
        
        // proportion to next waypoint
        float wp_proportion;
        
        // last time is_flying() returned true in milliseconds
        uint32_t last_flying_ms;

        // time stamp of when we start flying while in auto mode in milliseconds
        uint32_t started_flying_in_auto_ms;

        // barometric altitude at start of takeoff
        float baro_takeoff_alt;

        // are we in VTOL mode in AUTO?
        bool vtol_mode:1;

        // are we doing loiter mode as a VTOL?
        bool vtol_loiter:1;
    } auto_state;

    struct {
        // roll pitch yaw commanded from external controller in centidegrees
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        float forced_throttle;
        uint32_t last_forced_throttle_ms;
    } guided_state;

#if LANDING_GEAR_ENABLED == ENABLED
    // landing gear state
    struct {
        int8_t last_auto_cmd;
        int8_t last_cmd;
    } gear;
#endif
    
    struct {
        // on hard landings, only check once after directly a landing so you
        // don't trigger a crash when picking up the aircraft
        bool checkedHardLanding:1;

        // crash detection. True when we are crashed
        bool is_crashed:1;

        // impact detection flag. Expires after a few seconds via impact_timer_ms
        bool impact_detected:1;

        // debounce timer
        uint32_t debounce_timer_ms;

        // delay time for debounce to count to
        uint32_t debounce_time_total_ms;

        // length of time impact_detected has been true. Times out after a few seconds. Used to clip isFlyingProbability
        uint32_t impact_timer_ms;
    } crash_state;

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool auto_throttle_mode:1;

    // true if we are in an auto-navigation mode, which controls whether control input is ignored
    // with STICK_MIXING=0
    bool auto_navigation_mode:1;
    
    // this allows certain flight modes to mix RC input with throttle depending on airspeed_nudge_cm
    bool throttle_allows_nudging:1;

    // this controls throttle suppression in auto modes
    bool throttle_suppressed;
	
    // reduce throttle to eliminate battery over-current
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust
    uint32_t throttle_watt_limit_timer_ms;

    AP_Vehicle::FixedWing::FlightStage flight_stage = AP_Vehicle::FixedWing::FLIGHT_NORMAL;

    // probability of aircraft is currently in flight. range from 0 to
    // 1 where 1 is 100% sure we're in flight
    float isFlyingProbability;

    // previous value of is_flying()
    bool previous_is_flying;

    // time since started flying in any mode in milliseconds
    uint32_t started_flying_ms;

    // Navigation control variables
    // The instantaneous desired bank angle.  Hundredths of a degree
    int32_t nav_roll_cd;

    // The instantaneous desired pitch angle.  Hundredths of a degree
    int32_t nav_pitch_cd;

    // the aerodymamic load factor. This is calculated from the demanded
    // roll before the roll is clipped, using 1/sqrt(cos(nav_roll))
    float aerodynamic_load_factor = 1.0f;

    // a smoothed airspeed estimate, used for limiting roll angle
    float smoothed_airspeed;

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if PARACHUTE == ENABLED
    AP_Parachute parachute{relay};
#endif

    // terrain handling
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain{mission};
#endif

    AP_Landing landing{mission,ahrs,SpdHgt_Controller,nav_controller,aparm,
            FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
            FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
            FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};

    AP_ADSB adsb;

    // avoidance of adsb enabled vehicles (normally manned vheicles)
    AP_Avoidance_Plane avoidance_adsb{ahrs, adsb};

    // Outback Challenge Failsafe Support
    AP_AdvancedFailsafe_Plane afs {mission, gps};

    /*
      meta data to support counting the number of circles in a loiter
    */
    struct {
        // previous target bearing, used to update sum_cd
        int32_t old_target_bearing_cd;

        // Total desired rotation in a loiter.  Used for Loiter Turns commands. 
        int32_t total_cd;

        // total angle completed in the loiter so far
        int32_t sum_cd;

        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        int8_t direction;

        // when loitering and an altitude is involved, this flag is true when it has been reached at least once
        bool reached_target_alt;

        // check for scenarios where updrafts can keep you from loitering down indefinitely.
        bool unable_to_acheive_target_alt;

        // start time of the loiter.  Milliseconds.
        uint32_t start_time_ms;

        // altitude at start of loiter loop lap. Used to detect delta alt of each lap.
        // only valid when sum_cd > 36000
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        uint32_t time_max_ms;
    } loiter;


    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;

    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    uint32_t condition_start;
    // A value used in condition commands.  For example the rate at which to change altitude.
    int16_t condition_rate;

    // 3D Location vectors
    // Location structure defined in AP_Common
    const struct Location &home = ahrs.get_home();

    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    Location prev_WP_loc {};

    // The plane's current location
    struct Location current_loc {};

    // The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
    Location next_WP_loc {};

    // The location of the active waypoint in Guided mode.
    struct Location guided_WP_loc {};

    // Altitude control
    struct {
        // target altitude above sea level in cm. Used for barometric
        // altitude navigation
        int32_t amsl_cm;

        // Altitude difference between previous and current waypoint in
        // centimeters. Used for glide slope handling
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // are we trying to follow terrain?
        bool terrain_following;

        // target altitude above terrain in cm, valid if terrain_following
        // is set
        int32_t terrain_alt_cm;

        // lookahead value for height error reporting
        float lookahead;
#endif

        // last input for FBWB/CRUISE height control
        float last_elevator_input;

        // last time we checked for pilot control of height
        uint32_t last_elev_check_us;
    } target_altitude {};

    float relative_altitude = 0.0f;

    // INS variables
    // The main loop execution time.  Seconds
    // This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
    float G_Dt = 0.02f;

    // loop performance monitoring:
    AP::PerfInfo perf_info;
    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    // last time home was updated while disarmed
    uint32_t last_home_update_ms;

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    AP_Mount camera_mount{current_loc};
#endif

    // Arming/Disarming mangement class
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

    // time that rudder arming has been running
    uint32_t rudder_arm_timer;

    // support for quadcopter-plane
    QuadPlane quadplane{ahrs};

    // support for transmitter tuning
    AP_Tuning_Plane tuning;

    static const struct LogStructure log_structure[];

    // rudder mixing gain for differential thrust (0 - 1)
    float rudder_dt;

    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void send_fence_status(mavlink_channel_t chan);
    void update_sensor_status_flags(void);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_wind(mavlink_channel_t chan);
    void send_pid_info(const mavlink_channel_t chan, const AP_Logger::PID_Info *pid_info, const uint8_t axis, const float achieved);
    void send_pid_tuning(mavlink_channel_t chan);
    void send_rpm(mavlink_channel_t chan);

    void send_aoa_ssa(mavlink_channel_t chan);

    void gcs_send_airspeed_calibration(const Vector3f &vg);

    void Log_Write_Fast(void);
    void Log_Write_Attitude(void);
    void Log_Write_Performance();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Control_Tuning();
    void Log_Write_Nav_Tuning();
    void Log_Write_Status();
    void Log_Write_Sonar();
    void Log_Arm_Disarm();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Write_AOA_SSA();
    void Log_Write_AETR();

    void load_parameters(void);
    void convert_mixers(void);
    void adjust_altitude_target();
    void setup_glide_slope(void);
    int32_t get_RTL_altitude();
    float relative_ground_altitude(bool use_rangefinder_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_current_adjusted(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_fbwb_minimum_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc);
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void set_next_WP(const struct Location &loc);
    void set_guided_WP(void);
    void update_home();
    // set home location and store it persistently:
    void set_home_persistently(const Location &loc);
    void do_RTL(int32_t alt);
    bool verify_takeoff();
    bool verify_loiter_unlim(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_time();
    bool verify_loiter_turns(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd);
    bool verify_RTL();
    bool verify_continue_and_change_alt();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_altitude_wait(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(const AP_Mission::Mission_Command &cmd);
    void do_loiter_at_location();
    bool verify_loiter_heading(bool init);
    void exit_mission_callback();
    void mavlink_delay(uint32_t ms);
    void read_control_switch();
    uint8_t readSwitch(void);
    void reset_control_switch();
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    void failsafe_short_on_event(enum failsafe_state fstype, mode_reason_t reason);
    void failsafe_long_on_event(enum failsafe_state fstype, mode_reason_t reason);
    void failsafe_short_off_event(mode_reason_t reason);
    void failsafe_long_off_event(mode_reason_t reason);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    uint8_t max_fencepoints(void);
    Vector2l get_fence_point_with_index(unsigned i);
    void set_fence_point_with_index(const Vector2l &point, unsigned i);
    void geofence_load(void);
    bool geofence_present(void);
    void geofence_update_pwm_enabled_state();
    bool geofence_set_enabled(bool enable, GeofenceEnableReason r);
    bool geofence_enabled(void);
    bool geofence_set_floor_enabled(bool floor_enable);
    bool geofence_check_minalt(void);
    bool geofence_check_maxalt(void);
    void geofence_check(bool altitude_check_only);
    bool geofence_stickmixing(void);
    void geofence_send_status(mavlink_channel_t chan);
    bool geofence_breached(void);
    void geofence_disable_and_send_error_msg(const char *errorMsg);
    void disarm_if_autoland_complete();
    float tecs_hgt_afe(void);
    void set_nav_controller(void);
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void calc_airspeed_errors();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_cruise();
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);
    void set_control_channels(void);
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void rudder_arm_disarm_check();
    void read_radio();
    int16_t rudder_input(void);
    void control_failsafe();
    bool trim_radio();
    bool rc_failsafe_active(void) const;
    void read_rangefinder(void);
    void read_airspeed(void);
    void rpm_update(void);
    void init_ardupilot();
    void startup_ground(void);
    enum FlightMode get_previous_mode();
    void set_mode(enum FlightMode mode, mode_reason_t reason);
    void exit_mode(enum FlightMode mode);
    void check_long_failsafe();
    void check_short_failsafe();
    void startup_INS_ground(void);
    void update_notify();
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    void change_arm_state(void);
    bool disarm_motors(void);
    bool arm_motors(AP_Arming::ArmingMethod method, bool do_arming_checks=true);
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void landing_gear_update(void);
    void complete_auto_takeoff(void);
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
    void afs_fs_check(void);
    void compass_cal_update();
    void update_optical_flow(void);
    void one_second_loop(void);
    void airspeed_ratio_update(void);
    void compass_save(void);
    void update_logging1(void);
    void update_logging2(void);
    void avoidance_adsb_update(void);
    void update_flight_mode(void);
    void stabilize();
    void set_servos_idle(void);
    void set_servos();
    void set_servos_manual_passthrough(void);
    void set_servos_controlled(void);
    void set_servos_old_elevons(void);
    void set_servos_flaps(void);
    void change_landing_gear(AP_LandingGear::LandingGearCommand cmd);
    void set_landing_gear(void);
    void dspoiler_update(void);
    void servo_output_mixers(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    void handle_auto_mode(void);
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    void update_flight_stage();
    void update_navigation();
    void set_flight_stage(AP_Vehicle::FixedWing::FlightStage fs);
    bool is_flying(void);
    float get_speed_scaler(void);
    bool stick_mixing_enabled(void);
    void stabilize_roll(float speed_scaler);
    void stabilize_pitch(float speed_scaler);
    static void stick_mix_channel(RC_Channel *channel, int16_t &servo_out);
    void stabilize_stick_mixing_direct();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw(float speed_scaler);
    void stabilize_training(float speed_scaler);
    void stabilize_acro(float speed_scaler);
    void calc_nav_yaw_coordinated(float speed_scaler);
    void calc_nav_yaw_course(void);
    void calc_nav_yaw_ground(void);
    void throttle_slew_limit(SRV_Channel::Aux_servo_function_t func);
    bool suppress_throttle(void);
    void channel_function_mixer(SRV_Channel::Aux_servo_function_t func1_in, SRV_Channel::Aux_servo_function_t func2_in,
                                SRV_Channel::Aux_servo_function_t func1_out, SRV_Channel::Aux_servo_function_t func2_out);
    void flaperon_update(int8_t flap_percent);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
    void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_turns(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void notify_flight_mode(enum FlightMode mode);
    void log_init();
    void init_capabilities(void);
    void parachute_check();
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
    void parachute_release();
    bool parachute_manual_release();
#endif
#if OSD_ENABLED == ENABLED
    void publish_osd_info();
#endif
    void accel_cal_update(void);
    void update_soft_armed();
#if SOARING_ENABLED == ENABLED
    void update_soaring();
#endif

    bool reversed_throttle;
    bool have_reverse_throttle_rc_option;
    bool allow_reverse_thrust(void) const;
    bool have_reverse_thrust(void) const;
    int16_t get_throttle_input(bool no_deadzone=false) const;

    // support for AP_Avoidance custom flight mode, AVOID_ADSB
    bool avoid_adsb_init(bool ignore_checks);
    void avoid_adsb_run();

    enum Failsafe_Action {
        Failsafe_Action_None      = 0,
        Failsafe_Action_RTL       = 1,
        Failsafe_Action_Land      = 2,
        Failsafe_Action_Terminate = 3,
        Failsafe_Action_QLand     = 4,
    };

    // list of priorities, highest priority first
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_QLand,
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

public:
    void mavlink_delay_cb();
    void failsafe_check(void);
};

extern const AP_HAL::HAL& hal;
extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
