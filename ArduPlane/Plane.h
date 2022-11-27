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
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Scheduler/PerfInfo.h>                  // loop perf monitoring

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library

#include <AP_Soaring/AP_Soaring.h>
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming/AP_Arming.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_OSD/AP_OSD.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
#include <AP_Follow/AP_Follow.h>

#include "GCS_Mavlink.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include "tuning.h"

// Configuration
#include "config.h"

#if ADVANCED_FAILSAFE == ENABLED
#include "afs_plane.h"
#endif

// Local modules
#include "defines.h"
#include "mode.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel.h"     // RC Channel Library
#include "Parameters.h"
#if HAL_ADSB_ENABLED
#include "avoidance_adsb.h"
#endif
#include "AP_Arming.h"

/*
  main APM:Plane class
 */
class Plane : public AP_Vehicle {
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
    friend class Tailsitter;
    friend class Tiltrotor;
    friend class SLT_Transition;
    friend class Tailsitter_Transition;

    friend class Mode;
    friend class ModeCircle;
    friend class ModeStabilize;
    friend class ModeTraining;
    friend class ModeAcro;
    friend class ModeFBWA;
    friend class ModeFBWB;
    friend class ModeCruise;
    friend class ModeAutoTune;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeLoiter;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeInitializing;
    friend class ModeManual;
    friend class ModeQStabilize;
    friend class ModeQHover;
    friend class ModeQLoiter;
    friend class ModeQLand;
    friend class ModeQRTL;
    friend class ModeQAcro;
    friend class ModeQAutotune;
    friend class ModeTakeoff;
    friend class ModeThermal;
    friend class ModeLoiterAltQLand;

    Plane(void);

private:

    // key aircraft parameters passed to multiple libraries
    AP_FixedWing aparm;

    // Global parameters are all contained within the 'g' and 'g2' classes.
    Parameters g;
    ParametersG2 g2;

    // mapping between input channels
    RCMapper rcmap;

    // primary input channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;
    RC_Channel *channel_flap;
    RC_Channel *channel_airbrake;

    AP_Logger logger;

    // scaled roll limit based on pitch
    int32_t roll_limit_cd;
    int32_t pitch_limit_min_cd;

    // flight modes convenience array
    AP_Int8 *flight_modes = &g.flight_mode1;

    AP_FixedWing::Rangefinder_State rangefinder_state;

#if AP_RPM_ENABLED
    AP_RPM rpm_sensor;
#endif

    AP_TECS TECS_controller{ahrs, aparm, landing, MASK_LOG_TECS};
    AP_L1_Control L1_controller{ahrs, &TECS_controller};

    // Attitude to servo controllers
    AP_RollController rollController{aparm};
    AP_PitchController pitchController{aparm};
    AP_YawController yawController{aparm};
    AP_SteerController steerController{};

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

    // GCS selection
    GCS_Plane _gcs; // avoid using this; use gcs()
    GCS_Plane &gcs() { return _gcs; }

    // selected navigation controller
    AP_Navigation *nav_controller = &L1_controller;

    // Camera
#if AP_CAMERA_ENABLED
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

#if AP_OPTICALFLOW_ENABLED
    // Optical flow sensor
    AP_OpticalFlow optflow;
#endif

    // Rally Ponints
    AP_Rally rally;

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    ModeCircle mode_circle;
    ModeStabilize mode_stabilize;
    ModeTraining mode_training;
    ModeAcro mode_acro;
    ModeFBWA mode_fbwa;
    ModeFBWB mode_fbwb;
    ModeCruise mode_cruise;
    ModeAutoTune mode_autotune;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
    ModeLoiter mode_loiter;
#if HAL_ADSB_ENABLED
    ModeAvoidADSB mode_avoidADSB;
#endif
    ModeGuided mode_guided;
    ModeInitializing mode_initializing;
    ModeManual mode_manual;
#if HAL_QUADPLANE_ENABLED
    ModeQStabilize mode_qstabilize;
    ModeQHover mode_qhover;
    ModeQLoiter mode_qloiter;
    ModeQLand mode_qland;
    ModeQRTL mode_qrtl;
    ModeQAcro mode_qacro;
    ModeLoiterAltQLand mode_loiter_qland;
#if QAUTOTUNE_ENABLED
    ModeQAutotune mode_qautotune;
#endif  // QAUTOTUNE_ENABLED
#endif  // HAL_QUADPLANE_ENABLED
    ModeTakeoff mode_takeoff;
#if HAL_SOARING_ENABLED
    ModeThermal mode_thermal;
#endif

    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    Mode *control_mode = &mode_initializing;
    Mode *previous_mode = &mode_initializing;

    // time of last mode change
    uint32_t last_mode_change_ms;

    // Used to maintain the state of the previous control switch position
    // This is set to 254 when we need to re-read the switch
    uint8_t oldSwitchPosition = 254;

    // This is used to enable the inverted flight feature
    bool inverted_flight;

    // last time we ran roll/pitch stabilization
    uint32_t last_stabilize_ms;

    // Failsafe
    struct {
        // Used to track if the value on channel 3 (throtttle) has fallen below the failsafe threshold
        // RC receiver should be set up to output a low throttle value when signal is lost
        bool rc_failsafe;

        // true if an adsb related failsafe has occurred
        bool adsb;

        // saved flight mode
        enum Mode::Number saved_mode_number;

        // A tracking variable for type of failsafe active
        // Used for failsafe based on loss of RC signal or GCS signal
        int16_t state;

        // number of low throttle values
        uint8_t throttle_counter;

        // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
        uint32_t short_timer_ms;

        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

    enum Landing_ApproachStage {
        RTL,
        LOITER_TO_ALT,
        ENSURE_RADIUS,
        WAIT_FOR_BREAKOUT,
        APPROACH_LINE,
        VTOL_LANDING,
    };

#if HAL_QUADPLANE_ENABLED
    // Landing
    struct {
        enum Landing_ApproachStage approach_stage;
        float approach_direction_deg;
    } vtol_approach_s;
#endif

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
    int32_t new_airspeed_cm = -1;  //temp variable for AUTO and GUIDED mode speed changes

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

    // speed scaler for control surfaces, updated at 10Hz
    float surface_speed_scaler = 1.0;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
    } takeoff_state;

    // ground steering controller state
    struct {
        // Direction held during phases of takeoff and landing centidegrees
        // A value of -1 indicates the course has not been set/is not in use
        // this is a 0..36000 value, or -1 for disabled
        int32_t hold_course_cd = -1;

        // locked_course and locked_course_cd are used in stabilize mode
        // when ground steering is active, and for steering in auto-takeoff
        bool locked_course;
        float locked_course_err;
        uint32_t last_steer_ms;
    } steer_state;

    // flight mode specific
    struct {
        // Altitude threshold to complete a takeoff command in autonomous
        // modes.  Centimeters above home
        int32_t takeoff_altitude_rel_cm;

        // Begin leveling out the enforced takeoff pitch angle min at this height to reduce/eliminate overshoot
        int32_t height_below_takeoff_to_level_off_cm;

        // the highest airspeed we have reached since entering AUTO. Used
        // to control ground takeoff
        float highest_airspeed;

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

        // initial pitch. Used to detect if nose is rising in a tail dragger
        int16_t initial_pitch_cd;

        // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
        int16_t takeoff_pitch_cd;

        // used to 'wiggle' servos in idle mode to prevent them freezing
        // at high altitudes
        uint8_t idle_wiggle_stage;

        // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
        bool takeoff_complete;

        // are we headed to the land approach waypoint? Works for any nav type
        bool wp_is_land_approach;

        // should we fly inverted?
        bool inverted_flight;

        // should we enable cross-tracking for the next waypoint?
        bool next_wp_crosstrack;

        // should we use cross-tracking for this waypoint?
        bool crosstrack;

        // in FBWA taildragger takeoff mode
        bool fbwa_tdrag_takeoff_mode;

        // have we checked for an auto-land?
        bool checked_for_autoland;

        // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
        // are we in idle mode? used for balloon launch to stop servo
        // movement until altitude is reached
        bool idle_mode;

        // are we in VTOL mode in AUTO?
        bool vtol_mode;

        // are we doing loiter mode as a VTOL?
        bool vtol_loiter;

        // how much correction have we added for terrain data
        float terrain_correction;
    } auto_state;

#if AP_SCRIPTING_ENABLED
    // support for scripting nav commands, with verify
    struct {
        bool enabled;
        uint16_t id;
        float roll_rate_dps;
        float pitch_rate_dps;
        float yaw_rate_dps;
        float throttle_pct;
        uint32_t start_ms;
        uint32_t current_ms;
    } nav_scripting;
#endif

    struct {
        // roll pitch yaw commanded from external controller in centidegrees
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        float forced_throttle;
        uint32_t last_forced_throttle_ms;

#if OFFBOARD_GUIDED == ENABLED
        // airspeed adjustments
        float target_airspeed_cm = -1;  // don't default to zero here, as zero is a valid speed.
        float target_airspeed_accel;
        uint32_t target_airspeed_time_ms;

        // altitude adjustments
        float target_alt = -1;   // don't default to zero here, as zero is a valid alt.
        uint32_t last_target_alt = 0;
        float target_alt_accel;
        uint32_t target_alt_time_ms = 0;
        uint8_t target_alt_frame = 0;

        // heading track
        float target_heading = -4; // don't default to zero or -1 here, as both are valid headings in radians
        float target_heading_accel_limit;
        uint32_t target_heading_time_ms;
        guided_heading_type_t target_heading_type;
        bool target_heading_limit_low;
        bool target_heading_limit_high;
#endif // OFFBOARD_GUIDED == ENABLED
    } guided_state;

#if LANDING_GEAR_ENABLED == ENABLED
    // landing gear state
    struct {
        AP_FixedWing::FlightStage last_flight_stage;
    } gear;
#endif

    struct {
        // on hard landings, only check once after directly a landing so you
        // don't trigger a crash when picking up the aircraft
        bool checkedHardLanding;

        // crash detection. True when we are crashed
        bool is_crashed;

        // impact detection flag. Expires after a few seconds via impact_timer_ms
        bool impact_detected;

        // debounce timer
        uint32_t debounce_timer_ms;

        // delay time for debounce to count to
        uint32_t debounce_time_total_ms;

        // length of time impact_detected has been true. Times out after a few seconds. Used to clip isFlyingProbability
        uint32_t impact_timer_ms;
    } crash_state;

    // this controls throttle suppression in auto modes
    bool throttle_suppressed;

    // reduce throttle to eliminate battery over-current
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust
    uint32_t throttle_watt_limit_timer_ms;

    AP_FixedWing::FlightStage flight_stage = AP_FixedWing::FlightStage::NORMAL;

    // probability of aircraft is currently in flight. range from 0 to
    // 1 where 1 is 100% sure we're in flight
    float isFlyingProbability;

    // previous value of is_flying()
    bool previous_is_flying;

    // time since started flying in any mode in milliseconds
    uint32_t started_flying_ms;

    // ground mode is true when disarmed and not flying
    bool ground_mode;

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
    AP_Terrain terrain;
#endif

    AP_Landing landing{mission,ahrs,&TECS_controller,nav_controller,aparm,
            FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
            FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
            FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};
#if HAL_ADSB_ENABLED
    AP_ADSB adsb;

    // avoidance of adsb enabled vehicles (normally manned vehicles)
    AP_Avoidance_Plane avoidance_adsb{adsb};
#endif

    // Outback Challenge Failsafe Support
#if ADVANCED_FAILSAFE == ENABLED
    AP_AdvancedFailsafe_Plane afs;
#endif

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

    float relative_altitude;

    // loop performance monitoring:
    AP::PerfInfo perf_info;
    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    struct {
        bool done_climb;
    } rtl;

    // last time home was updated while disarmed
    uint32_t last_home_update_ms;

    // Camera/Antenna mount tracking and stabilisation stuff
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

    // Arming/Disarming mangement class
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

    // time that rudder arming has been running
    uint32_t rudder_arm_timer;

#if HAL_QUADPLANE_ENABLED
    // support for quadcopter-plane
    QuadPlane quadplane{ahrs};
#endif

    // support for transmitter tuning
    AP_Tuning_Plane tuning;

    static const struct LogStructure log_structure[];

    // rudder mixing gain for differential thrust (0 - 1)
    float rudder_dt;

    // soaring mode-change timer
    uint32_t soaring_mode_timer_ms;

    // terrain disable for non AUTO modes, set with an RC Option switch
    bool non_auto_terrain_disable;
    bool terrain_disabled();
#if AP_TERRAIN_AVAILABLE
    bool terrain_enabled_in_current_mode() const;
    bool terrain_enabled_in_mode(Mode::Number num) const;
    enum class terrain_bitmask {
        ALL             = 1U << 0,
        FLY_BY_WIRE_B   = 1U << 1,
        CRUISE          = 1U << 2,
        AUTO            = 1U << 3,
        RTL             = 1U << 4,
        AVOID_ADSB      = 1U << 5,
        GUIDED          = 1U << 6,
        LOITER          = 1U << 7,
        CIRCLE          = 1U << 8,
        QRTL            = 1U << 9,
        QLAND           = 1U << 10,
        QLOITER         = 1U << 11,
    };
    struct TerrainLookupTable{
       Mode::Number mode_num;
       terrain_bitmask bitmask;
    };
    static const TerrainLookupTable Terrain_lookup[];
#endif

    // Attitude.cpp
    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void adjust_altitude_target();
    void setup_glide_slope(void);
    int32_t get_RTL_altitude_cm() const;
    float relative_ground_altitude(bool use_rangefinder_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_current_adjusted(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_fbwb_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &start_loc, const Location &destination_loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc) const;
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void rangefinder_terrain_correction(float &height);
    void stabilize();
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    float calc_speed_scaler(void);
    float get_speed_scaler(void) const { return surface_speed_scaler; }
    bool stick_mixing_enabled(void);
    void stabilize_roll(float speed_scaler);
    float stabilize_roll_get_roll_out(float speed_scaler);
    void stabilize_pitch(float speed_scaler);
    float stabilize_pitch_get_pitch_out(float speed_scaler);
    void stabilize_stick_mixing_direct();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw(float speed_scaler);
    void stabilize_training(float speed_scaler);
    void stabilize_acro(float speed_scaler);
    void stabilize_acro_quaternion(float speed_scaler);
    void calc_nav_yaw_coordinated(float speed_scaler);
    void calc_nav_yaw_course(void);
    void calc_nav_yaw_ground(void);

    // Log.cpp
    uint32_t last_log_fast_ms;

    void Log_Write_FullRate(void);
    void Log_Write_Attitude(void);
    void Log_Write_Control_Tuning();
    void Log_Write_OFG_Guided();
    void Log_Write_Guided(void);
    void Log_Write_Nav_Tuning();
    void Log_Write_Status();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Write_AETR();
    void log_init();

    // Parameters.cpp
    void load_parameters(void) override;

    // commands_logic.cpp
    void set_next_WP(const struct Location &loc);
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
    void do_loiter_at_location();
    bool verify_loiter_heading(bool init);
    void exit_mission_callback();
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_land(const AP_Mission::Mission_Command& cmd);
#if HAL_QUADPLANE_ENABLED
    void do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
#endif
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
#if HAL_QUADPLANE_ENABLED
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
#endif
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    float get_wp_radius() const;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;
    
#if AP_SCRIPTING_ENABLED
    // nav scripting support
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif

    // commands.cpp
    void set_guided_WP(const Location &loc);
    void update_home();
    // set home location and store it persistently:
    bool set_home_persistently(const Location &loc) WARN_IF_UNUSED;

    // quadplane.cpp
#if HAL_QUADPLANE_ENABLED
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(const AP_Mission::Mission_Command &cmd);
#endif

    // control_modes.cpp
    void read_control_switch();
    uint8_t readSwitch(void) const;
    void reset_control_switch();
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }
    Mode *mode_from_mode_num(const enum Mode::Number num);

    // events.cpp
    void failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason);
    void failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason);
    void failsafe_short_off_event(ModeReason reason);
    void failsafe_long_off_event(ModeReason reason);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    bool failsafe_in_landing_sequence() const;  // returns true if the vehicle is in landing sequence.  Intended only for use in failsafe code.

#if AP_FENCE_ENABLED
    // fence.cpp
    void fence_check();
    bool fence_stickmixing() const;
    bool in_fence_recovery() const;
#endif

    // ArduPlane.cpp
    void disarm_if_autoland_complete();
    void get_osd_roll_pitch_rad(float &roll, float &pitch) const override;
    float tecs_hgt_afe(void);
    void efi_update(void);
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif
    void one_second_loop(void);
    void three_hz_loop(void);
#if AP_AIRSPEED_AUTOCAL_ENABLE
    void airspeed_ratio_update(void);
#endif
    void compass_save(void);
    void update_logging10(void);
    void update_logging25(void);
    void update_control_mode(void);
    void update_fly_forward(void);
    void update_flight_stage();
    void set_flight_stage(AP_FixedWing::FlightStage fs);

    // navigation.cpp
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void calc_airspeed_errors();
    float mode_auto_target_airspeed_cm();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_loiter_update_nav(uint16_t radius);
    void update_cruise();
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);

    // radio.cpp
    void set_control_channels(void) override;
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void rudder_arm_disarm_check();
    void read_radio();
    int16_t rudder_input(void);
    void control_failsafe();
    void trim_radio();
    bool rc_throttle_value_ok(void) const;
    bool rc_failsafe_active(void) const;

    // sensors.cpp
    void read_rangefinder(void);

    // system.cpp
    void init_ardupilot() override;
    void startup_ground(void);
    bool set_mode(Mode& new_mode, const ModeReason reason);
    bool set_mode(const uint8_t mode, const ModeReason reason) override;
    bool set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason);
    void check_long_failsafe();
    void check_short_failsafe();
    void startup_INS_ground(void);
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    void notify_mode(const Mode& mode);

    // takeoff.cpp
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void landing_gear_update(void);

    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);

    // servos.cpp
    void set_servos_idle(void);
    void set_servos();
    void set_servos_manual_passthrough(void);
    void set_servos_controlled(void);
    void set_servos_old_elevons(void);
    void set_servos_flaps(void);
    void set_landing_gear(void);
    void dspoiler_update(void);
    void airbrake_update(void);
    void landing_neutral_control_surface_servos(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void force_flare();
    void throttle_voltage_comp(int8_t &min_throttle, int8_t &max_throttle) const;
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    void throttle_slew_limit(SRV_Channel::Aux_servo_function_t func);
    bool suppress_throttle(void);
    void update_throttle_hover();
    void channel_function_mixer(SRV_Channel::Aux_servo_function_t func1_in, SRV_Channel::Aux_servo_function_t func2_in,
                                SRV_Channel::Aux_servo_function_t func1_out, SRV_Channel::Aux_servo_function_t func2_out) const;
    void flaperon_update();

    // is_flying.cpp
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    bool is_flying(void);

    // parachute.cpp
    void parachute_check();
#if PARACHUTE == ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
    void parachute_release();
    bool parachute_manual_release();
#endif

    // soaring.cpp
#if HAL_SOARING_ENABLED
    void update_soaring();
#endif

    // RC_Channel.cpp
    bool emergency_landing;

    // vehicle specific waypoint info helpers
    bool get_wp_distance_m(float &distance) const override;
    bool get_wp_bearing_deg(float &bearing) const override;
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;

    // reverse_thrust.cpp
    bool reversed_throttle;
    bool have_reverse_throttle_rc_option;
    bool allow_reverse_thrust(void) const;
    bool have_reverse_thrust(void) const;
    float get_throttle_input(bool no_deadzone=false) const;
    float get_adjusted_throttle_input(bool no_deadzone=false) const;

#if AP_SCRIPTING_ENABLED
    // support for NAV_SCRIPT_TIME mission command
    bool nav_scripting_active(void);
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;
    void nav_script_time_done(uint16_t id) override;

    // command throttle percentage and roll, pitch, yaw target
    // rates. For use with scripting controllers
    void set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps) override;
    bool nav_scripting_enable(uint8_t mode) override;
#endif
 
    enum Failsafe_Action {
        Failsafe_Action_None      = 0,
        Failsafe_Action_RTL       = 1,
        Failsafe_Action_Land      = 2,
        Failsafe_Action_Terminate = 3,
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_QLand     = 4,
#endif
        Failsafe_Action_Parachute = 5,
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_Loiter_alt_QLand = 6,
#endif
    };

    // list of priorities, highest priority first
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_Parachute,
#if HAL_QUADPLANE_ENABLED
                                                      Failsafe_Action_QLand,
#endif
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // EKF checks for loss of navigation performed in ekf_check.cpp
    // These are specific to VTOL operation
    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);

    enum class CrowMode {
        NORMAL,
        PROGRESSIVE,
        CROW_DISABLED,
    };

    enum class ThrFailsafe {
        Disabled    = 0,
        Enabled     = 1,
        EnabledNoFS = 2
    };

    CrowMode crow_mode = CrowMode::NORMAL;

    enum class FlareMode {
        FLARE_DISABLED = 0,
        ENABLED_NO_PITCH_TARGET,
        ENABLED_PITCH_TARGET
    };
    
    enum class AutoTuneAxis {
        ROLL  = 1U <<0,
        PITCH = 1U <<1,
        YAW   = 1U <<2,
    };

    FlareMode flare_mode;
    bool throttle_at_zero(void) const;

    // expo handling
    float roll_in_expo(bool use_dz) const;
    float pitch_in_expo(bool use_dz) const;
    float rudder_in_expo(bool use_dz) const;

    // mode reason for entering previous mode
    ModeReason previous_mode_reason = ModeReason::UNKNOWN;

    // last target alt we passed to tecs
    int32_t tecs_target_alt_cm;

public:
    void failsafe_check(void);
#if AP_SCRIPTING_ENABLED
    bool set_target_location(const Location& target_loc) override;
    bool get_target_location(Location& target_loc) override;
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;
    bool set_velocity_match(const Vector2f &velocity) override;
#endif // AP_SCRIPTING_ENABLED

};

extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
