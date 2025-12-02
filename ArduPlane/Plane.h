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
#include <AP_RangeFinder/AP_RangeFinder_config.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
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
#include <AP_Avoidance/AP_Avoidance_config.h>      // "ADSB" avoidance library
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
#include <AP_Follow/AP_Follow.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Plane.h"
#endif

#include <AC_PrecLand/AC_PrecLand_config.h>
#if AC_PRECLAND_ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
#endif

#include "GCS_MAVLink_Plane.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include <AP_Tuning/AP_Tuning_config.h>
#if AP_TUNING_ENABLED
#include "tuning.h"
#endif

// Configuration
#include "config.h"

#if AP_ADVANCEDFAILSAFE_ENABLED
#include "afs_plane.h"
#endif

// Local modules
#include "defines.h"
#include "mode.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel_Plane.h"     // RC Channel Library
#include "Parameters.h"
#if AP_ADSB_AVOIDANCE_ENABLED
#include "avoidance_adsb.h"
#endif  // AP_ADSB_AVOIDANCE_ENABLED
#include "AP_Arming_Plane.h"
#include "pullup.h"
#include "systemid.h"

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
    friend class VTOL_Assist;

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
#if MODE_AUTOLAND_ENABLED
    friend class ModeAutoLand;
#endif
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Plane;
#endif
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    friend class GliderPullup;
#endif
#if AP_PLANE_SYSTEMID_ENABLED
    friend class AP_SystemID;
#endif

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

    // scaled roll limit based on pitch
    int32_t roll_limit_cd;
    float pitch_limit_min;

    // flight modes convenience array
    AP_Int8 *flight_modes = &g.flight_mode1;
    const uint8_t num_flight_modes = 6;

#if AP_RANGEFINDER_ENABLED
    AP_FixedWing::Rangefinder_State rangefinder_state;

    /*
      orientation of rangefinder to use for landing
     */
    Rotation rangefinder_orientation(void) const {
        return Rotation(g2.rangefinder_land_orient.get());
    }

#endif

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    struct {
        // allow for external height above ground estimate
        float hagl;
        uint32_t last_update_ms;
        uint32_t timeout_ms;
    } external_hagl;
    bool get_external_HAGL(float &height_agl);
    void handle_external_hagl(const mavlink_command_int_t &packet);
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

    float get_landing_height(bool &using_rangefinder);

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

    // should throttle be pass-thru in guided?
    bool guided_throttle_passthru;

    // are we doing calibration? This is used to allow heartbeat to
    // external failsafe boards during baro and airspeed calibration
    bool in_calibration;

    // are we currently in long failsafe but have postponed it in MODE TAKEOFF until min level alt is reached
    bool long_failsafe_pending;

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

#if HAL_RALLY_ENABLED
    // Rally Points
    AP_Rally rally;
#endif

#if AC_PRECLAND_ENABLED
    void precland_update(void);
#endif

    // returns a Location for a rally point or home; if
    // HAL_RALLY_ENABLED is false, just home.
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt_amsl_cm) const;

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
#if MODE_AUTOLAND_ENABLED
    ModeAutoLand mode_autoland;
#endif
#if HAL_SOARING_ENABLED
    ModeThermal mode_thermal;
#endif

#if AP_QUICKTUNE_ENABLED
    AP_Quicktune quicktune;
#endif
    
    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    Mode *control_mode = &mode_initializing;
    Mode *previous_mode = &mode_initializing;

    // time of last mode change
    uint32_t last_mode_change_ms;

    // This is used to enable the inverted flight feature
    bool inverted_flight;

    // last time we ran roll/pitch stabilization
    uint32_t last_stabilize_ms;

    // Failsafe
    struct {
        // Used to track if the value on channel 3 (throttle) has fallen below the failsafe threshold
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

        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

#if HAL_QUADPLANE_ENABLED
    // Landing
    class VTOLApproach {
    public:
        enum class Stage {
            RTL,
            LOITER_TO_ALT,
            ENSURE_RADIUS,
            WAIT_FOR_BREAKOUT,
            APPROACH_LINE,
            VTOL_LANDING,
        };

        Stage approach_stage;
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
    bool groundspeed_undershoot_is_valid;
    float last_groundspeed_undershoot_offset;

    // speed scaler for control surfaces, updated at 10Hz
    float surface_speed_scaler = 1.0;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t rudder_takeoff_warn_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
        bool waiting_for_rudder_neutral;
        float throttle_lim_max;
        float throttle_lim_min;
        uint32_t throttle_max_timer_ms;
        uint32_t level_off_start_time_ms;
        // Good candidate for keeping the initial time for TKOFF_THR_MAX_T.
#if MODE_AUTOLAND_ENABLED
       struct {
            float heading; // deg
            bool initialized;
        } initial_direction;
#endif
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

        // last home altitude for detecting changes
        int32_t last_home_alt_cm;

        // have we finished the takeoff ratation (when it applies)?
        bool rotation_complete;
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
        float rudder_offset_pct;
        bool run_yaw_rate_controller;
    } nav_scripting;
#endif

    struct GuidedState {
        // roll pitch yaw commanded from external controller in centidegrees
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        float forced_throttle;
        uint32_t last_forced_throttle_ms;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        // airspeed adjustments
        float target_airspeed_cm = -1;  // don't default to zero here, as zero is a valid speed.
        float target_airspeed_accel;
        uint32_t target_airspeed_time_ms;

        // altitude adjustments
        Location target_location;
        // target_location altitude is uses to hold some flag values:
        bool target_location_alt_is_minus_one() const;

        float target_alt_rate;
        uint32_t target_alt_time_ms = 0;
        uint8_t target_mav_frame = -1;

        // heading track
        float target_heading = -4; // don't default to zero or -1 here, as both are valid headings in radians
        float target_heading_accel_limit;
        uint32_t target_heading_time_ms;
        guided_heading_type_t target_heading_type;
        bool target_heading_limit;
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } guided_state;

#if AP_LANDINGGEAR_ENABLED
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

#if AP_BATTERY_WATT_MAX_ENABLED
    // reduce throttle to eliminate battery over-current
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust
    uint32_t throttle_watt_limit_timer_ms;
#endif

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

    // the aerodynamic load factor. This is calculated from the demanded
    // roll before the roll is clipped, using 1/cos(nav_roll)
    float aerodynamic_load_factor = 1.0f;

    // a smoothed airspeed estimate, used for limiting roll angle
    float smoothed_airspeed;

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if HAL_PARACHUTE_ENABLED
    AP_Parachute parachute;
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
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    // avoidance of adsb enabled vehicles (normally manned vehicles)
    AP_Avoidance_Plane avoidance_adsb{adsb};
#endif  // AP_ADSB_AVOIDANCE_ENABLED

    // Outback Challenge Failsafe Support
#if AP_ADVANCEDFAILSAFE_ENABLED
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
        bool unable_to_achieve_target_alt;

        // start time of the loiter.  Milliseconds.
        uint32_t start_time_ms;

        // altitude at start of loiter loop lap. Used to detect delta alt of each lap.
        // only valid when sum_cd > 36000
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        uint32_t time_max_ms;

        // current value of loiter radius in metres used by the controller
        float radius;
    } loiter;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;

    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    uint32_t condition_start;

    // 3D Location vectors
    // Location structure defined in AP_Common
    const Location &home = ahrs.get_home();

    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    Location prev_WP_loc {};

    // The plane's current location
    Location current_loc {};

    // The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
    Location next_WP_loc {};

    // Altitude control
    struct {
        // target altitude above sea level in cm. Used for barometric
        // altitude navigation
        int32_t amsl_cm;

        // Altitude difference between previous and current waypoint in
        // centimeters. Used for altitude slope handling
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // are we trying to follow terrain?
        bool terrain_following;

        // are we waiting to load terrain data to init terrain following
        bool terrain_following_pending;

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

    // Arming/Disarming management class
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    // external control library
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Plane external_control;
#endif

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];

#if HAL_QUADPLANE_ENABLED
    // support for quadcopter-plane
    QuadPlane quadplane{ahrs};
#endif

#if AP_TUNING_ENABLED
    // support for transmitter tuning
    AP_Tuning_Plane tuning;
#endif

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
        AUTOLAND        = 1U << 12,
    };
    struct TerrainLookupTable{
       Mode::Number mode_num;
       terrain_bitmask bitmask;
    };
    static const TerrainLookupTable Terrain_lookup[];
#endif

#if AP_QUICKTUNE_ENABLED
    void update_quicktune(void);
#endif

    // Attitude.cpp
    void adjust_nav_pitch_throttle(void);
    void update_load_factor(void);
    void apply_load_factor_roll_limits(void);
    void adjust_altitude_target();
    void setup_alt_slope(void);
    int32_t get_RTL_altitude_cm() const;
    bool rangefinder_use(enum RangeFinderUse rangefinder_use) const;
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use);
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use, bool use_terrain_if_available);
    void set_target_altitude_current(void);
    void set_target_altitude_location(const Location &loc);
    int32_t relative_target_altitude_cm(void);
    void change_target_altitude(int32_t change_cm);
    void set_target_altitude_proportion(const Location &loc, float proportion);
#if AP_TERRAIN_AVAILABLE
    bool set_target_altitude_proportion_terrain(void);
#endif
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);
    int32_t calc_altitude_error_cm(void);
    void check_fbwb_altitude(void);
    void reset_offset_altitude(void);
    void set_offset_altitude_location(const Location &start_loc, const Location &destination_loc);
    bool above_location_current(const Location &loc);
    void setup_terrain_target_alt(Location &loc);
    int32_t adjusted_altitude_cm(void);
    int32_t adjusted_relative_altitude_cm(void);
    float mission_alt_offset(void);
    float height_above_target(void);
    float lookahead_adjustment(void);
    void fix_terrain_WP(Location &loc, uint32_t linenum);
#if AP_RANGEFINDER_ENABLED
    float rangefinder_correction(void);
    void rangefinder_height_update(void);
    void rangefinder_terrain_correction(float &height);
#endif
    void stabilize();
    void calc_throttle();
    void calc_nav_roll();
    void calc_nav_pitch();
    float calc_speed_scaler(void);
    float get_speed_scaler(void) const { return surface_speed_scaler; }
    bool stick_mixing_enabled(void);
    void stabilize_roll();
    float stabilize_roll_get_roll_out();
    void stabilize_pitch();
    float stabilize_pitch_get_pitch_out();
    void stabilize_stick_mixing_fbw();
    void stabilize_yaw();
    int16_t calc_nav_yaw_coordinated();
    int16_t calc_nav_yaw_course(void);
    int16_t calc_nav_yaw_ground(void);

#if HAL_LOGGING_ENABLED

    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;

    // Log.cpp
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

#if AP_PLANE_BLACKBOX_LOGGING
    void Log_Write_Blackbox(void);
#endif
#endif

    // Parameters.cpp
    void load_parameters(void) override;

    // commands_logic.cpp
    void set_next_WP(const Location &loc);
    void do_RTL(int32_t alt);
    bool verify_takeoff();
    bool verify_loiter_unlim(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_time();
    bool verify_loiter_turns(const AP_Mission::Mission_Command &cmd);
    bool verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd);
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
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
#if HAL_QUADPLANE_ENABLED
    // vtol takeoff from AP_Vehicle for quadplane.
    bool start_takeoff(const float alt_m) override;
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);
#endif
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    float get_wp_radius() const;

    bool is_land_command(uint16_t cmd) const;

    bool do_change_speed(SPEED_TYPE speedtype, float speed_target_ms, float rhtottle_pct);
    /*
      return true if in a specific AUTO mission command
    */
    bool in_auto_mission_id(uint16_t command) const;

#if AP_SCRIPTING_ENABLED
    // nav scripting support
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif

    // commands.cpp
    void set_guided_WP(const Location &loc);

    // update home position. Return true if update done
    bool update_home();

    // update current_loc
    void update_current_loc(void);

    // set home location and store it persistently:
    bool set_home_persistently(const Location &loc) WARN_IF_UNUSED;
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    // control_modes.cpp
    void autotune_start(void);
    void autotune_restore(void);
    void autotune_enable(bool enable);
    bool fly_inverted(void);
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }
    Mode *mode_from_mode_num(const enum Mode::Number num);
    bool current_mode_requires_mission() const override {
        return control_mode == &mode_auto;
    }

    bool autotuning;

    // events.cpp
    void rc_failsafe_short_on_event();
    void failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason);
    void rc_failsafe_short_off_event();
    void failsafe_long_off_event(ModeReason reason);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    bool failsafe_in_landing_sequence() const;  // returns true if the vehicle is in landing sequence.  Intended only for use in failsafe code.

#if AP_FENCE_ENABLED
    // fence.cpp
    void fence_check();
    void fence_checks_async() override;
    bool fence_stickmixing() const;
    bool in_fence_recovery() const;
    uint8_t orig_breaches;
#endif

    // Plane.cpp
    void disarm_if_autoland_complete();
    bool trigger_land_abort(const float climb_to_alt_m);
    void get_osd_roll_pitch_rad(float &roll, float &pitch) const override;
    float tecs_hgt_afe(void);
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void ahrs_update();
    void update_speed_height(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_compass(void);
    void update_alt(void);
#if AP_ADVANCEDFAILSAFE_ENABLED
    void afs_fs_check(void);
#endif
    void one_second_loop(void);
    void three_hz_loop(void);
#if AP_AIRSPEED_AUTOCAL_ENABLE
    void airspeed_ratio_update(void);
#endif
    void update_logging10(void);
    void update_logging25(void);
    void update_control_mode(void);
    void update_fly_forward(void);
    void update_flight_stage();
    void set_flight_stage(AP_FixedWing::FlightStage fs);
    bool flight_option_enabled(FlightOptions flight_option) const;

    // navigation.cpp
    void loiter_angle_reset(void);
    void loiter_angle_update(void);
    void navigate();
    void check_home_alt_change(void);
    void calc_airspeed_errors();
    float mode_auto_target_airspeed_cm();
    void calc_gndspeed_undershoot();
    void update_loiter(uint16_t radius);
    void update_loiter_update_nav(uint16_t radius);
    void update_fbwb_speed_height(void);
    void setup_turn_angle(void);
    bool reached_loiter_target(void);

    // radio.cpp
    void set_control_channels(void) override;
    void init_rc_in();
    void init_rc_out_main();
    void init_rc_out_aux();
    void read_radio();
    int16_t rudder_input(void);
    void control_failsafe();
    void trim_radio();
    bool rc_throttle_value_ok(void) const;
    bool rc_failsafe_active(void) const;

#if AP_RANGEFINDER_ENABLED
    // sensors.cpp
    void read_rangefinder(void);
#endif

    // system.cpp
    __INITFUNC__ void init_ardupilot() override;
    bool set_mode(Mode& new_mode, const ModeReason reason);
    bool set_mode(const uint8_t mode, const ModeReason reason) override;
    bool set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason);
    void check_long_failsafe();
    void check_short_rc_failsafe();
    void startup_INS(void);
    bool should_log(uint32_t mask);
    int8_t throttle_percentage(void);
    void notify_mode(const Mode& mode);
    bool gcs_mode_enabled(const Mode::Number mode_num) const;

    // takeoff.cpp
    bool auto_takeoff_check(void);
    void takeoff_calc_roll(void);
    void takeoff_calc_pitch(void);
    void takeoff_calc_throttle();
    int8_t takeoff_tail_hold(void);
    int16_t get_takeoff_pitch_min_cd(void);
    void landing_gear_update(void);
    bool check_takeoff_timeout(void);
    bool check_takeoff_timeout_level_off(void);

    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);

    // servos.cpp
    void set_servos();
    float apply_throttle_limits(float throttle_in);
    void set_throttle(void);
    void set_takeoff_expected(void);
    void set_servos_flaps(void);
    void dspoiler_update(void);
    void airbrake_update(void);
    void landing_neutral_control_surface_servos(void);
    void servos_output(void);
    void servos_auto_trim(void);
    void servos_twin_engine_mix();
    void force_flare();
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);
    void throttle_slew_limit();
    bool suppress_throttle(void);
    void update_throttle_hover();
    void channel_function_mixer(SRV_Channel::Function func1_in, SRV_Channel::Function func2_in,
                                SRV_Channel::Function func1_out, SRV_Channel::Function func2_out) const;
    void flaperon_update();
    void indicate_waiting_for_rud_neutral_to_takeoff(void);

    // is_flying.cpp
    void update_is_flying_5Hz(void);
    void crash_detection_update(void);
    bool in_preLaunch_flight_stage(void);
    bool is_flying(void);

    // parachute.cpp
    void parachute_check();
#if HAL_PARACHUTE_ENABLED
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
    bool reverse_thrust_enabled(UseReverseThrust use_reverse_thrust_option) const;

#if AP_SCRIPTING_ENABLED
    // support for NAV_SCRIPT_TIME mission command
    bool nav_scripting_active(void);
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;
    void nav_script_time_done(uint16_t id) override;

    // command throttle percentage and roll, pitch, yaw target
    // rates. For use with scripting controllers
    void set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps) override;
    void set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller) override;
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
        Failsafe_Action_AUTOLAND_OR_RTL = 7,
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

    using ThrFailsafe = Parameters::ThrFailsafe;

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
    bool is_landing() const override;
    bool is_taking_off() const override;
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
    bool set_target_location(const Location& target_loc) override;
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if AP_SCRIPTING_ENABLED
    bool get_target_location(Location& target_loc) override;
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;
    bool set_velocity_match(const Vector2f &velocity) override;

    // allow for landing descent rate to be overridden by a script, may be -ve to climb
    bool set_land_descent_rate(float descent_rate) override;

    // allow scripts to override mission/guided crosstrack behaviour
    // It's up to the Lua script to ensure the provided location makes sense
    bool set_crosstrack_start(const Location &new_start_location) override;

#endif // AP_SCRIPTING_ENABLED

    bool tkoff_option_is_set(AP_FixedWing::TakeoffOption option) const {
        return (aparm.takeoff_options & int32_t(option)) != 0;
    }
   

};

extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
