#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_QUADPLANE_ENABLED
#define HAL_QUADPLANE_ENABLED 1
#endif

#if HAL_QUADPLANE_ENABLED

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AC_AttitudeControl/AC_CommandModel.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_WeatherVane.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Proximity/AP_Proximity.h>
#include "qautotune.h"
#include "defines.h"
#include "tailsitter.h"
#include "tiltrotor.h"
#include "transition.h"

/*
  QuadPlane specific functionality
 */
class QuadPlane
{
public:
    friend class Plane;
    friend class AP_Tuning_Plane;
    friend class GCS_MAVLINK_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class QAutoTune;
    friend class AP_Arming_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channel;
    friend class Tailsitter;
    friend class Tiltrotor;
    friend class SLT_Transition;
    friend class Tailsitter_Transition;

    friend class Mode;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeQHover;
    friend class ModeQLand;
    friend class ModeQLoiter;
    friend class ModeQRTL;
    friend class ModeQStabilize;
    friend class ModeQAutotune;
    friend class ModeQAcro;
    friend class ModeLoiterAltQLand;
    
    QuadPlane(AP_AHRS &_ahrs);

    static QuadPlane *get_singleton() {
        return _singleton;
    }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    void control_auto(void);
    bool setup(void);

    void vtol_position_controller(void);
    void setup_target_position(void);
    void takeoff_controller(void);
    void waypoint_controller(void);
    void update_land_positioning(void);

    void update_throttle_mix(void);
    
    // update transition handling
    void update(void);

    // set motor arming
    void set_armed(bool armed);

    // is VTOL available?
    bool available(void) const {
        return initialised;
    }

    // is quadplane assisting?
    bool in_assisted_flight(void) const {
        return available() && assisted_flight;
    }

    // abort landing, only valid when in a VTOL landing descent
    bool abort_landing(void);

    /*
      return true if we are in a transition to fwd flight from hover
    */
    bool in_transition(void) const;

    bool handle_do_vtol_transition(enum MAV_VTOL_STATE state) const;

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(void);
    bool in_vtol_auto(void) const;
    bool in_vtol_mode(void) const;
    bool in_vtol_takeoff(void) const;
    bool in_vtol_posvel_mode(void) const;
    void update_throttle_hover();
    bool show_vtol_view() const;

    // vtol help for is_flying()
    bool is_flying(void);

    // return desired forward throttle percentage
    float forward_throttle_pct();
    float get_weathervane_yaw_rate_cds(void);

    // see if we are flying from vtol point of view
    bool is_flying_vtol(void) const;

    // user initiated takeoff for guided mode
    bool do_user_takeoff(float takeoff_altitude);

    // return true if the wp_nav controller is being updated
    bool using_wp_nav(void) const;

    // return true if the user has set ENABLE
    bool enabled(void) const { return enable != 0; }
    
    // is throttle controlled landing descent active?
    bool thr_ctrl_land;

    uint16_t get_pilot_velocity_z_max_dn() const;
    
    struct PACKED log_QControl_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float    throttle_in;
        float    angle_boost;
        float    throttle_out;
        float    throttle_hover;
        float    desired_alt;
        float    inav_alt;
        int32_t  baro_alt;
        int16_t  target_climb_rate;
        int16_t  climb_rate;
        float    throttle_mix;
        float    speed_scaler;
        uint8_t  transition_state;
        uint8_t  assist;
    };

    MAV_TYPE get_mav_type(void) const;

    enum Q_ASSIST_STATE_ENUM {
        Q_ASSIST_DISABLED,
        Q_ASSIST_ENABLED,
        Q_ASSIST_FORCE,
    };
    void set_q_assist_state(Q_ASSIST_STATE_ENUM state) {q_assist_state = state;};

    // called when we change mode (for any mode, not just Q modes)
    void mode_enter(void);

    // Check if servo auto trim is allowed
    bool allow_servo_auto_trim();

    /*
      are we in the descent phase of a VTOL landing?
     */
    bool in_vtol_land_descent(void) const;

    // Should we allow stick mixing from the pilot
    bool allow_stick_mixing() const;

private:
    AP_AHRS &ahrs;

    // key aircraft parameters passed to multiple libraries
    AP_MultiCopter aparm;

    AP_InertialNav inertial_nav{ahrs};

    AP_Enum<AP_Motors::motor_frame_class> frame_class;
    AP_Enum<AP_Motors::motor_frame_type> frame_type;

    // Initialise motors to allow passing it to tailsitter in its constructor
    AP_MotorsMulticopter *motors = nullptr;
    const struct AP_Param::GroupInfo *motors_var_info;

    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Loiter *loiter_nav;
    
    // maximum vertical velocity the pilot may request
    AP_Int16 pilot_velocity_z_max_up;
    AP_Int16 pilot_velocity_z_max_dn;

    // vertical acceleration the pilot may request
    AP_Int16 pilot_accel_z;

    // air mode state: OFF, ON, ASSISTED_FLIGHT_ONLY
    AirMode air_mode;

    // Command model parameter class
    // Default max rate, default expo, default time constant
    AC_CommandModel command_model_pilot{100.0, 0.25, 0.25};
    // helper functions to set and disable time constant from command model
    void set_pilot_yaw_rate_time_constant();
    void disable_yaw_rate_time_constant();

    // return true if airmode should be active
    bool air_mode_active() const;

    // check for quadplane assistance needed
    bool should_assist(float aspeed, bool have_airspeed);

    // check for an EKF yaw reset
    void check_yaw_reset(void);
    
    // hold hover (for transition)
    void hold_hover(float target_climb_rate_cms);

    // hold stabilize (for transition)
    void hold_stabilize(float throttle_in);

    // set climb rate in position controller
    void set_climb_rate_cms(float target_climb_rate_cms);

    // get pilot desired yaw rate in cd/s
    float get_pilot_input_yaw_rate_cds(void) const;

    // get overall desired yaw rate in cd/s
    float get_desired_yaw_rate_cds(bool weathervane=true);
    
    // get desired climb rate in cm/s
    float get_pilot_desired_climb_rate_cms(void) const;

    // get pilot lean angle
    void get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const;

    // get pilot throttle in for landing code. Return value on scale of 0 to 1
    float get_pilot_land_throttle(void) const;

    // initialise throttle_wait when entering mode
    void init_throttle_wait();

    // use multicopter rate controller
    void multicopter_attitude_rate_update(float yaw_rate_cds);

    float get_pilot_throttle(void);
    void control_hover(void);
    void relax_attitude_control();

    bool check_land_complete(void);
    bool land_detector(uint32_t timeout_ms);
    bool check_land_final(void);

    float assist_climb_rate_cms(void) const;

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(void) const;

    bool should_relax(void);
    void motors_output(bool run_rate_controller = true);
    void Log_Write_QControl_Tuning();
    void log_QPOS(void);
    float landing_descent_rate_cms(float height_above_ground);
    
    // setup correct aux channels for frame class
    void setup_default_channels(uint8_t num_motors);

    void guided_start(void);
    void guided_update(void);

    void update_throttle_suppression(void);

    void run_z_controller(void);
    void run_xy_controller(float accel_limit=0.0);

    void setup_defaults(void);

    // calculate a stopping distance for fixed-wing to vtol transitions
    float stopping_distance(float ground_speed_squared) const;
    float accel_needed(float stop_distance, float ground_speed_squared) const;
    float stopping_distance(void);

    // distance below which we don't do approach, based on stopping
    // distance for cruise speed
    float transition_threshold(void);

    AP_Int16 transition_time_ms;
    AP_Int16 back_trans_pitch_limit_ms;

    // transition deceleration, m/s/s
    AP_Float transition_decel;

    // transition failure handling
    struct TRANS_FAIL {
        enum ACTION {
            QLAND,
            QRTL
        };
        AP_Int16 timeout;
        AP_Enum<ACTION> action;
        bool warned;
    } transition_failure;


    // Quadplane trim, degrees
    AP_Float ahrs_trim_pitch;
    float _last_ahrs_trim_pitch;

    // fw landing approach radius
    AP_Float fw_land_approach_radius;

    AP_Int16 rc_speed;

    // speed below which quad assistance is given
    AP_Float assist_speed;

    // angular error at which quad assistance is given
    AP_Int8 assist_angle;
    uint32_t angle_error_start_ms;
    AP_Float assist_delay;

    // altitude to trigger assistance
    AP_Int16 assist_alt;
    uint32_t alt_error_start_ms;
    bool in_alt_assist;

    // landing speed in cm/s
    AP_Int16 land_speed_cms;

    // QRTL start altitude, meters
    AP_Int16 qrtl_alt;
    AP_Int16 qrtl_alt_min;
    
    // alt to switch to QLAND_FINAL
    AP_Float land_final_alt;
    AP_Float vel_forward_alt_cutoff;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;

    // control if a VTOL RTL will be used
    AP_Int8 rtl_mode;
    enum RTL_MODE{
        NONE,
        SWITCH_QRTL,
        VTOL_APPROACH_QRTL,
        QRTL_ALWAYS,
    };

    // control if a VTOL GUIDED will be used
    AP_Int8 guided_mode;

    // control ESC throttle calibration
    AP_Int8 esc_calibration;
    void run_esc_calibration(void);

    // ICEngine control on landing
    AP_Int8 land_icengine_cut;

    // HEARTBEAT mav_type override
    AP_Int8 mav_type;

    // manual throttle curve expo strength
    AP_Float throttle_expo;

    // manual forward throttle input
    AP_Float fwd_thr_max;
    RC_Channel *rc_fwd_thr_ch;

    // QACRO mode max roll/pitch/yaw rates
    AP_Float acro_roll_rate;
    AP_Float acro_pitch_rate;
    AP_Float acro_yaw_rate;

    // time we last got an EKF yaw reset
    uint32_t ekfYawReset_ms;

    struct {
        AP_Float gain;
        float integrator;
        uint32_t last_ms;
        float last_pct;
    } vel_forward;

    AC_WeatherVane *weathervane;

    bool initialised;

    Location last_auto_target;

    // when did we last run the attitude controller?
    uint32_t last_att_control_ms;

    // transition logic
    Transition *transition = nullptr;

    // true when waiting for pilot throttle
    bool throttle_wait:1;

    // true when quad is assisting a fixed wing mode
    bool assisted_flight:1;

    // true when in angle assist
    bool in_angle_assist:1;

    // are we in a guided takeoff?
    bool guided_takeoff:1;

    /* if we arm in guided mode when we arm then go into a "waiting
       for takeoff command" state. In this state we are waiting for
       one of the following:

       1) disarm
       2) guided takeoff command
       3) change to AUTO with a takeoff waypoint as first nav waypoint
       4) change to another mode

       while in this state we don't go to throttle unlimited, and will
       refuse a change to AUTO mode if the first waypoint is not a
       takeoff. If we try to switch to RTL then we will instead use
       QLAND

       This state is needed to cope with the takeoff sequence used
       by QGC on common controllers such as the MX16, which do this on a "takeoff" swipe:

          - changes mode to GUIDED
          - arms
          - changes mode to AUTO
    */
    bool guided_wait_takeoff;
    bool guided_wait_takeoff_on_mode_enter;

    struct {
        // time when motors reached lower limit
        uint32_t lower_limit_start_ms;
        uint32_t land_start_ms;
        float vpos_start_m;

        // landing detection threshold in meters
        AP_Float detect_alt_change;
    } landing_detect;

    // throttle mix acceleration filter
    LowPassFilterVector3f throttle_mix_accel_ef_filter{1.0};

    // time we last set the loiter target
    uint32_t last_loiter_ms;

    enum position_control_state {
        QPOS_NONE = 0,
        QPOS_APPROACH,
        QPOS_AIRBRAKE,
        QPOS_POSITION1,
        QPOS_POSITION2,
        QPOS_LAND_DESCEND,
        QPOS_LAND_ABORT,
        QPOS_LAND_FINAL,
        QPOS_LAND_COMPLETE
    };
    class PosControlState {
    public:
        enum position_control_state get_state() const {
            return state;
        }
        void set_state(enum position_control_state s);
        uint32_t time_since_state_start_ms() const {
            return AP_HAL::millis() - last_state_change_ms;
        }
        Vector3p target_cm;
        Vector2f xy_correction;
        Vector3f target_vel_cms;
        bool slow_descent:1;
        bool pilot_correction_active;
        bool pilot_correction_done;
        uint32_t thrust_loss_start_ms;
        uint32_t last_log_ms;
        bool reached_wp_speed;
        uint32_t last_run_ms;
        float pos1_speed_limit;
        bool done_accel_init;
        Vector2f velocity_match;
        uint32_t last_velocity_match_ms;
        float target_speed;
        float target_accel;
        uint32_t last_pos_reset_ms;
        bool overshoot;

        float override_descent_rate;
        uint32_t last_override_descent_ms;
    private:
        uint32_t last_state_change_ms;
        enum position_control_state state;
    } poscontrol;

    struct {
        bool running;
        uint32_t start_ms;            // system time the motor test began
        uint32_t timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
        uint8_t seq = 0;              // motor sequence number of motor being tested
        uint8_t throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
        uint16_t throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type
        uint8_t motor_count;          // number of motors to cycle
    } motor_test;

    // time of last control log message
    uint32_t last_ctrl_log_ms;

    // time of last QTUN log message
    uint32_t last_qtun_log_ms;

    // Tiltrotor control
    Tiltrotor tiltrotor{*this, motors};

    // tailsitter control
    Tailsitter tailsitter{*this, motors};

    // the attitude view of the VTOL attitude controller
    AP_AHRS_View *ahrs_view;

    // time when motors were last active
    uint32_t last_motors_active_ms;

    // time when we last ran the vertical accel controller
    uint32_t last_pidz_active_ms;
    uint32_t last_pidz_init_ms;

    // throttle scailing for vectored motors in FW flighy
    float FW_vector_throttle_scaling(void);

    void afs_terminate(void);
    bool guided_mode_enabled(void);

    // set altitude target to current altitude
    void set_alt_target_current(void);

    // additional options
    AP_Int32 options;
    enum class OPTION {
        LEVEL_TRANSITION=(1<<0),
        ALLOW_FW_TAKEOFF=(1<<1),
        ALLOW_FW_LAND=(1<<2),
        RESPECT_TAKEOFF_FRAME=(1<<3),
        MISSION_LAND_FW_APPROACH=(1<<4),
        FS_QRTL=(1<<5),
        IDLE_GOV_MANUAL=(1<<6),
        Q_ASSIST_FORCE_ENABLE=(1<<7),
        TAILSIT_Q_ASSIST_MOTORS_ONLY=(1<<8),
        AIRMODE_UNUSED=(1<<9),
        DISARMED_TILT=(1<<10),
        DELAY_ARMING=(1<<11),
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST=(1<<12),
        DISABLE_GROUND_EFFECT_COMP=(1<<13),
        INGORE_FW_ANGLE_LIMITS_IN_Q_MODES=(1<<14),
        THR_LANDING_CONTROL=(1<<15),
        DISABLE_APPROACH=(1<<16),
        REPOSITION_LANDING=(1<<17),
        ONLY_ARM_IN_QMODE_OR_AUTO=(1<<18),
        TRANS_FAIL_TO_FW=(1<<19),
        FS_RTL=(1<<20),
        DISARMED_TILT_UP=(1<<21),
    };
    bool option_is_set(OPTION option) const {
        return (options.get() & int32_t(option)) != 0;
    }

    AP_Float takeoff_failure_scalar;
    AP_Float maximum_takeoff_airspeed;
    uint32_t takeoff_start_time_ms;
    uint32_t takeoff_time_limit_ms;

    float last_land_final_agl;

    // AHRS alt for land abort and package place, meters
    float land_descend_start_alt;

    // min alt for navigation in takeoff
    AP_Float takeoff_navalt_min;
    uint32_t takeoff_last_run_ms;
    float takeoff_start_alt;

    // oneshot with duration ARMING_DELAY_MS used by quadplane to delay spoolup after arming:
    // ignored unless OPTION_DELAY_ARMING or OPTION_TILT_DISARMED is set
    bool delay_arming;

    /*
      return true if current mission item is a vtol takeoff
     */
    bool is_vtol_takeoff(uint16_t id) const;

    /*
      return true if current mission item is a vtol landing
     */
    bool is_vtol_land(uint16_t id) const;

#if QAUTOTUNE_ENABLED
    // qautotune mode
    QAutoTune qautotune;
#endif

    /*
      are we in the approach phase of a VTOL landing?
     */
    bool in_vtol_land_approach(void) const;

    /*
      are we in the final landing phase of a VTOL landing?
     */
    bool in_vtol_land_final(void) const;

    /*
      are we in any of the phases of a VTOL landing?
     */
    bool in_vtol_land_sequence(void) const;

    /*
      see if we are in the VTOL position control phase of a landing
    */
    bool in_vtol_land_poscontrol(void) const;

    /*
      are we in the airbrake phase of a VTOL landing?
     */
    bool in_vtol_airbrake(void) const;

    // returns true if the vehicle should currently be doing a spiral landing
    bool landing_with_fixed_wing_spiral_approach(void) const;

    // Q assist state, can be enabled, disabled or force. Default to enabled
    Q_ASSIST_STATE_ENUM q_assist_state = Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED;

    /*
      return true if we should use the fixed wing attitude control loop
     */
    bool use_fw_attitude_controllers(void) const;

    /*
      get the airspeed for landing approach
     */
    float get_land_airspeed(void);

    /*
      setup for landing approach
     */
    void poscontrol_init_approach(void);

    /*
      calculate our closing velocity vector on the landing
      point. Takes account of the landing point having a velocity
     */
    Vector2f landing_closing_velocity();

    /*
      calculate our desired closing velocity vector on the landing point.
    */
    Vector2f landing_desired_closing_velocity();

    /*
      change spool state, providing easy hook for catching changes in debug
     */
    void set_desired_spool_state(AP_Motors::DesiredSpoolState state);

    /*
      get a scaled Q_WP_SPEED based on direction of movement
     */
    float get_scaled_wp_speed(float target_bearing_deg) const;

    /*
      setup scaling of roll and pitch angle P gains to match fixed wing gains
     */
    void setup_rp_fw_angle_gains(void);

public:
    void motor_test_output();
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                        uint16_t throttle_value, float timeout_sec,
                                        uint8_t motor_count);
private:
    void motor_test_stop();

    static QuadPlane *_singleton;
};

#endif  // HAL_QUADPLANE_ENABLED
