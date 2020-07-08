#pragma once

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_Fence/AC_Fence.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Proximity/AP_Proximity.h>
#include "qautotune.h"
#include "defines.h"

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

    friend class Mode;
    friend class ModeAuto;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeQHover;
    friend class ModeQLand;
    friend class ModeQLoiter;
    friend class ModeQRTL;
    friend class ModeQStabilize;
    friend class ModeQAutotune;
    friend class ModeQAcro;
    
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    void control_run(void);
    void control_auto(void);
    bool init_mode(void);
    bool setup(void);

    void vtol_position_controller(void);
    void setup_target_position(void);
    void takeoff_controller(void);
    void waypoint_controller(void);

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

    /*
      return true if we are in a transition to fwd flight from hover
    */
    bool in_transition(void) const;

    /*
      return true if we are a tailsitter transitioning to VTOL flight
    */
    bool in_tailsitter_vtol_transition(void) const;
    
    bool handle_do_vtol_transition(enum MAV_VTOL_STATE state);

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(void);
    bool in_vtol_auto(void) const;
    bool in_vtol_mode(void) const;
    bool in_vtol_posvel_mode(void) const;
    void update_throttle_hover();

    // vtol help for is_flying()
    bool is_flying(void);

    // return current throttle as a percentate
    uint8_t throttle_percentage(void) const {
        return last_throttle * 100;
    }

    // return desired forward throttle percentage
    int8_t forward_throttle_pct();
    float get_weathervane_yaw_rate_cds(void);

    // see if we are flying from vtol point of view
    bool is_flying_vtol(void) const;

    // return true when tailsitter frame configured
    bool is_tailsitter(void) const;

    // return true when flying a control surface only tailsitter tailsitter
    bool is_contol_surface_tailsitter(void) const;

    // return true when flying a tailsitter in VTOL
    bool tailsitter_active(void);
    
    // create outputs for tailsitters
    void tailsitter_output(void);

    // handle different tailsitter input types
    void tailsitter_check_input(void);
    
    // check if we have completed transition to fixed wing
    bool tailsitter_transition_fw_complete(void);

    // check if we have completed transition to vtol
    bool tailsitter_transition_vtol_complete(void) const;

    // account for control surface speed scaling in VTOL modes
    void tailsitter_speed_scaling(void);

    // user initiated takeoff for guided mode
    bool do_user_takeoff(float takeoff_altitude);

    // return true if the wp_nav controller is being updated
    bool using_wp_nav(void) const;

    // return true if the user has set ENABLE
    bool enabled(void) const { return enable != 0; }
    
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
    };

    MAV_TYPE get_mav_type(void) const;

    enum Q_ASSIST_STATE_ENUM {
        Q_ASSIST_DISABLED,
        Q_ASSIST_ENABLED,
        Q_ASSIST_FORCE,
    };
    void set_q_assist_state(Q_ASSIST_STATE_ENUM state) {q_assist_state = state;};

private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AP_Int8 frame_class;
    AP_Int8 frame_type;
    
    AP_MotorsMulticopter *motors;
    const struct AP_Param::GroupInfo *motors_var_info;
    
    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    AC_Loiter *loiter_nav;
    
    // maximum vertical velocity the pilot may request
    AP_Int16 pilot_velocity_z_max;

    // vertical acceleration the pilot may request
    AP_Int16 pilot_accel_z;

     // air mode state: OFF, ON
    AirMode air_mode;

   // check for quadplane assistance needed
    bool assistance_needed(float aspeed, bool have_airspeed);

    // check if it is safe to provide assistance
    bool assistance_safe();

    // update transition handling
    void update_transition(void);

    // check for an EKF yaw reset
    void check_yaw_reset(void);
    
    // hold hover (for transition)
    void hold_hover(float target_climb_rate);    

    // hold stabilize (for transition)
    void hold_stabilize(float throttle_in);    

    // get pilot desired yaw rate in cd/s
    float get_pilot_input_yaw_rate_cds(void) const;

    // get overall desired yaw rate in cd/s
    float get_desired_yaw_rate_cds(void);
    
    // get desired climb rate in cm/s
    float get_pilot_desired_climb_rate_cms(void) const;

    // initialise throttle_wait when entering mode
    void init_throttle_wait();

    // use multicopter rate controller
    void multicopter_attitude_rate_update(float yaw_rate_cds);
    
    // main entry points for VTOL flight modes
    void init_stabilize(void);
    void control_stabilize(void);

    void check_attitude_relax(void);
    void init_qacro(void);
    float get_pilot_throttle(void);
    void control_qacro(void);
    void init_hover(void);
    void control_hover(void);

    void init_loiter(void);
    void init_qland(void);
    void control_loiter(void);
    bool check_land_complete(void);
    bool land_detector(uint32_t timeout_ms);
    bool check_land_final(void);

    void init_qrtl(void);
    void control_qrtl(void);
    
    float assist_climb_rate_cms(void) const;

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(void) const;

    bool should_relax(void);
    void motors_output(bool run_rate_controller = true);
    void Log_Write_QControl_Tuning();
    float landing_descent_rate_cms(float height_above_ground) const;
    
    // setup correct aux channels for frame class
    void setup_default_channels(uint8_t num_motors);

    void guided_start(void);
    void guided_update(void);

    void update_throttle_suppression(void);

    void run_z_controller(void);

    void setup_defaults(void);

    // calculate a stopping distance for fixed-wing to vtol transitions
    float stopping_distance(void);
    
    AP_Int16 transition_time_ms;

    // transition deceleration, m/s/s
    AP_Float transition_decel;

    // transition failure milliseconds
    AP_Int16 transition_failure;

    // Quadplane trim, degrees
    AP_Float ahrs_trim_pitch;
    float _last_ahrs_trim_pitch;

    // fw landing approach radius
    AP_Float fw_land_approach_radius;

    AP_Int16 rc_speed;

    // min and max PWM for throttle
    AP_Int16 thr_min_pwm;
    AP_Int16 thr_max_pwm;

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

    // maximum yaw rate in degrees/second
    AP_Float yaw_rate_max;

    // landing speed in cm/s
    AP_Int16 land_speed_cms;

    // QRTL start altitude, meters
    AP_Int16 qrtl_alt;
    
    // alt to switch to QLAND_FINAL
    AP_Float land_final_alt;
    AP_Float vel_forward_alt_cutoff;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;

    // control if a VTOL RTL will be used
    AP_Int8 rtl_mode;

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
        int8_t last_pct;
    } vel_forward;

    struct {
        AP_Float gain;
        AP_Float min_roll;
        uint32_t last_pilot_input_ms;
        float last_output;
    } weathervane;
    
    bool initialised;
    
    // timer start for transition
    uint32_t transition_start_ms;
    float transition_initial_pitch;
    uint32_t transition_low_airspeed_ms;

    Location last_auto_target;

    // last throttle value when active
    float last_throttle;

    // pitch when we enter loiter mode
    int32_t loiter_initial_pitch_cd;

    // when did we last run the attitude controller?
    uint32_t last_att_control_ms;

    // true if we have reached the airspeed threshold for transition
    enum {
        TRANSITION_AIRSPEED_WAIT,
        TRANSITION_TIMER,
        TRANSITION_ANGLE_WAIT_FW,
        TRANSITION_ANGLE_WAIT_VTOL,
        TRANSITION_DONE
    } transition_state;

    // true when waiting for pilot throttle
    bool throttle_wait:1;

    // true when quad is assisting a fixed wing mode
    bool assisted_flight:1;

    // true when in angle assist
    bool in_angle_assist:1;

    // are we in a guided takeoff?
    bool guided_takeoff:1;
    
    struct {
        // time when motors reached lower limit
        uint32_t lower_limit_start_ms;
        uint32_t land_start_ms;
        float vpos_start_m;
    } landing_detect;

    // throttle mix acceleration filter
    LowPassFilterVector3f throttle_mix_accel_ef_filter = LowPassFilterVector3f(1.0f);

    // time we last set the loiter target
    uint32_t last_loiter_ms;

    enum position_control_state {
        QPOS_POSITION1,
        QPOS_POSITION2,
        QPOS_LAND_DESCEND,
        QPOS_LAND_FINAL,
        QPOS_LAND_COMPLETE
    };
    struct {
        enum position_control_state state;
        float speed_scale;
        Vector2f target_velocity;
        float max_speed;
        Vector3f target;
        bool slow_descent:1;
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

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3
    };

    // tiltrotor control variables
    struct {
        AP_Int16 tilt_mask;
        AP_Int16 max_rate_up_dps;
        AP_Int16 max_rate_down_dps;
        AP_Int8  max_angle_deg;
        AP_Int8  tilt_type;
        AP_Float tilt_yaw_angle;
        float current_tilt;
        float current_throttle;
        bool motors_active:1;
    } tilt;

    // bit 0 enables plane mode and bit 1 enables body-frame roll mode
    enum tailsitter_input {
        TAILSITTER_INPUT_PLANE   = (1U<<0),
        TAILSITTER_INPUT_BF_ROLL = (1U<<1)
    };

    enum tailsitter_mask {
        TAILSITTER_MASK_AILERON  = (1U<<0),
        TAILSITTER_MASK_ELEVATOR = (1U<<1),
        TAILSITTER_MASK_THROTTLE = (1U<<2),
        TAILSITTER_MASK_RUDDER   = (1U<<3),
    };

    enum tailsitter_gscl_mask {
        TAILSITTER_GSCL_BOOST   = (1U<<0),
        TAILSITTER_GSCL_ATT_THR = (1U<<1),
        TAILSITTER_GSCL_INTERP  = (1U<<2),
    };

    // tailsitter control variables
    struct {
        AP_Int8 transition_angle;
        AP_Int8 input_type;
        AP_Int8 input_mask;
        AP_Int8 input_mask_chan;
        AP_Float vectored_forward_gain;
        AP_Float vectored_hover_gain;
        AP_Float vectored_hover_power;
        AP_Float throttle_scale_max;
        AP_Float gain_scaling_min;
        AP_Float max_roll_angle;
        AP_Int16 motor_mask;
        AP_Float scaling_speed_min;
        AP_Float scaling_speed_max;
        AP_Int16 gain_scaling_mask;
    } tailsitter;

    // tailsitter speed scaler
    float last_spd_scaler = 1.0f;

    // the attitude view of the VTOL attitude controller
    AP_AHRS_View *ahrs_view;

    // time when motors were last active
    uint32_t last_motors_active_ms;

    // time when we last ran the vertical accel controller
    uint32_t last_pidz_active_ms;
    uint32_t last_pidz_init_ms;

    // time when we were last in a vtol control mode
    uint32_t last_vtol_mode_ms;
    
    void tiltrotor_slew(float tilt);
    void tiltrotor_binary_slew(bool forward);
    void tiltrotor_update(void);
    void tiltrotor_continuous_update(void);
    void tiltrotor_binary_update(void);
    void tiltrotor_vectored_yaw(void);
    void tiltrotor_bicopter(void);
    void tilt_compensate_up(float *thrust, uint8_t num_motors);
    void tilt_compensate_down(float *thrust, uint8_t num_motors);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool is_motor_tilting(uint8_t motor) const {
        return (((uint8_t)tilt.tilt_mask.get()) & (1U<<motor));
    }
    bool tiltrotor_fully_fwd(void);
    float tilt_max_change(bool up);

    void afs_terminate(void);
    bool guided_mode_enabled(void);

    // set altitude target to current altitude
    void set_alt_target_current(void);
    
    // adjust altitude target smoothly
    void adjust_alt_target(float target_cm);

    // additional options
    AP_Int32 options;
    enum {
        OPTION_LEVEL_TRANSITION=(1<<0),
        OPTION_ALLOW_FW_TAKEOFF=(1<<1),
        OPTION_ALLOW_FW_LAND=(1<<2),
        OPTION_RESPECT_TAKEOFF_FRAME=(1<<3),
        OPTION_MISSION_LAND_FW_APPROACH=(1<<4),
        OPTION_FS_QRTL=(1<<5),
        OPTION_IDLE_GOV_MANUAL=(1<<6),
        OPTION_Q_ASSIST_FORCE_ENABLE=(1<<7),
        OPTION_TAILSIT_Q_ASSIST_MOTORS_ONLY=(1<<8),
        OPTION_AIRMODE=(1<<9),
    };

    AP_Float takeoff_failure_scalar;
    AP_Float maximum_takeoff_airspeed;
    uint32_t takeoff_start_time_ms;
    uint32_t takeoff_time_limit_ms;

    float last_land_final_agl;

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
      are we in the descent phase of a VTOL landing?
     */
    bool in_vtol_land_descent(void) const;

    /*
      are we in the final landing phase of a VTOL landing?
     */
    bool in_vtol_land_final(void) const;

    /*
      are we in any of the phases of a VTOL landing?
     */
    bool in_vtol_land_sequence(void) const;

    // Q assist state, can be enabled, disabled or force. Default to enabled
    Q_ASSIST_STATE_ENUM q_assist_state = Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED;

public:
    void motor_test_output();
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                        uint16_t throttle_value, float timeout_sec,
                                        uint8_t motor_count);
private:
    void motor_test_stop();
};
