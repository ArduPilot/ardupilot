/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_WPNav/AC_WPNav.h>

/*
  QuadPlane specific functionality
 */
class QuadPlane
{
public:
    friend class Plane;
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void control_run(void);
    void control_auto(const Location &loc);
    bool init_mode(void);
    bool setup(void);
    
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
    
    bool handle_do_vtol_transition(const mavlink_command_long_t &packet);

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(const AP_Mission::Mission_Command &cmd);
    bool in_vtol_auto(void);

    // vtol help for is_flying()
    bool is_flying(void);

    // return current throttle as a percentate
    uint8_t throttle_percentage(void) const {
        return last_throttle * 0.1f;
    }
    
private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;
    AC_PID        pid_rate_roll {0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_PID        pid_rate_pitch{0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_PID        pid_rate_yaw  {0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_P          p_stabilize_roll{4.5};
    AC_P          p_stabilize_pitch{4.5};
    AC_P          p_stabilize_yaw{4.5};

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AC_P                    p_pos_xy{1};
    AC_P                    p_alt_hold{1};
    AC_P                    p_vel_z{5};
    AC_PID                  pid_accel_z{0.5, 1, 0, 800, 20, 0.02};
    AC_PI_2D                pi_vel_xy{1.0, 0.5, 1000, 5, 0.02};

    AP_Int8 frame_class;
    AP_Int8 frame_type;
    
    AP_MotorsMulticopter *motors;
    AC_AttitudeControl_Multi *attitude_control;
    AC_PosControl *pos_control;
    AC_WPNav *wp_nav;
    
    // maximum vertical velocity the pilot may request
    AP_Int16 pilot_velocity_z_max;

    // vertical acceleration the pilot may request
    AP_Int16 pilot_accel_z;
    
    // update transition handling
    void update_transition(void);

    // hold hover (for transition)
    void hold_hover(float target_climb_rate);    

    // hold stabilize (for transition)
    void hold_stabilize(float throttle_in);    

    // get pilot desired yaw rate in cd/s
    float get_pilot_input_yaw_rate_cds(void);

    // get overall desired yaw rate in cd/s
    float get_desired_yaw_rate_cds(void);
    
    // get desired climb rate in cm/s
    float get_pilot_desired_climb_rate_cms(void);

    // initialise throttle_wait when entering mode
    void init_throttle_wait();

    // main entry points for VTOL flight modes
    void init_stabilize(void);
    void control_stabilize(void);

    void init_hover(void);
    void control_hover(void);

    void init_loiter(void);
    void control_loiter(void);

    float assist_climb_rate_cms(void);

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(void);

    bool should_relax(void);

    // setup correct aux channels for frame class
    void setup_default_channels(uint8_t num_motors);
    
    AP_Int16 transition_time_ms;

    AP_Int16 rc_speed;

    // min and max PWM for throttle
    AP_Int16 thr_min_pwm;
    AP_Int16 thr_max_pwm;
    AP_Int16 throttle_mid;

    // speed below which quad assistance is given
    AP_Float assist_speed;

    // maximum yaw rate in degrees/second
    AP_Float yaw_rate_max;

    // landing speed in cm/s
    AP_Int16 land_speed_cms;

    // alt to switch to QLAND_FINAL
    AP_Float land_final_alt;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;
    
    bool initialised;
    
    // timer start for transition
    uint32_t transition_start_ms;

    Location last_auto_target;

    // last throttle value when active
    float last_throttle;

    const float smoothing_gain = 6;

    // true if we have reached the airspeed threshold for transition
    enum {
        TRANSITION_AIRSPEED_WAIT,
        TRANSITION_TIMER,
        TRANSITION_DONE
    } transition_state;

    // true when waiting for pilot throttle
    bool throttle_wait;

    // true when quad is assisting a fixed wing mode
    bool assisted_flight;

    // time when motors reached lower limit
    uint32_t motors_lower_limit_start_ms;

    // time we last set the loiter target
    uint32_t last_loiter_ms;

    enum {
        QLAND_POSITION,
        QLAND_DESCEND,
        QLAND_FINAL,
        QLAND_COMPLETE
    } land_state;
    int32_t land_yaw_cd;
    float land_wp_proportion;

    enum frame_class {
        FRAME_CLASS_QUAD=0,
        FRAME_CLASS_HEXA=1,
        FRAME_CLASS_OCTA=2,
    };
};
