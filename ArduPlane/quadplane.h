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
    friend class Tuning;
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    void control_run(void);
    void control_auto(const Location &loc);
    bool init_mode(void);
    bool setup(void);
    void setup_defaults(void);
    
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
    bool in_vtol_mode(void);

    // vtol help for is_flying()
    bool is_flying(void);

    // return current throttle as a percentate
    uint8_t throttle_percentage(void) const {
        return last_throttle * 0.1f;
    }

    struct PACKED log_QControl_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float    angle_boost;
        float    throttle_out;
        float    desired_alt;
        float    inav_alt;
        int32_t  baro_alt;
        int16_t  desired_climb_rate;
        int16_t  climb_rate;
        float    dvx;
        float    dvy;
        float    dax;
        float    day;
    };
        
private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AC_P                    p_pos_xy{1};
    AC_P                    p_alt_hold{1};
    AC_P                    p_vel_z{5};
    AC_PID                  pid_accel_z{0.3, 1, 0, 800, 10, 0.02};
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
    void init_land(void);
    void control_loiter(void);
    void check_land_complete(void);

    float assist_climb_rate_cms(void);

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(void);

    bool should_relax(void);
    void motors_output(void);
    void Log_Write_QControl_Tuning();
    float landing_descent_rate_cms(float height_above_ground);
    
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
        QLAND_POSITION1,
        QLAND_POSITION2,
        QLAND_DESCEND,
        QLAND_FINAL,
        QLAND_COMPLETE
    } land_state;
    struct {
        int32_t yaw_cd;
        float speed_scale;
        Vector2f target_velocity;
    } land;

    enum frame_class {
        FRAME_CLASS_QUAD=0,
        FRAME_CLASS_HEXA=1,
        FRAME_CLASS_OCTA=2,
        FRAME_CLASS_OCTAQUAD=3,
    };

    struct {
        bool running;
        uint32_t start_ms;            // system time the motor test began
        uint32_t timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
        uint8_t seq = 0;              // motor sequence number of motor being tested
        uint8_t throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
        uint16_t throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type
        uint8_t motor_count;          // number of motors to cycle
    } motor_test;

public:
    void motor_test_output();
    uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                     uint16_t throttle_value, float timeout_sec,
                                     uint8_t motor_count);
private:
    void motor_test_stop();
};
