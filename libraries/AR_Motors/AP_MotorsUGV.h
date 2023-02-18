#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_MotorsUGV {
public:
    // Constructor
    AP_MotorsUGV(AP_ServoRelayEvents &relayEvents, AP_WheelRateControl& rate_controller);

    // singleton support
    static AP_MotorsUGV    *get_singleton(void) { return _singleton; }

    enum motor_test_order {
        MOTOR_TEST_THROTTLE = 1,
        MOTOR_TEST_STEERING = 2,
        MOTOR_TEST_THROTTLE_LEFT = 3,
        MOTOR_TEST_THROTTLE_RIGHT = 4,
        MOTOR_TEST_MAINSAIL = 5,
        MOTOR_TEST_LAST
    };

    // supported omni motor configurations
    enum frame_type {
        FRAME_TYPE_UNDEFINED = 0,
        FRAME_TYPE_OMNI3 = 1,
        FRAME_TYPE_OMNIX = 2,
        FRAME_TYPE_OMNIPLUS = 3,
    };

    // initialise motors
    void init(uint8_t ftype);

    // return true if motors are active
    bool active() const;

    // setup output in case of main CPU failure
    void setup_safety_output();

    // setup servo output ranges
    void setup_servo_output();

    // get or set steering as a value from -4500 to +4500
    //   apply_scaling should be set to false for manual modes where
    //   no scaling by speed or angle should e performed
    float get_steering() const { return _steering; }
    void set_steering(float steering, bool apply_scaling = true);

    // get or set throttle as a value from -100 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle);

    // get or set roll as a value from -1 to 1
    float get_roll() const { return _roll; }
    void set_roll(float roll);

    // get or set pitch as a value from -1 to 1
    float get_pitch() const { return _pitch; }
    void set_pitch(float pitch);

    // get or set walking_height as a value from -1 to 1
    float get_walking_height() const { return _walking_height; }
    void set_walking_height(float walking_height);

    // get or set lateral input as a value from -100 to +100
    float get_lateral() const { return _lateral; }
    void set_lateral(float lateral);

    // set or get mainsail input as a value from 0 to 100
    void set_mainsail(float mainsail);
    float get_mainsail() const { return _mainsail; }

    // set or get wingsail input as a value from -100 to 100
    void set_wingsail(float wingsail);
    float get_wingsail() const { return _wingsail; }

    // set or get mast rotation input as a value from -100 to 100
    void set_mast_rotation(float mast_rotation);
    float get_mast_rotation() const { return _mast_rotation; }

    // get slew limited throttle
    // used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
    // same as private slew_limit_throttle method (see below) but does not update throttle state
    float get_slew_limited_throttle(float throttle, float dt) const;

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // true if vehicle has vectored thrust (i.e. boat with motor on steering servo)
    bool have_vectored_thrust() const { return is_positive(_vector_angle_max); }

    // output to motors and steering servos
    // ground_speed should be the vehicle's speed over the surface in m/s
    // dt should be expected time between calls to this function
    void output(bool armed, float ground_speed, float dt);

    // test steering or throttle output as a percentage of the total (range -100 to +100)
    // used in response to DO_MOTOR_TEST mavlink command
    bool output_test_pct(motor_test_order motor_seq, float pct);

    // test steering or throttle output using a pwm value
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

    //  returns true if checks pass, false if they fail.  display_failure argument should be true to send text messages to GCS
    bool pre_arm_check(bool report) const;

    // return the motor mask
    uint32_t get_motor_mask() const { return _motor_mask; }

    // returns true if the configured PWM type is digital and should have fixed endpoints
    bool is_digital_pwm_type() const;

    // returns true if the vehicle is omni
    bool is_omni() const { return _frame_type != FRAME_TYPE_UNDEFINED && _motors_num > 0; }

    // structure for holding motor limit flags
    struct AP_MotorsUGV_limit {
        uint8_t steer_left      : 1; // we have reached the steering controller's left most limit
        uint8_t steer_right     : 1; // we have reached the steering controller's right most limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    enum pwm_type {
        PWM_TYPE_NORMAL = 0,
        PWM_TYPE_ONESHOT = 1,
        PWM_TYPE_ONESHOT125 = 2,
        PWM_TYPE_BRUSHED_WITH_RELAY = 3,
        PWM_TYPE_BRUSHED_BIPOLAR = 4,
        PWM_TYPE_DSHOT150 = 5,
        PWM_TYPE_DSHOT300 = 6,
        PWM_TYPE_DSHOT600 = 7,
        PWM_TYPE_DSHOT1200 = 8
    };

    // sanity check parameters
    void sanity_check_parameters();

    // setup pwm output type
    void setup_pwm_type();

    // setup for frames with omni motors
    void setup_omni();

    // add omni motor using separate throttle, steering and lateral factors
    void add_omni_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor);

    // add a motor and set up output function
    void add_omni_motor_num(int8_t motor_num);

    // disable omni motor and remove all throttle, steering and lateral factor for this motor
    void clear_omni_motors(int8_t motor_num);

    // output to regular steering and throttle channels
    void output_regular(bool armed, float ground_speed, float steering, float throttle);

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle, float dt);

    // output for omni motors
    void output_omni(bool armed, float steering, float throttle, float lateral);

    // output throttle (-100 ~ +100) to a throttle channel.  Sets relays if required
    // dt is the main loop time interval and is required when rate control is required
    void output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt = 0.0f);

    // output for sailboat's mainsail in the range of 0 to 100 and wing sail in the range +- 100
    void output_sail();

    // true if the vehicle has a mainsail or wing sail
    bool has_sail() const;

    // slew limit throttle for one iteration
    void slew_limit_throttle(float dt);

    // set limits based on steering and throttle input
    void set_limits_from_input(bool armed, float steering, float throttle);

    // scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
    float get_scaled_throttle(float throttle) const;

    // use rate controller to achieve desired throttle
    float get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt);

    // external references
    AP_ServoRelayEvents &_relayEvents;
    AP_WheelRateControl &_rate_controller;

    static const int8_t AP_MOTORS_NUM_MOTORS_MAX = 4;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int16 _slew_rate; // slew rate expressed as a percentage / second
    AP_Int8 _throttle_min; // throttle minimum percentage
    AP_Int8 _throttle_max; // throttle maximum percentage
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear
    AP_Float _vector_angle_max;  // angle between steering's middle position and maximum position when using vectored thrust.  zero to disable vectored thrust
    AP_Float _speed_scale_base;  // speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
    AP_Float _steering_throttle_mix; // Steering vs Throttle priorisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid for Skid Steering vehicles

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
    bool    _scale_steering = true; // true if we should scale steering by speed or angle
    float   _lateral;  // requested lateral input as a value from -100 to +100
    float   _roll;      // requested roll as a value from -1 to +1
    float   _pitch;     // requested pitch as a value from -1 to +1
    float   _walking_height; // requested height as a value from -1 to +1   
    float   _mainsail;  // requested mainsail input as a value from 0 to 100
    float   _wingsail;  // requested wing sail input as a value in the range +- 100
    float   _mast_rotation;  // requested mast rotation input as a value in the range +- 100
    uint32_t _motor_mask;   // mask of motors configured with pwm_type
    frame_type _frame_type; // frame type requested at initialisation

    // omni variables
    float   _throttle_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _steering_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _lateral_factor[AP_MOTORS_NUM_MOTORS_MAX];
    uint8_t   _motors_num;

    static AP_MotorsUGV *_singleton;
};

namespace AP {
    AP_MotorsUGV *motors_ugv();
};
