/// @file	AP_MotorsMulticopter.h
/// @brief	Motor control class for Multicopters
#pragma once

#include "AP_Motors_Class.h"

#ifndef AP_MOTORS_DENSITY_COMP
#define AP_MOTORS_DENSITY_COMP 1
#endif

#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200
#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_HOVER_DEFAULT    0.35f   // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_TC         10.0f   // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_MIN        0.125f  // minimum possible hover throttle
#define AP_MOTORS_THST_HOVER_MAX        0.6875f // maximum possible hover throttle
#define AP_MOTORS_SPIN_MIN_DEFAULT      0.15f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_MAX_DEFAULT      0.95f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_ARM_DEFAULT      0.10f   // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
#define AP_MOTORS_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#define AP_MOTORS_BAT_CURR_MAX_DEFAULT  0.0f    // current limiting max default
#define AP_MOTORS_BAT_CURR_TC_DEFAULT   5.0f    // Time constant used to limit the maximum current
#define AP_MOTORS_BATT_VOLT_FILT_HZ     0.5f    // battery voltage filtered at 0.5hz

// spool definition
#define AP_MOTORS_SPOOL_UP_TIME_DEFAULT 0.5f    // time (in seconds) for throttle to increase from zero to min throttle, and min throttle to full throttle.

/// @class      AP_MotorsMulticopter
class AP_MotorsMulticopter : public AP_Motors {
public:

    // Constructor
    AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // output - sends commands to the motors
    virtual void        output() override;

    // output_min - sends minimum values out to the motors
    void                output_min() override;

    // set_yaw_headroom - set yaw headroom (yaw is given at least this amount of pwm)
    void                set_yaw_headroom(int16_t pwm) { _yaw_headroom = pwm; }

    // set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    // also sets minimum and maximum pwm values that will be sent to the motors
    void                set_throttle_range(int16_t radio_min, int16_t radio_max);

    // update estimated throttle required to hover
    void                update_throttle_hover(float dt);
    virtual float       get_throttle_hover() const override { return _throttle_hover; }

    // passes throttle directly to all motors for ESC calibration.
    //   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
    void                set_throttle_passthrough_for_esc_calibration(float throttle_input);

    // get_lift_max - get maximum lift ratio - for logging purposes only
    float               get_lift_max() { return _lift_max; }

    // get_batt_voltage_filt - get battery voltage ratio - for logging purposes only
    float               get_batt_voltage_filt() const { return _batt_voltage_filt.get(); }

    // get throttle limit imposed by battery current limiting.  This is a number from 0 ~ 1 where 0 means hover throttle, 1 means full throttle (i.e. not limited)
    float               get_throttle_limit() const { return _throttle_limit; }

    // returns maximum thrust in the range 0 to 1
    float               get_throttle_thrust_max() const { return _throttle_thrust_max; }

    // return true if spool up is complete
    bool spool_up_complete() const { return _spool_mode == THROTTLE_UNLIMITED; }

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    virtual void        output_motor_mask(float thrust, uint8_t mask);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask() override;

    // get minimum or maximum pwm value that can be output to motors
    int16_t             get_pwm_output_min() const;
    int16_t             get_pwm_output_max() const;
    
    // set thrust compensation callback
    FUNCTOR_TYPEDEF(thrust_compensation_fn_t, void, float *, uint8_t);
    void                set_thrust_compensation_callback(thrust_compensation_fn_t callback) {
        _thrust_compensation_callback = callback;
    }
    
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // run spool logic
    void                output_logic();

    // output_to_motors - sends commands to the motors
    virtual void        output_to_motors() = 0;

    // update the throttle input filter
    virtual void        update_throttle_filter() override;

    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    virtual float       get_current_limit_max_throttle();

    // apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
    float               apply_thrust_curve_and_volt_scaling(float thrust) const;

    // update_lift_max_from_batt_voltage - used for voltage compensation
    void                update_lift_max_from_batt_voltage();

    // return gain scheduling gain based on voltage and air density
    float               get_compensation_gain() const;

    // convert thrust (0~1) range back to pwm range
    int16_t             calc_thrust_to_pwm(float thrust_in) const;

    // calculate spin up to pwm range
    int16_t             calc_spin_up_to_pwm() const;

    // apply any thrust compensation for the frame
    virtual void        thrust_compensation(void) {}

    // output booster throttle, if any
    virtual void        output_boost_throttle(void);
    
    // save parameters as part of disarming
    void save_params_on_disarm() override;

    // enum values for HOVER_LEARN parameter
    enum HoverLearn {
        HOVER_LEARN_DISABLED = 0,
        HOVER_LEARN_ONLY = 1,
        HOVER_LEARN_AND_SAVE = 2
    };

    // parameters
    AP_Int16            _yaw_headroom;          // yaw control is given at least this pwm range
    AP_Float            _thrust_curve_expo;     // curve used to linearize pwm to thrust conversion.  set to 0 for linear and 1 for second order approximation
    AP_Float            _spin_min;      // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _spin_max;      // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _spin_arm;      // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _batt_voltage_max;      // maximum voltage used to scale lift
    AP_Float            _batt_voltage_min;      // minimum voltage used to scale lift
    AP_Float            _batt_current_max;      // current over which maximum throttle is limited
    AP_Float            _batt_current_time_constant;    // Time constant used to limit the maximum current
    AP_Int8             _batt_idx;              // battery index used for compensation
    AP_Int16            _pwm_min;               // minimum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's min pwm used)
    AP_Int16            _pwm_max;               // maximum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's max pwm used)
    AP_Float            _throttle_hover;        // estimated throttle required to hover throttle in the range 0 ~ 1
    AP_Int8             _throttle_hover_learn;  // enable/disabled hover thrust learning
    AP_Int8             _disarm_disable_pwm;    // disable PWM output while disarmed

    // Maximum lean angle of yaw servo in degrees. This is specific to tricopter
    AP_Float            _yaw_servo_angle_max_deg;

    // time to spool motors to min throttle
    AP_Float            _spool_up_time;

    // scaling for booster motor throttle
    AP_Float            _boost_scale;
    
    // motor output variables
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];    // true if motor is enabled
    int16_t             _throttle_radio_min;        // minimum PWM from RC input's throttle channel (i.e. minimum PWM input from receiver, RC3_MIN)
    int16_t             _throttle_radio_max;        // maximum PWM from RC input's throttle channel (i.e. maximum PWM input from receiver, RC3_MAX)

    // spool variables
    float               _spin_up_ratio;      // throttle percentage (0 ~ 1) between zero and throttle_min

    // battery voltage, current and air pressure compensation variables
    LowPassFilterFloat  _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
    float               _lift_max;              // maximum lift ratio from battery voltage
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum
    float               _throttle_thrust_max;   // the maximum allowed throttle thrust 0.0 to 1.0 in the range throttle_min to throttle_max
    uint16_t            _disarm_safety_timer;

    // vehicle supplied callback for thrust compensation. Used for tiltrotors and tiltwings
    thrust_compensation_fn_t _thrust_compensation_callback;
};
