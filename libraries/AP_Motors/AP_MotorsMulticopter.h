// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsMulticopter.h
/// @brief	Motor control class for Multicopters
#pragma once

#include "AP_Motors_Class.h"

#ifndef AP_MOTORS_DENSITY_COMP
#define AP_MOTORS_DENSITY_COMP 1
#endif

#define AP_MOTORS_DEFAULT_MID_THROTTLE  500

#define AP_MOTORS_SPIN_WHEN_ARMED       70      // spin motors at this PWM value when armed
#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200
#define AP_MOTORS_THR_LOW_CMP_DEFAULT   0.5f    // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input
#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_SPIN_MAX_DEFAULT      0.95f   // throttle which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
#define AP_MOTORS_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#define AP_MOTORS_BAT_CURR_MAX_DEFAULT  0.0f    // current limiting max default
#define AP_MOTORS_CURRENT_LIMIT_P       0.2f    // replace with parameter - Sets the current limit P term
#define AP_MOTORS_BATT_VOLT_FILT_HZ     0.5f    // battery voltage filtered at 0.5hz
#define AP_MOTORS_THR_MIX_MIN_DEFAULT   0.1f    // minimum throttle mix
#define AP_MOTORS_THR_MIX_MID_DEFAULT   0.5f    // manual throttle mix
#define AP_MOTORS_THR_MIX_MAX_DEFAULT   0.5f    // maximum throttle mix default

// spool definition
#define AP_MOTORS_SPOOL_UP_TIME         0.5f    // time (in seconds) for throttle to increase from zero to min throttle, and min throttle to full throttle.

/// @class      AP_MotorsMulticopter
class AP_MotorsMulticopter : public AP_Motors {
public:

    // Constructor
    AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // output - sends commands to the motors
    virtual void        output();

    // output_min - sends minimum values out to the motors
    void                output_min();

    // output_to_motors - sends commands to the motors
    virtual void        output_to_motors() = 0;

    // set_yaw_headroom - set yaw headroom (yaw is given at least this amount of pwm)
    void                set_yaw_headroom(int16_t pwm) { _yaw_headroom = pwm; }

    // set_throttle_rpy_mix - set desired throttle_thr_mix (actual throttle_thr_mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void                set_throttle_mix_min() { _throttle_rpy_mix_desired = _thr_mix_min; }
    void                set_throttle_mix_mid() { _throttle_rpy_mix_desired = AP_MOTORS_THR_MIX_MID_DEFAULT; }
    void                set_throttle_mix_max() { _throttle_rpy_mix_desired = _thr_mix_max; }

    // get_throttle_rpy_mix - get low throttle compensation value
    bool                is_throttle_mix_min() const { return (_throttle_rpy_mix < 1.25f*_thr_mix_min); }

    // set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    // also sets minimum and maximum pwm values that will be sent to the motors
    void                set_throttle_range(uint16_t min_throttle, int16_t radio_min, int16_t radio_max);

    // set_hover_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    void                set_hover_throttle(uint16_t hov_thr) { _hover_out = hov_thr; }

    // spool up states
    enum spool_up_down_mode {
        SHUT_DOWN = 0,                      // all motors stop
        SPIN_WHEN_ARMED = 1,                // all motors at spin when armed
        SPOOL_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        SPOOL_DOWN = 4,                     // decreasing maximum throttle while stabilizing
    };

    void                output_logic();

    // passes throttle directly to all motors for ESC calibration.
    //   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
    void                set_throttle_passthrough_for_esc_calibration(float throttle_input);

    // get_lift_max - get maximum lift ratio - for logging purposes only
    float               get_lift_max() { return _lift_max; }

    // get_batt_voltage_filt - get battery voltage ratio - for logging purposes only
    float               get_batt_voltage_filt() const { return _batt_voltage_filt.get(); }

    // get_batt_resistance - get battery resistance approximation - for logging purposes only
    float               get_batt_resistance() const { return _batt_resistance; }

    // get_throttle_limit - throttle limit ratio - for logging purposes only
    float               get_throttle_limit() const { return _throttle_limit; }

    // returns maximum thrust in the range 0 to 1
    float               get_throttle_thrust_max() const { return _throttle_thrust_max; }

    // return true if spool up is complete
    bool spool_up_complete() const { return _multicopter_flags.spool_mode == THROTTLE_UNLIMITED; }

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    void                output_motor_mask(float thrust, uint8_t mask);

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

    // update the throttle input filter
    virtual void        update_throttle_filter();

    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle();

    // apply_thrust_curve_and_volt_scaling - returns throttle in the range 0 ~ 1
    float               apply_thrust_curve_and_volt_scaling(float thrust) const;

    // update_lift_max_from_batt_voltage - used for voltage compensation
    void                update_lift_max_from_batt_voltage();

    // update_battery_resistance - calculate battery resistance when throttle is above hover_out
    void                update_battery_resistance();

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void                update_throttle_rpy_mix();

    // return gain scheduling gain based on voltage and air density
    float               get_compensation_gain() const;

    // get_hover_throttle_as_high_end_pct - return hover throttle in the 0 to 1 range
    float               get_hover_throttle_as_high_end_pct() const;

    // convert thrust (0~1) range back to pwm range
    int16_t             calc_thrust_to_pwm(float thrust_in) const;

    // apply any thrust compensation for the frame
    virtual void        thrust_compensation(void) {}
    
    // flag bitmask
    struct {
        spool_up_down_mode     spool_mode       : 3;    // motor's current spool mode
    } _multicopter_flags;

    // parameters
    AP_Int16            _spin_when_armed;       // used to control whether the motors always spin when armed.  pwm value above radio_min

    AP_Int16            _yaw_headroom;          // yaw control is given at least this pwm range
    AP_Float            _thrust_curve_expo;     // curve used to linearize pwm to thrust conversion.  set to 0 for linear and 1 for second order approximation
    AP_Float            _thrust_curve_max;      // throttle which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _batt_voltage_max;      // maximum voltage used to scale lift
    AP_Float            _batt_voltage_min;      // minimum voltage used to scale lift
    AP_Float            _batt_current_max;      // current over which maximum throttle is limited
    AP_Float            _thr_mix_min;           // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float            _thr_mix_max;           // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    AP_Int16            _pwm_min;               // minimum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's min pwm used)
    AP_Int16            _pwm_max;               // maximum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's max pwm used)

    // internal variables
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];    // true if motor is enabled
    float               _throttle_rpy_mix_desired;  // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds
    float               _throttle_rpy_mix;          // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
    int16_t             _min_throttle;              // the minimum throttle to be sent to the motors when they're on (prevents motors stalling while flying)
    int16_t             _hover_out;                 // the estimated hover throttle as pct * 10 (i.e. 0 ~ 1000)
    int16_t             _throttle_radio_min;        // minimum PWM from RC input's throttle channel (i.e. minimum PWM input from receiver, RC3_MIN)
    int16_t             _throttle_radio_max;        // maximum PWM from RC input's throttle channel (i.e. maximum PWM input from receiver, RC3_MAX)
    float               _throttle_thrust_max;       // the maximum allowed throttle thrust 0.0 to 1.0 in the range throttle_min to throttle_max

    // spool variables
    float               _throttle_low_end_pct;      // throttle percentage (0 ~ 1) between zero and throttle_min

    // battery voltage, current and air pressure compensation variables
    float               _batt_voltage_resting;  // battery voltage reading at minimum throttle
    LowPassFilterFloat  _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
    float               _batt_current_resting;  // battery's current when motors at minimum
    float               _batt_resistance;       // battery's resistance calculated by comparing resting voltage vs in flight voltage
    int16_t             _batt_timer;            // timer used in battery resistance calcs
    float               _lift_max;              // maximum lift ratio from battery voltage
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum

    // vehicle supplied callback for thrust compensation. Used for tiltrotors and tiltwings
    thrust_compensation_fn_t _thrust_compensation_callback;
};
