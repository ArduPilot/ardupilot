// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsMulticopter.h
/// @brief	Motor control class for Multicopters

#ifndef __AP_MOTORS_MULTICOPTER_H__
#define __AP_MOTORS_MULTICOPTER_H__

#include "AP_Motors_Class.h"

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MID_THROTTLE  500
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  1000

#define AP_MOTORS_SPIN_WHEN_ARMED       70      // spin motors at this PWM value when armed
#define AP_MOTORS_YAW_HEADROOM_DEFAULT  200
#define AP_MOTORS_THR_LOW_CMP_DEFAULT   0.5f    // ratio controlling the max throttle output during competing requests of low throttle from the pilot (or autopilot) and higher throttle for attitude control.  Higher favours Attitude over pilot input
#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_MAX_DEFAULT      0.95f   // throttle which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_THST_BAT_MAX_DEFAULT  0.0f
#define AP_MOTORS_THST_BAT_MIN_DEFAULT  0.0f
#define AP_MOTORS_CURR_MAX_DEFAULT      0.0f    // current limiting max default
#define AP_MOTORS_BATT_VOLT_FILT_HZ     0.5f    // battery voltage filtered at 0.5hz
#define AP_MOTORS_THR_MIX_MIN_DEFAULT   0.1f    // minimum throttle mix
#define AP_MOTORS_THR_MIX_MID_DEFAULT   0.5f    // manual throttle mix
#define AP_MOTORS_THR_MIX_MAX_DEFAULT   0.5f    // maximum throttle mix default

// To-Do: replace this hard coded counter with a timer
#if HAL_CPU_CLASS < HAL_CPU_CLASS_75 || CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 // slow start increments - throttle increase per (100hz) iteration.  i.e. 5 = full speed in 2 seconds
 #define AP_MOTOR_SLOW_START_INCREMENT           10      // max throttle ramp speed (i.e. motors can reach full throttle in 1 second)
 #define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   2       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 1 second)
#else
 // slow start increments - throttle increase per (400hz) iteration.  i.e. 1 = full speed in 2.5 seconds
 #define AP_MOTOR_SLOW_START_INCREMENT           3       // max throttle ramp speed (i.e. motors can reach full throttle in 0.8 seconds)
 #define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   1       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 0.3 second)
#endif

/// @class      AP_MotorsMulticopter
class AP_MotorsMulticopter : public AP_Motors {
public:

    // Constructor
    AP_MotorsMulticopter(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // output - sends commands to the motors
    virtual void        output();

    // set_yaw_headroom - set yaw headroom (yaw is given at least this amount of pwm)
    void                set_yaw_headroom(int16_t pwm) { _yaw_headroom = pwm; }

    // set_throttle_thr_mix - set desired throttle_thr_mix (actual throttle_thr_mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void                set_throttle_mix_min() { _throttle_thr_mix_desired = _thr_mix_min; }
    void                set_throttle_mix_mid() { _throttle_thr_mix_desired = AP_MOTORS_THR_MIX_MID_DEFAULT; }
    void                set_throttle_mix_max() { _throttle_thr_mix_desired = _thr_mix_max; }

    // get_throttle_thr_mix - get low throttle compensation value
    bool                is_throttle_mix_min() const { return (_throttle_thr_mix < 1.25f*_thr_mix_min); }

    // returns warning throttle
    float               get_throttle_warn() const { return rel_pwm_to_thr_range(_spin_when_armed); }

    int16_t             throttle_max() const { return _max_throttle; }
    int16_t             throttle_min() const { return rel_pwm_to_thr_range(_min_throttle); }

    // set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    // also sets throttle channel minimum and maximum pwm
    void                set_throttle_range(uint16_t min_throttle, int16_t radio_min, int16_t radio_max);

    // set_hover_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    void                set_hover_throttle(uint16_t hov_thr) { _hover_out = hov_thr; }

    // slow_start - set to true to slew motors from current speed to maximum
    // Note: this must be set immediately before a step up in throttle
    void                slow_start(bool true_false);

    // throttle_pass_through - passes provided pwm directly to all motors - dangerous but used for initialising ESCs
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void                throttle_pass_through(int16_t pwm);

    // get_lift_max - get maximum lift ratio - for logging purposes only
    float               get_lift_max() { return _lift_max; }

    // get_batt_voltage_filt - get battery voltage ratio - for logging purposes only
    float               get_batt_voltage_filt() const { return _batt_voltage_filt.get(); }

    // get_batt_resistance - get battery resistance approximation - for logging purposes only
    float               get_batt_resistance() const { return _batt_resistance; }

    // get_throttle_limit - throttle limit ratio - for logging purposes only
    float               get_throttle_limit() const { return _throttle_limit; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // update the throttle input filter
    virtual void        update_throttle_filter();

    // update_max_throttle - updates the limits on _max_throttle for slow_start and current limiting flag
    void                update_max_throttle();

    // current_limit_max_throttle - current limit maximum throttle (called from update_max_throttle)
    void                current_limit_max_throttle();

    // apply_thrust_curve_and_volt_scaling - thrust curve and voltage adjusted pwm value (i.e. 1000 ~ 2000)
    int16_t             apply_thrust_curve_and_volt_scaling(int16_t pwm_out, int16_t pwm_min, int16_t pwm_max) const;

    // update_lift_max_from_batt_voltage - used for voltage compensation
    void                update_lift_max_from_batt_voltage();

    // update_battery_resistance - calculate battery resistance when throttle is above hover_out
    void                update_battery_resistance();

    // update_throttle_thr_mix - updates thr_low_comp value towards the target
    void                update_throttle_thr_mix();

    // return gain scheduling gain based on voltage and air density
    float               get_compensation_gain() const;

    // get_hover_throttle_as_pwm - converts hover throttle to pwm (i.e. range 1000 ~ 2000)
    int16_t             get_hover_throttle_as_pwm() const;

    float               rel_pwm_to_thr_range(float pwm) const;
    float               thr_range_to_rel_pwm(float thr) const;

    // convert RPY and Throttle servo ranges from legacy controller scheme back into PWM values
    // RPY channels typically +/-45 degrees servo travel between +/-400 PWM
    // Throttle channel typically 0-1000 range converts to 1100-1900 PWM for final output signal to motors
    // ToDo: this should all be handled as floats +/- 1.0 instead of PWM and fake angle ranges
    float               calc_roll_pwm() { return (_roll_control_input * _rpy_pwm_scalar); }
    float               calc_pitch_pwm() { return (_pitch_control_input * _rpy_pwm_scalar); }
    float               calc_yaw_pwm() { return (_yaw_control_input * _rpy_pwm_scalar); }
    int16_t             calc_throttle_radio_output() { return (_throttle_control_input * _throttle_pwm_scalar) + _throttle_radio_min;}

    // flag bitmask
    struct {
        uint8_t slow_start         : 1;    // 1 if slow start is active
        uint8_t slow_start_low_end : 1;    // 1 just after arming so we can ramp up the spin_when_armed value
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

    // internal variables
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];    // true if motor is enabled
    int16_t             _spin_when_armed_ramped;    // equal to _spin_when_armed parameter but slowly ramped up from zero
    float               _throttle_thr_mix_desired;  // desired throttle_low_comp value, actual throttle_low_comp is slewed towards this value over 1~2 seconds
    float               _throttle_thr_mix;          // mix between throttle and hover throttle for 0 to 1 and ratio above hover throttle for >1
    int16_t             _min_throttle;              // the minimum throttle to be sent to the motors when they're on (prevents motors stalling while flying)
    int16_t             _max_throttle;              // the maximum throttle to be sent to the motors (sometimes limited by slow start)
    int16_t             _hover_out;                 // the estimated hover throttle as pct * 10 (i.e. 0 ~ 1000)

    // battery voltage, current and air pressure compensation variables
    float               _batt_voltage_resting;  // battery voltage reading at minimum throttle
    LowPassFilterFloat  _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
    float               _batt_current_resting;  // battery's current when motors at minimum
    float               _batt_resistance;       // battery's resistance calculated by comparing resting voltage vs in flight voltage
    int16_t             _batt_timer;            // timer used in battery resistance calcs
    float               _lift_max;              // maximum lift ratio from battery voltage
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum
};
#endif  // __AP_MOTORS_MULTICOPTER_H__
