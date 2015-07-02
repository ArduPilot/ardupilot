// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_CLASS_H__
#define __AP_MOTORS_CLASS_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Notify.h>      // Notify library
#include <RC_Channel.h>     // RC Channel Library
#include <Filter.h>         // filter library

// offsets for motors in motor_out, _motor_filtered and _motor_to_channel_map arrays
#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MOT_5 4
#define AP_MOTORS_MOT_6 5
#define AP_MOTORS_MOT_7 6
#define AP_MOTORS_MOT_8 7

#define MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_5,CH_6,CH_7,CH_8

#define AP_MOTORS_MAX_NUM_MOTORS 8

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MID_THROTTLE  500
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  1000

// frame definitions
#define AP_MOTORS_PLUS_FRAME        0
#define AP_MOTORS_X_FRAME           1
#define AP_MOTORS_V_FRAME           2
#define AP_MOTORS_H_FRAME           3   // same as X frame but motors spin in opposite direction
#define AP_MOTORS_VTAIL_FRAME       4   // Lynxmotion Hunter VTail 400/500
#define AP_MOTORS_ATAIL_FRAME       5   // A-Shaped VTail Quads
#define AP_MOTORS_NEW_PLUS_FRAME    10  // NEW frames are same as original 4 but with motor orders changed to be clockwise from the front
#define AP_MOTORS_NEW_X_FRAME       11
#define AP_MOTORS_NEW_V_FRAME       12
#define AP_MOTORS_NEW_H_FRAME       13   // same as X frame but motors spin in opposite direction

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors

#define THROTTLE_CURVE_ENABLED      1   // throttle curve disabled by default
#define THROTTLE_CURVE_MID_THRUST   52  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)
#define THROTTLE_CURVE_MAX_THRUST   93  // throttle which produces the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)

// bit mask for recording which limits we have reached when outputting to motors
#define AP_MOTOR_NO_LIMITS_REACHED  0x00
#define AP_MOTOR_ROLLPITCH_LIMIT    0x01
#define AP_MOTOR_YAW_LIMIT          0x02
#define AP_MOTOR_THROTTLE_LIMIT     0x04
#define AP_MOTOR_ANY_LIMIT          0xFF

/// @class      AP_Motors
class AP_Motors {
public:

    // Constructor
    AP_Motors(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // set update rate to motors - a value in hertz
    virtual void        set_update_rate( uint16_t speed_hz ) { _speed_hz = speed_hz; };

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation ) { _flags.frame_orientation = new_orientation; };

    // arm, disarm or check status status of motors
    bool                armed() const { return _flags.armed; };
    void                armed(bool arm);

    // set motor interlock status
    void                set_interlock(bool set) { _flags.interlock = set;}

    // get motor interlock status.  true means motors run, false motors don't run
    bool                get_interlock() const { return _flags.interlock; };

    // set_throttle_range - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    // also sets throttle channel minimum and maximum pwm
    virtual void        set_throttle_range(uint16_t min_throttle, int16_t radio_min, int16_t radio_max) {}

    // set_hover_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    void                set_hover_throttle(uint16_t hov_thr) { _hover_out = hov_thr; }

    // get_hover_throttle_as_pwm - converts hover throttle to pwm (i.e. range 1000 ~ 2000)
    int16_t             get_hover_throttle_as_pwm() const;

    int16_t             throttle_min() const { return rel_pwm_to_thr_range(_min_throttle); }
    int16_t             throttle_max() const { return _max_throttle;}

    // set_roll, set_pitch, set_yaw, set_throttle
    void                set_roll(int16_t roll_in) { _roll_control_input = roll_in; };                   // range -4500 ~ 4500
    void                set_pitch(int16_t pitch_in) { _pitch_control_input = pitch_in; };               // range -4500 ~ 4500
    void                set_yaw(int16_t yaw_in) { _yaw_control_input = yaw_in; };                       // range -4500 ~ 4500
    void                set_throttle(float throttle_in) { _throttle_in = constrain_float(throttle_in,-100.0f,1100.0f); };   // range 0 ~ 1000
    void                set_stabilizing(bool stabilizing) { _flags.stabilizing = stabilizing; }

    // accessors for roll, pitch, yaw and throttle inputs to motors
    float               get_roll() const { return _roll_control_input; }
    float               get_pitch() const { return _pitch_control_input; }
    float               get_yaw() const { return _yaw_control_input; }
    float               get_throttle() const { return _throttle_control_input; }

    void                set_throttle_filter_cutoff(float filt_hz) { _throttle_filter.set_cutoff_frequency(filt_hz); }

    // set_voltage - set voltage to be used for output scaling
    virtual void        set_voltage(float volts){ _batt_voltage = volts; }

    // set_current - set current to be used for output scaling
    virtual void        set_current(float current){ _batt_current = current; }

    // set_density_ratio - sets air density as a proportion of sea level density
    void                set_air_density_ratio(float ratio) { _air_density_ratio = ratio; }

    // get_lift_max - get maximum lift ratio
    float               get_lift_max() { return _lift_max; }

    // get_batt_voltage_filt - get battery voltage ratio
    float               get_batt_voltage_filt() { return _batt_voltage_filt.get(); }

    // get_batt_resistance - get battery resistance approximation
    float               get_batt_resistance() { return _batt_resistance; }

    // get_throttle_limit - throttle limit ratio
    float               get_throttle_limit() { return _throttle_limit; }

    // 1 if motor is enabled, 0 otherwise
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];

    // structure for holding motor limit flags
    struct AP_Motors_limit {
        uint8_t roll_pitch      : 1; // we have reached roll or pitch limit
        uint8_t yaw             : 1; // we have reached yaw limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    ////////////////////////////////////////////////////////////////////////
    // Virtual Functions to be overloaded by child classes
    // in most cases this is done because of global usage of multirotor specific
    // functions which do not apply to helicopters

    // init
    virtual void        Init() {}

    // enable - starts allowing signals to be sent to motors
    virtual void        enable() = 0;

    // output - sends commands to the motors
    virtual void        output() = 0;

    // output_min - sends minimum values out to the motors
    virtual void        output_min() = 0;

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm) = 0;

    virtual int16_t     throttle_min() { return AP_MOTORS_DEFAULT_MIN_THROTTLE;};
    virtual int16_t     throttle_max() { return AP_MOTORS_DEFAULT_MAX_THROTTLE;};

    // set_hover_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    virtual void        set_hover_throttle(uint16_t hov_thr) {}

    // throttle_pass_through - passes provided pwm directly to all motors - dangerous but used for initialising ESCs
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        throttle_pass_through(int16_t pwm) {}

    // returns warning throttle
    virtual float       get_throttle_warn() { return 0; }

    // set_throttle_thr_mix - set desired throttle_thr_mix (actual throttle_thr_mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    virtual void        set_throttle_mix_min() {}
    virtual void        set_throttle_mix_mid() {}
    virtual void        set_throttle_mix_max() {}

    // slow_start - set to true to slew motors from current speed to maximum
    // Note: this must be set immediately before a step up in throttle
    virtual void                slow_start(bool true_false) {};

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask() = 0;

protected:
    // output functions that should be overloaded by child classes
    virtual void        output_armed_stabilizing()=0;
    virtual void        output_armed_not_stabilizing()=0;
    virtual void        output_armed_zero_throttle() { output_min(); }
    virtual void        output_disarmed()=0;

    // update the throttle input filter
    virtual void        update_throttle_filter() = 0;

    // convert RPY and Throttle servo ranges from legacy controller scheme back into PWM values
    // RPY channels typically +/-45 degrees servo travel between +/-400 PWM
    // Throttle channel typically 0-1000 range converts to 1100-1900 PWM for final output signal to motors
    // ToDo: this should all be handled as floats +/- 1.0 instead of PWM and fake angle ranges
    int16_t             calc_roll_pwm() { return (_roll_control_input / 11.25f);}
    int16_t             calc_pitch_pwm() { return (_pitch_control_input / 11.25f);}
    int16_t             calc_yaw_pwm() { return (_yaw_control_input / 11.25f);}
    int16_t             calc_throttle_radio_output() { return (_throttle_control_input * _throttle_pwm_scalar) + _throttle_radio_min;}

    // flag bitmask
    struct AP_Motors_flags {
        uint8_t armed              : 1;    // 0 if disarmed, 1 if armed
        uint8_t stabilizing        : 1;    // 0 if not controlling attitude, 1 if controlling attitude
        uint8_t frame_orientation  : 4;    // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2, H_FRAME 3, NEW_PLUS_FRAME 10, NEW_X_FRAME, NEW_V_FRAME, NEW_H_FRAME
        uint8_t slow_start         : 1;    // 1 if slow start is active
        uint8_t slow_start_low_end : 1;    // 1 just after arming so we can ramp up the spin_when_armed value
        uint8_t interlock          : 1;    // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
    } _flags;

    // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
    static const uint8_t _motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS] PROGMEM;

    // internal variables
    float               _roll_control_input;        // desired roll control from attitude controllers, +/- 4500
    float               _pitch_control_input;       // desired pitch control from attitude controller, +/- 4500
    float               _throttle_control_input;    // desired throttle (thrust) control from attitude controller, 0-1000
    float               _yaw_control_input;         // desired yaw control from attitude controller, +/- 4500
    float               _throttle_pwm_scalar;       // scalar used to convert throttle channel pwm range into 0-1000 range, ~0.8 - 1.0
    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    int16_t             _throttle_radio_min;        // minimum radio channel pwm
    int16_t             _throttle_radio_max;        // maximum radio channel pwm
    int16_t             _spin_when_armed_ramped;    // equal to _spin_when_armed parameter but slowly ramped up from zero

    // battery voltage compensation variables
    float               _batt_voltage;          // latest battery voltage reading
    float               _batt_voltage_resting;  // battery voltage reading at minimum throttle
    LowPassFilterFloat  _batt_voltage_filt;     // filtered battery voltage expressed as a percentage (0 ~ 1.0) of batt_voltage_max
    float               _batt_current;          // latest battery current reading
    float               _batt_current_resting;  // battery's current when motors at minimum
    float               _batt_resistance;       // battery's resistance calculated by comparing resting voltage vs in flight voltage
    int16_t             _batt_timer;            // timer used in battery resistance calcs
    float               _air_density_ratio;     // air density / sea level density - decreases in altitude
    float               _lift_max;              // maximum lift ratio from battery voltage
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum
    float               _throttle_in;           // last throttle input from set_throttle caller
    LowPassFilterFloat  _throttle_filter;       // throttle input filter
};
#endif  // __AP_MOTORS_CLASS_H__
