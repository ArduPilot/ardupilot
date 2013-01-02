// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#ifndef __AP_MOTORS_CLASS_H__
#define __AP_MOTORS_CLASS_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>       // Curve used to linearlise throttle pwm to thrust
#include <RC_Channel.h>     // RC Channel Library

// offsets for motors in motor_out, _motor_filtered and _motor_to_channel_map arrays
#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MOT_5 4
#define AP_MOTORS_MOT_6 5
#define AP_MOTORS_MOT_7 6
#define AP_MOTORS_MOT_8 7

#define APM1_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_7,CH_8,CH_10,CH_11
#define APM2_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_5,CH_6,CH_7,CH_8

#define AP_MOTORS_MAX_NUM_MOTORS 8

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  850

// APM board definitions
#define AP_MOTORS_APM1  1
#define AP_MOTORS_APM2  2

// frame definitions
#define AP_MOTORS_PLUS_FRAME 0
#define AP_MOTORS_X_FRAME 1
#define AP_MOTORS_V_FRAME 2

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT 490

// top-bottom ratio (for Y6)
#define AP_MOTORS_TOP_BOTTOM_RATIO      1.0

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
    AP_Motors( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    virtual void        Init();

    // set mapping from motor number to RC channel
    virtual void        set_motor_to_channel_map( uint8_t mot_1, uint8_t mot_2, uint8_t mot_3, uint8_t mot_4, uint8_t mot_5, uint8_t mot_6, uint8_t mot_7, uint8_t mot_8 ) {
        _motor_to_channel_map[AP_MOTORS_MOT_1] = mot_1;
        _motor_to_channel_map[AP_MOTORS_MOT_2] = mot_2;
        _motor_to_channel_map[AP_MOTORS_MOT_3] = mot_3;
        _motor_to_channel_map[AP_MOTORS_MOT_4] = mot_4;
        _motor_to_channel_map[AP_MOTORS_MOT_5] = mot_5;
        _motor_to_channel_map[AP_MOTORS_MOT_6] = mot_6;
        _motor_to_channel_map[AP_MOTORS_MOT_7] = mot_7;
        _motor_to_channel_map[AP_MOTORS_MOT_8] = mot_8;
    }

    // set update rate to motors - a value in hertz
    virtual void        set_update_rate( uint16_t speed_hz ) {
        _speed_hz = speed_hz;
    };

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation ) {
        _frame_orientation = new_orientation;
    };

    // enable - starts allowing signals to be sent to motors
    virtual void        enable() {
    };

    // arm, disarm or check status status of motors
    virtual bool        armed() {
        return _armed;
    };
    virtual void        armed(bool arm) {
        _armed = arm;
    };

    // check or set status of auto_armed - controls whether autopilot can take control of throttle
    // Note: this should probably be moved out of this class as it has little to do with the motors
    virtual bool        auto_armed() {
        return _auto_armed;
    };
    virtual void        auto_armed(bool arm) {
        _auto_armed = arm;
    };

    // set_min_throttle - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    virtual void        set_min_throttle(uint16_t min_throttle) {
        _min_throttle = min_throttle;
    };
    virtual void        set_max_throttle(uint16_t max_throttle) {
        _max_throttle = max_throttle;
    };

    // output - sends commands to the motors
    virtual void        output() {
        if( _armed && _auto_armed ) { output_armed(); }else{ output_disarmed(); }
    };

    // output_min - sends minimum values out to the motors
    virtual void        output_min() {
    };

    // reached_limits - return whether we hit the limits of the motors
    virtual uint8_t     reached_limit( uint8_t which_limit = AP_MOTOR_ANY_LIMIT ) {
        return _reached_limit & which_limit;
    }

    // get basic information about the platform
    virtual uint8_t        get_num_motors() {
        return 0;
    };

    // motor test
    virtual void        output_test() {
    };

    // throttle_pass_through - passes throttle through to motors - dangerous but required for initialising ESCs
    virtual void        throttle_pass_through();

	// setup_throttle_curve - used to linearlise thrust output by motors
    //      returns true if curve is created successfully
	virtual bool setup_throttle_curve();

    // 1 if motor is enabled, 0 otherwise
    AP_Int8             motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];

    // final output values sent to the motors.  public (for now) so that they can be access for logging
    int16_t             motor_out[AP_MOTORS_MAX_NUM_MOTORS];

    // power ratio of upper vs lower motors (only used by y6 and octa quad copters)
    AP_Float            top_bottom_ratio;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // output functions that should be overloaded by child classes
    virtual void        output_armed() {
    };
    virtual void        output_disarmed() {
    };

    RC_Channel*         _rc_roll, *_rc_pitch, *_rc_throttle, *_rc_yaw;  // input in from users
    uint8_t             _motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS];        // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
    uint16_t            _speed_hz;                      // speed in hz to send updates to motors
    bool                _armed;                         // true if motors are armed
    bool                _auto_armed;            // true is throttle is above zero, allows auto pilot to take control of throttle
    uint8_t             _frame_orientation;     // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2
    int16_t             _min_throttle;          // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)
    int16_t             _max_throttle;          // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)
    AP_CurveInt16_Size4 _throttle_curve;                // curve used to linearize the pwm->thrust
    AP_Int8             _throttle_curve_enabled;        // enable throttle curve
    AP_Int8             _throttle_curve_mid;  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    AP_Int8             _throttle_curve_max;  // throttle which produces the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    uint8_t             _reached_limit;                // bit mask to record which motor limits we hit (if any) during most recent output.  Used to provide feedback to attitude controllers
};
#endif  // __AP_MOTORS_CLASS_H__
