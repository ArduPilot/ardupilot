// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_CLASS_H__
#define __AP_MOTORS_CLASS_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Notify/AP_Notify.h>      // Notify library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <Filter/Filter.h>         // filter library

// offsets for motors in motor_out and _motor_filtered arrays
#define AP_MOTORS_MOT_1 0U
#define AP_MOTORS_MOT_2 1U
#define AP_MOTORS_MOT_3 2U
#define AP_MOTORS_MOT_4 3U
#define AP_MOTORS_MOT_5 4U
#define AP_MOTORS_MOT_6 5U
#define AP_MOTORS_MOT_7 6U
#define AP_MOTORS_MOT_8 7U

#define AP_MOTORS_MAX_NUM_MOTORS 8

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
#define AP_MOTORS_QUADPLANE         14   // motors on 5..8

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors

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

    //
    // voltage, current and air pressure compensation or limiting features - multicopters only
    //
    // set_voltage - set voltage to be used for output scaling
    void                set_voltage(float volts){ _batt_voltage = volts; }

    // set_current - set current to be used for output scaling
    void                set_current(float current){ _batt_current = current; }

    // set_density_ratio - sets air density as a proportion of sea level density
    void                set_air_density_ratio(float ratio) { _air_density_ratio = ratio; }

    // structure for holding motor limit flags
    struct AP_Motors_limit {
        uint8_t roll_pitch      : 1; // we have reached roll or pitch limit
        uint8_t yaw             : 1; // we have reached yaw limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    //
    // virtual functions that should be implemented by child classes
    //

    // init
    virtual void        Init() = 0;

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

    // slow_start - set to true to slew motors from current speed to maximum
    // Note: this must be set immediately before a step up in throttle
    virtual void        slow_start(bool true_false) = 0;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask() = 0;

protected:
    // output functions that should be overloaded by child classes
    virtual void        output_armed_stabilizing()=0;
    virtual void        output_armed_not_stabilizing()=0;
    virtual void        output_armed_zero_throttle() { output_min(); }
    virtual void        output_disarmed()=0;
    virtual void        rc_write(uint8_t chan, uint16_t pwm);
    virtual void        rc_set_freq(uint32_t mask, uint16_t freq_hz);
    virtual void        rc_enable_ch(uint8_t chan);
    virtual uint32_t    rc_map_mask(uint32_t mask) const;
    
    // update the throttle input filter
    virtual void        update_throttle_filter() = 0;

    // flag bitmask
    struct AP_Motors_flags {
        uint8_t armed              : 1;    // 0 if disarmed, 1 if armed
        uint8_t stabilizing        : 1;    // 0 if not controlling attitude, 1 if controlling attitude
        uint8_t frame_orientation  : 4;    // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2, H_FRAME 3, NEW_PLUS_FRAME 10, NEW_X_FRAME, NEW_V_FRAME, NEW_H_FRAME
        uint8_t interlock          : 1;    // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
    } _flags;

    // internal variables
    float               _roll_control_input;        // desired roll control from attitude controllers, +/- 4500
    float               _pitch_control_input;       // desired pitch control from attitude controller, +/- 4500
    float               _throttle_control_input;    // desired throttle (thrust) control from attitude controller, 0-1000
    float               _yaw_control_input;         // desired yaw control from attitude controller, +/- 4500
    float               _throttle_pwm_scalar;       // scalar used to convert throttle channel pwm range into 0-1000 range, ~0.8 - 1.0
    float               _rpy_pwm_scalar;            // scaler used to convert roll, pitch, yaw inputs to pwm range
    uint16_t            _loop_rate;                 // rate at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    int16_t             _throttle_radio_min;        // minimum radio channel pwm
    int16_t             _throttle_radio_max;        // maximum radio channel pwm
    float               _throttle_in;               // last throttle input from set_throttle caller
    LowPassFilterFloat  _throttle_filter;           // throttle input filter

    // battery voltage, current and air pressure compensation variables
    float               _batt_voltage;          // latest battery voltage reading
    float               _batt_current;          // latest battery current reading
    float               _air_density_ratio;     // air density / sea level density - decreases in altitude

    // mapping to output channels
    uint8_t             _motor_map[AP_MOTORS_MAX_NUM_MOTORS];
    uint16_t            _motor_map_mask;
};
#endif  // __AP_MOTORS_CLASS_H__
