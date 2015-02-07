// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_HELI_RSC_H__
#define __AP_MOTORS_HELI_RSC_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

// default rotor ramp up time in seconds
#define AP_MOTORS_HELI_RSC_RAMP_TIME            1       // 1 second to ramp output to rotor ESC to full power (most people use exterrnal govenors so we can ramp up quickly)
#define AP_MOTORS_HELI_RSC_RUNUP_TIME           10      // 10 seconds for rotor to reach full speed

class AP_MotorsHeli_RSC {
public:
    AP_MotorsHeli_RSC(RC_Channel&   servo_output,
                      int8_t        servo_output_channel) :
        _servo_output(servo_output),
        _servo_output_channel(servo_output_channel)
    {};

    // get_speed_target
    int16_t     get_speed_target() { return _speed_target; }

    // set_speed_target
    void        set_speed_target(int16_t speed_target) { _speed_target = speed_target; }

    // get_speed_estimate
    int16_t     get_speed_estimate() { return _speed_estimate; }

    // is_runup_complete
    bool        is_runup_complete() { return _runup_complete; }

    // set_dt for setting main loop rate time
    void        set_dt(float dt) { _dt = dt; }

    // set_ramp_time
    void        set_ramp_time (int8_t ramp_time) { _ramp_time = ramp_time; }

    // set_runup_time
    void        set_runup_time (int8_t runup_time) { _runup_time = runup_time; }

    // recalc_scalers
    void        recalc_scalers();

    // output
    void        output ();

    // var_info
    static const struct AP_Param::GroupInfo        var_info[];

private:

    // external
    RC_Channel&     _servo_output;
    int8_t          _servo_output_channel;  // output channel to rotor esc

    // internal variables
    int16_t         _speed_target;          // latest desired rotor speed from pilot
    float           _speed_out;             // latest output sent to the main rotor or an estimate of the rotors actual speed (whichever is higher) (0 ~ 1000)
    float           _speed_estimate;        // estimated speed of the main rotor (0~1000)
    float           _dt;                    // main loop time
    float           _ramp_increment;        // the amount we can increase the rotor output during each 100hz iteration
    int8_t          _ramp_time;             // time in seconds for the output to the main rotor's ESC to reach full speed
    int8_t          _runup_time;            // time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    float           _runup_increment;       // the amount we can increase the rotor's estimated speed during each 100hz iteration
    bool            _runup_complete;        // flag for determining if runup is complete

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1000
    void            write_rsc(int16_t servo_out);
};

#endif // AP_MOTORS_HELI_RSC_H