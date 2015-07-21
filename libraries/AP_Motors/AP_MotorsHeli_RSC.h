// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_HELI_RSC_H__
#define __AP_MOTORS_HELI_RSC_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

class AP_MotorsHeli_RSC {
public:
        AP_MotorsHeli_RSC(RC_Channel&   servo_output,
                      int8_t        servo_output_channel,
                      uint16_t      loop_rate) :
        _servo_output(servo_output),
        _servo_output_channel(servo_output_channel),
        _loop_rate(loop_rate)
    {};

    // set_critical_speed
    void        set_critical_speed(int16_t critical_speed) { _critical_speed = critical_speed; }
    
    // get_critical_speed
    int16_t     get_critical_speed() const { return _critical_speed; }

    // get_desired_speed
    int16_t     get_desired_speed() const { return _desired_speed; }

    // set_desired_speed
    void        set_desired_speed(int16_t desired_speed) { _desired_speed = desired_speed; }

    // get_estimated_speed
    int16_t     get_estimated_speed() const { return _estimated_speed; }

    // is_runup_complete
    bool        is_runup_complete() const { return _runup_complete; }

    // set_ramp_time
    void        set_ramp_time (int8_t ramp_time) { _ramp_time = ramp_time; }

    // set_runup_time
    void        set_runup_time (int8_t runup_time) { _runup_time = runup_time; }

    // recalc_scalers
    void        recalc_scalers();

    // output_armed
    void        output_armed ();

    // output_disarmed
    void        output_disarmed ();

private:

    // external variables
    RC_Channel&     _servo_output;
    int8_t          _servo_output_channel;      // output channel to rotor esc
    float           _loop_rate;                 // main loop rate

    // internal variables
    int16_t         _critical_speed = 0;        // rotor speed below which flight is not possible
    int16_t         _desired_speed = 0;         // latest desired rotor speed from pilot
    float           _speed_out = 0;             // latest output sent to the main rotor or an estimate of the rotors actual speed (whichever is higher) (0 ~ 1000)
    float           _estimated_speed = 0;       // estimated speed of the main rotor (0~1000)
    float           _ramp_increment = 0;        // the amount we can increase the rotor output during each 100hz iteration
    int8_t          _ramp_time = 0;             // time in seconds for the output to the main rotor's ESC to reach full speed
    int8_t          _runup_time = 0;            // time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    float           _runup_increment = 0;       // the amount we can increase the rotor's estimated speed during each 100hz iteration
    bool            _runup_complete = false;    // flag for determining if runup is complete

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1000
    void            write_rsc(int16_t servo_out);
};

#endif // AP_MOTORS_HELI_RSC_H