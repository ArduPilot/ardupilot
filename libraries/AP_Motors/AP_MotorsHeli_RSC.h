// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_HELI_RSC_H__
#define __AP_MOTORS_HELI_RSC_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>      // RC Channel Library

// rotor controller states
#define ROTOR_CONTROL_STOP                      0
#define ROTOR_CONTROL_IDLE                      1
#define ROTOR_CONTROL_ACTIVE                    2

// rotor control modes
#define ROTOR_CONTROL_MODE_DISABLED             0
#define ROTOR_CONTROL_MODE_PASSTHROUGH          1
#define ROTOR_CONTROL_MODE_SETPOINT             2

class AP_MotorsHeli_RSC {
public:
        AP_MotorsHeli_RSC(RC_Channel&   servo_output,
                      int8_t        servo_output_channel,
                      uint16_t      loop_rate) :
        _servo_output(servo_output),
        _servo_output_channel(servo_output_channel),
        _loop_rate(loop_rate)
    {};

    // init_servo - servo initialization on start-up
    void        init_servo();

    // set_control_mode - sets control mode
    void        set_control_mode(int8_t mode) { _control_mode = mode; }

    // set_critical_speed
    void        set_critical_speed(int16_t critical_speed) { _critical_speed = critical_speed; }
    
    // get_critical_speed
    int16_t     get_critical_speed() const { return _critical_speed; }

    // set_idle_speed
    void        set_idle_speed(int16_t idle_speed) { _idle_speed = idle_speed; }

    // get_desired_speed
    int16_t     get_desired_speed() const { return _desired_speed; }

    // set_desired_speed
    void        set_desired_speed(int16_t desired_speed) { _desired_speed = desired_speed; }

    // get_control_speed
    int16_t     get_control_speed() const { return _control_speed; }

    // get_rotor_speed - return estimated or measured rotor speed
    int16_t     get_rotor_speed() const;

    // is_runup_complete
    bool        is_runup_complete() const { return _runup_complete; }

    // set_ramp_time
    void        set_ramp_time(int8_t ramp_time) { _ramp_time = ramp_time; }

    // set_runup_time
    void        set_runup_time(int8_t runup_time) { _runup_time = runup_time; }

    // recalc_scalers
    void        recalc_scalers();

    // output - update value to send to ESC/Servo
    void        output(uint8_t state);

private:

    // external variables
    RC_Channel&     _servo_output;
    int8_t          _servo_output_channel;      // output channel to rotor esc
    float           _loop_rate;                 // main loop rate

    // internal variables
    int8_t          _control_mode = 0;          // motor control mode, Passthrough or Setpoint
    int16_t         _critical_speed = 0;        // rotor speed below which flight is not possible
    int16_t         _idle_speed = 0;            // motor output idle speed
    int16_t         _max_speed = 1000;          // rotor maximum speed. Placeholder value until we have measured speed input (ToDo)
    int16_t         _desired_speed = 0;         // latest desired rotor speed from pilot
    int16_t         _control_speed = 0;         // latest logic controlled rotor speed
    float           _rotor_ramp_output = 0;     // scalar used to ramp rotor speed between _rsc_idle and full speed (0.0-1.0f)
    float           _rotor_runup_output = 0;    // scalar used to store status of rotor run-up time (0.0-1.0f)
    float           _ramp_increment = 0;        // the amount to increase/decrease the rotor ramp scalar during each iteration
    int8_t          _ramp_time = 0;             // time in seconds for the output to the main rotor's ESC to reach full speed
    int8_t          _runup_time = 0;            // time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    float           _runup_increment = 0;       // the amount to increase/decrease the rotor run-up scalar during each iteration
    bool            _runup_complete = false;    // flag for determining if runup is complete

    // update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
    void            update_rotor_ramp(float rotor_ramp_input);

    // update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
    void            update_rotor_runup();

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1000
    void            write_rsc(int16_t servo_out);
};

#endif // AP_MOTORS_HELI_RSC_H