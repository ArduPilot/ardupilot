#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

// default main rotor speed (ch8 out) as a number from 0 ~ 100
#define AP_MOTORS_HELI_RSC_SETPOINT             70

// default main rotor critical speed
#define AP_MOTORS_HELI_RSC_CRITICAL             50

// RSC output defaults
#define AP_MOTORS_HELI_RSC_IDLE_DEFAULT         0

// default main rotor ramp up time in seconds
#define AP_MOTORS_HELI_RSC_RAMP_TIME            1       // 1 second to ramp output to main rotor ESC to setpoint
#define AP_MOTORS_HELI_RSC_RUNUP_TIME           10      // 10 seconds for rotor to reach full speed
#define AP_MOTORS_HELI_RSC_BAILOUT_TIME         1       // time in seconds to ramp motors when bailing out of autorotation

// Throttle Curve Defaults
#define AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT     25
#define AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT    32
#define AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT    38
#define AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT    50
#define AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT   100

// RSC governor defaults
#define AP_MOTORS_HELI_RSC_GOVERNOR_SETPNT_DEFAULT    1500
#define AP_MOTORS_HELI_RSC_GOVERNOR_DISENGAGE_DEFAULT 25
#define AP_MOTORS_HELI_RSC_GOVERNOR_DROOP_DEFAULT     30
#define AP_MOTORS_HELI_RSC_GOVERNOR_TCGAIN_DEFAULT    90
#define AP_MOTORS_HELI_RSC_GOVERNOR_RANGE_DEFAULT     100

// rotor controller states
enum RotorControlState {
    ROTOR_CONTROL_STOP = 0,
    ROTOR_CONTROL_IDLE,
    ROTOR_CONTROL_ACTIVE
};

// rotor control modes
enum RotorControlMode {
    ROTOR_CONTROL_MODE_DISABLED = 0,
    ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH,
    ROTOR_CONTROL_MODE_SPEED_SETPOINT,
    ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT,
    ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT
};

class AP_MotorsHeli_RSC {
public:
    friend class AP_MotorsHeli_Single;
    friend class AP_MotorsHeli_Dual;
    friend class AP_MotorsHeli_Quad;

    AP_MotorsHeli_RSC(SRV_Channel::Aux_servo_function_t aux_fn,
                      uint8_t default_channel) :
        _aux_fn(aux_fn),
        _default_channel(default_channel)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init_servo - servo initialization on start-up
    void        init_servo();

    // set_control_mode - sets control mode
    void        set_control_mode(RotorControlMode mode) { _control_mode = mode; }

    // reset_rsc_mode_param - resets rsc mode param to current control mode
    void        reset_rsc_mode_param() { _rsc_mode.set((uint8_t)_control_mode); }

    // get_control_mode - gets control mode
    uint8_t     get_control_mode() const { return _control_mode; }

    // set_critical_speed
    void        set_critical_speed(float critical_speed) { _critical_speed = critical_speed; }

    // set_idle_output
    void        set_idle_output(float idle_output) { _idle_output = idle_output; }

    // set rotor speed governor parameters
    void        set_governor_output(float governor_output) {_governor_output = governor_output; }

    // get_desired_speed
    float       get_desired_speed() const { return _desired_speed; }

    // set_desired_speed - this requires input to be 0-1
    void        set_desired_speed(float desired_speed) { _desired_speed = desired_speed; }

    // get_control_speed
    float       get_control_output() const { return _control_output; }

    // get_rotor_speed - estimated rotor speed when no governor or rpm sensor is used
    float       get_rotor_speed() const;
    
    // set_rotor_rpm - when speed sensor is available for governor
    void        set_rotor_rpm(float rotor_rpm) {_rotor_rpm = (float)rotor_rpm; }
    
    // get_governor_output
    float       get_governor_output() const { return _governor_output; }

    // is_runup_complete
    bool        is_runup_complete() const { return _runup_complete; }

    // set_ramp_time
    void        set_ramp_time(int8_t ramp_time) { _ramp_time = ramp_time; }

    // set_runup_time
    void        set_runup_time(int8_t runup_time) { _runup_time = runup_time; }

    // set_throttle_curve
    void        set_throttle_curve();

    // set_collective. collective for throttle curve calculation
    void        set_collective(float collective) { _collective_in = collective; }

    // use bailout ramp time
    void        use_bailout_ramp_time(bool enable) { _use_bailout_ramp = enable; }

    // use external governor autorotation window
    void        set_autorotaion_flag(bool flag) { _in_autorotaion = flag; }

    // set the throttle percentage to be sent to external governor to signal that autorotation bailout ramp should be used within this instance of Heli_RSC
    void        set_ext_gov_arot_bail(int16_t pct) { _rsc_arot_bailout_pct = pct; }

    // output - update value to send to ESC/Servo
    void        output(RotorControlState state);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int16        _rsc_setpoint;            // rotor speed when RSC mode is set to is enabled
    AP_Int8         _rsc_mode;                // Which main rotor ESC control mode is active
    AP_Int8         _ramp_time;               // Time in seconds for the output to the main rotor's ESC to reach setpoint
    AP_Int8         _runup_time;              // Time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    AP_Int16        _critical_speed;          // Rotor speed below which flight is not possible
    AP_Int16        _idle_output;             // Rotor control output while at idle
    AP_Int16        _ext_gov_arot_pct;        // Percent value sent to external governor when in autorotation

private:
    uint64_t        _last_update_us;

    // channel setup for aux function
    SRV_Channel::Aux_servo_function_t _aux_fn;
    uint8_t         _default_channel;

    // internal variables
    RotorControlMode _control_mode = ROTOR_CONTROL_MODE_DISABLED;   // motor control mode, Passthrough or Setpoint
    float           _desired_speed;               // latest desired rotor speed from pilot
    float           _control_output;              // latest logic controlled output
    float           _rotor_ramp_output;           // scalar used to ramp rotor speed between _rsc_idle_output and full speed (0.0-1.0f)
    float           _rotor_runup_output;          // scalar used to store status of rotor run-up time (0.0-1.0f)
    bool            _runup_complete;              // flag for determining if runup is complete
    float           _thrcrv_poly[4][4];           // spline polynomials for throttle curve interpolation
    float           _collective_in;               // collective in for throttle curve calculation, range 0-1.0f
    float           _rotor_rpm;                   // rotor rpm from speed sensor for governor
    float           _governor_output;             // governor output for rotor speed control
    bool            _governor_engage;             // RSC governor status flag for soft-start
    bool            _use_bailout_ramp;            // true if allowing RSC to quickly ramp up engine
    bool            _in_autorotaion;              // true if vehicle is currently in an autorotaion
    int16_t         _rsc_arot_bailout_pct;        // the throttle percentage sent to the external governor to signal that autorotation bailout ramp should be used

    // update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
    void            update_rotor_ramp(float rotor_ramp_input, float dt);

    // update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
    void            update_rotor_runup(float dt);

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1
    void            write_rsc(float servo_out);

    // calculate_desired_throttle - uses throttle curve and collective input to determine throttle setting
    float           calculate_desired_throttle(float collective_in);

    // parameters
    AP_Int16        _power_slewrate;          // throttle slew rate (percentage per second)
    AP_Int16        _thrcrv[5];               // throttle value sent to throttle servo at 0, 25, 50, 75 and 100 percent collective
    AP_Int16        _governor_reference;      // sets rotor speed for governor
    AP_Float        _governor_range;          // RPM range +/- governor rpm reference setting where governor is operational
    AP_Float        _governor_disengage;      // sets the throttle percent where the governor disengages for return to flight idle
    AP_Float        _governor_droop_response; // governor response to droop under load
    AP_Float        _governor_tcgain;       // governor throttle curve weighting, range 50-100%

    // parameter accessors to allow conversions
    float       get_critical_speed() const { return _critical_speed * 0.01; }
    float       get_idle_output() { return _idle_output * 0.01; }
    float       get_governor_disengage() { return _governor_disengage * 0.01; }
    float       get_governor_droop_response() { return _governor_droop_response * 0.01; }
    float       get_governor_tcgain() { return _governor_tcgain * 0.01; }

};

