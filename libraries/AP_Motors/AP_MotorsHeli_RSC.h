#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AC_Autorotation/RSC_Autorotation.h>
#include "AP_Motors_Class.h"

// rotor control modes
enum RotorControlMode {
    ROTOR_CONTROL_MODE_DISABLED = 0,
    ROTOR_CONTROL_MODE_PASSTHROUGH,
    ROTOR_CONTROL_MODE_SETPOINT,
    ROTOR_CONTROL_MODE_THROTTLECURVE,
    ROTOR_CONTROL_MODE_AUTOTHROTTLE
};

class AP_MotorsHeli_RSC {
public:
    friend class AP_MotorsHeli_Single;
    friend class AP_MotorsHeli_Dual;
    friend class AP_MotorsHeli_Quad;

    AP_MotorsHeli_RSC(SRV_Channel::Function aux_fn,
                      uint8_t default_channel,
                      uint8_t inst) :
        _instance(inst),
        _aux_fn(aux_fn),
        _default_channel(default_channel)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    using DesiredRSCSpoolState = AP_Motors::DesiredSpoolState;
    using RSCSpoolState = AP_Motors::SpoolState;

    // initialize - servo initialization and parameter setup on start-up
    void        initialize();

    // configure - configures RSC for current control mode, should be called on each update of control mode and on start-up after initialize() to setup parameters and scalars for the current control mode
    void        configure();

    // configure - configure the RSC with specific settings, allows caller to specify settings instead of using parameters.
    void        configure(RotorControlMode control_mode,
                          int8_t ramp_time,
                          int8_t runup_time,
                          float critical_speed,
                          float idle_output);

    // configure - configures the parameters for the armed state, should be called on each update of control mode and on start-up after initialize() to setup parameters and scalars for the current control mode
    void        configure_armed();

    // update - runs RSC logic and outputs to motors, returns current spool state for use in motor output logic
    RSCSpoolState        update(DesiredRSCSpoolState desired_spool_state);

    // output_to_servo - outputs pwm onto output rsc channel.
    void        output_to_servo() { write_rsc(_control_output);}

    // get_rsc_control_mode - gets RSC control mode
    uint8_t     get_rsc_control_mode() const { return _rsc_control_mode; }

    // get_desired_rotor_speed
    float       get_desired_rotor_speed() const { return _desired_rotor_speed; }

    // set_passthru_desired_rotor_speed - this requires input to be 0-1
    void        set_passthru_desired_rotor_speed(float desired_rotor_speed) { _passthru_desired_rotor_speed = desired_rotor_speed; }

    // set_collective. collective for throttle curve calculation
    void        set_collective(float collective) { _collective_in = collective; }

    // true if we are considered to be autorotating or bailing out of an autorotation
    bool        in_autorotation(void) const;

    // turbine start initialize sequence
    void        set_turbine_start(bool turbine_start) {_turbine_start = turbine_start; }

    // Return mask of output channels which the RSC is outputting on
    uint32_t    get_output_mask() const;

#if HAL_LOGGING_ENABLED
    // RSC logging
    void write_log(void) const;
#endif

    RSC_Autorotation autorotation;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // parameters
    AP_Int16        _rsc_setpoint;            // rotor speed when RSC mode is set to is enabled
    AP_Int8         _rsc_mode;                // Which main rotor ESC control mode is active
    AP_Int8         _ramp_time;               // Time in seconds for the output to the main rotor's ESC to reach setpoint
    AP_Int8         _runup_time;              // Time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    AP_Int16        _critical_speed;          // Rotor speed below which flight is not possible
    AP_Int16        _idle_output;             // Rotor control output while at idle

private:

    // update_spool_state - updates the spool state machine based on the desired spool state and current spool state
    void            update_spool_state();

    // set_rsc_control_mode - sets RSC control mode
    void            set_rsc_control_mode(RotorControlMode mode) { _rsc_control_mode = mode; }

    // reset_rsc_mode_param - resets rsc mode param to current control mode
    void            reset_rsc_mode_param() { _rsc_mode.set((uint8_t)_rsc_control_mode); }

    // set_critical_speed
    void            set_critical_speed(float critical_speed) { _critical_speed.set(critical_speed); }

    // functions for autothrottle, throttle curve, governor, idle speed, output to servo
    void            set_governor_output(float governor_output) {_governor_output = governor_output; }
    void            governor_reset();
    float           get_control_output() const { return _control_output; }
    void            set_idle_output(float idle_output) { _idle_output.set(idle_output); }
    void            autothrottle_run();
    void            set_throttle_curve();

    // functions for ramp and runup timers
    void            set_ramp_time(int8_t ramp_time) { _ramp_time.set(ramp_time); }
    void            set_runup_time(int8_t runup_time) { _runup_time.set(runup_time); }

    // update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
    void            update_rotor_ramp(float rotor_ramp_input, float dt);

    // update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
    void            update_rotor_runup(float dt);

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1
    void            write_rsc(float servo_out);

    // calculate_throttlecurve - uses throttle curve and collective input to determine throttle setting
    float           calculate_throttlecurve(float collective_in);

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool            rotor_speed_above_critical(void) const { return _rotor_runup_output >= get_critical_speed(); }

    uint64_t        _last_update_us;
    const uint8_t   _instance;

    // channel setup for aux function
    const SRV_Channel::Function _aux_fn;
    const uint8_t  _default_channel;

    RSCSpoolState          _spool_state;               // current spool state
    DesiredRSCSpoolState   _desired_spool_state;       // desired spool state

    // internal variables
    RotorControlMode _rsc_control_mode = ROTOR_CONTROL_MODE_DISABLED;   // RSC control mode, Passthrough, Setpoint, Throttle Curve or Autothrottle
    float           _passthru_desired_rotor_speed;// latest pilot desired rotor speed, used for passthrough mode
    float           _desired_rotor_speed;         // latest desired rotor speed
    float           _control_output;              // latest logic controlled output
    float           _rotor_ramp_output;           // scalar used to ramp rotor speed between _rsc_idle_output and full speed (0.0-1.0f)
    float           _rotor_runup_output;          // scalar used to store status of rotor run-up time (0.0-1.0f)
    bool            _runup_complete;              // flag for determining if runup is complete
    float           _thrcrv_poly[4][4];           // spline polynomials for throttle curve interpolation
    float           _collective_in;               // collective in for throttle curve calculation, range 0-1.0f
    float           _rotor_rpm;                   // rotor rpm from speed sensor for governor
    bool            _turbine_start;               // initiates starting sequence
    bool            _starting;                    // tracks if starting sequence has been used
    float           _governor_output;             // governor output for rotor speed control
    bool            _governor_engage;             // RSC governor status flag
    bool            _autothrottle;                // autothrottle status flag
    bool            _governor_fault;              // governor fault status flag
    bool            _spooldown_complete;          // flag for determining if spooldown is complete
    float           _fast_idle_timer;             // cooldown timer variable
    uint8_t         _governor_fault_count;        // variable for tracking governor speed sensor faults
    float           _governor_torque_reference;   // governor reference for load calculations
    float           _idle_throttle;               // current idle throttle setting
    bool            _save_rsc_mode;               // flag to determine if we should save RSC mode changes to EEPROM, used to prevent saving changes to RSC mode param while armed.

    // parameters
    AP_Int16        _power_slewrate;            // throttle slew rate (percentage per second)
    AP_Int16        _thrcrv[5];                 // throttle value sent to throttle servo at 0, 25, 50, 75 and 100 percent collective
    AP_Int16        _governor_rpm;              // governor reference for speed calculations
    AP_Float        _governor_torque;           // governor torque rise setting
    AP_Float        _governor_compensator;      // governor torque compensator variable
    AP_Float        _governor_droop_response;   // governor response to droop under load
    AP_Float        _governor_ff;               // governor feedforward variable
    AP_Float        _governor_range;            // RPM range +/- governor rpm reference setting where governor is operational
    AP_Int16        _cooldown_time;             // cooldown time to provide a fast idle

    // parameter accessors to allow conversions
    float           get_critical_speed() const { return _critical_speed * 0.01; }
    float           get_idle_output() const { return _idle_output * 0.01; }
    float           get_governor_torque() const { return _governor_torque * 0.01; }
    float           get_governor_compensator() const { return _governor_compensator * 0.000001; }

};
