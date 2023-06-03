/// @file	AC_ADRC.h
/// @brief	Generic adaptive control algorithm, with EEPROM-backed storage of constants.

#include "AC_ADRC.h"
#include <AP_Math/AP_Math.h>

const AP_Param::GroupInfo AC_ADRC::var_info[] = {

    // @Param: WC
    // @Description: Response bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WC", 0, AC_ADRC, _wc, default_wc),

    // @Param: WO
    // @Description: ESO bandwidth
    // @Units: rad/s
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WO", 1, AC_ADRC, _wo, default_wo),
    
    // @Param: B0
    // @Description: Control input gain
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B0", 2, AC_ADRC, _b0, default_b0),

    // @Param: DELTA
    // @Description: Linear deadzone
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("DELTA", 3, AC_ADRC, _delta, default_delta),

    // @Param: ORDER
    // @Description: Model order
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ORDR", 4, AC_ADRC, _order, default_order),

    // @Param: LM
    // @Description: Control output bound
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("LM", 5, AC_ADRC, _limit, default_limit),

    AP_GROUPEND

};

AC_ADRC::AC_ADRC(float initial_wc, float initial_wo, float initial_b0, int8_t initial_order, 
                 float initial_delta, float initial_limit):
    default_wc(initial_wc),
    default_wo(initial_wo),
    default_b0(initial_b0),
    default_order(initial_order),
    default_delta(initial_delta),
    default_limit(initial_limit)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_debug_info, 0, sizeof(_debug_info));
}

float AC_ADRC::update_all(float target,float measurement, float dt)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // get controller error
    float e1 = target - _z1;
    
    // control derivation error
    float e2 = -_z2;      

    // state estimation error                
    float e  = _z1 - measurement;         

    float output = 0.0f;
    float output_limited = 0;
    float sigma = 1.0f /(sq(e) + 1.0f);

    switch (_order) {
    case 1: {
        // nonlinear control law
        output = (_wc * fal(e1, 0.5f, _delta)  - sigma * _z2) / _b0;

        // limit output 
        const float control_output_limit = _limit.get();
        if (is_zero(control_output_limit)) {
            output_limited = output;
        } else {
            output_limited = constrain_value(output, -control_output_limit, control_output_limit);
        }
        
        // state estimation
        float fe = fal(e, 0.5, _delta);
        float beta1 = 2 * _wo;
        float beta2 = _wo * _wo;
        _z1 = _z1 + dt * (_z2 - beta1 * e + _b0 * output_limited);
        _z2 = _z2 + dt * (-beta2 * fe);

        _debug_info.P      = _z1;
        _debug_info.I      = _z2;
        _debug_info.D      = _z3;
        _debug_info.FF     = output_limited;
        break;
    }
    case 2: {
        float kp  = sq(_wc);
        float kd  = 2 * _wc;

        // nonlinear control law
        output = (kp * fal(e1, 0.5f, _delta) + kd * fal(e2, 0.25, _delta) - sigma * _z3)/ _b0;

        // limit output 
        const float control_output_limit = _limit.get();
        if (is_zero(control_output_limit)) {
            output_limited = output;
        } else {
            output_limited = constrain_value(output, -control_output_limit, control_output_limit);
        }

        // state estimation
        float beta1 = 3 * _wo;
        float beta2 = 3 * _wo * _wo;
        float beta3 = _wo * _wo * _wo;
        float fe  = fal(e, 0.5, _delta);
        float fe1 = fal(e, 0.25, _delta);
        _z1  = _z1 + dt * (_z2 - beta1 * e);
        _z2  = _z2 + dt * (_z3 - beta2 * fe + _b0 * output_limited);
        _z3  = _z3 + dt * (- beta3 * fe1);

        _debug_info.P      = _z1;
        _debug_info.I      = _z2;
        _debug_info.D      = _z3;
        _debug_info.FF     = output_limited;
        break;
    }
       
    default:
        output_limited = 0.0f;
        break;
    }


    // For loggers
    _debug_info.target = target;
    _debug_info.actual = measurement;
    _debug_info.error  = target - measurement;

    return output_limited;
}

void AC_ADRC::reset_eso(float measurement)
{
    _z1 = measurement;
    _z2 = 0.0f;
    _z3 = 0.0f;
    memset(&_debug_info, 0, sizeof(_debug_info));
}