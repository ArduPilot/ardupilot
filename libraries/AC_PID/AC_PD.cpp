/*
  Generic PD algorithm with a LPF filter on the derivative term.
    Used for tailsitter angle control.
*/

#include <AP_Math/AP_Math.h>
#include "AC_PD.h"

const AP_Param::GroupInfo AC_PD::var_info[] = {
    // @Param: P
    // @DisplayName: PD Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    1, AC_PD, kP, default_kp),

    // @Param: D
    // @DisplayName: PD Integral Gain
    // @Description: D Gain which produces an output value that is proportional to the rate of change of the error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    2, AC_PD, kD, default_kd),

    // @Param: alpha
    // @DisplayName: PD Alpha
    // @Description: Alpha value for the low-pass filter applied to the derivative term
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("alpha", 3, AC_PD, alpha, default_alpha),
    AP_GROUPEND
};

// Constructor
AC_PD::AC_PD(float initial_p, float initial_d, float initial_alpha) :
    default_kp(initial_p),
    default_kd(initial_d),
    default_alpha(initial_alpha)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
}

float AC_PD::update(float err, float dt)
{
    static float error_last = 0.0f,derivative_last = 0.0f;
    derivative = (err - error_last) / dt;
    error_last = err;
    derivative = alpha * derivative + (1.0f - alpha) * derivative_last;
    derivative_last = derivative;
    output_P = kP * err;

    return output_P + (kD * derivative);
}
void AC_PD::save_gains()
{
    kP.save();
}
