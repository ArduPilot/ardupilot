/*
  Generic PI algorithm for controllers that don't need filtering (such as heaters)
*/

#include <AP_Math/AP_Math.h>
#include "AC_PI.h"

const AP_Param::GroupInfo AC_PI::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    1, AC_PI, kP, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    2, AC_PI, kI, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 3, AC_PI, imax, 0),

    AP_GROUPEND
};

// Constructor
AC_PI::AC_PI(float initial_p, float initial_i, float initial_imax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    kP.set(initial_p);
    kI.set(initial_i);
    imax.set(initial_imax);
}

float AC_PI::update(float measurement, float target, float dt)
{
    const float err = target - measurement;

    integrator += kI * err * dt;
    integrator = constrain_float(integrator, 0, imax);
    output_P = kP * err;

    return output_P + integrator;
}
