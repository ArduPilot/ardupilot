#pragma once

/// @file	AC_P_1D.h
/// @brief	Generic P controller, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class	AC_P_1D
/// @brief	Object managing one P controller
class AC_P_1D {
public:

    // constructor
    AC_P_1D(float initial_p);

    CLASS_NO_COPY(AC_P_1D);

    // update_all - set target and measured inputs to P controller and calculate outputs
    // target and measurement are filtered
    float update_all(float &target, float measurement) WARN_IF_UNUSED;

    // set_limits - sets the maximum error to limit output and first and second derivative of output
    void set_limits(float output_min, float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    // set_error_limits - reduce maximum position error to error_max
    // to be called after setting limits
    void set_error_limits(float error_min, float error_max);

    // get_error_min - return minimum position error
    float get_error_min() const { return _error_min; }

    // get_error_max - return maximum position error
    float get_error_max() const { return _error_max; }

    // save gain to eeprom
    void save_gains() { _kp.save(); }

    // accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    float get_error() const { return _error; }

    // set accessors
    void kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float _kp;

    // internal variables
    float _error;       // time step in seconds
    float _error_min; // error limit in negative direction
    float _error_max; // error limit in positive direction
    float _D1_max;      // maximum first derivative of output

    const float default_kp;
};
