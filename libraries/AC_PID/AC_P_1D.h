#pragma once

/// @file	AC_P_1D.h
/// @brief	Position-based P controller with optional limits on output and its first derivative.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class	AC_P_1D
/// @brief	Object managing one P controller
class AC_P_1D {
public:

    /// Constructor for 1D P controller with initial gain.
    AC_P_1D(float initial_p);

    CLASS_NO_COPY(AC_P_1D);

    // Computes the P controller output given a target and measurement.
    // Applies position error clamping based on configured limits.
    // Optionally constrains output slope using the sqrt_controller.
    float update_all(postype_t &target, postype_t measurement) WARN_IF_UNUSED;

    // Sets limits on output, output slope (D1), and output acceleration (D2).
    // For position controllers: output = velocity, D1 = acceleration, D2 = jerk.
    void set_limits(float output_min, float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    // Reduces error limits to user-specified bounds, respecting previously computed limits.
    // Intended to be called after `set_limits()`.
    void set_error_limits(float error_min, float error_max);

    // Returns the current minimum error clamp, in controller units.
    float get_error_min() const { return _error_min; }

    // Returns the current maximum error clamp, in controller units.
    float get_error_max() const { return _error_max; }

    // Saves controller configuration from EEPROM. (not used)
    void save_gains() { _kp.save(); }

    // accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    float get_error() const { return _error; }

    // set accessors
    void set_kP(float v) { _kp.set(v); }

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
