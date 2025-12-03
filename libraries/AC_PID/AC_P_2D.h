#pragma once

/// @file   AC_P_2D.h
/// @brief  2D P controller with output limiting, derivative constraint, and EEPROM-backed gain storage.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class  AC_P_2D
/// @brief  2-axis P controller
class AC_P_2D {
public:

    /// Constructor for 2D P controller with initial gain.
    AC_P_2D(float initial_p);

    CLASS_NO_COPY(AC_P_2D);

    // Computes the P controller output given a target and measurement.
    // Applies position error clamping based on configured limits.
    // Optionally constrains output slope using the sqrt_controller.
    Vector2f update_all(Vector2p &target, const Vector2p &measurement) WARN_IF_UNUSED;

    // Sets limits on output, output slope (D1), and output acceleration (D2).
    // For position controllers: output = velocity, D1 = acceleration, D2 = jerk.
    void set_limits(float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    // Reduces the maximum allowable error after limit initialization.
    // Intended to be called after set_limits().
    void set_error_max(float error_max);

    // Returns the current error clamp limit in controller units.
    float get_error_max() const { return _error_max; }

    // Saves controller configuration from EEPROM. (not used)
    void save_gains() { _kp.save(); }

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    const Vector2f& get_error() const { return _error; }

    // set accessors
    void set_kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float    _kp;

    // internal variables
    Vector2f _error;    // error between target and measured
    float _error_max;   // error limit in positive direction
    float _D1_max;      // maximum first derivative of output

    const float default_kp;
};
