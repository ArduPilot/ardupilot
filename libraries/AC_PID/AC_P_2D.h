#pragma once

/// @file   AC_P_2D.h
/// @brief  2-axis P controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class  AC_P_2D
/// @brief  2-axis P controller
class AC_P_2D {
public:

    // constructor
    AC_P_2D(float initial_p);

    CLASS_NO_COPY(AC_P_2D);

    // set target and measured inputs to P controller and calculate outputs
    Vector2f update_all(postype_t &target_x, postype_t &target_y, const Vector2f &measurement) WARN_IF_UNUSED;

    // set target and measured inputs to P controller and calculate outputs
    // measurement is provided as 3-axis vector but only x and y are used
    Vector2f update_all(postype_t &target_x, postype_t &target_y, const Vector3f &measurement) WARN_IF_UNUSED {
        return update_all(target_x, target_y, Vector2f{measurement.x, measurement.y});
    }

    // set_limits - sets the maximum error to limit output and first and second derivative of output
    void set_limits(float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    // set_error_max - reduce maximum position error to error_max
    // to be called after setting limits
    void set_error_max(float error_max);

    // get_error_max - return maximum position error
    float get_error_max() { return _error_max; }

    // save gain to eeprom
    void save_gains() { _kp.save(); }

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    const Vector2f& get_error() const { return _error; }

    // set accessors
    void kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float    _kp;

    // internal variables
    Vector2f _error;    // time step in seconds
    float _error_max;   // error limit in positive direction
    float _D1_max;      // maximum first derivative of output

    const float default_kp;
};
