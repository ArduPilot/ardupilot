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
    AC_P_2D(float initial_p, float dt);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    // set target and measured inputs to P controller and calculate outputs
    Vector2f update_all(float &target_x, float &target_y, const Vector2f &measurement, float error_max, float D2_max) WARN_IF_UNUSED;

    // set target and measured inputs to P controller and calculate outputs
    // measurement is provided as 3-axis vector but only x and y are used
    Vector2f update_all(float &target_x, float &target_y, const Vector3f &measurement, float error_max, float D2_max) WARN_IF_UNUSED {
        return update_all(target_x, target_y, Vector2f(measurement.x, measurement.y), error_max, D2_max);
    }

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }

    // set accessor
    void kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // parameters
    AP_Float    _kp;

    // internal variables
    float _dt;          // time step in seconds
};
