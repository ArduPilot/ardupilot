#pragma once

/// @file   AC_P_2D.h
/// @brief  Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

/// @class  AC_P_2D
/// @brief  Copter PID control class
class AC_P_2D {
public:

    // Constructor for PID
    AC_P_2D(float initial_p, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    Vector2f update_all(float &target_x, float &target_y, Vector2f measurement, float max);
    Vector2f update_all(float &target_x, float &target_y, Vector3f measurement, float max) {return update_all(target_x, target_y, Vector2f(measurement.x, measurement.y), max);}

    // get_pi - get results from pid controller
    Vector2f get_p() const;
    Vector2f get_error() const {return _error;}
    float get_error_mag() const {return _error.length();}

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void        reset_filter();

    // load gain from eeprom
    void        load_gains();

    // save gain to eeprom
    void        save_gains();

    /// operator function call for easy initialisation
    void operator()(float initial_p, float dt);

    // get accessors
    AP_Float    &kP() { return _kp; }

    // set accessors
    void        kP(const float v) { _kp.set(v); }

    /// limit vector to a given length, returns true if vector was limited
    static bool limit_vector_length(float& vector_x, float& vector_y, float max_length);

    /// Proportional controller with piecewise sqrt sections to constrain second derivative
    static Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha();

    // parameters
    AP_Float        _kp;

    // internal variables
    float           _dt;        // time step in seconds
    Vector2f        _error;         // error value to enable filtering
    float           _D_max;     // maximum first differential of output
    float           _D2_max;    // maximum second differential of output
};
