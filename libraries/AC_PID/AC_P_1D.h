#pragma once

/// @file	AC_P_1D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

/// @class	AC_P_1D
/// @brief	Object managing one P controller
class AC_P_1D {
public:

    // Constructor for PID
    AC_P_1D(float initial_p, float dt);

    // set_dt - set time step in seconds
    void set_dt(float dt);

    void set_limits_error(float error_min, float error_max, float output_min, float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and _error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    float update_all(float &target, float measurement, bool limit_min, bool limit_max);

    // get_pi - get results from pid controller
    float get_p() const;
    float get_error() const {return _error;}

    // load gain from eeprom
    void load_gains();

    // save gain to eeprom
    void save_gains();

    /// Overload the function call operator to permit easy initialisation
    void operator()(float initial_p, float dt);

    /// asymetricLimit - set limits based on seperate max and min
    bool asymetricLimit(float &input, float min, float max, bool &limitMin, bool &limitMax );

    // Proportional controller with piecewise sqrt sections to constrain second derivative
    float sqrt_controller(float error, float p, float D_max, float dt);
    float inv_sqrt_controller(float output, float p, float D_max);

    // accessors
    AP_Float        &kP() { return _kp; }
    const AP_Float  &kP() const { return _kp; }
    void            kP(const float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha();

    // parameters
    AP_Float        _kp;

    // internal variables
    float           _dt;        // time step in seconds
    float           _error;     // error value to enable filtering
    float           _lim_err_neg;     // error value to enable filtering
    float           _lim_err_pos;     // error value to enable filtering
    float           _lim_out_neg;     // error value to enable filtering
    float           _lim_out_pos;     // error value to enable filtering
    float           _lim_D_Out;     // maximum first differential of output
};
