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
    AC_P_1D(float initial_p, float dt);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    // get maximum error
    float get_error_min() const {return _error_min;}
    float get_error_max() const {return _error_max;}

    // update_all - set target and measured inputs to P controller and calculate outputs
    // target and measurement are filtered
    // if measurement is further than error_min or error_max (see set_limits method)
    //   the target is moved closer to the measurement and limit_min or limit_max will be set true
    float update_all(float &target, float measurement, bool &limit_min, bool &limit_max) WARN_IF_UNUSED;

    // set limits on error, output and output from D term
    void set_limits(float error_min, float error_max, float output_min, float output_max, float D1_max = 0.0f, float D2_max = 0.0f);
    float get_error_min() {return _error_min;}
    float get_error_max() {return _error_max;}

    // save gain to eeprom
    void save_gains() { _kp.save(); }

    // accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    void kP(float v) { _kp.set(v); }
    float get_error() const {return _error;}

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float _kp;

    // internal variables
    float _dt;          // time step in seconds
    float _error;       // time step in seconds
    float _error_min; // error limit in negative direction
    float _error_max; // error limit in positive direction
    float _D1_max;      // maximum first derivative of output
};
