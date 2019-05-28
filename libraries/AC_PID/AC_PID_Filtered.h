#pragma once

/// @file	AC_PID_Filtered.h
/// @brief	Multicopter Specific Rate PID algorithm with advanced filtering, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>
#include <Filter/NotchFilter.h>
#include "AC_PID.h"

#define AC_PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_FILT_HZ_MIN      0.01f   // minimum input filter frequency

/// @class	AC_PID_Fiteredd
/// @brief	PID control class with D-term notch filter
class AC_PID_Filtered : public AC_PID {
public:

    /// Constructor for PID
    AC_PID_Filtered(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff=0);

   // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input_filter_all - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_all(float input);

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(float input);

    // set accessors
    void        filt_hz(const float v);

    // load gain from eeprom
    virtual void        load_gains();

    // save gain to eeprom
    virtual void        save_gains();

    float       get_derivative() const { return _derivative; }
    float       get_raw_derivative() const { return _raw_derivative; }
    bool        notch_requires_update() const {
        return !is_equal(notch_center_freq_hz.get(), _last_notch_center_freq_hz) ||
            !is_equal(notch_bandwidth_hz.get(), _last_notch_bandwidth_hz) ||
            !is_equal(notch_attenuation_dB.get(), _last_notch_attenuation_dB);
    }
    void        notch_update_params();

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

    AP_Int8     notch_enable;
    AP_Int16    notch_center_freq_hz;
    AP_Int16    notch_bandwidth_hz;
    AP_Float    notch_attenuation_dB;
    AP_Float    filt_hz2;                   // 2nd PID Input filter frequency in Hz

private:
    // internal variables
    float           _raw_input;             // last filtered output value
    float           _raw_derivative;        // last raw input derivative
    int16_t         _last_notch_center_freq_hz;
    int16_t         _last_notch_bandwidth_hz;
    float           _last_notch_attenuation_dB;

private:
    LowPassFilterFloat _pid_filter;
    LowPassFilterFloat _pid_filter2;
    NotchFilterFloat _pid_notch_filter;
};
