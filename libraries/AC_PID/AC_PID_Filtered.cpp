/// @file	AC_PID_Filtered.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_Filtered.h"

const AP_Param::GroupInfo AC_PID_Filtered::var_info[] = {
    // parameters from parent PID
    AP_NESTEDGROUPINFO(AC_PID, 0),

    // @Param: FLT2
    // @DisplayName: Second PID Input filter frequency in Hz
    // @Description: Second Input filter frequency in Hz. Filter is applied before the main filter. For small copters a frequency double that of FILT is a good starting point. This filter can be set at most to half the loop frequency.
    // @Range: 0 200
    // @Units: Hz
    AP_GROUPINFO("FLT2", 1, AC_PID_Filtered, filt_hz2, 0.0f),

    // @Param: NTCH
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("NTCH", 2, AC_PID_Filtered, notch_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: NTHZ
    // @DisplayName: Frequency
    // @Description: Notch center frequency in Hz
    // @Range: 10 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTHZ", 3, AC_PID_Filtered, notch_center_freq_hz, 80),

    // @Param: NTBW
    // @DisplayName: Bandwidth
    // @Description: Notch bandwidth in Hz
    // @Range: 5 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTBW", 4, AC_PID_Filtered, notch_bandwidth_hz, 20),

    // @Param: NTAT
    // @DisplayName: Attenuation
    // @Description: Notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("NTAT", 5, AC_PID_Filtered, notch_attenuation_dB, 15),


    AP_GROUPEND
};

// Constructor
AC_PID_Filtered::AC_PID_Filtered(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff) :
    AC_PID(initial_p, initial_i, initial_d, initial_imax, initial_filt_hz, dt, initial_ff),
    _raw_input(0.0f),
    _raw_derivative(0.0f)
{
    // By default 2nd filter tracks the first
    filt_hz2.set(0.0f);

    AP_Param::setup_object_defaults(this, var_info);
    set_dt(dt); // Initializes all the filters as well
}

// update the notch parameters
void AC_PID_Filtered::notch_update_params()
{
    _pid_notch_filter.init(1.0/_dt, notch_center_freq_hz, notch_bandwidth_hz, notch_attenuation_dB);
    _last_notch_center_freq_hz = notch_center_freq_hz;
    _last_notch_bandwidth_hz = notch_bandwidth_hz;
    _last_notch_attenuation_dB = notch_attenuation_dB;
}

// set_dt - set time step in seconds
void AC_PID_Filtered::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    AC_PID::set_dt(dt);
    _pid_filter.set_cutoff_frequency(1.0/dt, _filt_hz);
    if (is_zero(filt_hz2.get())) {
        _pid_filter2.set_cutoff_frequency(1.0/dt, _filt_hz);
    }
    else {
        _pid_filter2.set_cutoff_frequency(1.0/dt, filt_hz2);
    }
    notch_update_params();
}

// filt_hz - set input filter hz, used by autotune, does not affect _pid_filter2
void AC_PID_Filtered::filt_hz(float hz)
{
    AC_PID::filt_hz(hz);
    _pid_filter.set_cutoff_frequency(1.0/_dt, _filt_hz);
    if (is_zero(filt_hz2.get())) {
        _pid_filter2.set_cutoff_frequency(1.0/_dt, _filt_hz);
    }
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_Filtered::set_input_filter_all(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // update notch parameters if necessary
    if (notch_requires_update()) {
        notch_update_params();
        _flags._reset_filter = true;
    }

    if (!is_equal(_filt_hz.get(), _pid_filter.get_cutoff_freq())) {
        _pid_filter.set_cutoff_frequency(1.0/_dt, _filt_hz);
        // FILT2 may track FILT
        if (is_zero(filt_hz2.get())) {
            _pid_filter2.set_cutoff_frequency(1.0/_dt, _filt_hz);
        }
        _flags._reset_filter = true;
    }

    if (!is_zero(filt_hz2.get()) && !is_equal(filt_hz2.get(), _pid_filter2.get_cutoff_freq())) {
        _pid_filter2.set_cutoff_frequency(1.0/_dt, filt_hz2);
        _flags._reset_filter = true;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _pid_filter.reset();
        _pid_filter2.reset();
        _pid_notch_filter.reset();
        _flags._reset_filter = false;
        _input = input;
        _raw_input = input;
        _derivative = 0.0f;
        _raw_derivative = 0.0f;
    }

    // update filter and calculate derivative
    float next_input = _pid_filter2.apply(input);
    next_input = _pid_filter.apply(next_input);
    if (notch_enable.get()) {
        next_input = _pid_notch_filter.apply(next_input);
    }
    if (_dt > 0.0f) {
        _derivative = (next_input - _input) / _dt;
        _raw_derivative = (input - _raw_input) / _dt;        
    }
    _input = next_input;
    _raw_input = input;
}

// set_input_filter_d - set input to PID controller
// only input to the D portion of the controller is filtered
// this should be called before any other calls to get_p, get_i or get_d
void AC_PID_Filtered::set_input_filter_d(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // update notch parameters if necessary
    if (notch_requires_update()) {
        notch_update_params();
        _flags._reset_filter = true;
    }

    if (!is_equal(_filt_hz.get(), _pid_filter.get_cutoff_freq())) {
        _pid_filter.set_cutoff_frequency(1.0/_dt, _filt_hz);
        // FILT2 may track FILT
        if (is_zero(filt_hz2.get())) {
            _pid_filter2.set_cutoff_frequency(1.0/_dt, _filt_hz);
        }
        _flags._reset_filter = true;
    }

    if (!is_zero(filt_hz2.get()) && !is_equal(filt_hz2.get(), _pid_filter2.get_cutoff_freq())) {
        _pid_filter2.set_cutoff_frequency(1.0/_dt, filt_hz2);
        _flags._reset_filter = true;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _pid_filter.reset();
        _pid_filter2.reset();
        _pid_notch_filter.reset();
        _flags._reset_filter = false;
        _input = input;
        _derivative = 0.0f;
        _raw_input = input;
        _raw_derivative = 0.0f;
    }

    // update filter and calculate derivative
    if (_dt > 0.0f) {
        _raw_derivative = (input - _input) / _dt;
        _derivative = _pid_filter2.apply(_raw_derivative);
        _derivative = _pid_filter.apply(_derivative);
        if (notch_enable.get()) {
            _derivative = _pid_notch_filter.apply(_derivative);
       }
    }

    _raw_input = input;
    _input = input;
}

void AC_PID_Filtered::load_gains()
{
    AC_PID::load_gains();
    // not sure these are really necessary, but filt frequency is important to yaw autotune
    filt_hz2.load();
    notch_attenuation_dB.load();
    notch_bandwidth_hz.load();
    notch_center_freq_hz.load();
    _pid_filter.set_cutoff_frequency(1.0/_dt, _filt_hz);
    _pid_filter2.set_cutoff_frequency(1.0/_dt, filt_hz2);
}

// save_gains - save gains to eeprom
void AC_PID_Filtered::save_gains()
{
    AC_PID::save_gains();
    // not sure these are really necessary, but filt frequency is important to yaw autotune
    filt_hz2.save();
    notch_attenuation_dB.save();
    notch_bandwidth_hz.save();
    notch_center_freq_hz.save();
}




