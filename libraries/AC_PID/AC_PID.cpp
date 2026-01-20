/// @file	AC_PID.cpp
/// @brief	General-purpose PID controller with input, error, and derivative filtering, plus slew rate limiting and EEPROM gain storage.

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"

#define AC_PID_DEFAULT_NOTCH_ATTENUATION 40

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_PID, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 1, AC_PID, _ki, default_ki),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, AC_PID, _kd, default_kd),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, AC_PID, _kff, default_kff),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 5, AC_PID, _kimax, default_kimax),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the target input (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 9, AC_PID, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the error (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 10, AC_PID, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the derivative (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 11, AC_PID, _filt_D_hz, default_filt_D_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 12, AC_PID, _slew_rate_max, default_slew_rate_max),

    // @Param: PDMX
    // @DisplayName: PD sum maximum
    // @Description: The maximum/minimum value that the sum of the P and D term can output
    // @User: Advanced
    AP_GROUPINFO("PDMX", 13, AC_PID, _kpdmax, 0),

    // @Param: D_FF
    // @DisplayName: PID Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_FF", 14, AC_PID, _kdff, default_kdff),

#if AP_FILTER_ENABLED
    // @Param: NTF
    // @DisplayName: PID Target notch filter index
    // @Description: PID Target notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NTF", 15, AC_PID, _notch_T_filter, 0),

    // @Param: NEF
    // @DisplayName: PID Error notch filter index
    // @Description: PID Error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NEF", 16, AC_PID, _notch_E_filter, 0),
#endif

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float initial_srmax, float initial_srtau, float initial_dff) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_kd(initial_d),
    default_kff(initial_ff),
    default_kdff(initial_dff),
    default_kimax(initial_imax),
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_slew_rate_max(initial_srmax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // this param is not in the table, so its default is no loaded in the call above
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// Sets the target low-pass filter frequency (Hz)
void AC_PID::set_filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// Sets the error low-pass filter frequency (Hz)
void AC_PID::set_filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// Sets the derivative low-pass filter frequency (Hz)
void AC_PID::set_filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_PID::set_slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

// Configures optional notch filters for target and error signals using the given sample rate.
// Filters are dynamically allocated and validated via the AP_Filter API.
void AC_PID::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_notch_T_filter == 0 && _notch_E_filter == 0) {
        return;
    }

    if (_notch_T_filter != 0) {
        if (_target_notch == nullptr) {
            _target_notch = NEW_NOTHROW NotchFilterFloat();
        }
        // Lookup filter definition and initialize if valid
        AP_Filter* filter = AP::filters().get_filter(_notch_T_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_target_notch, sample_rate)) {
            delete _target_notch;
            _target_notch = nullptr;
            _notch_T_filter.set(0);  // disable filter if setup fails
        }
    }

    if (_notch_E_filter != 0) {
        if (_error_notch == nullptr) {
            _error_notch = NEW_NOTHROW NotchFilterFloat();
        }
        // Lookup filter definition and initialize if valid
        AP_Filter* filter = AP::filters().get_filter(_notch_E_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_error_notch, sample_rate)) {
            delete _error_notch;
            _error_notch = nullptr;
            _notch_E_filter.set(0);  // disable filter if setup fails
        }
    }
#endif
}

// Computes the PID output using a target and measurement input.
// Applies filters to the target and error, calculates the derivative and updates the integrator.
// If `limit` is true, the integrator is allowed to shrink but not grow.
float AC_PID::update_all(float target, float measurement, float dt, bool limit, float pd_scale, float i_scale)
{
    // Return zero if input is invalid (NaN or infinite)
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // Flag used for logging to indicate a filter reset occurred
    _pid_info.reset = _flags._reset_filter;
    // Reset filters to match the current input (first sample or after reset)
    if (_flags._reset_filter) {
        // Reset filters to match the current inputs
        _flags._reset_filter = false;

        // Reset target filter
        _target = target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            _target_notch->reset();
            _target = _target_notch->apply(_target);
        }
#endif

        // Calculate error and reset error filter
        _error = _target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            _error_notch->reset();
            _error = _error_notch->apply(_error);
        }
#endif
        // Clear derivative history to avoid spikes after reset
        _derivative = 0.0f;
        _target_derivative = 0.0f;

    } else {

        // Apply target filters
        const float target_last = _target;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            // Allocate and set up target notch filter
            target = _target_notch->apply(target);
        }
#endif
        // Apply first-order low-pass filter to target value
        _target += get_filt_T_alpha(dt) * (target - _target);

        // Calculate error and apply error filter
        const float error_last = _error;
        float error = _target - measurement;
#if AP_FILTER_ENABLED
        if (_error_notch != nullptr) {
            // Allocate and set up error notch filter
            error = _error_notch->apply(error);
        }
#endif
        // apply notch filters before FTLD/FLTE to minimize shot noise
        _error += get_filt_E_alpha(dt) * (error - _error);

        if (is_positive(dt)) {
            // Compute and low-pass filter the error derivative (D term)
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
            // Calculate target derivative for D_FF contribution
            _target_derivative = (_target - target_last) / dt;
        }
    }

    // Integrate error (with wind-up protection if limit is active)
    // If limit is active, allow I-term to shrink but not grow
    update_i(dt, limit, i_scale);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);
    float I_out = _integrator;

    // Calculate dynamic modifier to reduce P+D output based on slew rate limiter
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    // This modifier is used to reduce control effort under fast transients
    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // scale pd output if required
    P_out *= pd_scale;
    D_out *= pd_scale;

    _pid_info.PD_limit = false;
    // Apply PD sum limit if enabled
    if (is_positive(_kpdmax)) {
        const float PD_sum_abs = fabsf(P_out + D_out);
        if (PD_sum_abs > _kpdmax) {
            const float pd_limit_scale = _kpdmax / PD_sum_abs;
            P_out *= pd_limit_scale;
            D_out *= pd_limit_scale;
            _pid_info.PD_limit = true;
        }
    }

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;
    _pid_info.I = I_out;
    _pid_info.limit = limit;
    // Set I set flag for logging and clear
    _pid_info.I_term_set = _flags._I_set;
    _flags._I_set = false;
    _pid_info.FF = _target * _kff;
    _pid_info.DFF = _target_derivative * _kdff;

    return P_out + D_out + I_out;
}

// Computes the PID output from an error input only (target assumed to be zero).
// Applies error filtering and updates the derivative and integrator.
// Target and measurement must be set separately for logging.
// todo: remove function when it is no longer used.
float AC_PID::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    // Reuse update all code path, zero target and pass negative error as measurement
    // Pass negative error as "measurement" so that error = target - measurement evaluates correctly
    // Bypasses target filtering for legacy compatibility
    _target = 0.0;
    const float output = update_all(0.0, -error, dt, limit);

    // Make sure logged target and actual are still 0 to maintain behaviour
    _pid_info.target = 0.0;
    _pid_info.actual = 0.0;

    return output;
}

// Updates the integrator based on current error and dt.
// If `limit` is true, the integrator is only allowed to shrink to avoid wind-up.
// i_scale can be used to temporarily scale the updated I-term, by default is 1 - should not be set to 0
void AC_PID::update_i(float dt, bool limit, float i_scale)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Allow integrator growth only if not limited, or if error opposes the integrator direction
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * i_scale * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
}

float AC_PID::get_p() const
{
    return _pid_info.P;
}

float AC_PID::get_i() const
{
    return _integrator;
}

float AC_PID::get_d() const
{
    return _pid_info.D;
}

float AC_PID::get_ff() const
{
    return  _pid_info.FF + _pid_info.DFF;
}

float AC_PID::get_ff_component() const
{
    return  _pid_info.FF;
}

float AC_PID::get_dff_component() const
{
    return  _pid_info.DFF;
}

// Used to fully zero the I term between mode changes or initialization
void AC_PID::reset_I()
{
    _flags._I_set = true;
    _integrator = 0.0;
}

// Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// Saves controller configuration from EEPROM, including gains and filter frequencies. Used by autotune to save gains before tuning.
void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

// Returns alpha value for the target low-pass filter (based on filter frequency and dt)
float AC_PID::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// Returns alpha value for the error low-pass filter (based on filter frequency and dt)
float AC_PID::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
float AC_PID::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

// Sets the integrator directly, clamped to the IMAX bounds. Also flags I-term as externally set.
void AC_PID::set_integrator(float integrator)
{
    _flags._I_set = true;
    _integrator = constrain_float(integrator, -_kimax, _kimax);
}

// Gradually adjust the integrator toward a desired value using a time constant.
// Typically used to "relax" the I-term in dynamic conditions.
void AC_PID::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _flags._I_set = true;
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}
