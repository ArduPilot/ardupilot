// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_PID_H__
#define __AC_PID_H__

#include <AP_Common.h>
#include <AP_Param.h>
#include <stdlib.h>
#include <math.h>               // for fabs()

// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373
#define AC_PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency

/// @class	AC_PID
/// @brief	Object managing one PID control
class AC_PID {
public:

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note	PIDs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    AC_PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0.0):
        _integrator(0),
        _last_input(0),
        _last_derivative(0),
        _d_lpf_alpha(AC_PID_D_TERM_FILTER)
    {
		AP_Param::setup_object_defaults(this, var_info);

        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = abs(initial_imax);

		// derivative is invalid on startup
		_last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    float       get_pid(float error, float dt);
    float       get_pi(float error, float dt);
    float       get_p(float error) const;
    float       get_i(float error, float dt);
    float       get_d(float error, float dt);

    /// Reset the PID integrator
    ///
    void        reset_I();

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();
    
    /// Sets filter Alpha for D-term LPF
    void        set_d_lpf_alpha(int16_t cutoff_frequency, float time_step);

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
    }

    // accessors
    float       kP() const { return _kp.get(); }
    float       kI() const { return _ki.get(); }
    float       kD() const { return _kd.get(); }
    int16_t     imax() const { return _imax.get(); }
    void        kP(const float v) { _kp.set(v); }
    void        kI(const float v) { _ki.set(v); }
    void        kD(const float v) { _kd.set(v); }
    void        imax(const int16_t v) { _imax.set(abs(v)); }
    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }

    static const struct AP_Param::GroupInfo        var_info[];

protected:
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Int16        _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
};

#endif // __AC_PID_H__
