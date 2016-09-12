// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <DataFlash/DataFlash.h>
#include <stdlib.h>
#include <cmath>

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:

    PID(const float &   initial_p = 0.0f,
        const float &   initial_i = 0.0f,
        const float &   initial_d = 0.0f,
        const int16_t & initial_imax = 0)
    {
		AP_Param::setup_object_defaults(this, var_info);
        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = initial_imax;

		// set _last_derivative as invalid when we startup
		_last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    float        get_pid(float error, float scaler = 1.0);

    /// Reset the PID integrator
    ///
    void        reset_I();

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = imaxval;
    }

    float        kP() const {
        return _kp.get();
    }
    float        kI() const {
        return _ki.get();
    }
    float        kD() const {
        return _kd.get();
    }
    int16_t        imax() const {
        return _imax.get();
    }

    void        kP(const float v)               {
        _kp.set(v);
    }
    void        kI(const float v)               {
        _ki.set(v);
    }
    void        kD(const float v)               {
        _kd.set(v);
    }
    void        imax(const int16_t v)   {
        _imax.set(abs(v));
    }

    float        get_integrator() const {
        return _integrator;
    }

    static const struct AP_Param::GroupInfo        var_info[];

    const DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }

private:
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Int16        _imax;

    float           _integrator;///< integrator value
    float           _last_error;///< last error for derivative
    float           _last_derivative;///< last derivative for low-pass filter
    uint32_t        _last_t;///< last time get_pid() was called in millis

    float           _get_pid(float error, uint16_t dt, float scaler);

    DataFlash_Class::PID_Info _pid_info {};

    /// Low pass filter cut frequency for derivative calculation.
    ///
    /// 20 Hz because anything over that is probably noise, see
    /// http://en.wikipedia.org/wiki/Low-pass_filter.
    ///
    static const uint8_t        _fCut = 20;
};
