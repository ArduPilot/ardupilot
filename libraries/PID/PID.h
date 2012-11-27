// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef PID_h
#define PID_h

#include <AP_Common.h>
#include <math.h>               // for fabs()

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:

    PID(const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0)
    {
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
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    int32_t        get_pid(int32_t error, float scaler = 1.0);

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

private:
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Int16        _imax;

    float           _integrator;                                ///< integrator value
    int32_t         _last_error;                                ///< last error for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    uint32_t        _last_t;                                    ///< last time get_pid() was called in millis

    int32_t         _get_pid(int32_t error, uint16_t dt, float scaler);

    /// Low pass filter cut frequency for derivative calculation.
    ///
    /// 20 Hz becasue anything over that is probably noise, see
    /// http://en.wikipedia.org/wiki/Low-pass_filter.
    ///
    static const uint8_t        _fCut = 20;
};

#endif
