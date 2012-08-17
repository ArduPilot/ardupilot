// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef AC_PID_h
#define AC_PID_h

#include <AP_Common.h>
#include <math.h>               // for fabs()

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
        const int16_t & initial_imax = 0.0)
    {
        _kp = initial_p;
        _ki = initial_i;
        _kd = initial_d;
        _imax = abs(initial_imax);
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
    int32_t         get_pid(int32_t error, float dt);
    int32_t         get_pi(int32_t error, float dt);
    int32_t         get_p(int32_t error);
    int32_t         get_i(int32_t error, float dt);
    int32_t         get_d(int32_t error, float dt);


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
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
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
    void        set_integrator(float i) {
        _integrator = i;
    }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Int16        _imax;

    float           _integrator;                                ///< integrator value
    int32_t         _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _output;
    float           _derivative;

    /// Low pass filter cut frequency for derivative calculation.
    ///
    static const float        _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3
};

#endif
