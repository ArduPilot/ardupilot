// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef AP_PID_h
#define AP_PID_h

#include <AP_Common.h>
#include <math.h>               // for fabs()

/// @class	AP_PID
/// @brief	Object managing one PID control
class AP_PID {
public:

    AP_PID();

    long        get_pid(int32_t error, uint16_t dt, float scaler = 1.0);

    /// Reset the PID integrator
    ///
    void        reset_I();

    void        kP(const float v)               {
        _kp = v;
    }
    void        kI(const float v)               {
        _ki = v;
    }
    void        kD(const float v)               {
        _kd = v;
    }
    void        imax(const int16_t v)   {
        _imax = v;
    }

    float           kP()                    {
        return _kp;
    }
    float           kI()                    {
        return _ki;
    }
    float           kD()                    {
        return _kd;
    }
    float           imax()                  {
        return _imax;
    }

    float        get_integrator() const {
        return _integrator;
    }

private:
    float           _kp;
    float           _ki;
    float           _kd;
    float           _imax;

    float           _integrator;                                ///< integrator value
    int32_t         _last_error;                                ///< last error for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter

    /// Low pass filter cut frequency for derivative calculation.
    ///
    /// 20 Hz becasue anything over that is probably noise, see
    /// http://en.wikipedia.org/wiki/Low-pass_filter.
    ///
    static const uint8_t        _fCut = 20;
};

#endif
