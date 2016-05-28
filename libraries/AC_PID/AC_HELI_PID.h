// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_HELI_PID.h
/// @brief	Helicopter Specific Rate PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_HELI_PID_H__
#define __AC_HELI_PID_H__

#include <AP_Common.h>
#include <AP_Param.h>
#include <stdlib.h>
#include <math.h>
#include <AC_PID.h>

// Examples for _filter:
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
#define AC_HELI_PID_D_TERM_FILTER 0.00795770f    // 20hz filter on D term

/// @class	AC_HELI_PID
/// @brief	Object managing one PID control
class AC_HELI_PID : public AC_PID {
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
    AC_HELI_PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0.0,
        const float &   initial_ff = 0.0) :
        AC_PID(initial_p, initial_i, initial_d, initial_imax)
    {
		 _ff = initial_ff;
    }
    
    /// get_ff - return FeedForward Term 
    float       get_ff(float requested_rate) const;
    
    /// get_leaky_i - replacement for get_i but output is leaded at leak_rate
    float       get_leaky_i(float error, float dt, float leak_rate);
    
    // accessors
    float       ff() const { return _ff.get(); }
    void        ff(const float v) { _ff.set(v); }

    static const struct AP_Param::GroupInfo        var_info[];
    
private:
    AP_Float        _ff;
    
};

#endif // __AC_HELI_PID_H__
