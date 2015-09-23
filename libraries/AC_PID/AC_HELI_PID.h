// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_HELI_PID.h
/// @brief	Helicopter Specific Rate PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_HELI_PID_H__
#define __AC_HELI_PID_H__

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <math.h>
#include "AC_PID.h"

#define AC_PID_LEAK_MIN     500.0  // Default I-term Leak Minimum

/// @class	AC_HELI_PID
/// @brief	Heli PID control class
class AC_HELI_PID : public AC_PID {
public:

    /// Constructor for PID
    AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff);

    /// get_vff - return Velocity FeedForward Term 
    float       get_vff(float requested_rate);

    /// get_avff - return Acceleration FeedForward Term 
    float       get_aff(float requested_rate);
    
    /// get_leaky_i - replacement for get_i but output is leaded at leak_rate
    float       get_leaky_i(float leak_rate);
    
    // accessors
    float       ff() const { return _vff.get(); }
    void        ff(const float v) { _vff.set(v); }

    static const struct AP_Param::GroupInfo        var_info[];
    
private:
    AP_Float        _vff;
    AP_Float        _leak_min;
    AP_Float        _aff;

    float           _last_requested_rate;       // Requested rate from last iteration, used to calculate rate change of requested rate
    
};

#endif // __AC_HELI_PID_H__
