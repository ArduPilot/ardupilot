#pragma once

/// @file	AC_HELI_PID.h
/// @brief	Helicopter Specific Rate PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AC_PID.h"

static const float AC_PID_LEAK_MIN = 0.1f;  // Default I-term Leak Minimum

/// @class	AC_HELI_PID
/// @brief	Heli PID control class
class AC_HELI_PID : public AC_PID {
public:

    /// Constructor for PID
    AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz);

    CLASS_NO_COPY(AC_HELI_PID);

    /// update_leaky_i - replacement for get_i but output is leaked at leak_rate
    void       update_leaky_i(float leak_rate);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _leak_min;

    float           _last_requested_rate;       // Requested rate from last iteration, used to calculate rate change of requested rate
};
