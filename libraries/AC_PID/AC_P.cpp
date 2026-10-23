/// @file	AC_P.cpp
/// @brief	Single-axis P controller with EEPROM-backed gain storage.

#include <AP_Math/AP_Math.h>
#include "AC_P.h"

const AP_Param::GroupInfo AC_P::var_info[] = {
    // @Param: P
    // @DisplayName: P Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_P, _kp, default_kp),
    AP_GROUPEND
};

float AC_P::get_p(float error) const
{
    return (float)error * _kp;
}

// Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
void AC_P::load_gains()
{
    _kp.load();
}

// Saves controller configuration from EEPROM. Used by autotune to save gains before tuning.
void AC_P::save_gains()
{
    _kp.save();
}
