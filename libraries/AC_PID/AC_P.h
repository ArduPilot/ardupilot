#pragma once

/// @file	AC_PD.h
/// @brief	Single-axis P controller with EEPROM-backed gain storage.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

/// @class	AC_P
/// @brief	Object managing one P controller
class AC_P {
public:

    /// Constructor for P controller with EEPROM-backed gain.
    /// Parameters are initialized from defaults or EEPROM at runtime.
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    ///
    AC_P(const float &initial_p = 0.0f) :
        default_kp(initial_p)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    CLASS_NO_COPY(AC_P);

    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @returns		The updated control output.
    ///
    float       get_p(float error) const;

    // Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
    void        load_gains();

    // Saves controller configuration from EEPROM. Used by autotune to save gains before tuning.
    void        save_gains();

    /// @name	parameter accessors
    //@{

    // accessors
    AP_Float    &kP() { return _kp; }
    const AP_Float &kP() const { return _kp; }
    void        set_kP(const float v) { _kp.set(v); }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _kp;

    const float default_kp;
};
