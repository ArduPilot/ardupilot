// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AC_InputManager_Heli.h
/// @brief  Pilot manual control input library for Conventional Helicopter

#ifndef AC_INPUTMANAGER_HELI_H
#define AC_INPUTMANAGER_HELI_H

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include "AC_InputManager.h"

# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_MIN_DEFAULT     0
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_LOW_DEFAULT     400
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_HIGH_DEFAULT    600
# define AC_ATTITUDE_HELI_STAB_COLLECTIVE_MAX_DEFAULT     1000

/// @class  AP_InputManager_Heli
/// @brief  Class managing the pilot's control inputs   for Conventional Helicopter
class AC_InputManager_Heli : public AC_InputManager {
public:
    // Constructor
    AC_InputManager_Heli(uint16_t loop_rate):
        AC_InputManager(loop_rate)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    // get_pilot_desired_collective - rescale's pilot collective pitch input in Stabilize and Acro modes
    int16_t get_pilot_desired_collective(int16_t control_in);

    // set_use_stab_col - setter function
    void set_use_stab_col(bool use) { _im_flags_heli.use_stab_col = use; }

    // set_heli_stab_col_ramp - setter function
    void set_stab_col_ramp(float ramp) { _stab_col_ramp = constrain_float(ramp, 0.0, 1.0); }

    static const struct AP_Param::GroupInfo        var_info[];

private:

    struct InputManagerHeliFlags {
        uint8_t use_stab_col        :   1;  // 1 if we should use Stabilise mode collective range, 0 for Acro range
    } _im_flags_heli;

    //  factor used to smoothly ramp collective from Acro value to Stab-Col value
    float _stab_col_ramp = 0;

    AP_Int16        _heli_stab_col_min;             // minimum collective pitch setting at zero throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_low;             // collective pitch setting at mid-low throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_high;            // collective pitch setting at mid-high throttle input in Stabilize mode
    AP_Int16        _heli_stab_col_max;             // maximum collective pitch setting at full throttle input in Stabilize mode
    AP_Float        _acro_col_expo;                 // used to soften collective pitch inputs near center point in Acro mode

};

#endif /* AC_INPUTMANAGER_HELI_H */