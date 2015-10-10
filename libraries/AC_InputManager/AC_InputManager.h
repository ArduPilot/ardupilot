// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AC_InputManager.h
/// @brief  Pilot manual control input library

#ifndef AC_INPUTMANAGER_H
#define AC_INPUTMANAGER_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/// @class  AC_InputManager
/// @brief  Class managing the pilot's control inputs
class AC_InputManager{
public:
    AC_InputManager(uint16_t loop_rate):
        _loop_rate(loop_rate)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // internal variables
    uint16_t            _loop_rate;             // rate at which output() function is called (normally 400hz)

};

#endif /* AC_INPUTMANAGER_H */
