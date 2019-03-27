#pragma once

/// @file   AC_InputManager.h
/// @brief  Pilot manual control input library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/// @class  AC_InputManager
/// @brief  Class managing the pilot's control inputs
class AC_InputManager{
public:
    AC_InputManager() {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AC_InputManager(const AC_InputManager &other) = delete;
    AC_InputManager &operator=(const AC_InputManager&) = delete;

    static const struct AP_Param::GroupInfo        var_info[];
    void set_loop_rate(uint16_t loop_rate) { _loop_rate = loop_rate; }

protected:
    // internal variables
    uint16_t            _loop_rate;             // rate at which output() function is called (normally 400hz)

};
