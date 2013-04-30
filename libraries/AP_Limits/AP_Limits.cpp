// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/// @file	ap_limits.cpp
/// @brief	Imposes limits on location (geofence), altitude and other parameters
/// Each breach will trigger an action or set of actions to recover.
/// Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#include <AP_Limits.h>
#include <AP_Limit_Module.h>

const AP_Param::GroupInfo AP_Limits::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Enable Limits Library
    // @Description: Setting this to Enabled(1) will enable the limits system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED",         0,      AP_Limits,      _enabled, 0),

    // @Param: REQUIRED
    // @DisplayName: Limits Library Required
    // @Description: Setting this to 1 will enable the limits pre-arm checklist
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("REQUIRED",        1,      AP_Limits,      _required, 0),

    // @Param: DEBUG
    // @DisplayName: Enable Limits Debug
    // @Description: Setting this to 1 will turn on debugging messages on the console and via MAVLink STATUSTEXT messages
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DEBUG",           2,      AP_Limits,      _debug,    0),

    // @Param: SAFETIME
    // @DisplayName: Limits Safetime
    // @Description: Automatic return of controls to pilot. Set to 0 to disable (full RTL) or a number of seconds after complete recovery to return the controls to the pilot in STABILIZE
    // @Values: 0:Disabled,1-255:Seconds before returning control
    // @User: Standard
    AP_GROUPINFO("SAFETIME",        3,      AP_Limits,      _safetime, 0),

    // @Param: CHANNEL
    // @DisplayName: Limits Channel
    // @Description: Channel for Limits on/off control. If channel exceeds LIMITS_ENABLE_PWM, it turns limits on, and vice-versa.
    // @Range: 1-8
    // @User: Standard
    AP_GROUPINFO("CHANNEL",         4,      AP_Limits,      _channel,  0),

    // @Param: RECMODE
    // @DisplayName: Limits Recovery Mode
    // @Description: Select how Limits should "recover". Set to 0 for RTL-like mode, where the vehicle navigates back to home until it is "safe". Set to 1, for bounce-mode, where the vehicle will hold position when it hits a limit. RTL mode is better for large fenced areas, Bounce mode for smaller spaces. Note: RTL mode will cause the vehicle to yaw 180 degrees  (turn around) to navigate towards home when it hits a limit.
    // @Values: 0:RTL mode, 1: Bounce mode
    // @User: Standard
    AP_GROUPINFO("RECMODE",         5,      AP_Limits,      _recmode,  0),

    AP_GROUPEND
};

AP_Limits::AP_Limits() {
    AP_Param::setup_object_defaults(this, var_info);
    _state = LIMITS_INIT;
}

void AP_Limits::modules(AP_Limit_Module *m)
{
    _modules_head = m;
}

bool AP_Limits::init() {
    AP_Limit_Module *m = modules_first();

    while (m) {
        m->init();
        m = modules_next();
    }
    return true;
}


bool AP_Limits::enabled() {
    return _enabled;
}

bool AP_Limits::debug() {
    return _debug;
}


AP_Limit_Module *AP_Limits::modules_first() {
     // reset current to head of list
    _modules_current = _modules_head;
    return _modules_head;
}

AP_Limit_Module *AP_Limits::modules_current() {
    return _modules_current;
}

AP_Limit_Module *AP_Limits::modules_next() {
    if (_modules_current) {
         // move to "next" (which might be NULL)
        _modules_current = _modules_current->next();
    }
    return _modules_current;
}

uint8_t AP_Limits::modules_count() {
    _modules_count = 0;
    AP_Limit_Module *m = _modules_head;

    while (m) {
        _modules_count++;
        m = m->next();
    }
    return _modules_count;
}

AP_Limit_Module *AP_Limits::modules_last() {
    AP_Limit_Module *m = _modules_head;
    while (m && m->next()) {
        m = m->next();
    }
    return m;
}

void AP_Limits::modules_add(AP_Limit_Module *m) {
    if (_modules_head) {
        // if there is a module linked add to the end of the list
        modules_last()->link(m);
    } else {
        // otherwise, this will be the "head"
        _modules_head = m;
    }
}

bool AP_Limits::required() {
    return _required;
}

bool AP_Limits::check_all() {
    // required=false, means "all"
    return check_triggered(false);
}

bool AP_Limits::check_required() {
    // required=true, means "only required modules"
    return check_triggered(true);
}

bool AP_Limits::check_triggered(bool only_required) {

    // check all enabled modules for triggered limits
    AP_Limit_Module *mod = _modules_head;

    // reset bit fields
    mods_triggered = 0;
    mods_enabled = 0;
    mods_required = 0;

    while (mod) {

        unsigned module_id = mod->get_module_id();

        // We check enabled, triggered and required across all modules
        // We set all the bit-fields each time we check, keeping them up to date

        if (mod->enabled()) {
            mods_enabled |= module_id;

            if (mod->required()) mods_required |= module_id;
            if (mod->triggered()) mods_triggered |= module_id;
        }

        mod = mod->next();
    }

    if (only_required) {
        // just modules that are both required AND triggered. (binary AND)
        return (mods_required & mods_triggered) != 0;
    } else {
        return mods_triggered != 0;
    }
}

int8_t AP_Limits::state() {
    return _state.get();
}

int8_t AP_Limits::safetime() {
    return _safetime.get();
}

void AP_Limits::set_state(int s) {
    _state.set(s);
}


int8_t AP_Limits::channel() {
    return _channel.get();
}

int8_t AP_Limits::recmode() {
    return _recmode.get();
}
