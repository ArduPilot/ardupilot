// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @brief Imposes limits on location (geofence), altitude and other parameters.
/// Each breach will trigger an action or set of actions to recover.
/// dapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef __AP_LIMITS_H__
#define __AP_LIMITS_H__

// XXX this isn't where enabled/disabled goes.
#ifndef AP_LIMITS
 #define AP_LIMITS ENABLED
#endif

#include <stdint.h>
#include <AP_Param.h>
#include "AP_Limit_Module.h"
#include "AP_Limit_Altitude.h"
#include "AP_Limit_Geofence.h"
#include "AP_Limit_GPSLock.h"

#ifndef HAVE_ENUM_LIMITS_STATE
 #define HAVE_ENUM_LIMITS_STATE
enum LimitState {
    LIMITS_INIT,       // pre-initialization
    LIMITS_DISABLED,   // disabled
    LIMITS_ENABLED,    // checking limits
    LIMITS_TRIGGERED,  // a limit has been breached
    LIMITS_RECOVERING, // taking action, eg. RTL
    LIMITS_RECOVERED,  // we're no longer in breach of a limit
};
#endif

class AP_Limits {
public:
    AP_Limits();

    // access functions
    bool        enabled();
    bool        debug();
    bool        required();
    int8_t     state();
    int8_t     safetime();
    int8_t     channel();
    int8_t     recmode();

    // module linked list management methods
    // pointer to the first module in linked list of modules
    void                    modules(AP_Limit_Module *m);
    AP_Limit_Module *       modules_first();
    AP_Limit_Module *       modules_current();
    AP_Limit_Module *       modules_last();
    AP_Limit_Module *       modules_next();
    void                    modules_add(AP_Limit_Module *m);
    uint8_t                 modules_count();

    // main limit methods
    // initialize self and all modules
    bool                    init(); 
    // change state
    void                    set_state(int s);
    // check if any limit is triggered
    bool                    check_all();
    // check if any of the required modules is triggered
    bool                    check_required();
    // the function that does the checking for the two above
    bool                    check_triggered(bool required);

    // time of last limit breach (trigger)
    uint32_t                last_trigger;
    // time of last recovery action taken
    uint32_t                last_action;
    // time of last recovery success
    uint32_t                last_recovery;
    // time of last recovery success
    uint32_t                last_status_update;
    // last time all triggers were clear (or 0 if never clear)
    uint32_t                last_clear;
    // count of total breaches
    uint16_t                breach_count;

    uint8_t                 old_mode_switch;

    LimitModuleBits         mods_enabled;
    LimitModuleBits         mods_required;
    LimitModuleBits         mods_triggered;
    LimitModuleBits         mods_recovering;

    // module parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // the entire AP_Limits system on/off switch
    AP_Int8                 _enabled;
    // master switch for pre-arm checks of limits. Will not allow vehicle to
    // "arm" if breaching limits.
    AP_Int8                 _required;
    // enable debug console messages
    AP_Int8                 _debug;
    // how long after recovered before switching to stabilize
    AP_Int8                 _safetime;
    // overall state - used in the main loop state machine inside the
    // vehicle's slow loop.
    AP_Int8                 _state;
    // channel used for switching limits on/off
    AP_Int8                 _channel;
    // recovery mode: 0=RTL mode, 1=bounce mode
    AP_Int8                 _recmode;

private:
    // points to linked list of modules
    AP_Limit_Module *       _modules_head;
    // points to linked list of modules
    AP_Limit_Module *       _modules_current;
    uint8_t                 _modules_count;

};


#endif // __AP_LIMITS_H__
