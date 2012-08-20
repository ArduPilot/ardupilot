// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @brief	Imposes limits on location (geofence), altitude and other parameters.
///         Each breach will trigger an action or set of actions to recover. Adapted from geofence.
/// @author Andrew Tridgell
///         Andreas Antonopoulos

#ifndef AP_LIMITS_H
#define AP_LIMITS_H

#ifndef AP_LIMITS
 #define AP_LIMITS ENABLED
#endif

#include <stdint.h>
#include <AP_Param.h>
#include <AP_Limit_Module.h>

// MAVLink messages, trying to pull into library
//#include "../GCS_MAVLink/include/mavlink/v1.0/protocol.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/common/mavlink.h"
//#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink_msg_limits_status.h"



#ifndef HAVE_ENUM_LIMITS_STATE
 #define HAVE_ENUM_LIMITS_STATE
enum LimitState {
    LIMITS_INIT,                        // pre-initialization
    LIMITS_DISABLED,                    // disabled
    LIMITS_ENABLED,                     // checking limits
    LIMITS_TRIGGERED,                   // a limit has been breached
    LIMITS_RECOVERING,                  // taking action, eg. RTL
    LIMITS_RECOVERED,                   // we're no longer in breach of a limit
};
#endif

class AP_Limits {

public:

    AP_Limits();

    // access functions
    bool        enabled();
    bool        debug();
    bool        required();
    AP_Int8        state();
    AP_Int8        safetime();
    AP_Int8        channel();
    AP_Int8        recmode();

    // module linked list management methods
    void                    modules(AP_Limit_Module *m);             // pointer to the first module in linked list of modules
    AP_Limit_Module *       modules_first();
    AP_Limit_Module *       modules_current();
    AP_Limit_Module *       modules_last();
    AP_Limit_Module *       modules_next();
    void                    modules_add(AP_Limit_Module *m);
    uint8_t                 modules_count();

    // main limit methods
    bool                    init();                                     // initialize self and all modules
    void                    set_state(int s);             // change state
    bool                    check_all();                        // check if any limit is triggered
    bool                    check_required();                   // check if any of the required modules is triggered
    bool                    check_triggered(bool required);             // the function that does the checking for the two above

    uint32_t                last_trigger;             // time of last limit breach (trigger)
    uint32_t                last_action;             // time of last recovery action taken
    uint32_t                last_recovery;             // time of last recovery success
    uint32_t                last_status_update;             // time of last recovery success
    uint32_t                last_clear;             // last time all triggers were clear (or 0 if never clear)

    uint16_t                breach_count;             // count of total breaches

    byte                    old_mode_switch;

    LimitModuleBits         mods_enabled;
    LimitModuleBits         mods_required;
    LimitModuleBits         mods_triggered;
    LimitModuleBits         mods_recovering;


    static const struct AP_Param::GroupInfo        var_info[]; // module parameters

protected:
    AP_Int8                 _enabled;             // the entire AP_Limits system on/off switch
    AP_Int8                 _required;             // master switch for pre-arm checks of limits. Will not allow vehicle to "arm" if breaching limits.
    AP_Int8                 _debug;              // enable debug console messages
    AP_Int8                 _safetime;             // how long after recovered before switching to stabilize
    AP_Int8                 _state;             // overall state - used in the main loop state machine inside the vehicle's slow loop.
    AP_Int8                 _channel;             // channel used for switching limits on/off
    AP_Int8                 _recmode;             // recovery mode: 0=RTL mode, 1=bounce mode

private:
    AP_Limit_Module *       _modules_head;      // points to linked list of modules
    AP_Limit_Module *       _modules_current;      // points to linked list of modules
    uint8_t                 _modules_count;

};


#endif // AP_LIMITS_H
