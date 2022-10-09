/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include <AP_Param/AP_Param.h>

#define AP_RELAY_NUM_RELAYS 6

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    AP_Relay();

    /* Do not allow copies */
    AP_Relay(const AP_Relay &other) = delete;
    AP_Relay &operator=(const AP_Relay&) = delete;

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(uint8_t instance) { set(instance, true); }

    // de-activate the relay
    void        off(uint8_t instance) { set(instance, false); }

    // get state of relay
    uint8_t     get(uint8_t instance) const {
        return instance < AP_RELAY_NUM_RELAYS ? _pin_states & (1U<<instance) : 0;
    }
    
    // see if the relay is enabled
    bool        enabled(uint8_t instance) { return instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1; }

    // toggle the relay status
    void        toggle(uint8_t instance);

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_Relay *singleton;

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;
    uint8_t _pin_states;
    uint8_t _last_logged_pin_states;
    uint32_t _last_log_ms;

    void set(uint8_t instance, bool value);
};

namespace AP {
    AP_Relay *relay();
};
