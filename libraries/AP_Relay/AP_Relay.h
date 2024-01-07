/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include "AP_Relay_config.h"

#if AP_RELAY_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay_Params.h>
#include <AP_Common/Bitmask.h>

#ifndef AP_RELAY_NUM_RELAYS
  #define AP_RELAY_NUM_RELAYS 6
#endif

#if AP_RELAY_NUM_RELAYS < 1
  #error There must be at least one relay instance if using AP_Relay
#endif

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    AP_Relay();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Relay);

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(uint8_t instance) { set(instance, true); }

    // de-activate the relay
    void        off(uint8_t instance) { set(instance, false); }

    // get state of relay
    bool        get(uint8_t instance) const;
    
    // see if the relay is enabled
    bool        enabled(uint8_t instance) const;

    // toggle the relay status
    void        toggle(uint8_t instance);

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

    bool send_relay_status(const class GCS_MAVLINK &link) const;

    void set(AP_Relay_Params::FUNCTION function, bool value);

    // see if the relay is enabled
    bool enabled(AP_Relay_Params::FUNCTION function) const;

private:
    static AP_Relay *singleton;

    AP_Relay_Params _params[AP_RELAY_NUM_RELAYS];

    void set(uint8_t instance, bool value);

    void set_defaults();
    void convert_params();

    void set_pin_by_instance(uint8_t instance, bool value);
};

namespace AP {
    AP_Relay *relay();
};

#endif  // AP_RELAY_ENABLED
