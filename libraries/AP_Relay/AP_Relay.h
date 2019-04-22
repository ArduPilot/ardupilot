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
    void        on(uint8_t relay);

    // de-activate the relay
    void        off(uint8_t relay);

    // see if the relay is enabled
    bool        enabled(uint8_t relay) { return relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1; }

    // toggle the relay status
    void        toggle(uint8_t relay);

    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_Relay *singleton;

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;
};
