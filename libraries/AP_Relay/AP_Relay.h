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

#define AP_RELAY_NUM_RELAYS 4

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    static AP_Relay create() { return AP_Relay{}; }

    constexpr AP_Relay(AP_Relay &&other) = default;

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

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Relay();

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;
};
