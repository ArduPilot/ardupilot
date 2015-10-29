// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class

#ifndef __AP_RELAY_H__
#define __AP_RELAY_H__

#include <AP_Param/AP_Param.h>

#define AP_RELAY_NUM_RELAYS 4

/// @class	AP_Relay
/// @brief	Class to manage the APM relay
class AP_Relay {
public:
    AP_Relay();

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
    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;
};

#endif /* AP_RELAY_H_ */
