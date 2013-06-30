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

#include <AP_Param.h>

/// @class	AP_Relay
/// @brief	Class to manage the APM relay
class AP_Relay {
public:
    AP_Relay();

    // setup the relay pin
    void        init();

    // activate the relay
    void        on();

    // de-activate the relay
    void        off();

    // toggle the relay status
    void        toggle();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Int8 _pin;
};

#endif /* AP_RELAY_H_ */
