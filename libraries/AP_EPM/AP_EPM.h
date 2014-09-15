// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.h
 *
 *  Created on: DEC 06, 2013
 *      Author: Andreas Jochum
 */

/// @file	AP_EPM.h
/// @brief	AP_EPM control class

#ifndef __AP_EPM_h__
#define __AP_EPM_h__

#include <AP_Math.h>
#include <AP_Common.h>
#include <RC_Channel.h>

// EPM PWM definitions
#define EPM_GRAB_PWM                1900
#define EPM_RELEASE_PWM             1100
#define EPM_NEUTRAL_PWM             1500

// EPM PWM returns to neutral position this many milliseconds after grab or release
#define EPM_RETURN_TO_NEUTRAL_MS     500

/// @class	AP_EPM
/// @brief	Class to manage the EPM_CargoGripper 
class AP_EPM {
public:
    AP_EPM();

    // setup the epm
    void        init();

    // enabled - true if the epm is enabled
    bool        enabled() { return _enabled; }

    // grab - move the epm pwm output to the grab position
    void        grab();

    // release - move the epm pwm output to the release position
    void        release();

    // neutral - return the epm pwm output to the neutral position
    void        neutral();

    // update - moves the pwm back to neutral after the timeout has passed
    // should be called at at least 10hz
    void        update();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // parameters
    AP_Int8     _enabled;

    // internal variables
    uint32_t    _last_grab_or_release;
};

#endif /* _AP_EPM_H_ */
