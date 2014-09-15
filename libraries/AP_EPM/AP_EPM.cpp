// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.cpp
 *
 *  Created on: DEC 6, 2013
 *      Author: Andreas Jochum
 */

#include <AP_EPM.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_EPM::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: EPM Enable/Disable
    // @Description: EPM enable/disable.  Note enabling will disable the external LEDs on the APM2
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO("ENABLE",  0, AP_EPM, _enabled, 0),

    AP_GROUPEND
};

AP_EPM::AP_EPM(void) :
        _last_grab_or_release(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_EPM::init() 
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // initialise the EPM to the neutral position
    neutral();
}

// grab - move the epm pwm output to the grab position
void AP_EPM::grab()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // capture time
    _last_grab_or_release = hal.scheduler->millis();

    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, EPM_GRAB_PWM);
}

// release - move the epm pwm output to the release position
void AP_EPM::release()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // capture time
    _last_grab_or_release = hal.scheduler->millis();

    // move the servo to the release position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, EPM_RELEASE_PWM);
}

// neutral - return the epm pwm output to the neutral position
void AP_EPM::neutral() 
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // move the servo to the off position
    RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, EPM_NEUTRAL_PWM);
}

// update - moves the pwm back to neutral after the timeout has passed
// should be called at at least 10hz
void AP_EPM::update()
{
    // move EPM PWM output back to neutral 2 seconds after the last grab or release
    if ((_last_grab_or_release != 0) && (hal.scheduler->millis() - _last_grab_or_release > EPM_RETURN_TO_NEUTRAL_MS)) {
        neutral();
    }
}

