// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.cpp
 *
 *  Created on: DEC 6, 2013
 *      Author: Andreas Jochum
 */

#include <AP_HAL.h>
#include "AP_EPM.h"

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

AP_EPM::AP_EPM(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_EPM::init() 
{
    // return immediately if not enabled
    if (!_enabled || !EPM_SUPPORTED) {
        return;
    }

    hal.gpio->pinMode(EPM_PIN_1, GPIO_OUTPUT);
    hal.gpio->pinMode(EPM_PIN_2, GPIO_OUTPUT);

    neutral();
}

bool AP_EPM::enabled()
{
     if (!EPM_SUPPORTED) {
        return false;
     }

     return _enabled;
}

void AP_EPM::on() 
{
    // return immediately if not enabled
    if (!_enabled || !EPM_SUPPORTED) {
        return;
    }

    hal.gpio->write(EPM_PIN_1, 1);
    hal.gpio->write(EPM_PIN_2, 0);
}

void AP_EPM::off() 
{
    // return immediately if not enabled
    if (!_enabled || !EPM_SUPPORTED) {
        return;
    }

    hal.gpio->write(EPM_PIN_1, 0);
    hal.gpio->write(EPM_PIN_2, 1);
}

void AP_EPM::neutral() 
{
    // return immediately if not enabled
    if (!_enabled || !EPM_SUPPORTED) {
        return;
    }

    hal.gpio->write(EPM_PIN_1, 0);
    hal.gpio->write(EPM_PIN_2, 0);
}
