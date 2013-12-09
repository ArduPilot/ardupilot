// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_HAL.h>
#include "AP_Relay.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define RELAY_PIN 47
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define RELAY_PIN 13
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define RELAY_PIN 111
#else
// no relay for this board
#define RELAY_PIN -1
#endif

const AP_Param::GroupInfo AP_Relay::var_info[] PROGMEM = {
    // @Param: PIN
    // @DisplayName: Relay Pin
    // @Description: Digital pin number for relay control.
    // @User: Standard
    // @Values: 13:APM2 A9 pin,47:APM1 relay,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin, RELAY_PIN),

    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;

AP_Relay::AP_Relay(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Relay::init() 
{
    if (_pin != -1) {
        hal.gpio->pinMode(_pin, GPIO_OUTPUT);
        off();
    }
}

void AP_Relay::on() 
{    
    if (_pin != -1) {
        hal.gpio->write(_pin, 1);
    }
}


void AP_Relay::off() 
{
    if (_pin != -1) {
        hal.gpio->write(_pin, 0);
    }
}


void AP_Relay::toggle() 
{
    if (_pin != -1) {
        bool ison = hal.gpio->read(_pin);
        if (ison)
            off();
        else
            on();
    }
}

