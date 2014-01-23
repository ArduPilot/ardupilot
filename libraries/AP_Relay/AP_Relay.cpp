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
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define RELAY_PIN 111
#else
#define RELAY_PIN 54
#endif
#else
// no relay for this board
#define RELAY_PIN -1
#endif

const AP_Param::GroupInfo AP_Relay::var_info[] PROGMEM = {
    // @Param: PIN
    // @DisplayName: First Relay Pin
    // @Description: Digital pin number for first relay control. This is the pin used for camera control.
    // @User: Standard
    // @Values: 13:APM2 A9 pin,47:APM1 relay,54:Pixhawk FMU AUX1,55:Pixhawk FMU AUX2,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin[0], RELAY_PIN),

    // @Param: PIN2
    // @DisplayName: Second Relay Pin
    // @Description: Digital pin number for 2nd relay control.
    // @User: Standard
    // @Values: 13:APM2 A9 pin,47:APM1 relay,54:Pixhawk FMU AUX1,55:Pixhawk FMU AUX2,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN2",  1, AP_Relay, _pin[1], -1),

    // @Param: PIN3
    // @DisplayName: Third Relay Pin
    // @Description: Digital pin number for 3rd relay control.
    // @User: Standard
    // @Values: 13:APM2 A9 pin,47:APM1 relay,54:Pixhawk FMU AUX1,55:Pixhawk FMU AUX2,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN3",  2, AP_Relay, _pin[2], -1),

    // @Param: PIN4
    // @DisplayName: Fourth Relay Pin
    // @Description: Digital pin number for 4th relay control.
    // @User: Standard
    // @Values: 13:APM2 A9 pin,47:APM1 relay,54:Pixhawk FMU AUX1,55:Pixhawk FMU AUX2,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN4",  3, AP_Relay, _pin[3], -1),

    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;

AP_Relay::AP_Relay(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Relay::init() 
{
    for (uint8_t i=0; i<AP_RELAY_NUM_RELAYS; i++) {
        if (_pin[i].get() != -1) {
            off(i);
        }
    }
}

void AP_Relay::on(uint8_t relay) 
{    
    if (relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1) {
        hal.gpio->pinMode(_pin[relay], GPIO_OUTPUT);
        hal.gpio->write(_pin[relay], 1);
    }
}


void AP_Relay::off(uint8_t relay) 
{
    if (relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1) {
        hal.gpio->pinMode(_pin[relay], GPIO_OUTPUT);
        hal.gpio->write(_pin[relay], 0);
    }
}


void AP_Relay::toggle(uint8_t relay) 
{
    if (relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1) {
        bool ison = hal.gpio->read(_pin[relay]);
        if (ison)
            off(relay);
        else
            on(relay);
    }
}

