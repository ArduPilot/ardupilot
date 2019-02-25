/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Relay.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define RELAY1_PIN_DEFAULT 13

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    #define RELAY1_PIN_DEFAULT 57
    #define RELAY2_PIN_DEFAULT 49
    #define RELAY3_PIN_DEFAULT 116
    #define RELAY4_PIN_DEFAULT 113
  #endif
#endif

#ifndef RELAY1_PIN_DEFAULT
  #define RELAY1_PIN_DEFAULT -1
#endif

#ifndef RELAY2_PIN_DEFAULT
  #define RELAY2_PIN_DEFAULT -1
#endif

#ifndef RELAY3_PIN_DEFAULT
  #define RELAY3_PIN_DEFAULT -1
#endif

#ifndef RELAY4_PIN_DEFAULT
  #define RELAY4_PIN_DEFAULT -1
#endif

#ifndef RELAY5_PIN_DEFAULT
  #define RELAY5_PIN_DEFAULT -1
#endif

#ifndef RELAY6_PIN_DEFAULT
  #define RELAY6_PIN_DEFAULT -1
#endif


const AP_Param::GroupInfo AP_Relay::var_info[] = {
    // @Param: PIN
    // @DisplayName: First Relay Pin
    // @Description: Digital pin number for first relay control. This is the pin used for camera control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin[0], RELAY1_PIN_DEFAULT),

    // @Param: PIN2
    // @DisplayName: Second Relay Pin
    // @Description: Digital pin number for 2nd relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN2",  1, AP_Relay, _pin[1], RELAY2_PIN_DEFAULT),

    // @Param: PIN3
    // @DisplayName: Third Relay Pin
    // @Description: Digital pin number for 3rd relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN3",  2, AP_Relay, _pin[2], RELAY3_PIN_DEFAULT),

    // @Param: PIN4
    // @DisplayName: Fourth Relay Pin
    // @Description: Digital pin number for 4th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN4",  3, AP_Relay, _pin[3], RELAY4_PIN_DEFAULT),

    // @Param: DEFAULT
    // @DisplayName: Default relay state
    // @Description: The state of the relay on boot.
    // @User: Standard
    // @Values: 0:Off,1:On,2:NoChange
    AP_GROUPINFO("DEFAULT",  4, AP_Relay, _default, 0),

    // @Param: PIN5
    // @DisplayName: Fifth Relay Pin
    // @Description: Digital pin number for 5th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN5",  5, AP_Relay, _pin[4], RELAY5_PIN_DEFAULT),

    // @Param: PIN6
    // @DisplayName: Sixth Relay Pin
    // @Description: Digital pin number for 6th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,115:PX4IO ACC1.116:BB Blue GP0 pin 5
    AP_GROUPINFO("PIN6",  6, AP_Relay, _pin[5], RELAY6_PIN_DEFAULT),

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
            switch (_default) {
            case 0:
                off(i);
                break;
            case 1:
                on(i);
                break;
            default:
                break;
            }
        }
    }
}

void AP_Relay::on(uint8_t relay) 
{    
    if (relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1) {
        hal.gpio->pinMode(_pin[relay], HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin[relay], 1);
    }
}


void AP_Relay::off(uint8_t relay) 
{
    if (relay < AP_RELAY_NUM_RELAYS && _pin[relay] != -1) {
        hal.gpio->pinMode(_pin[relay], HAL_GPIO_OUTPUT);
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

