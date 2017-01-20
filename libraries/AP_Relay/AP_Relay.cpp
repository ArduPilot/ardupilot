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
#define RELAY2_PIN_DEFAULT -1
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define RELAY1_PIN_DEFAULT 111
#define RELAY2_PIN_DEFAULT -1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define RELAY1_PIN_DEFAULT -1
#define RELAY2_PIN_DEFAULT -1
#else
#define RELAY1_PIN_DEFAULT 54
#define RELAY2_PIN_DEFAULT 55
#endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#define RELAY1_PIN_DEFAULT 33
#define RELAY2_PIN_DEFAULT -1
#else
// no relay for this board
#define RELAY1_PIN_DEFAULT -1
#define RELAY2_PIN_DEFAULT -1
#endif

const AP_Param::GroupInfo AP_Relay::var_info[] = {
    // @Param: PIN
    // @DisplayName: First Relay Pin
    // @Description: Digital pin number for first relay control. This is the pin used for camera control.
    // @User: Standard
    // @Values: -1:Disabled,13:APM2 A9 pin,47:APM1 relay,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin[0], RELAY1_PIN_DEFAULT),

    // @Param: PIN2
    // @DisplayName: Second Relay Pin
    // @Description: Digital pin number for 2nd relay control.
    // @User: Standard
    // @Values: -1:Disabled,13:APM2 A9 pin,47:APM1 relay,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN2",  1, AP_Relay, _pin[1], RELAY2_PIN_DEFAULT),

    // @Param: PIN3
    // @DisplayName: Third Relay Pin
    // @Description: Digital pin number for 3rd relay control.
    // @User: Standard
    // @Values: -1:Disabled,13:APM2 A9 pin,47:APM1 relay,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN3",  2, AP_Relay, _pin[2], -1),

    // @Param: PIN4
    // @DisplayName: Fourth Relay Pin
    // @Description: Digital pin number for 4th relay control.
    // @User: Standard
    // @Values: -1:Disabled,13:APM2 A9 pin,47:APM1 relay,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    AP_GROUPINFO("PIN4",  3, AP_Relay, _pin[3], -1),

    // @Param: DEFAULT
    // @DisplayName: Default relay state
    // @Description: The state of the relay on boot. 
    // @User: Standard
    // @Values: 0:Off,1:On,2:NoChange
    AP_GROUPINFO("DEFAULT",  4, AP_Relay, _default, 0),

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

