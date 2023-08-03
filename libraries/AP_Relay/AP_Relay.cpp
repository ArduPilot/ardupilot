/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include "AP_Relay_config.h"

#if AP_RELAY_ENABLED

#include "AP_Relay.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define RELAY1_PIN_DEFAULT 13

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    #define RELAY1_PIN_DEFAULT 57
    #define RELAY2_PIN_DEFAULT 49
    #define RELAY3_PIN_DEFAULT 116
    #define RELAY4_PIN_DEFAULT 113
  #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    #define RELAY1_PIN_DEFAULT 27
    #define RELAY2_PIN_DEFAULT 65
    #define RELAY3_PIN_DEFAULT 22
    #define RELAY4_PIN_DEFAULT 81
    #define RELAY5_PIN_DEFAULT 23
    #define RELAY6_PIN_DEFAULT 26
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
    // @Description: Digital pin number for first relay control. This is the pin used for camera shutter control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,27:BBBMini Pin P8.17,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin[0], RELAY1_PIN_DEFAULT),

    // @Param: PIN2
    // @DisplayName: Second Relay Pin
    // @Description: Digital pin number for 2nd relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,65:BBBMini Pin P8.18,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN2",  1, AP_Relay, _pin[1], RELAY2_PIN_DEFAULT),

    // @Param: PIN3
    // @DisplayName: Third Relay Pin
    // @Description: Digital pin number for 3rd relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,22:BBBMini Pin P8.19,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN3",  2, AP_Relay, _pin[2], RELAY3_PIN_DEFAULT),

    // @Param: PIN4
    // @DisplayName: Fourth Relay Pin
    // @Description: Digital pin number for 4th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,63:BBBMini Pin P8.34,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN4",  3, AP_Relay, _pin[3], RELAY4_PIN_DEFAULT),

    // @Param: DEFAULT
    // @DisplayName: Default relay state
    // @Description: The state of the relay on boot.
    // @User: Standard
    // @Values: 0:Off,1:On,2:NoChange
    AP_GROUPINFO("DEFAULT",  4, AP_Relay, _default, 0),

    // @Param: PIN5
    // @DisplayName: Fifth Relay Pin
    // @Description: Digital pin number for 5th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,62:BBBMini Pin P8.13,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN5",  5, AP_Relay, _pin[4], RELAY5_PIN_DEFAULT),

    // @Param: PIN6
    // @DisplayName: Sixth Relay Pin
    // @Description: Digital pin number for 6th relay control. Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,37:BBBMini Pin P8.14,101:MainOut1,102:MainOut2,103:MainOut3,104:MainOut4,105:MainOut5,106:MainOut6,107:MainOut7,108:MainOut8
    AP_GROUPINFO("PIN6",  6, AP_Relay, _pin[5], RELAY6_PIN_DEFAULT),

    AP_GROUPEND
};

AP_Relay *AP_Relay::singleton;

extern const AP_HAL::HAL& hal;

AP_Relay::AP_Relay(void)
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Relay must be singleton");
    }
#endif
    singleton = this;
}


void AP_Relay::init()
{
    if (_default != 0 && _default != 1) {
        return;
    }
    for (uint8_t i=0; i<AP_RELAY_NUM_RELAYS; i++) {
        set(i, _default);
    }
}

void AP_Relay::set(const uint8_t instance, const bool value)
{
    if (instance >= AP_RELAY_NUM_RELAYS) {
        return;
    }
    if (_pin[instance] == -1) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    _pin_states = value ? _pin_states | (1U<<instance) : _pin_states & ~(1U<<instance);
    if ((_pin_states != _last_logged_pin_states) && (now - _last_log_ms > 10)) {
       AP::logger().Write("RELY", "TimeUS,State", "s-", "F-", "QB",
                                        AP_HAL::micros64(),
                                        _pin_states);
       _last_log_ms = now;
       _last_logged_pin_states = _pin_states;
    }
#if AP_SIM_ENABLED && (CONFIG_HAL_BOARD != HAL_BOARD_SITL)
    return;
#endif
    hal.gpio->pinMode(_pin[instance], HAL_GPIO_OUTPUT);
    hal.gpio->write(_pin[instance], value);
}

void AP_Relay::toggle(uint8_t instance)
{
    if (instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1) {
        bool ison = hal.gpio->read(_pin[instance]);
        set(instance, !ison);
    }
}

// check settings are valid
bool AP_Relay::arming_checks(size_t buflen, char *buffer) const
{
    for (uint8_t i=0; i<AP_RELAY_NUM_RELAYS; i++) {
        int8_t pin = _pin[i].get();
        if (pin != -1 && !hal.gpio->valid_pin(pin)) {
            char param_name_buf[11] = "RELAY_PIN";
            if (i > 0) {
                hal.util->snprintf(param_name_buf, ARRAY_SIZE(param_name_buf), "RELAY_PIN%u", unsigned(i+1));
            }
            uint8_t servo_ch;
            if (hal.gpio->pin_to_servo_channel(pin, servo_ch)) {
                hal.util->snprintf(buffer, buflen, "%s=%d, set SERVO%u_FUNCTION=-1", param_name_buf, int(pin), unsigned(servo_ch+1));
            } else {
                hal.util->snprintf(buffer, buflen, "%s=%d invalid", param_name_buf, int(pin));
            }
            return false;
        }
    }
    return true;
}


#if AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
// this method may only return false if there is no space in the
// supplied link for the message.
bool AP_Relay::send_relay_status(const GCS_MAVLINK &link) const
{
    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), RELAY_STATUS)) {
        return false;
    }

    uint16_t present_mask = 0;
    uint16_t on_mask = 0;
    for (auto i=0; i<AP_RELAY_NUM_RELAYS; i++) {
        if (!enabled(i)) {
            continue;
        }
        const uint16_t relay_bit_mask = 1U << i;
        present_mask |= relay_bit_mask;

        if (_pin_states & relay_bit_mask) {
            on_mask |= relay_bit_mask;
        }
    }

    mavlink_msg_relay_status_send(
        link.get_chan(),
        AP_HAL::millis(),
        on_mask,
        present_mask
        );
    return true;
}
#endif  // AP_MAVLINK_MSG_RELAY_STATUS_ENABLED

namespace AP {

AP_Relay *relay()
{
    return AP_Relay::get_singleton();
}

}

#endif  // AP_RELAY_ENABLED
