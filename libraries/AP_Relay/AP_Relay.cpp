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
    // 0 was PIN
    // 1 was PIN2
    // 2 was PIN3
    // 3 was PIN4
    // 4 was DEFAULT
    // 5 was PIN5
    // 6 was PIN6

    // @Group: 1_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 7, AP_Relay, AP_Relay_Params),

    // @Group: 2_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 8, AP_Relay, AP_Relay_Params),

    // @Group: 3_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 9, AP_Relay, AP_Relay_Params),

    // @Group: 4_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 10, AP_Relay, AP_Relay_Params),

    // @Group: 5_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 11, AP_Relay, AP_Relay_Params),

    // @Group: 6_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 12, AP_Relay, AP_Relay_Params),

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

void AP_Relay::convert_params()
{
    // Find old default param
    int8_t default_state = 0; // off was the old behaviour
    const bool have_default = AP_Param::get_param_by_index(this, 4, AP_PARAM_INT8, &default_state);

    // grab the old values if they were set
    for (uint8_t i = 0; i < MIN(AP_RELAY_NUM_RELAYS, 6); i++) {

        uint8_t param_index = i;
        if (i > 3) {
            // Skip over the old DEFAULT parameter at index 4
            param_index += 1;
        }

        int8_t pin = 0;
        if (!_params[i].pin.configured() && AP_Param::get_param_by_index(this, param_index, AP_PARAM_INT8, &pin) && (pin >= 0)) {
            // if the pin isn't negative we can assume the user might have been using it, map it to the old school relay interface
            _params[i].pin.set_and_save(pin);
            _params[i].function.set_and_save(int8_t(AP_Relay_Params::Function::relay));

            if (have_default) {
                _params[i].default_state.set_and_save(default_state);
            }
        }

    }
}

void AP_Relay::set_defaults() {
    const int8_t pins[] = { RELAY1_PIN_DEFAULT,
                             RELAY2_PIN_DEFAULT,
                             RELAY3_PIN_DEFAULT,
                             RELAY4_PIN_DEFAULT,
                             RELAY5_PIN_DEFAULT,
                             RELAY6_PIN_DEFAULT };

    for (uint8_t i = 0; i < MIN(uint8_t(AP_RELAY_NUM_RELAYS), ARRAY_SIZE(pins)); i++) {
        // set the default
        if (pins[i] != -1) {
            _params[i].pin.set_default(pins[i]);
        }
    }
}

void AP_Relay::init()
{
    set_defaults();

    convert_params();

    // setup the actual default values of all the pins
    for (uint8_t instance = 0; instance < AP_RELAY_NUM_RELAYS; instance++) {
        const int8_t pin = _params[instance].pin;
        if (pin == -1) {
            // no valid pin to set it on, skip it
            continue;
        }

        const AP_Relay_Params::Function function = _params[instance].function;
        if (function < AP_Relay_Params::Function::relay || function >= AP_Relay_Params::Function::num_functions) {
            // invalid function, skip it
            continue;
        }

        if (function == AP_Relay_Params::Function::relay) {
            // relay by instance number, set the state to match our output
            const AP_Relay_Params::DefaultState default_state = _params[instance].default_state;
            if ((default_state == AP_Relay_Params::DefaultState::OFF) ||
                (default_state == AP_Relay_Params::DefaultState::ON)) {

                set_pin_by_instance(instance, (bool)default_state);
            }
        } else {
            // all functions are supposed to be off by default
            // this will need revisiting when we support inversion
            set_pin_by_instance(instance, false);
        }
    }
}

void AP_Relay::set(const AP_Relay_Params::Function function, const bool value) {
    if (function <= AP_Relay_Params::Function::none && function >= AP_Relay_Params::Function::num_functions) {
        // invalid function
        return;
    }

    for (uint8_t instance = 0; instance < AP_RELAY_NUM_RELAYS; instance++) {
        if (function != _params[instance].function) {
            continue;
        }

        set_pin_by_instance(instance, value);
    }
}

// set a pins output state by instance and log if required
// this is an internal helper, instance must have already been validated to be in range
void AP_Relay::set_pin_by_instance(uint8_t instance, bool value)
{
    const int8_t pin = _params[instance].pin;
    if (pin == -1) {
        // no valid pin to set it on, skip it
        return;
    }

#if AP_SIM_ENABLED
    if (!(AP::sitl()->on_hardware_relay_enable_mask & (1U << instance))) {
        return;
    }
#endif

    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    const bool initial_value = (bool)hal.gpio->read(pin);

    if (initial_value != value) {
        hal.gpio->write(pin, value);
        AP::logger().Write("RELY", "TimeUS,Instance,State", "s#-", "F--", "QBB",
                            AP_HAL::micros64(),
                            instance,
                            value);
    }
}

void AP_Relay::set(const uint8_t instance, const bool value)
{
    if (instance >= AP_RELAY_NUM_RELAYS) {
        return;
    }

    if (_params[instance].function != AP_Relay_Params::Function::relay) {
        return;
    }

    set_pin_by_instance(instance, value);
}

void AP_Relay::toggle(uint8_t instance)
{
    if (instance < AP_RELAY_NUM_RELAYS) {
        set(instance, !get(instance));
    }
}

// check settings are valid
bool AP_Relay::arming_checks(size_t buflen, char *buffer) const
{
    for (uint8_t i=0; i<AP_RELAY_NUM_RELAYS; i++) {
        const int8_t pin = _params[i].pin.get();
        if (pin != -1 && !hal.gpio->valid_pin(pin)) {
            char param_name_buf[14];
            hal.util->snprintf(param_name_buf, ARRAY_SIZE(param_name_buf), "RELAY%u_PIN", unsigned(i+1));
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

bool AP_Relay::get(uint8_t instance) const
{
    if (instance >= AP_RELAY_NUM_RELAYS) {
        // invalid instance
        return false;
    }

    const int8_t pin = _params[instance].pin.get();

    if (pin < 0) {
        // invalid pin
        return false;
    }

    return (bool)hal.gpio->read(pin);
}

// see if the relay is enabled
bool AP_Relay::enabled(uint8_t instance) const 
{
    // Must be a valid instance with function relay and pin set
    return (instance < AP_RELAY_NUM_RELAYS) && (_params[instance].pin != -1) && (_params[instance].function == AP_Relay_Params::Function::relay);
}

// see if the relay is enabled
bool AP_Relay::enabled(AP_Relay_Params::Function function) const
{
    bool valid = false;
    for (uint8_t instance = 0; instance < AP_RELAY_NUM_RELAYS; instance++) {
        if ((_params[instance].function == function) && (_params[instance].pin != -1)) {
            valid = true;
        }
    }
    return valid;
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

        if (get(i)) {
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
