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
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Camera/AP_Camera.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AR_Motors/AP_MotorsUGV.h>
#endif

#if AP_RELAY_DRONECAN_ENABLED
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_CANManager/AP_CANManager.h>
#endif

#if AP_SIM_ENABLED
#include <SITL/SITL.h>
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

#if AP_RELAY_NUM_RELAYS > 1
    // @Group: 2_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 8, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 2
    // @Group: 3_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 9, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 3
    // @Group: 4_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 10, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 4
    // @Group: 5_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 11, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 5
    // @Group: 6_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 12, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 6
    // @Group: 7_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 13, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 7
    // @Group: 8_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 14, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 8
    // @Group: 9_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 15, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 9
    // @Group: 10_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[9], "10_", 16, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 10
    // @Group: 11_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[10], "11_", 17, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 11
    // @Group: 12_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[11], "12_", 18, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 12
    // @Group: 13_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[12], "13_", 19, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 13
    // @Group: 14_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[13], "14_", 20, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 14
    // @Group: 15_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[14], "15_", 21, AP_Relay, AP_Relay_Params),
#endif

#if AP_RELAY_NUM_RELAYS > 15
    // @Group: 16_
    // @Path: AP_Relay_Params.cpp
    AP_SUBGROUPINFO(_params[15], "16_", 22, AP_Relay, AP_Relay_Params),
#endif

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
    // PARAMETER_CONVERSION - Added: Dec-2023
#ifndef HAL_BUILD_AP_PERIPH
    // Dont need this conversion on periph as relay support is more recent

    // Before converting local params we must find any relays being used by index from external libs
    int8_t ice_relay = -1;
#if AP_ICENGINE_ENABLED
    AP_ICEngine *ice = AP::ice();
    int8_t ice_relay_index;
    if (ice != nullptr && ice->get_legacy_ignition_relay_index(ice_relay_index)) {
        ice_relay = ice_relay_index;
    }
#endif

    int8_t chute_relay = -1;
#if HAL_PARACHUTE_ENABLED
    AP_Parachute *parachute = AP::parachute();
    int8_t parachute_relay_index;
    if (parachute != nullptr && parachute->get_legacy_relay_index(parachute_relay_index)) {
        chute_relay = parachute_relay_index;
    }
#endif

    int8_t cam_relay = -1;
#if AP_CAMERA_ENABLED
    AP_Camera *camera = AP::camera();
    int8_t camera_relay_index;
    if ((camera != nullptr) && (camera->get_legacy_relay_index(camera_relay_index))) {
        cam_relay = camera_relay_index;
    }
#endif

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    int8_t rover_relay[] = { -1, -1, -1, -1 };
    AP_MotorsUGV *motors = AP::motors_ugv();
    if (motors != nullptr) {
        motors->get_legacy_relay_index(rover_relay[0], rover_relay[1], rover_relay[2], rover_relay[3]);
    }
#endif

    // Find old default param
    AP_Relay_Params::DefaultState default_state = AP_Relay_Params::DefaultState::OFF; // off was the old behaviour
    const bool have_default = AP_Param::get_param_by_index(this, 4, AP_PARAM_INT8, &default_state);

    // grab the old values if they were set
    for (uint8_t i = 0; i < MIN(ARRAY_SIZE(_params), 6U); i++) {
        if (_params[i].function.configured()) {
            // Conversion already done, or user has configured manually
            continue;
        }

        uint8_t param_index = i;
        if (i > 3) {
            // Skip over the old DEFAULT parameter at index 4
            param_index += 1;
        }

        int8_t pin = 0;
        if (!_params[i].pin.configured() && AP_Param::get_param_by_index(this, param_index, AP_PARAM_INT8, &pin) && (pin >= 0)) {
            // Copy old pin parameter if valid
            _params[i].pin.set_and_save(pin);
        }

        // Work out what function this relay should be
        AP_Relay_Params::FUNCTION new_fun;
        if (i == chute_relay) {
            new_fun = AP_Relay_Params::FUNCTION::PARACHUTE;

        } else if (i == ice_relay) {
            new_fun = AP_Relay_Params::FUNCTION::IGNITION;

        } else if (i == cam_relay) {
            new_fun = AP_Relay_Params::FUNCTION::CAMERA;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
        } else if (i == rover_relay[0]) {
            new_fun = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1;

        } else if (i == rover_relay[1]) {
            new_fun = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2;

        } else if (i == rover_relay[2]) {
            new_fun = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_3;

        } else if (i == rover_relay[3]) {
            new_fun = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_4;
#endif

        } else {
            if (_params[i].pin < 0) {
                // Don't enable as numbered relay if pin is invalid
                // Other functions should be enabled with a invalid pin
                // This will result in a pre-arm promoting the user to resolve
                continue;
            }
            new_fun = AP_Relay_Params::FUNCTION::RELAY;

        }
        _params[i].function.set_and_save(new_fun);


        // Set the default state
        if (have_default) {
            _params[i].default_state.set_and_save(default_state);
        }
    }
#endif // HAL_BUILD_AP_PERIPH
}

void AP_Relay::set_defaults() {
    const int8_t pins[] = { RELAY1_PIN_DEFAULT,
                             RELAY2_PIN_DEFAULT,
                             RELAY3_PIN_DEFAULT,
                             RELAY4_PIN_DEFAULT,
                             RELAY5_PIN_DEFAULT,
                             RELAY6_PIN_DEFAULT };

    for (uint8_t i = 0; i < MIN(ARRAY_SIZE(_params), ARRAY_SIZE(pins)); i++) {
        // set the default
        if (pins[i] != -1) {
            _params[i].pin.set_default(pins[i]);
        }
    }
}

// Return true is function is valid
bool AP_Relay::function_valid(AP_Relay_Params::FUNCTION function) const
{
    return (function > AP_Relay_Params::FUNCTION::NONE) && (function < AP_Relay_Params::FUNCTION::NUM_FUNCTIONS);
}

void AP_Relay::init()
{
    set_defaults();

    convert_params();

    // setup the actual default values of all the pins
    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        const int16_t pin = _params[instance].pin;
        if (pin == -1) {
            // no valid pin to set it on, skip it
            continue;
        }

        const AP_Relay_Params::FUNCTION function = _params[instance].function;
        if (!function_valid(function)) {
            // invalid function, skip it
            continue;
        }

        bool use_default_param = (function == AP_Relay_Params::FUNCTION::RELAY);
#ifdef HAL_BUILD_AP_PERIPH
        use_default_param |= (function >= AP_Relay_Params::FUNCTION::DroneCAN_HARDPOINT_0 && function <= AP_Relay_Params::FUNCTION::DroneCAN_HARDPOINT_15);
#endif
        if (use_default_param) {
            // relay by instance number, set the state to match our output
            const AP_Relay_Params::DefaultState default_state = _params[instance].default_state;
            if ((default_state == AP_Relay_Params::DefaultState::OFF) ||
                (default_state == AP_Relay_Params::DefaultState::ON)) {

                set_instance_state(instance, (bool)default_state);
            }
        } else {
            // all functions are supposed to be off by default
            set_instance_state(instance, false);
        }

        // Make sure any DroneCAN pin is enabled for streaming
#if AP_RELAY_DRONECAN_ENABLED
        dronecan.enable_pin(pin);
#endif

    }
}

void AP_Relay::set(const AP_Relay_Params::FUNCTION function, const bool value) {
    if (!function_valid(function)) {
        // invalid function
        return;
    }

    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        if (function != _params[instance].function) {
            continue;
        }

        set_instance_state(instance, value);
    }
}

// set a pins output state by instance and log if required
// this is an internal helper, instance must have already been validated to be in range
void AP_Relay::set_instance_state(uint8_t instance, bool value)
{
    const int16_t pin = _params[instance].pin;
    if (pin == -1) {
        // no valid pin to set it on, skip it
        return;
    }

#if AP_SIM_ENABLED
    if (!(AP::sitl()->on_hardware_relay_enable_mask & (1U << instance))) {
        return;
    }
#endif

    const bool initial_value = get_pin_state(pin);

    if (_params[instance].inverted > 0) {
        value = !value;
    }

    if (initial_value != value) {
        set_pin_state(pin, value);
#if HAL_LOGGING_ENABLED
// @LoggerMessage: RELY
// @Description: Relay state
// @Field: TimeUS: Time since system startup
// @Field: Instance: relay instance number
// @Field: State: current state
        AP::logger().Write(
            "RELY",
            "TimeUS," "Instance," "State",
            "s"       "#"         "-",
            "F"       "-"         "-",
            "Q"       "B"         "B",
            AP_HAL::micros64(),
            instance,
            value);
#endif  // HAL_LOGGING_ENABLED
    }
}

void AP_Relay::set(const uint8_t instance, const bool value)
{
    if (instance >= ARRAY_SIZE(_params)) {
        return;
    }

    if (_params[instance].function != AP_Relay_Params::FUNCTION::RELAY) {
        return;
    }

    set_instance_state(instance, value);
}

void AP_Relay::toggle(uint8_t instance)
{
    if (instance < ARRAY_SIZE(_params)) {
        set(instance, !get(instance));
    }
}

// check settings are valid
bool AP_Relay::arming_checks(size_t buflen, char *buffer) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(_params); i++) {
        if (!function_valid(_params[i].function)) {
            // Relay disabled
            continue;
        }

        const int16_t pin = _params[i].pin.get();
        if (pin == -1) {
            // Pin disabled, may want to pre-arm this in the future as function enabled with invalid pin
            // User should set function to none to disable
            continue;
        }

#if AP_RELAY_DRONECAN_ENABLED
        const bool DroneCAN_pin = dronecan.valid_pin(pin);
#else
        const bool DroneCAN_pin = false;
#endif

        if (!DroneCAN_pin && !hal.gpio->valid_pin(pin)) {
            // Check GPIO pin is valid
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

        // Each pin can only be used by a single relay
        for (uint8_t j=i+1; j<ARRAY_SIZE(_params); j++) {
            if (!function_valid((AP_Relay_Params::FUNCTION)_params[j].function.get())) {
                // Relay disabled
                continue;
            }
            if (pin == _params[j].pin.get()) {
                hal.util->snprintf(buffer, buflen, "pin conflict RELAY%u_PIN = RELAY%u_PIN", int(i+1), int(j+1));
                return false;
            }
        }
    }
    return true;
}

bool AP_Relay::get(uint8_t instance) const
{
    if (instance >= ARRAY_SIZE(_params)) {
        // invalid instance
        return false;
    }

    if (_params[instance].inverted > 0) {
        return !get_pin_state(_params[instance].pin.get());
    }

    return get_pin_state(_params[instance].pin.get());
}

// Get relay state from pin number
bool AP_Relay::get_pin_state(const int16_t pin) const
{
    if (pin < 0) {
        // invalid pin
        return false;
    }

#if AP_RELAY_DRONECAN_ENABLED
    if (dronecan.valid_pin(pin)) {
        // Virtual DroneCAN pin
        return dronecan.get_pin_state(pin);
    }
#endif

    // Real GPIO pin
    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    return (bool)hal.gpio->read(pin);
}

// Set relay state from pin number
void AP_Relay::set_pin_state(const int16_t pin, const bool value)
{
    if (pin < 0) {
        // invalid pin
        return;
    }

#if AP_RELAY_DRONECAN_ENABLED
    if (dronecan.valid_pin(pin)) {
        // Virtual DroneCAN pin
        dronecan.set_pin_state(pin, value);
        return;
    }
#endif

    // Real GPIO pin
    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(pin, value);
}

// Get GPIO pin from instance
bool AP_Relay::get_pin_by_instance(uint8_t instance, uint8_t &pin) const
{
    if (instance >= ARRAY_SIZE(_params)) {
        return false;
    }
    pin = _params[instance].pin;
    return true;
}
// see if the relay is enabled
bool AP_Relay::enabled(uint8_t instance) const 
{
    // Must be a valid instance with function relay and pin set
    return (instance < ARRAY_SIZE(_params)) && (_params[instance].pin != -1) && (_params[instance].function == AP_Relay_Params::FUNCTION::RELAY);
}

// see if the relay is enabled
bool AP_Relay::enabled(AP_Relay_Params::FUNCTION function) const
{
    for (uint8_t instance = 0; instance < ARRAY_SIZE(_params); instance++) {
        if ((_params[instance].function == function) && (_params[instance].pin != -1)) {
            return true;
        }
    }
    return false;
}

#if AP_RELAY_DRONECAN_ENABLED
// Return true if pin number is a virtual DroneCAN pin
bool AP_Relay::DroneCAN::valid_pin(int16_t pin) const
{
    switch(pin) {
        case (int16_t)AP_Relay_Params::VIRTUAL_PINS::DroneCAN_0 ... (int16_t)AP_Relay_Params::VIRTUAL_PINS::DroneCAN_15:
            return true;
        default:
            return false;
    }
}

// Enable streaming of pin number
void AP_Relay::DroneCAN::enable_pin(int16_t pin)
{
    if (!valid_pin(pin)) {
        return;
    }

    const uint8_t index = hardpoint_index(pin);
    state[index].enabled = true;
}

// Get the hardpoint index of given pin number
uint8_t AP_Relay::DroneCAN::hardpoint_index(const int16_t pin) const
{
    return pin - (int16_t)AP_Relay_Params::VIRTUAL_PINS::DroneCAN_0;
}

// Set DroneCAN relay state from pin number
void AP_Relay::DroneCAN::set_pin_state(const int16_t pin, const bool value)
{
    const uint8_t index = hardpoint_index(pin);

    // Set pin and ensure enabled for streaming
    state[index].enabled = true;
    state[index].value = value;

    // Broadcast msg on all channels
    // Just a single send, rely on streaming to fill in any lost packet

    uavcan_equipment_hardpoint_Command msg {};
    msg.hardpoint_id = index;
    msg.command = state[index].value;

    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto *ap_dronecan = AP_DroneCAN::get_dronecan(i);
        if (ap_dronecan != nullptr) {
            ap_dronecan->relay_hardpoint.broadcast(msg);
        }
    }

}

// Get relay state from pin number, this relies on a cached value, assume remote pin is in sync
bool AP_Relay::DroneCAN::get_pin_state(const int16_t pin) const
{
    const uint8_t index = hardpoint_index(pin);
    return state[index].value;
}

// Populate message and update index with the sent command
// This will allow the caller to cycle through each enabled pin
bool AP_Relay::DroneCAN::populate_next_command(uint8_t &index, uavcan_equipment_hardpoint_Command &msg) const
{
    // Find the next enabled index
    for (uint8_t i = 0; i < ARRAY_SIZE(state); i++) {
        // Look for next index, wrapping back to 0 as needed
        const uint8_t new_index = (index + 1 + i) % ARRAY_SIZE(state);
        if (!state[new_index].enabled) {
            // This index is not being used
            continue;
        }

        // Update command and index then return
        msg.hardpoint_id = new_index;
        msg.command = state[new_index].value;
        index = new_index;
        return true;
    }

    return false;
}
#endif // AP_RELAY_DRONECAN_ENABLED

#if AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
// this method may only return false if there is no space in the
// supplied link for the message.
bool AP_Relay::send_relay_status(const GCS_MAVLINK &link) const
{
    static_assert(AP_RELAY_NUM_RELAYS <= 16, "Too many relays for MAVLink status reporting to work.");

    if (!HAVE_PAYLOAD_SPACE(link.get_chan(), RELAY_STATUS)) {
        return false;
    }

    uint16_t present_mask = 0;
    uint16_t on_mask = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(_params); i++) {
        if (!function_valid(_params[i].function)) {
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
