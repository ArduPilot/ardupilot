/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include "AP_Relay_config.h"

#if AP_RELAY_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Relay/AP_Relay_Params.h>

#ifndef AP_RELAY_NUM_RELAYS
  #define AP_RELAY_NUM_RELAYS 6
#endif

#if AP_RELAY_NUM_RELAYS < 1
  #error There must be at least one relay instance if using AP_Relay
#endif

#if AP_RELAY_DRONECAN_ENABLED
#include <AP_DroneCAN/AP_DroneCAN.h>
#endif

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
#if AP_RELAY_DRONECAN_ENABLED
    // Allow DroneCAN to directly access private DroneCAN state
    friend class AP_DroneCAN;
#endif
public:
    AP_Relay();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Relay);

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(uint8_t instance) { set(instance, true); }

    // de-activate the relay
    void        off(uint8_t instance) { set(instance, false); }

    // get state of relay
    bool        get(uint8_t instance) const;
    
    // see if the relay is enabled
    bool        enabled(uint8_t instance) const;

    // toggle the relay status
    void        toggle(uint8_t instance);

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

    bool send_relay_status(const class GCS_MAVLINK &link) const;

    // Set the state of all relays that are configured as the specified function type
    void set(AP_Relay_Params::FUNCTION function, bool value);

    // see if the relay is enabled
    bool enabled(AP_Relay_Params::FUNCTION function) const;

    // Get the pin number of the instance (if assigned to a pin)
    bool get_pin_by_instance(uint8_t instance, uint8_t &pin) const;

private:
    static AP_Relay *singleton;

    AP_Relay_Params _params[AP_RELAY_NUM_RELAYS];

    // Return true if function is valid
    bool function_valid(AP_Relay_Params::FUNCTION function) const;

    // Set the state of the specified instance, if it is a valid relay
    void set(uint8_t instance, bool value);

    void set_defaults();
    void convert_params();

    // Internal helper: set the instance state, accounting for configured inversion
    void set_instance_state(uint8_t instance, bool value);

    // Internal helper: directly set the state of the specified pin
    void set_pin_state(const int16_t pin, const bool value);

    // Get the state of the specified pin
    bool get_pin_state(const int16_t pin) const;

#if AP_RELAY_DRONECAN_ENABLED
    // Virtual DroneCAN pins
    class DroneCAN {
    public:
        // Return true if pin number is a virtual DroneCAN pin
        bool valid_pin(int16_t pin) const;

        // Enable streaming of pin number
        void enable_pin(int16_t pin);

        // Populate message and update index with the sent command
        bool populate_next_command(uint8_t &index, uavcan_equipment_hardpoint_Command &msg) const;

        // Set DroneCAN relay state from pin number
        void set_pin_state(const int16_t pin, const bool value);

        // Get relay state from pin number
        bool get_pin_state(const int16_t pin) const;

    private:

        // Get the hardpoint index of given pin number
        uint8_t hardpoint_index(const int16_t pin) const;

        // Send DroneCAN hardpoint message for given index on all interfaces
        void send_index(const uint8_t index);

        static constexpr uint8_t num_pins = (int16_t)AP_Relay_Params::VIRTUAL_PINS::DroneCAN_15 - (int16_t)AP_Relay_Params::VIRTUAL_PINS::DroneCAN_0;

        struct {
            bool value;
            bool enabled;
        } state[num_pins];

    } dronecan;
#endif // AP_RELAY_DRONECAN_ENABLED

};

namespace AP {
    AP_Relay *relay();
};

#endif  // AP_RELAY_ENABLED
