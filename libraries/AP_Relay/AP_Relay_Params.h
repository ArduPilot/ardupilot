#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_Relay_config.h"

class AP_Relay_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Relay_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Relay_Params);

    enum class DefaultState : uint8_t {
        OFF = 0,
        ON = 1,
        NO_CHANGE = 2,
    };

    enum class Function : uint8_t {
        none     = 0,
        relay    = 1,
        num_functions // must be the last entry
    };

    AP_Enum<Function> function;            // relay function
    AP_Int16 pin;                          // gpio pin number
    AP_Enum<DefaultState> default_state;  // default state
};
