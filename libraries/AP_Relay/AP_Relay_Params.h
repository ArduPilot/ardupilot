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

    enum class FUNCTION : uint8_t {
        NONE     = 0,
        RELAY    = 1,
        IGNITION = 2,
        PARACHUTE = 3,
        CAMERA = 4,
        BRUSHED_REVERSE_1 = 5,
        BRUSHED_REVERSE_2 = 6,
        BRUSHED_REVERSE_3 = 7,
        BRUSHED_REVERSE_4 = 8,
        NUM_FUNCTIONS // must be the last entry
    };

    AP_Enum<FUNCTION> function;            // relay function
    AP_Int16 pin;                          // gpio pin number
    AP_Enum<DefaultState> default_state;  // default state
};
