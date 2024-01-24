#pragma once

#include <AP_Param/AP_Param.h>

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
        ICE_STARTER = 9,
        DroneCAN_HARDPOINT_0  = 10,
        DroneCAN_HARDPOINT_1  = 11,
        DroneCAN_HARDPOINT_2  = 12,
        DroneCAN_HARDPOINT_3  = 13,
        DroneCAN_HARDPOINT_4  = 14,
        DroneCAN_HARDPOINT_5  = 15,
        DroneCAN_HARDPOINT_6  = 16,
        DroneCAN_HARDPOINT_7  = 17,
        DroneCAN_HARDPOINT_8  = 18,
        DroneCAN_HARDPOINT_9  = 19,
        DroneCAN_HARDPOINT_10 = 20,
        DroneCAN_HARDPOINT_11 = 21,
        DroneCAN_HARDPOINT_12 = 22,
        DroneCAN_HARDPOINT_13 = 23,
        DroneCAN_HARDPOINT_14 = 24,
        DroneCAN_HARDPOINT_15 = 25,
        NUM_FUNCTIONS // must be the last entry
    };

    // Pins that do not go via GPIO
    enum class VIRTUAL_PINS {
        DroneCAN_0  = 1000,
        DroneCAN_1  = 1001,
        DroneCAN_2  = 1002,
        DroneCAN_3  = 1003,
        DroneCAN_4  = 1004,
        DroneCAN_5  = 1005,
        DroneCAN_6  = 1006,
        DroneCAN_7  = 1007,
        DroneCAN_8  = 1008,
        DroneCAN_9  = 1009,
        DroneCAN_10 = 1010,
        DroneCAN_11 = 1011,
        DroneCAN_12 = 1012,
        DroneCAN_13 = 1013,
        DroneCAN_14 = 1014,
        DroneCAN_15 = 1015,
    };

    AP_Enum<FUNCTION> function;            // relay function
    AP_Int16 pin;                          // gpio pin number
    AP_Enum<DefaultState> default_state;  // default state
};
