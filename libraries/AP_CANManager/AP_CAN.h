#pragma once

/*
 * this header contains data common to ArduPilot CAN, rather than to a
 * specific implementation of the protocols.  So we try to share
 * enumeration values where possible to make parameters similar across
 * Periph and main firmwares, for example.
 *
 * this is *not* to be a one-stop-shop for including all things CAN...
 */

#include <stdint.h>

class AP_CAN {
public:
    enum class Protocol : uint8_t {
        None = 0,
        DroneCAN = 1,
        // 2 was KDECAN -- do not re-use
        // 3 was ToshibaCAN -- do not re-use
        PiccoloCAN = 4,
        // 5 was CANTester
        EFI_NWPMU = 6,
        USD1 = 7,
        KDECAN = 8,
        // 9 was MPPT_PacketDigital
        Scripting = 10,
        Benewake = 11,
        Scripting2 = 12,
    };
};
