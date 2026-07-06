#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// IBus2 needs the AP_IBus2 library, which only full vehicle firmwares
// link; gate on program size like other larger optional features so
// IOMCU/peripheral/small-board builds exclude the RC backend too.
#ifndef AP_IBUS2_ENABLED
#define AP_IBUS2_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#ifndef AP_IBUS2_MASTER_ENABLED
#define AP_IBUS2_MASTER_ENABLED AP_IBUS2_ENABLED
#endif

#ifndef AP_IBUS2_SLAVE_ENABLED
#define AP_IBUS2_SLAVE_ENABLED AP_IBUS2_ENABLED
#endif

// Maximum RC channels the slave will decode and expose to the RC subsystem.
// Must not exceed MAX_RCIN_CHANNELS (18) defined in AP_RCProtocol/AP_RCProtocol.h.
// Kept here to avoid a circular include: AP_RCProtocol → AP_RCProtocol_IBus2 → AP_IBus2_Slave.
#define AP_IBUS2_MAX_CHANNELS 18

#ifndef AP_IBUS2_MAX_MASTER_INSTANCES
#define AP_IBUS2_MAX_MASTER_INSTANCES 4
#endif
