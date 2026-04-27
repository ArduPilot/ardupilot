#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_IBUS2_ENABLED
#define AP_IBUS2_ENABLED 1
#endif

#ifndef AP_IBUS2_MASTER_ENABLED
#define AP_IBUS2_MASTER_ENABLED AP_IBUS2_ENABLED
#endif

#ifndef AP_IBUS2_SLAVE_ENABLED
#define AP_IBUS2_SLAVE_ENABLED AP_IBUS2_ENABLED
#endif

// Maximum RC channels the slave will decode and expose to the RC subsystem.
// Must not exceed MAX_RCIN_CHANNELS (18) defined in AP_RCProtocol/AP_RCProtocol.h.
// Kept here to avoid a circular include: AP_RCProtocol → AP_RCProtocol_IBUS2 → AP_IBus2_Slave.
#define AP_IBUS2_MAX_CHANNELS 18
