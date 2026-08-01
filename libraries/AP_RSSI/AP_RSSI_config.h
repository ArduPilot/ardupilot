#pragma once

// board-specific values for this subsystem are generated into a hwdef
// fragment so they only enter the include closure of code using them
#if __has_include(<hwdef_rssi.h>)
#include <hwdef_rssi.h>
#endif

#ifndef BOARD_RSSI_ANA_PIN
#define BOARD_RSSI_ANA_PIN -1
#endif

#ifndef BOARD_RSSI_ANA_PIN_HIGH
#define BOARD_RSSI_ANA_PIN_HIGH 5.0f
#endif


#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_RSSI_ENABLED
#define AP_RSSI_ENABLED 1
#endif
