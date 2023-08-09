#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// UDP only on SITL for now
#ifndef AP_DDS_UDP_ENABLED
#define AP_DDS_UDP_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
