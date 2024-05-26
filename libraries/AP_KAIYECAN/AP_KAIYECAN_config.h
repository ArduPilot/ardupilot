
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_KAIYECAN_ENABLED
#define AP_KAIYECAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && BOARD_FLASH_SIZE > 1024)
#endif

