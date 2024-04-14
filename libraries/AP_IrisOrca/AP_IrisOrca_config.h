#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_IRISORCA_ENABLED
#define HAL_IRISORCA_ENABLED BOARD_FLASH_SIZE > 1024
#endif
