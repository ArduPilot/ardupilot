#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_VISUALODOM_ENABLED
#define HAL_VISUALODOM_ENABLED BOARD_FLASH_SIZE > 1024
#endif
