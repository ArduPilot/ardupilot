#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_EXTERNAL_AHRS_ENABLED
#define HAL_EXTERNAL_AHRS_ENABLED BOARD_FLASH_SIZE > 1024
#endif
