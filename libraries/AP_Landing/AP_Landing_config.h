#pragma once

#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
#define HAL_LANDING_DEEPSTALL_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif
