#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED BOARD_FLASH_SIZE > 1024
#endif
