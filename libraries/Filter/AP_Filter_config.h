#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_FILTER_NUM_FILTERS
#if BOARD_FLASH_SIZE > 1024
#define AP_FILTER_NUM_FILTERS 8
#else
#define AP_FILTER_NUM_FILTERS 0
#endif
#endif

#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED AP_FILTER_NUM_FILTERS > 0
#endif
