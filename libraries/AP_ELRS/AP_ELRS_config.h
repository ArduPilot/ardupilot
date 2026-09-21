#pragma once

#ifdef CONFIG_HAL_BOARD
#include <AP_HAL/AP_HAL_Boards.h>
#endif

#ifndef AP_ELRS_ENABLED
#if defined(AP_PERIPH_ELRS_ENABLED) && AP_PERIPH_ELRS_ENABLED
#define AP_ELRS_ENABLED 1
#else
#define AP_ELRS_ENABLED 0
#endif
#endif
