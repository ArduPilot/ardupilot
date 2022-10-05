#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_COMPASS_DIAGONALS_ENABLED
#define AP_COMPASS_DIAGONALS_ENABLED (!defined(HAL_BUILD_AP_PERIPH))
#endif
