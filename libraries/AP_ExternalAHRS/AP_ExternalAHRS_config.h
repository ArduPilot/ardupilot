#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_EXTERNAL_AHRS_ENABLED
<<<<<<< Updated upstream
#define HAL_EXTERNAL_AHRS_ENABLED BOARD_FLASH_SIZE > 1024
=======
#define HAL_EXTERNAL_AHRS_ENABLED !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
>>>>>>> Stashed changes
#endif
