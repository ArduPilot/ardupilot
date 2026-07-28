#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// externally commanded descent rate override
#ifndef AP_TECS_DESCENT_RATE_ENABLED
#define AP_TECS_DESCENT_RATE_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 2048)
#endif
