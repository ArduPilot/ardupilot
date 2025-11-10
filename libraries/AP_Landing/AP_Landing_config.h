#pragma once

#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
#define HAL_LANDING_DEEPSTALL_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
