#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_ORCAMOTOR_ENABLED
#define HAL_ORCAMOTOR_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif