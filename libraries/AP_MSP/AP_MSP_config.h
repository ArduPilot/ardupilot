#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 1
#endif

// define for enabling MSP sensor drivers
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED HAL_MSP_ENABLED
#endif

// define for enabling MSP DisplayPort
#ifndef HAL_WITH_MSP_DISPLAYPORT
#define HAL_WITH_MSP_DISPLAYPORT HAL_MSP_ENABLED
#endif
