#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Enabled 0 is compiled out (disabled)
// Enabled 1 is always enabled on all vehicles
// Enabled 2 is enabled with dummy methods for all vehicles except Sub and SITL

#ifndef AP_TEMPERATURE_SENSOR_ENABLED
#if HAL_MINIMIZE_FEATURES || BOARD_FLASH_SIZE <= 1024
    #define AP_TEMPERATURE_SENSOR_ENABLED 0
#elif (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
    #define AP_TEMPERATURE_SENSOR_ENABLED 1
#else
    #define AP_TEMPERATURE_SENSOR_ENABLED 2
#endif
#endif

#ifndef AP_TEMPERATURE_SENSOR_MAX31865_ENABLED
    #define AP_TEMPERATURE_SENSOR_MAX31865_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif



// maximum number of Temperature Sensors
#ifndef AP_TEMPERATURE_SENSOR_MAX_INSTANCES
#define AP_TEMPERATURE_SENSOR_MAX_INSTANCES             3
#endif

// first sensor is always the primary sensor
#ifndef AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE
#define AP_TEMPERATURE_SENSOR_PRIMARY_INSTANCE          0
#endif

