#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

// Needs SerialManager + (AHRS or GPS)
#ifndef HAL_NMEA_OUTPUT_ENABLED
#define HAL_NMEA_OUTPUT_ENABLED BOARD_FLASH_SIZE>1024 && HAL_GCS_ENABLED
#endif
