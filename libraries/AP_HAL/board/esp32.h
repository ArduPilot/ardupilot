#pragma once

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

#ifndef CONFIG_HAL_BOARD_SUBTYPE
// allow for generic boards
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_GENERIC
#endif

#define HAL_STORAGE_SIZE (128*1024)
#define HAL_BARO_ALLOW_INIT_NO_BARO

#define HAL_WITH_UAVCAN 0

#define HAL_HAVE_SAFETY_SWITCH 0

#define HAL_HAVE_BOARD_VOLTAGE 0

#define HAL_HAVE_SERVO_VOLTAGE 0

// allow for static semaphores
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_Semaphore_Recursive ESP32::Semaphore_Recursive
