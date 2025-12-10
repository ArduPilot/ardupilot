#pragma once

#define HAL_BOARD_NAME "EMBOX"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_NONE
#define HAL_BARO_DEFAULT HAL_BARO_NONE

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 1

#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_BOARD_STORAGE_DIRECTORY "."

#define HAL_NUM_CAN_IFACES 0
// #define HAL_WITH_MCU_MONITORING 0
// #define HAL_USE_QUADSPI 0

#ifdef __cplusplus
#include <AP_HAL_Embox/Semaphores.h>
#define HAL_Semaphore Embox::Semaphore
#define HAL_BinarySemaphore Embox::BinarySemaphore
#endif
