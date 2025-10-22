#pragma once

#define HAL_BOARD_NAME "SITL"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_OS_SOCKETS 1

#define AP_FLASHSTORAGE_TYPE 3

#if AP_FLASHSTORAGE_TYPE == 1
// emulate F1/F3 flash
#define HAL_STORAGE_SIZE 15360
#define HAL_FLASH_SECTOR_SIZE 16*1024
#define HAL_FLASH_MIN_WRITE_SIZE 1
#define HAL_FLASH_ALLOW_UPDATE 0

#elif AP_FLASHSTORAGE_TYPE == 2
// emulate F4/F7 flash
#define HAL_STORAGE_SIZE 15360
#define HAL_FLASH_SECTOR_SIZE 16*1024
#define HAL_FLASH_MIN_WRITE_SIZE 1
#define HAL_FLASH_ALLOW_UPDATE 1

#elif AP_FLASHSTORAGE_TYPE == 3
// emulate H7 flash
#define HAL_STORAGE_SIZE 16384
#define HAL_FLASH_SECTOR_SIZE 128*1024
#define HAL_FLASH_MIN_WRITE_SIZE 32
#define HAL_FLASH_ALLOW_UPDATE 0
#endif

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 4096
#endif

#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            32768
#endif

#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_PARAM_DEFAULTS_PATH nullptr
#define HAL_INS_DEFAULT HAL_INS_NONE

// simulated LEDs are disabled by default as they lead to a large
// amount of SIM_GPIO_MASK mavlink traffic

// #define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1
#define AP_NOTIFY_GPIO_LED_RGB_RED_PIN    8  // these are set in SIM_PIN_MASK
#define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN  9
#define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN  10

// #define AP_NOTIFY_GPIO_LED_1_ENABLED 1
#define AP_NOTIFY_GPIO_LED_A_PIN          8  // these are set in SIM_PIN_MASK

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SERVO_VOLTAGE 1
#define HAL_HAVE_SAFETY_SWITCH 1

// only include if compiling C++ code
#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_SITL/Semaphores.h>
#define HAL_Semaphore HALSITL::Semaphore
#define HAL_BinarySemaphore HALSITL::BinarySemaphore
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "."
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED 1
#endif

#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 1
#endif

#define HAL_SOLO_GIMBAL_ENABLED 1

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 1
#endif

#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 1
#endif

#ifndef AP_NOTIFY_TONEALARM_ENABLED
#define AP_NOTIFY_TONEALARM_ENABLED 1
#endif

#ifndef AP_NOTIFY_BUZZER_ENABLED
#define AP_NOTIFY_BUZZER_ENABLED 1
#endif

#define HAL_BATT_VOLT_PIN                  13
#define HAL_BATT_CURR_PIN                  12
#define HAL_BATT_VOLT_SCALE                10.1f
#define HAL_BATT_CURR_SCALE                17.0f

#define RELAY1_PIN_DEFAULT 13
