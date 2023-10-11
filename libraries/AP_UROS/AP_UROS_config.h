#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_UROS_ENABLED
#define AP_UROS_ENABLED 1
#endif  // AP_UROS_ENABLED

// UDP only on SITL for now
#ifndef AP_UROS_UDP_ENABLED
#define AP_UROS_UDP_ENABLED AP_UROS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif  // AP_UROS_UDP_ENABLED

#include <AP_VisualOdom/AP_VisualOdom_config.h>
#ifndef AP_UROS_VISUALODOM_ENABLED
#define AP_UROS_VISUALODOM_ENABLED HAL_VISUALODOM_ENABLED && AP_UROS_ENABLED
#endif  // AP_UROS_VISUALODOM_ENABLED

// Only enable parameter server on SITL or EPS32
#ifndef AP_UROS_PARAM_SRV_ENABLED
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#define AP_UROS_PARAM_SRV_ENABLED 1
#else
#define AP_UROS_PARAM_SRV_ENABLED 0
#endif
#endif  // AP_UROS_PARAM_SRV_ENABLED

// esp32 - free-rtos
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#endif
