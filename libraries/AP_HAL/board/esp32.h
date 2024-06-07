#pragma once


#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_DIY
#include "esp32diy.h" // Charles
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_BUZZ
#include "esp32buzz.h" //Buzz
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_ICARUS
#include "esp32icarus.h" //Alex
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_EMPTY
#include "esp32empty.h" //wiktor-m
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_TOMTE76
#include "esp32tomte76.h" //tomte76 on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_NICK
#include "esp32nick.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT
#include "esp32s3devkit.h" //Nick K. on discord
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_ESP32_S3EMPTY
#include "esp32s3empty.h"
#else
#error "Invalid CONFIG_HAL_BOARD_SUBTYPE for esp32"
#endif

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_WITH_DRONECAN 0
#define HAL_WITH_UAVCAN 0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0

#define HAL_WITH_IO_MCU 0

#define O_CLOEXEC 0
#define HAL_STORAGE_SIZE (16384)

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_BinarySemaphore ESP32::BinarySemaphore
#endif

#define HAL_NUM_CAN_IFACES 0
#define HAL_MEM_CLASS HAL_MEM_CLASS_192

// disable uncommon stuff that we'd otherwise get 
#define HAL_EXTERNAL_AHRS_ENABLED 0
#define HAL_GENERATOR_ENABLED 0

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

// whenver u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#define XCHAL_ERRATUM_453 0
//#define CONFIG_FREERTOS_CORETIMER_0 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#define CONFIG_SYSVIEW_ENABLE 0
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#define USE_LIBC_REALLOC 0
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_LWIP_PPP_SUPPORT 0
#define CONFIG_LWIP_STATS 0
//#define CONFIG_ESP32_WIFI_CSI_ENABLED 0
//#define CONFIG_ESP32_WIFI_NVS_ENABLED 0
#define CONFIG_NEWLIB_NANO_FORMAT 0
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#define CONFIG_LWIP_STATS 0
#define LWIP_COMPAT_SOCKET_INET 0
#define LWIP_COMPAT_SOCKET_ADDR 0
//#define CONFIG_ESP32_WIFI_TX_BA_WIN 0
//#define CONFIG_ESP32_WIFI_RX_BA_WIN 0


// turn off all the compasses by default.. 
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// we don't need 32, 16 is enough
#define NUM_SERVO_CHANNELS 16

// disble temp cal of gyros by default
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

//turn off a bunch of advanced plane scheduler table things. see ArduPlane.cpp
#define AP_ADVANCEDFAILSAFE_ENABLED 0
#define AP_ICENGINE_ENABLED 0
#define AP_OPTICALFLOW_ENABLED 0
#define AP_RPM_ENABLED 0
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#define HAL_MOUNT_ENABLED 0
#define AP_CAMERA_ENABLED 0
#define HAL_SOARING_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
#define HAL_ADSB_ENABLED 0
#define HAL_BUTTON_ENABLED 0 
#define AP_GRIPPER_ENABLED 0
#define AP_LANDINGGEAR_ENABLED 0

// disable avoid-fence-follow in copter, these all kinda need each other, so its all or none.
#define AP_AVOIDANCE_ENABLED 0
#define AP_FENCE_ENABLED 0
#define MODE_FOLLOW_ENABLED 0
#define AP_OAPATHPLANNER_ENABLED 0


// other big things..
#define HAL_QUADPLANE_ENABLED 0
#define HAL_GYROFFT_ENABLED 0
