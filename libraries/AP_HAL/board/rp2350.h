#pragma once

#include <hwdef.h>

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

#ifndef __RAMFUNC__
#define __RAMFUNC__ __attribute__((noinline, section(".time_critical")))
#endif

#ifndef __FASTFUNC__
  #define __FASTFUNC__ __RAMFUNC__
#endif

#ifndef HAL_BOARD_NAME
	#define HAL_BOARD_NAME "RP2350"
#endif

#define HAL_OS_POSIX_IO 0

#ifndef HAL_HAVE_FILESYSTEM
  #define HAL_HAVE_FILESYSTEM 0
#endif

#ifndef HAL_USE_WSPI
#define HAL_USE_WSPI TRUE
#endif
#define HAL_WSPI_DEVICE_LIST "qspi_flash"
#define AP_FLASHSTORAGE_TYPE 2

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

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 4096
#endif

#define HAL_BOARD_STORAGE_DIRECTORY "APM"
#define HAL_BOARD_LOG_DIRECTORY HAL_BOARD_STORAGE_DIRECTORY "/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY HAL_BOARD_STORAGE_DIRECTORY "/terrain"

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_RP/Semaphores.h>
#define HAL_Semaphore RP::Semaphore
#define HAL_BinarySemaphore RP::BinarySemaphore
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#define HAL_NUM_CAN_IFACES 0
#define HAL_MEM_CLASS HAL_MEM_CLASS_500

// disable uncommon stuff that we'd otherwise get 
#define AP_EXTERNAL_AHRS_ENABLED 0
#define HAL_GENERATOR_ENABLED 0

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

// whenver u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#define XCHAL_ERRATUM_453 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#define CONFIG_SYSVIEW_ENABLE 0
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_LWIP_PPP_SUPPORT 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_NEWLIB_NANO_FORMAT 0
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#define CONFIG_LWIP_STATS 0
#define LWIP_COMPAT_SOCKET_INET 0
#define LWIP_COMPAT_SOCKET_ADDR 0


// turn off all the compasses by default.. 
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// we don't need 32, 16 is enough
#define NUM_SERVO_CHANNELS 16

// disble temp cal of gyros by default
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

//turn off a bunch of advanced plane scheduler table things. see Plane.cpp
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

#ifndef AP_NOTIFY_BUZZER_ENABLED
#define AP_NOTIFY_BUZZER_ENABLED 1
#endif
