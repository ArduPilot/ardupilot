/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * It's not an elegant solution but we can improve it in future.
 */
#pragma once

#define HAL_BOARD_SITL     3
#define HAL_BOARD_SMACCM   4  // unused
#define HAL_BOARD_PX4      5  // unused
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_CHIBIOS  10
#define HAL_BOARD_F4LIGHT  11 // reserved
#define HAL_BOARD_ESP32	   12
#define HAL_BOARD_EMPTY    99

/* Default board subtype is -1 */
#define HAL_BOARD_SUBTYPE_NONE -1

/* HAL Linux sub-types, starting at 1000 */
#define HAL_BOARD_SUBTYPE_LINUX_NONE       1000
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD  1001
#define HAL_BOARD_SUBTYPE_LINUX_PXF        1002
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO      1003
#define HAL_BOARD_SUBTYPE_LINUX_ZYNQ       1004
#define HAL_BOARD_SUBTYPE_LINUX_BBBMINI    1005
#define HAL_BOARD_SUBTYPE_LINUX_BEBOP      1006
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 1009
#define HAL_BOARD_SUBTYPE_LINUX_BH         1010
#define HAL_BOARD_SUBTYPE_LINUX_PXFMINI    1012
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO2     1013
#define HAL_BOARD_SUBTYPE_LINUX_DISCO      1014
#define HAL_BOARD_SUBTYPE_LINUX_AERO       1015
#define HAL_BOARD_SUBTYPE_LINUX_DARK       1016
#define HAL_BOARD_SUBTYPE_LINUX_BLUE       1018
#define HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ 1019
#define HAL_BOARD_SUBTYPE_LINUX_EDGE       1020
#define HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ   1021
#define HAL_BOARD_SUBTYPE_LINUX_POCKET     1022
#define HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR  1023
#define HAL_BOARD_SUBTYPE_LINUX_VNAV       1024
#define HAL_BOARD_SUBTYPE_LINUX_OBAL_V1    1025

/* HAL CHIBIOS sub-types, starting at 5000

   NOTE!! Do not add more subtypes unless they are really needed. Most
   boards do not need a subtype defined. It is only needed if we need
   to use #ifdef'd code to change behaviour
*/
#define HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412	5000
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3         5001
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4         5002
#define HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC       5009
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV5         5013
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V51   5016
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52   5017
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51  5018
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRCORE_V10    5019
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V54   5020

#define HAL_BOARD_SUBTYPE_ESP32_DIY             6001
#define HAL_BOARD_SUBTYPE_ESP32_ICARUS          6002
#define HAL_BOARD_SUBTYPE_ESP32_BUZZ            6003

/* InertialSensor driver types */
#define HAL_INS_NONE         0
#define HAL_INS_MPU60XX_SPI  2
#define HAL_INS_MPU60XX_I2C  3
#define HAL_INS_HIL_UNUSED   4  // unused
#define HAL_INS_VRBRAIN      8
#define HAL_INS_MPU9250_SPI  9
#define HAL_INS_MPU9250_I2C 13
#define HAL_INS_MPU6500     19
#define HAL_INS_INV2_I2C    24
#define HAL_INS_INV2_SPI    25


/* Barometer driver types */
#define HAL_BARO_NONE        0
#define HAL_BARO_HIL_UNUSED  6  // unused
#define HAL_BARO_20789_I2C_I2C  14
#define HAL_BARO_20789_I2C_SPI  15
#define HAL_BARO_LPS25H_IMU_I2C 17

/* Heat Types */
#define HAL_LINUX_HEAT_PWM 1

/* CPU classes, used to select if CPU intensive algorithms should be used
 * Note that these are only approximate, not exact CPU speeds. */

/* 150Mhz: STM32F4 or similar. Assumes:
 *  - hardware floating point
 *  - tens of kilobytes of memory available
*/
#define HAL_CPU_CLASS_150  3

/* GigaHz class: SITL, BeagleBone etc. Assumes megabytes of memory available. */
#define HAL_CPU_CLASS_1000 4


/*
  memory classes, in kbytes. Board must have at least the given amount
  of memory
*/
#define HAL_MEM_CLASS_20   1
#define HAL_MEM_CLASS_64   2
#define HAL_MEM_CLASS_192  3
#define HAL_MEM_CLASS_300  4
#define HAL_MEM_CLASS_500  5
#define HAL_MEM_CLASS_1000 6

/* Operating system features
 *
 * HAL implementations may define the following extra feature defines to 1 if
 * available:
 *
 * - HAL_OS_POSIX_IO : has posix-like filesystem IO
 * - HAL_OS_SOCKETS  : has posix-like sockets */

/* DEFINITIONS FOR BOARDS */

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #include <AP_HAL/board/sitl.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL/board/linux.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
    #include <AP_HAL/board/empty.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    #include <AP_HAL/board/vrbrain.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
	#include <AP_HAL/board/chibios.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    #include <AP_HAL/board/esp32.h>
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

#ifndef CONFIG_HAL_BOARD_SUBTYPE
#error "No CONFIG_HAL_BOARD_SUBTYPE set"
#endif

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

#ifndef HAL_OS_SOCKETS
#define HAL_OS_SOCKETS 0
#endif

#ifndef HAL_PARAM_DEFAULTS_PATH
#define HAL_PARAM_DEFAULTS_PATH nullptr
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif

#ifndef HAL_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_RCINPUT_WITH_AP_RADIO
#define HAL_RCINPUT_WITH_AP_RADIO 0
#endif

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

// this is used as a general mechanism to make a 'small' build by
// dropping little used features. We use this to allow us to keep
// FMUv2 going for as long as possible
#ifndef HAL_MINIMIZE_FEATURES
#define HAL_MINIMIZE_FEATURES       0
#endif

#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 2048
#endif

#ifndef HAL_WITH_DSP
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || defined(HAL_BOOTLOADER_BUILD) || defined(HAL_BUILD_AP_PERIPH) || BOARD_FLASH_SIZE <= 1024
#define HAL_WITH_DSP 0
#else
#define HAL_WITH_DSP !HAL_MINIMIZE_FEATURES
#endif
#endif

#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef HAL_BARO_DEFAULT
#define HAL_BARO_DEFAULT HAL_BARO_NONE
#endif

#ifndef HAL_INS_DEFAULT
#define HAL_INS_DEFAULT HAL_INS_NONE
#endif

#ifndef HAL_GPS_TYPE_DEFAULT
#define HAL_GPS_TYPE_DEFAULT 1
#endif

#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
#if defined(HAL_BOOTLOADER_BUILD)
    #define HAL_MAX_CAN_PROTOCOL_DRIVERS 0
#else
    #define HAL_MAX_CAN_PROTOCOL_DRIVERS HAL_NUM_CAN_IFACES
#endif
#endif

#ifndef HAL_CANMANAGER_ENABLED
#define HAL_CANMANAGER_ENABLED ((HAL_MAX_CAN_PROTOCOL_DRIVERS > 0) && !defined(HAL_BUILD_AP_PERIPH))
#endif

#ifndef HAL_ENABLE_LIBUAVCAN_DRIVERS
#define HAL_ENABLE_LIBUAVCAN_DRIVERS HAL_CANMANAGER_ENABLED
#endif

#ifndef AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 1
#endif

#ifdef HAVE_LIBDL
#define AP_MODULE_SUPPORTED 1
#else
#define AP_MODULE_SUPPORTED 0
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif


#ifndef HAL_HAVE_DUAL_USB_CDC
#define HAL_HAVE_DUAL_USB_CDC 0
#endif

#if HAL_NUM_CAN_IFACES && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_UAVCAN_SLCAN_ENABLED 1
#else
#define AP_UAVCAN_SLCAN_ENABLED 0
#endif

#ifndef USE_LIBC_REALLOC
#define USE_LIBC_REALLOC 1
#endif

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS 0
#endif

#ifndef HAL_INS_ENABLED
#define HAL_INS_ENABLED (!defined(HAL_BUILD_AP_PERIPH))
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED (!defined(HAL_BUILD_AP_PERIPH))
#endif

#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 0
#endif

#ifndef HAL_HNF_MAX_FILTERS
// On an F7 The difference in CPU load between 1 notch and 24 notches is about 2%
// The difference in CPU load between 1Khz backend and 2Khz backend is about 10%
// So at 1Khz almost all notch combinations can be supported on F7 and certainly H7
#if defined(STM32H7) || CONFIG_HAL_BOARD == HAL_BOARD_SITL
// Enough for a double-notch per motor on an octa using three IMUs and one harmonics
// plus one static notch with one double-notch harmonics
#define HAL_HNF_MAX_FILTERS 54
#elif defined(STM32F7)
// Enough for a notch per motor on an octa using three IMUs and one harmonics
// plus one static notch with one harmonics
#define HAL_HNF_MAX_FILTERS 27
#else
// Enough for a notch per motor on an octa quad using two IMUs and one harmonic
// plus one static notch with one harmonic
#define HAL_HNF_MAX_FILTERS 18
#endif
#endif // HAL_HNF_MAX_FILTERS

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL // allow SITL to have all the CANFD options
#define HAL_CANFD_SUPPORTED 8
#elif !defined(HAL_CANFD_SUPPORTED)
#define HAL_CANFD_SUPPORTED 0
#endif

#ifndef __RAMFUNC__
#define __RAMFUNC__
#endif

#ifndef __FASTRAMFUNC__
#define __FASTRAMFUNC__
#endif
