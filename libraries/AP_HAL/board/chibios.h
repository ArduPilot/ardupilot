#pragma once

#include <hwdef.h>

#define HAL_BOARD_NAME "ChibiOS"

#if HAL_MEMORY_TOTAL_KB >= 1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#elif HAL_MEMORY_TOTAL_KB >= 500
#define HAL_MEM_CLASS HAL_MEM_CLASS_500
#elif HAL_MEMORY_TOTAL_KB >= 300
#define HAL_MEM_CLASS HAL_MEM_CLASS_300
#elif HAL_MEMORY_TOTAL_KB >= 192
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#elif HAL_MEMORY_TOTAL_KB >= 64
#define HAL_MEM_CLASS HAL_MEM_CLASS_64
#else
#define HAL_MEM_CLASS HAL_MEM_CLASS_20
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
#endif

#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif

#ifdef HAL_GPIO_PIN_SAFETY_IN
#define HAL_HAVE_SAFETY_SWITCH 1
#endif

#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 0
#endif

#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

#ifndef HAL_WITH_RAMTRON
#define HAL_WITH_RAMTRON 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_ChibiOS/Semaphores.h>
#define HAL_Semaphore ChibiOS::Semaphore
#define HAL_BinarySemaphore ChibiOS::BinarySemaphore
#endif

/* string names for well known SPI devices */
#define HAL_BARO_MS5611_NAME "ms5611"
#ifndef HAL_BARO_MS5611_SPI_INT_NAME
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#endif
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"
#define HAL_BARO_BMP280_NAME "bmp280"

#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"

#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"

#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"

#define HAL_INS_MPU6500_NAME "mpu6500"

#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"

#define HAL_COMPASS_HMC5843_NAME "hmc5843"
#define HAL_COMPASS_LIS3MDL_NAME "lis3mdl"

// allow for short names overridden in hwdef.dat
#ifndef CHIBIOS_SHORT_BOARD_NAME
#define CHIBIOS_SHORT_BOARD_NAME CHIBIOS_BOARD_NAME
#endif

#ifndef CONFIG_HAL_BOARD_SUBTYPE
// allow for generic boards
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC
#endif

// we support RC serial for BLHeli pass-thru
#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 1
#endif

// by default assume first I2C bus is internal
#ifndef HAL_I2C_INTERNAL_MASK
#define HAL_I2C_INTERNAL_MASK 1
#endif

// put all storage of files under /APM directory
#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "/APM"
#endif

#if defined(STM32_WSPI_USE_QUADSPI1) && STM32_WSPI_USE_QUADSPI1
#define HAL_USE_QUADSPI1 TRUE
#else
#define HAL_USE_QUADSPI1 FALSE
#endif
#if defined(STM32_WSPI_USE_QUADSPI2) && STM32_WSPI_USE_QUADSPI2
#define HAL_USE_QUADSPI2 TRUE
#else
#define HAL_USE_QUADSPI2 FALSE
#endif
#if defined(STM32_WSPI_USE_OCTOSPI1) && STM32_WSPI_USE_OCTOSPI1
#define HAL_USE_OCTOSPI1 TRUE
#else
#define HAL_USE_OCTOSPI1 FALSE
#endif
#if defined(STM32_WSPI_USE_OCTOSPI2) && STM32_WSPI_USE_OCTOSPI2
#define HAL_USE_OCTOSPI2 TRUE
#else
#define HAL_USE_OCTOSPI2 FALSE
#endif
#define HAL_USE_QUADSPI (HAL_USE_QUADSPI1 || HAL_USE_QUADSPI2)
#define HAL_USE_OCTOSPI (HAL_USE_OCTOSPI1 || HAL_USE_OCTOSPI2)
