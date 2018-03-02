#pragma once

#include <hwdef.h>

#define HAL_BOARD_NAME "ChibiOS"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150

#ifndef HAL_GPIO_A_LED_PIN
#define HAL_GPIO_A_LED_PIN        0
#endif
#ifndef HAL_GPIO_B_LED_PIN
#define HAL_GPIO_B_LED_PIN        0
#endif
#ifndef HAL_GPIO_C_LED_PIN
#define HAL_GPIO_C_LED_PIN        0
#endif
#ifndef HAL_GPIO_LED_ON
#define HAL_GPIO_LED_ON           0
#endif
#ifndef HAL_GPIO_LED_OFF
#define HAL_GPIO_LED_OFF          1
#endif

#ifndef HAL_WITH_UAVCAN
#define HAL_WITH_UAVCAN 0
#endif

#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
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

/* string names for well known SPI devices */
#define HAL_BARO_MS5611_NAME "ms5611"
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"

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


#ifndef CONFIG_HAL_BOARD_SUBTYPE
// allow for generic boards
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC
#endif
