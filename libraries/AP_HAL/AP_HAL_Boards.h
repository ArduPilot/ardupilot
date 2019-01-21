/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * It's not an elegant solution but we can improve it in future.
 */
#pragma once

#define HAL_BOARD_SITL     3
#define HAL_BOARD_SMACCM   4  // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_CHIBIOS  10
#define HAL_BOARD_F4LIGHT  11 // reserved
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

/* HAL PX4 sub-types, starting at 2000 */
#define HAL_BOARD_SUBTYPE_PX4_V1           2000
#define HAL_BOARD_SUBTYPE_PX4_V2           2001
#define HAL_BOARD_SUBTYPE_PX4_V4           2002
#define HAL_BOARD_SUBTYPE_PX4_V3           2003
#define HAL_BOARD_SUBTYPE_PX4_AEROFC_V1    2004
#define HAL_BOARD_SUBTYPE_PX4_V4PRO        2005

/* HAL VRBRAIN sub-types, starting at 4000 */
#define HAL_BOARD_SUBTYPE_VRBRAIN_V45      4000
#define HAL_BOARD_SUBTYPE_VRBRAIN_V51      4001
#define HAL_BOARD_SUBTYPE_VRBRAIN_V52      4002
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V51     4003
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V52     4004
#define HAL_BOARD_SUBTYPE_VRCORE_V10       4005
#define HAL_BOARD_SUBTYPE_VRBRAIN_V54      4006
#define HAL_BOARD_SUBTYPE_VRBRAIN_V52E     4007


/* HAL CHIBIOS sub-types, starting at 5000 */
#define HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412	5000
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3         5001
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4         5002
#define HAL_BOARD_SUBTYPE_CHIBIOS_MINDPXV2      5003
#define HAL_BOARD_SUBTYPE_CHIBIOS_SPARKY2       5004
#define HAL_BOARD_SUBTYPE_CHIBIOS_REVOMINI      5005
#define HAL_BOARD_SUBTYPE_CHIBIOS_MINIPIX       5006
#define HAL_BOARD_SUBTYPE_CHIBIOS_CRAZYFLIE2    5007
#define HAL_BOARD_SUBTYPE_CHIBIOS_OMNIBUSF7V2   5008
#define HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC       5009
#define HAL_BOARD_SUBTYPE_CHIBIOS_F4BY          5010
#define HAL_BOARD_SUBTYPE_CHIBIOS_OMNIBUSF4PRO  5011
#define HAL_BOARD_SUBTYPE_CHIBIOS_AIRBOTF4      5012
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV5         5013
#define HAL_BOARD_SUBTYPE_CHIBIOS_MATEKF405WING 5014
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4PRO      5015
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52   5016
#define HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51  5017

/* InertialSensor driver types */
#define HAL_INS_NONE         0
#define HAL_INS_MPU60XX_SPI  2
#define HAL_INS_MPU60XX_I2C  3
#define HAL_INS_HIL          4
#define HAL_INS_PX4          5
#define HAL_INS_L3G4200D     7
#define HAL_INS_VRBRAIN      8
#define HAL_INS_MPU9250_SPI  9
#define HAL_INS_L3GD20      10
#define HAL_INS_LSM9DS0     11
#define HAL_INS_MPU9250_I2C 13
#define HAL_INS_BH          14
#define HAL_INS_BBBMINI     17
#define HAL_INS_AERO        18
#define HAL_INS_MPU6500     19
#define HAL_INS_EDGE        20
#define HAL_INS_RST         21
#define HAL_INS_LSM9DS1     22
#define HAL_INS_ICM20789_SPI 23

/* Barometer driver types */
#define HAL_BARO_NONE        0
#define HAL_BARO_BMP085      1
#define HAL_BARO_MS5611_I2C  2
#define HAL_BARO_MS5611_SPI  3
#define HAL_BARO_MS5607_I2C  4
#define HAL_BARO_PX4         5
#define HAL_BARO_HIL         6
#define HAL_BARO_VRBRAIN     7
#define HAL_BARO_MS5637_I2C  8
#define HAL_BARO_BMP280_I2C 11
#define HAL_BARO_BMP280_SPI 12
#define HAL_BARO_LPS25H     13
#define HAL_BARO_20789_I2C_I2C  14
#define HAL_BARO_20789_I2C_SPI  15
#define HAL_BARO_LPS22H_SPI	16
#define HAL_BARO_LPS25H_IMU_I2C 17
#define HAL_BARO_FBM320_I2C 18
#define HAL_BARO_DPS280_I2C 19

/* Compass driver types */
#define HAL_COMPASS_NONE                0
#define HAL_COMPASS_HMC5843             1
#define HAL_COMPASS_PX4                 2
#define HAL_COMPASS_HIL                 3
#define HAL_COMPASS_VRBRAIN             4
#define HAL_COMPASS_AK8963_MPU9250      5
#define HAL_COMPASS_AK8963_I2C          6
#define HAL_COMPASS_HMC5843_MPU6000     7
#define HAL_COMPASS_AK8963_MPU9250_I2C  9
#define HAL_COMPASS_BH                 10
#define HAL_COMPASS_BBBMINI            13
#define HAL_COMPASS_NAVIO2             14
#define HAL_COMPASS_NAVIO              15
#define HAL_COMPASS_AERO               16
#define HAL_COMPASS_OCPOC_ZYNQ         17
#define HAL_COMPASS_EDGE               18
#define HAL_COMPASS_LIS3MDL            19
#define HAL_COMPASS_MAG3110            20
#define HAL_COMPASS_BMM150_I2C         21
#define HAL_COMPASS_QMC5883L           22
#define HAL_COMPASS_IST8310            23

/* Heat Types */
#define HAL_LINUX_HEAT_PWM 1

/* CPU classes, used to select if CPU intensive algorithms should be used
 * Note that these are only approximate, not exact CPU speeds. */

/* 150Mhz: PX4 or similar. Assumes:
 *  - hardware floating point
 *  - tens of kilobytes of memory available */
#define HAL_CPU_CLASS_150  3
/* GigaHz class: SITL, BeagleBone etc. Assumes megabytes of memory available. */
#define HAL_CPU_CLASS_1000 4

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
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
    #include <AP_HAL/board/px4.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL/board/linux.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
    #include <AP_HAL/board/empty.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    #include <AP_HAL/board/vrbrain.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
	#include <AP_HAL/board/chibios.h>
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

#ifndef HAL_WITH_UAVCAN
#define HAL_WITH_UAVCAN 0
#endif

#ifndef HAL_RCINPUT_WITH_AP_RADIO
#define HAL_RCINPUT_WITH_AP_RADIO 0
#endif

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

#ifndef HAL_HAVE_GETTIME_SETTIME
#define HAL_HAVE_GETTIME_SETTIME 0
#endif

// this is used as a general mechanism to make a 'small' build by
// dropping little used features. We use this to allow us to keep
// FMUv2 going for as long as possible
#ifndef HAL_MINIMIZE_FEATURES
#define HAL_MINIMIZE_FEATURES       0
#endif

#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef HAL_PX4_HAVE_PX4IO
#define HAL_PX4_HAVE_PX4IO 0
#endif

#ifndef HAL_COMPASS_DEFAULT
#define HAL_COMPASS_DEFAULT HAL_COMPASS_NONE
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

#ifdef HAVE_LIBDL
#define AP_MODULE_SUPPORTED 1
#else
#define AP_MODULE_SUPPORTED 0
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif

