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
#define HAL_BOARD_QURT     9
#define HAL_BOARD_EMPTY   99

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
#define HAL_BOARD_SUBTYPE_LINUX_RASPILOT   1007
#define HAL_BOARD_SUBTYPE_LINUX_MINLURE    1008
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 1009
#define HAL_BOARD_SUBTYPE_LINUX_BH         1010
#define HAL_BOARD_SUBTYPE_LINUX_QFLIGHT    1011
#define HAL_BOARD_SUBTYPE_LINUX_PXFMINI    1012
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO2     1013
#define HAL_BOARD_SUBTYPE_LINUX_DISCO      1014
#define HAL_BOARD_SUBTYPE_LINUX_AERO       1015
#define HAL_BOARD_SUBTYPE_LINUX_DARK       1016
#define HAL_BOARD_SUBTYPE_LINUX_URUS       1017
#define HAL_BOARD_SUBTYPE_LINUX_BLUE       1018

/* HAL PX4 sub-types, starting at 2000 */
#define HAL_BOARD_SUBTYPE_PX4_V1           2000
#define HAL_BOARD_SUBTYPE_PX4_V2           2001
#define HAL_BOARD_SUBTYPE_PX4_V4           2002
#define HAL_BOARD_SUBTYPE_PX4_V3           2003
#define HAL_BOARD_SUBTYPE_PX4_AEROFC_V1    2004

/* HAL VRBRAIN sub-types, starting at 4000 */
#define HAL_BOARD_SUBTYPE_VRBRAIN_V45      4000
#define HAL_BOARD_SUBTYPE_VRBRAIN_V51      4001
#define HAL_BOARD_SUBTYPE_VRBRAIN_V52      4002
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V51     4003
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V52     4004
#define HAL_BOARD_SUBTYPE_VRCORE_V10       4005
#define HAL_BOARD_SUBTYPE_VRBRAIN_V54      4006

/* InertialSensor driver types */
#define HAL_INS_MPU60XX_SPI  2
#define HAL_INS_MPU60XX_I2C  3
#define HAL_INS_HIL          4
#define HAL_INS_PX4          5
#define HAL_INS_L3G4200D     7
#define HAL_INS_VRBRAIN      8
#define HAL_INS_MPU9250_SPI  9
#define HAL_INS_L3GD20      10
#define HAL_INS_LSM9DS0     11
#define HAL_INS_RASPILOT    12
#define HAL_INS_MPU9250_I2C 13
#define HAL_INS_BH          14
#define HAL_INS_QFLIGHT     15
#define HAL_INS_QURT        16
#define HAL_INS_BBBMINI     17
#define HAL_INS_AERO        18
#define HAL_INS_MPU6500     19

/* Barometer driver types */
#define HAL_BARO_BMP085      1
#define HAL_BARO_MS5611_I2C  2
#define HAL_BARO_MS5611_SPI  3
#define HAL_BARO_MS5607_I2C  4
#define HAL_BARO_PX4         5
#define HAL_BARO_HIL         6
#define HAL_BARO_VRBRAIN     7
#define HAL_BARO_MS5637_I2C  8
#define HAL_BARO_QFLIGHT     9
#define HAL_BARO_QURT       10
#define HAL_BARO_BMP280_I2C 11
#define HAL_BARO_BMP280_SPI 12

/* Compass driver types */
#define HAL_COMPASS_HMC5843             1
#define HAL_COMPASS_PX4                 2
#define HAL_COMPASS_HIL                 3
#define HAL_COMPASS_VRBRAIN             4
#define HAL_COMPASS_AK8963_MPU9250      5
#define HAL_COMPASS_AK8963_I2C          6
#define HAL_COMPASS_HMC5843_MPU6000     7
#define HAL_COMPASS_RASPILOT            8
#define HAL_COMPASS_AK8963_MPU9250_I2C  9
#define HAL_COMPASS_BH                 10
#define HAL_COMPASS_QFLIGHT            11
#define HAL_COMPASS_QURT               12
#define HAL_COMPASS_BBBMINI            13
#define HAL_COMPASS_NAVIO2             14
#define HAL_COMPASS_NAVIO              15
#define HAL_COMPASS_AERO               16

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
#elif CONFIG_HAL_BOARD == HAL_BOARD_QURT
    #include <AP_HAL/board/qurt.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    #include <AP_HAL/board/vrbrain.h>
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

#ifndef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
#define HAL_RANGEFINDER_LIGHTWARE_I2C_BUS 1
#endif

#ifndef HAL_COMPASS_HMC5843_I2C_ADDR
#define HAL_COMPASS_HMC5843_I2C_ADDR 0x1E
#endif

#ifndef HAL_WITH_UAVCAN
#define HAL_WITH_UAVCAN 0
#endif

// this is used as a general mechanism to make a 'small' build by
// dropping little used features. We use this to allow us to keep
// FMUv2 going for as long as possible
#ifndef HAL_MINIMIZE_FEATURES
#define HAL_MINIMIZE_FEATURES       0
#endif
