#pragma once

#include <AP_HAL_QURT/AP_HAL_QURT_Main.h>

#define HAL_BOARD_NAME "QURT"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_STORAGE_SIZE            32768
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

// only include if compiling C++ code
#ifdef __cplusplus
#include <AP_HAL_QURT/Semaphores.h>
#define HAL_Semaphore QURT::Semaphore
#define HAL_BinarySemaphore QURT::BinarySemaphore
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

/*
  disable features for initial port
 */
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_WITH_MCU_MONITORING 0
#define HAL_USE_QUADSPI 0
#define HAL_WITH_DSP 0

#define HAL_CANFD_SUPPORTED 0
#define HAL_NUM_CAN_IFACES 0

#define AP_CRASHDUMP_ENABLED 0
#define HAL_ENABLE_DFU_BOOT 0


#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_LOGGING_FILESYSTEM_ENABLED 1

#define AP_FILESYSTEM_POSIX_HAVE_UTIME 0
#define AP_FILESYSTEM_POSIX_HAVE_FSYNC 0
#define AP_FILESYSTEM_POSIX_HAVE_STATFS 0
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 0

#define AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC 1
#define AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR "/data"
#define HAL_BOARD_STORAGE_DIRECTORY "APM"
#define HAL_BOARD_LOG_DIRECTORY "APM/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "APM/terrain"

#define SCRIPTING_DIRECTORY "APM/scripts"

/*
  a defaults file for this vehicle
 */
#ifndef HAL_PARAM_DEFAULTS_PATH
// this is an absolute path, as required by AP_Param
#define HAL_PARAM_DEFAULTS_PATH AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR "/APM/defaults.parm"
#endif

#define HAL_WITH_ESC_TELEM 1

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 1
#endif

#define HAL_OS_LITTLEFS_IO 0

/*
  battery monitoring setup, comes in via ESCs
 */
#define HAL_BATT_VOLT_PIN 1
#define HAL_BATT_CURR_PIN 2
#define HAL_BATT_MONITOR_DEFAULT 4
#define HAL_BATT_VOLT_SCALE 1
#define HAL_BATT_CURR_SCALE 1

#define AP_COMPASS_PROBING_ENABLED 1

/*
  compass list
 */
#define PROBE_MAG_I2C(driver, bus, addr, args ...) add_backend(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args)); RETURN_IF_NO_SPACE;
#define HAL_MAG_PROBE_LIST PROBE_MAG_I2C(QMC5883L, 0, 0x0d, true, ROTATION_NONE)

/*
  barometer list
 */
#define HAL_BARO_PROBE_LIST \
    probe_i2c_dev(AP_Baro_ICP101XX::probe, 2, 0x63);

/*
  IMU list
 */
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensensev3, "INV3", ROTATION_NONE)

/*
  bring in missing standard library functions
 */
#include <AP_HAL_QURT/replace.h>

#define DEFAULT_SERIAL4_PROTOCOL 23 // RC input
