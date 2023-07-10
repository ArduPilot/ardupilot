#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Logger/AP_Logger_config.h>

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundant sensors may be available
 */

#ifndef INS_AUX_INSTANCES
#define INS_AUX_INSTANCES 0
#endif

#ifndef INS_MAX_INSTANCES
#define INS_MAX_INSTANCES (3+INS_AUX_INSTANCES)
#endif

#if INS_MAX_INSTANCES < 3 && INS_AUX_INSTANCES > 0
#error "INS_AUX_INSTANCES must be zero if INS_MAX_INSTANCES is less than 3"
#endif

#if INS_MAX_INSTANCES > 3 && INS_AUX_INSTANCES == 0
#error "INS_AUX_INSTANCES must be non-zero if INS_MAX_INSTANCES is greater than 3"
#endif

#define INS_MAX_BACKENDS  2*INS_MAX_INSTANCES
#define INS_MAX_NOTCHES 12
#ifndef INS_VIBRATION_CHECK_INSTANCES
  #if HAL_MEM_CLASS >= HAL_MEM_CLASS_300
    #define INS_VIBRATION_CHECK_INSTANCES INS_MAX_INSTANCES
  #else
    #define INS_VIBRATION_CHECK_INSTANCES 1
  #endif
#endif
#define XYZ_AXIS_COUNT    3
// The maximum we need to store is gyro-rate / loop-rate, worst case ArduCopter with BMI088 is 2000/400
#define INS_MAX_GYRO_WINDOW_SAMPLES 8

#define DEFAULT_IMU_LOG_BAT_MASK 0

#ifndef HAL_INS_TEMPERATURE_CAL_ENABLE
#define HAL_INS_TEMPERATURE_CAL_ENABLE BOARD_FLASH_SIZE > 1024
#endif

#ifndef HAL_INS_NUM_HARMONIC_NOTCH_FILTERS
#define HAL_INS_NUM_HARMONIC_NOTCH_FILTERS 2
#endif

// time for the estimated gyro rates to converge
#ifndef HAL_INS_CONVERGANCE_MS
#define HAL_INS_CONVERGANCE_MS 30000
#endif

#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 1
#endif

#ifndef AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
#define AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED (AP_INERTIALSENSOR_ENABLED && HAL_LOGGING_ENABLED)
#endif

#ifndef AP_INERTIALSENSOR_KILL_IMU_ENABLED
#define AP_INERTIALSENSOR_KILL_IMU_ENABLED 1
#endif
