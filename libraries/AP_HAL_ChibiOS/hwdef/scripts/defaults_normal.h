// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for "normal" builds - typically vehicle binaries but
// also examples.

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED (HAL_PWM_COUNT>0)
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#endif

// enable terrain only if there's an SD card available:
#define AP_TERRAIN_AVAILABLE HAL_OS_FATFS_IO
