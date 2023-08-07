// this file is inserted (by chibios_hwdef.py) into hwdef.h when
// configuring for bootloader builds

#define HAL_DSHOT_ALARM_ENABLED 0
#define HAL_LOGGING_ENABLED 0
#define HAL_SCHEDULER_ENABLED 0

// bootloaders *definitely* don't use the FFT library:
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

// bootloaders don't talk to the GCS:
#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

// by default bootloaders don't use INS:
#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// bootloader does not save temperature cals etc:
#ifndef HAL_ENABLE_SAVE_PERSISTENT_PARAMS
#define HAL_ENABLE_SAVE_PERSISTENT_PARAMS 0
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

// make diagnosing Faults (e.g. HardFault) harder, but save bytes:
#ifndef AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED
#define AP_FAULTHANDLER_DEBUG_VARIABLES_ENABLED 0
#endif

#ifndef AP_WATCHDOG_SAVE_FAULT_ENABLED
#define AP_WATCHDOG_SAVE_FAULT_ENABLED 0
#endif
