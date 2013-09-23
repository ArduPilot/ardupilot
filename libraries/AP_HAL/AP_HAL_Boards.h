
#ifndef __AP_HAL_BOARDS_H__
#define __AP_HAL_BOARDS_H__

/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * Its not an elegant solution but we cant get too fancy if we want to
 * work with the Arduino mk and IDE builds without too much modification.
 */
 
#define HAL_BOARD_APM1     1
#define HAL_BOARD_APM2     2
#define HAL_BOARD_AVR_SITL 3
#define HAL_BOARD_SMACCM   4
#define HAL_BOARD_PX4      5
#define HAL_BOARD_FLYMAPLE 6
#define HAL_BOARD_EMPTY    99

/*
  define AP_HAL_BOARD_DRIVER to the right hal type for this
  board. This prevents us having a mess of ifdefs in every example
  sketch
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM1
#define HAL_BOARD_NAME "APM 1"
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM2
#define HAL_BOARD_NAME "APM 2"
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_SITL
#define HAL_BOARD_NAME "SITL"
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#define AP_HAL_BOARD_DRIVER AP_HAL_FLYMAPLE
#define HAL_BOARD_NAME "FLYMAPLE"
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define AP_HAL_BOARD_DRIVER AP_HAL_PX4
#define HAL_BOARD_NAME "PX4"
#elif CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
#define AP_HAL_BOARD_DRIVER AP_HAL_SMACCM
#define HAL_BOARD_NAME "SMACCM"
#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
#define AP_HAL_BOARD_DRIVER AP_HAL_Empty
#define HAL_BOARD_NAME "EMPTY"
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

#endif // __AP_HAL_BOARDS_H__

