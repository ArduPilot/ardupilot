
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
#define HAL_BOARD_EMPTY    99

/*
  define AP_HAL_BOARD_DRIVER to the right hal type for this
  board. This prevents us having a mess of ifdefs in every example
  sketch
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM1
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM2
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_SITL
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

#endif // __AP_HAL_BOARDS_H__

