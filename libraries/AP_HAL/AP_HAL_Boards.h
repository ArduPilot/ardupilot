
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
#define HAL_BOARD_SMACCM   4 // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_FLYMAPLE 6
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_EMPTY    99


/**
   CPU classes, used to select if CPU intensive algorithms should be used

   Note that these are only approximate, not exact CPU speeds. 
 */
#define HAL_CPU_CLASS_16   1   // 16Mhz, AVR2560 or similar
#define HAL_CPU_CLASS_75   2   // 75Mhz, Flymaple or similar
#define HAL_CPU_CLASS_150  3   // 150Mhz, PX4 or similar, assumes
                               // hardware floating point. Assumes tens
                               // of kilobytes of memory available
#define HAL_CPU_CLASS_1000 4   // GigaHz class, SITL, BeagleBone etc,
                               // assumes megabytes of memory available

/**
   operating system features:

   HAL implementations may define the following extra feature defines
   to 1 if available

  HAL_OS_POSIX_IO    :  has posix-like filesystem IO
 */


/*
  define AP_HAL_BOARD_DRIVER to the right hal type for this
  board. This prevents us having a mess of ifdefs in every example
  sketch
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM1
#define HAL_BOARD_NAME "APM 1"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16

#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM2
#define HAL_BOARD_NAME "APM 2"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16

#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_SITL
#define HAL_BOARD_NAME "SITL"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_OS_POSIX_IO 1

#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#define AP_HAL_BOARD_DRIVER AP_HAL_FLYMAPLE
#define HAL_BOARD_NAME "FLYMAPLE"
#define HAL_CPU_CLASS HAL_CPU_CLASS_75

#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define AP_HAL_BOARD_DRIVER AP_HAL_PX4
#define HAL_BOARD_NAME "PX4"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_HAL_BOARD_DRIVER AP_HAL_Linux
#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_OS_POSIX_IO 1

#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
#define AP_HAL_BOARD_DRIVER AP_HAL_Empty
#define HAL_BOARD_NAME "EMPTY"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16

#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif


#endif // __AP_HAL_BOARDS_H__

