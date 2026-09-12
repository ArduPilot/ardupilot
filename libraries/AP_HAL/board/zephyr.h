#pragma once

#include <hwdef.h>
#if defined(AP_BUILD_MINIMIZE) && AP_BUILD_MINIMIZE
#include <AP_HAL/board/minimize.h>
// hwdef.h is included first, so a board that opts into a feature there beats
// minimize.h. Minimized tool builds (CPUInfo) set AP_VEHICLE_ENABLED 0, and
// lua_generated_bindings.h declares AP_Vehicle::custom_mode_state whenever
// scripting is on, so a board hwdef enabling scripting fails to compile.
#undef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef HAL_BOARD_NAME
#define HAL_BOARD_NAME "Zephyr"
#endif

#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000

#ifndef HAL_STORAGE_SIZE
// Keep storage shadow small — on ESP32-S3 DRAM is only ~27KB free at boot.
// ZMS uses 256-byte chunks so 8192 = 32 chunks, enough for all AP_Param data.
#define HAL_STORAGE_SIZE 8192
#endif
#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE

// Disable backup parameter storage — RAM-only storage, no persistent media
#define AP_PARAM_STORAGE_BAK_ENABLED 0

// Disable AP_Stats — stats.init() hangs spinning on save_queue (no IO drain yet)
#define AP_STATS_ENABLED 0

// Disable I2C LED notify backends — none of these ICs are present on ESP32S3
#define AP_NOTIFY_LP5562_ENABLED     0
#define AP_NOTIFY_IS31FL3195_ENABLED 0
#define AP_NOTIFY_NCP5623_ENABLED    0
#define AP_NOTIFY_PCA9685_ENABLED    0
#define AP_NOTIFY_TOSHIBALED_ENABLED 0

// Allow boot without a physical barometer (no sensor on DevKit)
#define HAL_BARO_ALLOW_INIT_NO_BARO

// Allow boot without physical IMU sensors
#define AP_INERTIALSENSOR_ALLOW_NO_SENSORS 1

// Mirror esp32.h: disable features not present on this hardware
#ifndef HAL_WITH_DRONECAN
#define HAL_WITH_DRONECAN         0
#endif
#ifndef HAL_WITH_UAVCAN
#define HAL_WITH_UAVCAN           0
#endif
#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES        0
#endif
/* Was hardcoded to 0 regardless of HAL_NUM_CAN_IFACES, so a board that declared
 * CAN interfaces still built without CAN. */
#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
#define HAL_MAX_CAN_PROTOCOL_DRIVERS HAL_NUM_CAN_IFACES
#endif
#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH    0
#endif
#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE    0
#endif
#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE    0
#endif
/* #ifndef-guarded like its neighbours: a board whose hwdef declares
   IOMCU_UART has hwdef.h set this to 1 above, and a hard override here threw
   that away - AP_IOMCU.cpp compiled to an empty object and the IO
   co-processor was never driven. */
#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU           0
#endif
#define HAL_GENERATOR_ENABLED     0
/* #ifndef-guarded rather than a hard override so a board hwdef can turn
   ExternalAHRS back on where the board has one attached. */
#ifndef AP_EXTERNAL_AHRS_ENABLED
#define AP_EXTERNAL_AHRS_ENABLED  0
#endif
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0
/* #ifndef-guarded rather than hard overrides so a board hwdef can opt in: DSP
 * needs CMSIS-DSP, which is ARM-only, so it stays off for the HAL by default. */
#ifndef HAL_WITH_DSP
#define HAL_WITH_DSP              0
#endif
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED       0
#endif
#define AP_FRSKY_TELEM_ENABLED    0
#define HAL_QUADPLANE_ENABLED     0
#define HAL_SOARING_ENABLED       0
#define HAL_MOUNT_ENABLED         0
#define HAL_ADSB_ENABLED          0
#define HAL_BUTTON_ENABLED        0
#define AP_ADVANCEDFAILSAFE_ENABLED 0
#define AP_ICENGINE_ENABLED       0
#define AP_OPTICALFLOW_ENABLED    0
#define AP_RPM_ENABLED            0
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#define AP_CAMERA_ENABLED         0
#define AP_TERRAIN_AVAILABLE      0
#define AP_GRIPPER_ENABLED        0
#define AP_LANDINGGEAR_ENABLED    0
#define AP_AVOIDANCE_ENABLED      0
#define AP_FENCE_ENABLED          0
#define MODE_FOLLOW_ENABLED       0
#define AP_OAPATHPLANNER_ENABLED  0
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif
#define AP_SCHEDULER_OVERTIME_MARGIN_US 50000UL

// AP_Filter_config.h uses #ifndef AP_FILTER_ENABLED, so this must appear before
// HAL_PROGRAM_SIZE_LIMIT_KB is defined (which would otherwise make it 1).
#define AP_FILTER_ENABLED 0

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

#ifndef HAL_BOARD_STATE_DIRECTORY
#define HAL_BOARD_STATE_DIRECTORY "/APM"
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/logs"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/terrain"
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY HAL_BOARD_STATE_DIRECTORY
#endif

#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 0
#endif

#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif

#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 0
#endif

// minimize.h sets this to 0, and it is included above whenever AP_BUILD_MINIMIZE
// is set, so redefining it bare warned in every translation unit of a minimal
// build. #undef first, matching the HAL_OS_FATFS_IO pattern further down.
// Deliberately NOT an #ifndef guard: that would silently leave probing off in
// minimize builds, which is a behaviour change, not a warning fix.
#undef AP_COMPASS_PROBING_ENABLED
#define AP_COMPASS_PROBING_ENABLED 1

/*
  HAL thread enables - same macro names and polarity as AP_HAL_ChibiOS, whose
  Scheduler::init() guards every thread individually. AP_HAL_Zephyr previously
  created ALL SIX unconditionally, so minimal tools (CPUInfo, which sets
  AP_BUILD_MINIMIZE) spawned rcin, rcout, storage and monitor threads they never
  use - burning RAM and CPU and polluting the console of a benchmark whose whole
  job is precise timing.
*/
#if defined(AP_BUILD_MINIMIZE) && AP_BUILD_MINIMIZE
  #ifndef HAL_RCIN_THREAD_ENABLED
  #define HAL_RCIN_THREAD_ENABLED 0
  #endif
  #ifndef HAL_NO_RCOUT_THREAD
  #define HAL_NO_RCOUT_THREAD
  #endif
  #ifndef HAL_USE_EMPTY_STORAGE
  #define HAL_USE_EMPTY_STORAGE
  #endif
  #ifndef HAL_MONITOR_THREAD_ENABLED
  #define HAL_MONITOR_THREAD_ENABLED 0
  #endif
  #ifndef HAL_NO_TIMER_THREAD
  #define HAL_NO_TIMER_THREAD
  #endif
  #ifndef HAL_USE_EMPTY_IO
  #define HAL_USE_EMPTY_IO
  #endif
#endif

#ifndef HAL_RCIN_THREAD_ENABLED
#define HAL_RCIN_THREAD_ENABLED 1
#endif
#ifndef HAL_MONITOR_THREAD_ENABLED
#define HAL_MONITOR_THREAD_ENABLED 1
#endif

/* Fast rate thread (ArduCopter/rate_thread.cpp), mirroring chibios.h. */
#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 1
#endif

/* Per-ARCHITECTURE, not per-port: this used to assert 1 for every Zephyr board,
 * including those with no hardware double. */
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#if defined(__XTENSA__)
#define HAL_HAVE_HARDWARE_DOUBLE 0
#elif defined(__riscv) && (!defined(__riscv_flen) || __riscv_flen < 64)
/* RISC-V with no FPU (ESP32-C6) or single-precision-only FPU: doubles are
   libgcc soft-float, same trap as Xtensa above. __riscv_flen is 32 for an
   F-extension core, 64 for FD - only the latter has hardware doubles. */
#define HAL_HAVE_HARDWARE_DOUBLE 0
#else
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

/* Explicitly OFF on soft-double cores (maintainer directive 2026-08-15): the EKF
 * in software double is far slower than the loop budget allows. */
#ifndef AP_MATH_ALLOW_DOUBLE_FUNCTIONS
#if !HAL_HAVE_HARDWARE_DOUBLE
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 0
#endif
#endif

/* Zephyr defines main() in AP_HAL_Zephyr/zephyr/src/main.c; it calls
 * ardupilot_entry() after USB init.  AP_HAL_MAIN() expands to define
 * ardupilot_entry() via this alias. */
#define AP_MAIN ardupilot_entry

#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif
#if HAL_NUM_CAN_IFACES >= 1
#define HAL_CAN_IFACE0_ENABLE 1
#endif
#if HAL_NUM_CAN_IFACES >= 2
#define HAL_CAN_IFACE1_ENABLE 1
#endif

/* Logging. Enabled 2026-08-05. */
#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 1
#endif

/* SD card / FATFS. Uses Zephyr's FatFs, not ArduPilot's, because Zephyr's CMake
 * already builds ff.c plus the diskio glue that bridges to disk_access. */
/* Guard the OTHER direction: ArduPilot also ships a FatFs, and only one of the
 * two may be linked or the ff.c symbols collide. */
#if defined(HAL_OS_FATFS_IO) && HAL_OS_FATFS_IO && !defined(CONFIG_FAT_FILESYSTEM_ELM)
#error "HAL_OS_FATFS_IO needs a FatFs. Enable CONFIG_FAT_FILESYSTEM_ELM \
(plus CONFIG_SDHC, CONFIG_DISK_ACCESS, CONFIG_DISK_DRIVER_SDMMC) in the board \
prj.conf. Do NOT additionally compile ArduPilot's ff.c - two FatFs copies \
collide on every symbol."
#endif

/* CORRECTION 2026-08-13: this force used to be UNCONDITIONAL, which broke every
 * board that did not want it. */
#ifdef CONFIG_FAT_FILESYSTEM_ELM
#undef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 1
#undef AP_FILESYSTEM_FATFS_ENABLED
#define AP_FILESYSTEM_FATFS_ENABLED 1
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

// picolibc declares errno thread-local in errno.h
#ifndef AP_HAL_LIBC_DECLARES_ERRNO
#define AP_HAL_LIBC_DECLARES_ERRNO 1
#endif

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef HAL_OS_SOCKETS
#define HAL_OS_SOCKETS 0
#endif

#ifdef __cplusplus
#include <AP_HAL_Zephyr/Semaphores.h>
#define HAL_Semaphore Zephyr::Semaphore
#define HAL_BinarySemaphore Zephyr::BinarySemaphore

/* zephyr/kernel_structs.h (via Semaphores.h) collides with AP macro names, so it
 * must be included before they are defined. */
#ifdef _current
#undef _current
#endif
#endif

// AP-level feature flags are NOT forced off here — let the default config
// headers and per-board hwdef.h decide.  Only Zephyr HAL-layer things that
// have no Zephyr implementation and would cause link errors are disabled.
