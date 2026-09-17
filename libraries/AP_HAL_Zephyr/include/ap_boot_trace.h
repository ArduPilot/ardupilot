/*
 * BOOT_TRACE - boot/bringup tracing for the Zephyr HAL.
 *
 * printk, NOT DEV_PRINTF/hal.console: these fire before the console exists and
 * from threads that must never block on it. Compiled out entirely unless
 * ./waf configure --debug (HAL_DEBUG_BUILD), so a release build carries
 * neither the calls nor the format strings.
 *
 * Lives here, not in AP_HAL/board/zephyr.h, because it has to serve BOTH
 * compilers. The waf-built HAL sources reach it through board/zephyr.h, which
 * includes this file. The sources under zephyr/src are built by Zephyr's CMake
 * with -nostdinc++ and cannot include board/zephyr.h at all: that header pulls
 * in AP_HAL_Zephyr/Semaphores.h and the rest of the AP_HAL C++ stack, which
 * ends at AP_Common/missing/type_traits and a missing <type_traits>. They
 * include this header directly instead.
 *
 * HAL_DEBUG_BUILD reaches the two sides by different routes: boards.py puts it
 * in env.DEFINES for the waf TUs, and zephyr.py forwards it to CMake as
 * AP_HAL_DEBUG_BUILD for the Zephyr TUs. Both must be in place or the tracing
 * goes quiet on one side only, with no diagnostic.
 */
#pragma once

#if defined(HAL_DEBUG_BUILD) && HAL_DEBUG_BUILD
#include <zephyr/sys/printk.h>
#define BOOT_TRACE(fmt, args ...)  printk(fmt, ## args)
#else
#define BOOT_TRACE(fmt, args ...)  do {} while (0)
#endif
