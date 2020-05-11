/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  wrappers for stdio functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */
#include <string.h>
#include <hal.h>
#include <chprintf.h>
#include <ctype.h>
#include "hwdef/common/stdio.h"
#include <AP_HAL/AP_HAL.h>
#if HAL_USE_SERIAL_USB == TRUE
#include <AP_HAL_ChibiOS/hwdef/common/usbcfg.h>
#include "UARTDriver.h"
#endif

extern const AP_HAL::HAL& hal;

int __wrap_snprintf(char *str, size_t size, const char *fmt, ...)
{
   va_list arg;
   int done;

   va_start (arg, fmt);
   done =  hal.util->vsnprintf(str, size, fmt, arg);
   va_end (arg);

   return done;
}

int __wrap_vsnprintf(char *str, size_t size, const char *fmt, va_list ap)
{
#ifdef HAL_BOOTLOADER_BUILD
    return chvsnprintf(str, size, fmt, ap);
#else
  return hal.util->vsnprintf(str, size, fmt, ap);
#endif
}

int __wrap_vasprintf(char **strp, const char *fmt, va_list ap)
{
    int len = vsnprintf(NULL, 0, fmt, ap);
    if (len <= 0) {
        return -1;
    }
    char *buf = (char*)calloc(len+1, 1);
    if (!buf) {
        return -1;
    }
    vsnprintf(buf, len+1, fmt, ap);
    (*strp) = buf;
    return len;
}

int __wrap_asprintf(char **strp, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int ret = vasprintf(strp, fmt, ap);
    va_end(ap);
    return ret;
}

int __wrap_vprintf(const char *fmt, va_list arg)
{
#ifdef HAL_STDOUT_SERIAL
  return chvprintf((BaseSequentialStream*)&HAL_STDOUT_SERIAL, fmt, arg);
#elif HAL_USE_SERIAL_USB == TRUE
  usb_initialise();
  return chvprintf((BaseSequentialStream*)&SDU1, fmt, arg);
#else
  (void)arg;
  return strlen(fmt);
#endif
}

// hook to allow for printf() on systems without HAL_STDOUT_SERIAL
int (*vprintf_console_hook)(const char *fmt, va_list arg) = vprintf;

int __wrap_printf(const char *fmt, ...)
{
#ifndef HAL_NO_PRINTF
   va_list arg;
   int done;

   va_start (arg, fmt);
   done =  vprintf_console_hook(fmt, arg);
   va_end (arg);

   return done;
#else
   (void)fmt;
   return 0;
#endif
}

/*
  we assume stdout or stderr. For output to files use the AP_Fileystem
  posix_compat headers
 */
int __wrap_fprintf(void *f, const char *fmt, ...)
{
#ifndef HAL_NO_PRINTF
   va_list arg;
   int done;

   va_start (arg, fmt);
   done =  vprintf_console_hook(fmt, arg);
   va_end (arg);

   return done;
#else
   (void)fmt;
   return 0;
#endif
}

//just a stub for scanf
int __wrap_scanf(const char *fmt, ...)
{
    (void)fmt;
    return 0;
}

extern "C" {
    // alias fiprintf() to fprintf(). This saves flash space
    int __wrap_fiprintf(const char *fmt, ...) __attribute__((alias("__wrap_fprintf")));
}
