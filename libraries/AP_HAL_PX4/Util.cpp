
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <stdio.h>
#include <stdarg.h>

static int libc_vsnprintf(char* str, size_t size, const char *format, va_list ap) 
{
    return vsnprintf(str, size, format, ap);
}

#include "Util.h"
using namespace PX4;

int PX4Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int PX4Util::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}


int PX4Util::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

int PX4Util::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
