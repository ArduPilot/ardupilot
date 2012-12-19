
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#include <stdio.h>
#include <stdarg.h>

int libc_vsnprintf(char* str, size_t size, const char *format, va_list ap) {
    vsnprintf(str, size, format, ap);
}

#include "Util.h"
using namespace AVR_SITL;

int SITLUtil::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int SITLUtil::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}


int SITLUtil::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

int SITLUtil::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
