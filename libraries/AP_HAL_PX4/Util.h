
#ifndef __AP_HAL_PX4_UTIL_H__
#define __AP_HAL_PX4_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_PX4_Namespace.h"

class PX4::PX4Util : public AP_HAL::Util {
public:
    int snprintf(char* str, size_t size, const char *format, ...);
    int snprintf_P(char* str, size_t size, const prog_char_t *format, ...);
    int vsnprintf(char* str, size_t size, const char *format, va_list ap);
    int vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap);
    bool run_debug_shell(AP_HAL::BetterStream *stream);
};

#endif // __AP_HAL_PX4_UTIL_H__
