
#ifndef __AP_HAL_AVR_UTIL_H__
#define __AP_HAL_AVR_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRUtil : public AP_HAL::Util {
public:
    int snprintf(char* str, size_t size, const char *format, ...);
    int snprintf_P(char* str, size_t size, const prog_char_t *format, ...);
    int vsnprintf(char* str, size_t size, const char *format, va_list ap);
    int vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap);
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

#endif // __AP_HAL_AVR_UTIL_H__
