
#ifndef __AP_HAL_SMACCM_UTIL_H__
#define __AP_HAL_SMACCM_UTIL_H__

#include <AP_HAL.h>
#include "AP_HAL_SMACCM_Namespace.h"

class SMACCM::SMACCMUtil : public AP_HAL::Util {
public:
    int snprintf(char* str, size_t size, const char *format, ...);
    int snprintf_P(char* str, size_t size, const prog_char_t *format, ...);
    int vsnprintf(char* str, size_t size, const char *format, va_list ap);
    int vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap);
};

#endif // __AP_HAL_SMACCM_UTIL_H__
