
#ifndef __AP_HAL_UTILITY_VPRINTF_H__
#define __AP_HAL_UTILITY_VPRINTF_H__

#include <AP_HAL.h>
#include <stdarg.h>

void print_vprintf (AP_HAL::Print *s, unsigned char in_progmem, const char *fmt, va_list ap);


#endif //__AP_HAL_UTILITY_VPRINTF_H__
