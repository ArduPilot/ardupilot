#pragma once

#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap);
