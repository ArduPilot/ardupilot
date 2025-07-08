#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

void init();

void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;

uint16_t micros16();
uint32_t micros();
uint32_t millis();
uint16_t millis16();
uint64_t micros64();
uint64_t millis64();

void dump_stack_trace();
void dump_core_file();

// Function to reliably determine whether a timeout has expired using any unsigned time type and interval
// The template makes sure that callers do not mix up different types for storing time or assessing the
// current time.
// The comparison is guaranteed to work even if the present and the past cross a wrap boundary
template <typename T, typename S, typename R>
inline bool timeout_expired(const T past_time, const S now, const R timeout)
{
    static_assert(std::is_same<T, S>::value, "timeout_expired() must compare values of the same unsigned type");
    static_assert(std::is_unsigned<T>::value, "timeout_expired() must use unsigned times");
    static_assert(std::is_unsigned<R>::value, "timeout_expired() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout);
}

// Function to reliably determine whether how much of a timeout is left using any unsigned time type and interval
// The template makes sure that callers do not mix up different types for storing time or assessing the
// current time.
// The comparison is guaranteed to work even if the present and the past cross a wrap boundary
template <typename T, typename S, typename R>
inline T timeout_remaining(const T past_time, const S now, const R timeout)
{
    static_assert(std::is_same<T, S>::value, "timeout_remaining() must compare values of the same unsigned type");
    static_assert(std::is_unsigned<T>::value, "timeout_remaining() must use unsigned times");
    static_assert(std::is_unsigned<R>::value, "timeout_remaining() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout) ? T(0) : (timeout - dt);
}

} // namespace AP_HAL
