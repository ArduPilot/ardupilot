#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

void init();

void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;

uint32_t micros();
uint32_t millis();
uint16_t millis16();
uint64_t micros64();
uint64_t millis64();

uint32_t native_micros();
uint32_t native_millis();
uint16_t native_millis16();
uint64_t native_micros64();
uint64_t native_millis64();

void dump_stack_trace();

} // namespace AP_HAL
