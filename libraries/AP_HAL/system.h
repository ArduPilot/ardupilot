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

} // namespace AP_HAL
