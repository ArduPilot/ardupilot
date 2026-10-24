// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_analog_open(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size);
int32_t binding_analog_set_pin(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_analog_voltage_average(void *context, const uint8_t *input, uint32_t input_size,
                                       uint8_t *output, uint32_t output_size);
int32_t binding_analog_voltage_latest(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size);
int32_t binding_analog_voltage_average_ratiometric(void *context, const uint8_t *input, uint32_t input_size,
                                                   uint8_t *output, uint32_t output_size);
int32_t binding_pwm_open(void *context, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size);
int32_t binding_pwm_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size);
int32_t binding_pwm_set_pin(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size);
int32_t binding_pwm_get_us(void *context, const uint8_t *input, uint32_t input_size,
                           uint8_t *output, uint32_t output_size);
int32_t binding_pwm_get_average_us(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
void binding_analog_pwm_cleanup(void *context);
}
#endif
