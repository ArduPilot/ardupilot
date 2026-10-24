// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#include <stdint.h>

#if AP_MODULE_ENABLED
namespace AP_Module
{
int32_t binding_serial_find(const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size);
int32_t binding_serial_find_simulated(const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size);
int32_t binding_serial_begin(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_serial_configure_parity(const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size);
int32_t binding_serial_set_stop_bits(const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_serial_set_unbuffered_writes(const uint8_t *input, uint32_t input_size,
                                             uint8_t *output, uint32_t output_size);
int32_t binding_serial_set_flow_control(const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size);
int32_t binding_serial_available(const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size);
int32_t binding_serial_read(const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size);
int32_t binding_serial_write(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
}
#endif
