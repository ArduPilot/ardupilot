// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_i2c_open(void *context, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size);
int32_t binding_i2c_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size);
int32_t binding_i2c_set_retries(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_i2c_set_address(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_i2c_write_register(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_i2c_read_registers(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_i2c_transfer(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);

// Called by the manager only after the module worker has joined.
void binding_i2c_cleanup(void *context);
}
#endif
