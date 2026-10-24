// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_shared_state_open(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_shared_state_read(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_shared_state_write(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_shared_state_close(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);

// Called only after the module worker has joined.
void binding_shared_state_cleanup(void *context);
}
#endif
