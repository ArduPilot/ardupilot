// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_can_get_device(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_can_get_device2(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_can_read_frame(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_can_write_frame(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_can_add_filter(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_can_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size);
// Called by the manager only after the module worker has joined.
void binding_can_cleanup(void *context);
#if defined(AP_MODULE_BINDING_TESTING)
void binding_can_test_set_next_token(uint32_t token);
#endif
}
#endif
