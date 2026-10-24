// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_serialled_set_num_neopixel(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size);
int32_t binding_serialled_set_num_neopixel_rgb(void *context, const uint8_t *input, uint32_t input_size,
                                               uint8_t *output, uint32_t output_size);
int32_t binding_serialled_set_num_profiled(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size);

// Called by the manager only after the module worker has joined. Driver memory
// cannot be reclaimed; cleanup releases only this module's channel claims.
void binding_serialled_cleanup(void *context);
}
#endif
