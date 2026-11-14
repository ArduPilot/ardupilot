// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_custom_mode_register(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_custom_mode_get_allow_entry(void *context, const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size);
int32_t binding_custom_mode_set_allow_entry(void *context, const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size);
void binding_custom_mode_cleanup(void *context);
}
#endif
