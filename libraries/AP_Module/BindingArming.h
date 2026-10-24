// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_arming_get_aux_auth_id(void *context, const uint8_t *input, uint32_t input_size,
                                       uint8_t *output, uint32_t output_size);
int32_t binding_arming_set_aux_auth_passed(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size);
int32_t binding_arming_set_aux_auth_failed(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size);

// Called by the manager only after the module worker has joined.
void binding_arming_cleanup(void *context);
}
#endif
