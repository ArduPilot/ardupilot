// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#if AP_MODULE_ENABLED
#include <AP_Module/BindingIds.h>
#include <stdint.h>

namespace AP_Module
{
// Load-time hash lookup only. UINT32_MAX means no compatible public contract.
uint32_t binding_resolve(void *context, uint64_t hash);
// Firmware-only dispatcher. Non-overlapping buffers are borrowed for this call,
// never retained. These trusted native pointers are not sandbox-validated.
// Module validates its stop token before forwarding to this entry point.
int32_t binding_dispatch(void *context, BindingId id, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size);
int32_t binding_math_dispatch(BindingId id, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size);
}
#endif
