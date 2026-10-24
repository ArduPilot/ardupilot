// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#include <stdint.h>

#if AP_MODULE_ENABLED
namespace AP_Module
{
int32_t binding_file_read_at(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
}
#endif
