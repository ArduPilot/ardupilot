// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{

int32_t binding_airspeed_get_backend(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_airspeed_script_value(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size, uint8_t operation);

}
#endif // AP_MODULE_ENABLED
