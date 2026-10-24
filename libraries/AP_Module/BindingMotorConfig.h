// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{

int32_t binding_motors_dynamic_init(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_dynamic_add_motor(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_dynamic_load_factors(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_matrix_init(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_matrix_add_motor_raw(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_matrix_set_throttle_factor(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_6dof_init(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_6dof_add_motor(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
int32_t binding_motors_set_frame_string(const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);

}
#endif // AP_MODULE_ENABLED
