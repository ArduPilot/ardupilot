// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_crsf_add_menu(void *context, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size);
int32_t binding_crsf_menu_add_menu(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_crsf_menu_add_parameter(void *context, const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size);
int32_t binding_crsf_parameter_id(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_crsf_parameter_data(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size);
int32_t binding_crsf_peek_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_crsf_pop_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size);
int32_t binding_crsf_send_write_response(void *context, const uint8_t *input, uint32_t input_size,
                                         uint8_t *output, uint32_t output_size);
int32_t binding_crsf_send_response(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_crsf_get_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size);
int32_t binding_crsf_menu_id(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_crsf_menu_name(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_crsf_menu_num_params(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);

// Called only after the module worker joins. It prunes queued parameter
// pointers before deleting menus and invalidating opaque handles.
void binding_crsf_cleanup(void *context);
}
#endif
