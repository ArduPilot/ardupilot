// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#include <stdint.h>

#if AP_MODULE_ENABLED
namespace AP_Module
{
int32_t binding_filesystem_stat(const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_read_at(const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_dir_entry(const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_truncate(const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_append(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_unlink(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_filesystem_format(const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
}
#endif
