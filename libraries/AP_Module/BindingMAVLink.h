// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>

namespace AP_Module
{
int32_t binding_mavlink_init(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_mavlink_register_rx_msgid(void *context, const uint8_t *input, uint32_t input_size,
                                          uint8_t *output, uint32_t output_size);
int32_t binding_mavlink_receive_chan(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_mavlink_send_chan(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_mavlink_block_command(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size);

// Called for every locally routed packet. No module pointers are retained.
void binding_mavlink_handle_message(const mavlink_message_t &msg, mavlink_channel_t chan);

// True when any live Native module owns a block for this command.
bool binding_mavlink_is_handling_command(uint16_t command);

// Called by the manager only after the module worker has joined.
void binding_mavlink_cleanup(void *context);
}
#endif
