// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#include <stdint.h>

#if AP_MODULE_ENABLED
namespace AP_Module
{
int32_t binding_network_open(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_network_close(void *context, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size);
int32_t binding_network_bind(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_network_connect(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_network_listen(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_network_set_blocking(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_network_is_connected(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size);
int32_t binding_network_pollin(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_network_pollout(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);
int32_t binding_network_accept(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size);
int32_t binding_network_recv(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_network_send(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size);
int32_t binding_network_sendfile(void *context, const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size);
int32_t binding_network_recv_from(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size);
int32_t binding_network_is_pending(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size);
int32_t binding_network_reuse_address(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size);
int32_t binding_network_send_to(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size);

// Called by the manager only after the module worker has joined.
void binding_network_cleanup(void *context);
}
#endif
