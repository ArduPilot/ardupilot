// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingArming.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Arming/AP_Arming_config.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint8_t FAILURE_MESSAGE_MAX = 41;
static constexpr uint8_t FAILURE_WIRE_SIZE = 2 + FAILURE_MESSAGE_MAX;

int32_t binding_arming_get_aux_auth_id(void *context, const uint8_t *input, uint32_t input_size,
                                       uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr || output_size != 2) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_ARMING_ENABLED && AP_ARMING_AUX_AUTH_ENABLED
    auto *arming = AP_Arming::get_singleton();
    uint8_t auth_id;
    if (arming != nullptr && arming->get_aux_auth_id(auth_id, context)) {
        output[0] = 1;
        output[1] = auth_id;
    }
    return arming == nullptr ? AP_MODULE_STATUS_UNAVAILABLE : AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_arming_set_aux_auth_passed(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 1 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_ARMING_ENABLED && AP_ARMING_AUX_AUTH_ENABLED
    auto *arming = AP_Arming::get_singleton();
    if (arming == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    return arming->set_aux_auth_passed(input[0], context) ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_INVALID;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_arming_set_aux_auth_failed(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != FAILURE_WIRE_SIZE ||
        output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint8_t length = input[1];
    if (length > FAILURE_MESSAGE_MAX || memchr(input + 2, 0, length) != nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint8_t i = length; i < FAILURE_MESSAGE_MAX; i++) {
        if (input[2 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    char message[FAILURE_MESSAGE_MAX + 1] {};
    memcpy(message, input + 2, length);
#if AP_ARMING_ENABLED && AP_ARMING_AUX_AUTH_ENABLED
    auto *arming = AP_Arming::get_singleton();
    if (arming == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    return arming->set_aux_auth_failed(input[0], message, context) ?
           AP_MODULE_STATUS_OK : AP_MODULE_STATUS_INVALID;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

void binding_arming_cleanup(void *context)
{
#if AP_ARMING_ENABLED && AP_ARMING_AUX_AUTH_ENABLED
    auto *arming = AP_Arming::get_singleton();
    if (context != nullptr && arming != nullptr) {
        arming->release_aux_auths(context);
    }
#else
    (void)context;
#endif
}

}
#endif
