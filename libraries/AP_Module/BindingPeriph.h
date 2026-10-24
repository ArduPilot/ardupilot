// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#if defined(HAL_BUILD_AP_PERIPH)
#include <../Tools/AP_Periph/AP_Periph.h>
#endif

namespace AP_Module
{

static inline int32_t binding_periph_get_vehicle_state(const uint8_t *input, uint32_t input_size,
                                                        uint8_t *output, uint32_t output_size)
{
    if (input != nullptr || input_size != 0 || output == nullptr || output_size != 8) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if defined(HAL_BUILD_AP_PERIPH) && AP_PERIPH_NOTIFY_ENABLED
    auto *periph = AP_Periph_FW::get_singleton();
    if (periph == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const uint64_t value = periph->get_vehicle_state();
    BindingWire::write(output, uint32_t(value), 4);
    BindingWire::write(output + 4, uint32_t(value >> 32), 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
