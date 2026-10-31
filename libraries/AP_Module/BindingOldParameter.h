// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Param/AP_Param.h>

namespace AP_Module
{

template <typename T>
static bool binding_old_parameter_value(const AP_Param::ConversionInfo &info, float &result)
{
    T value;
    if (!AP_Param::find_old_parameter(&info, &value)) {
        return false;
    }
    result = float(value.get());
    return true;
}

static inline int32_t binding_parameter_old_value(const uint8_t *input, uint32_t input_size,
                                                   uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 7 || output == nullptr || output_size != 5 ||
        input[6] < AP_PARAM_INT8 || input[6] > AP_PARAM_FLOAT) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const auto type = static_cast<ap_var_type>(input[6]);
    const AP_Param::ConversionInfo info = {
        uint16_t(BindingWire::read(input, 2)),
        BindingWire::read(input + 2, 4),
        type,
        nullptr,
    };
    float value = 0;
    bool found = false;
    switch (type) {
    case AP_PARAM_INT8:
        found = binding_old_parameter_value<AP_Int8>(info, value);
        break;
    case AP_PARAM_INT16:
        found = binding_old_parameter_value<AP_Int16>(info, value);
        break;
    case AP_PARAM_INT32:
        found = binding_old_parameter_value<AP_Int32>(info, value);
        break;
    case AP_PARAM_FLOAT:
        found = binding_old_parameter_value<AP_Float>(info, value);
        break;
    default:
        return AP_MODULE_STATUS_INVALID;
    }
    if (found) {
        output[0] = 1;
        BindingWire::write_float(output + 1, value);
    }
    return AP_MODULE_STATUS_OK;
}

}
#endif
