// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <AP_Param/AP_Param.h>
#include <cmath>

namespace AP_Module
{

struct BindingParam {
    // AP_Param's by-name setters implicitly convert floats to integers. Reject
    // unrepresentable values before calling them; the int32 upper bound must
    // be exclusive because INT32_MAX rounds up when represented as a float.
    static bool valid_value(const char *name, float value)
    {
        if (!std::isfinite(value)) {
            return false;
        }
        ap_var_type type;
        if (AP_Param::find(name, &type) == nullptr) {
            return true; // preserve the underlying setter's false result
        }
        return valid_value(type, value);
    }

    static bool valid_value(ap_var_type type, float value)
    {
        if (!std::isfinite(value)) {
            return false;
        }
        switch (type) {
        case AP_PARAM_INT8:
            return value >= -128.0f && value <= 127.0f;
        case AP_PARAM_INT16:
            return value >= -32768.0f && value <= 32767.0f;
        case AP_PARAM_INT32:
            return value >= -2147483648.0f && value < 2147483648.0f;
        default:
            return true;
        }
    }
};

}
