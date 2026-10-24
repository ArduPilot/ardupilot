// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingPrintf.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace AP_Module
{
int32_t binding_string_format_printf(const uint8_t *input, uint32_t, uint8_t *output, uint32_t)
{
    // Fixed 585-byte input / 258-byte output buffers are guaranteed by the
    // SDK wrapper and resolved wire-contract hash, as for generated bindings.
    const char *format = BindingWire::read_cstring(input);
    const uint8_t count = input[256];
    using Arg = AP_HAL::Util::FormatArg;
    if (format == nullptr || count > AP_HAL::Util::FORMAT_MAX_ARGS) {
        return AP_MODULE_STATUS_INVALID;
    }
    Arg arguments[AP_HAL::Util::FORMAT_MAX_ARGS];
    for (uint8_t i = 0; i < count; i++) {
        Arg &arg = arguments[i];
        arg.type = static_cast<Arg::Type>(input[257 + i]);
        uint64_t bits = 0;
        for (uint8_t b = 0; b < 8; b++) {
            bits |= uint64_t(input[265 + i * 8 + b]) << (b * 8);
        }
        switch (arg.type) {
        case Arg::Type::INT32: {
            const uint32_t low = uint32_t(bits);
            memcpy(&arg.i32, &low, sizeof(low));
            break;
        }
        case Arg::Type::UINT32:
            arg.u32 = uint32_t(bits);
            break;
        case Arg::Type::INT64:
            memcpy(&arg.i64, &bits, sizeof(bits));
            break;
        case Arg::Type::UINT64:
            arg.u64 = bits;
            break;
        case Arg::Type::FLOAT64:
            memcpy(&arg.real, &bits, sizeof(bits));
            break;
        case Arg::Type::STRING:
            if (bits >= 256 || memchr(input + 329 + bits, 0, 256 - bits) == nullptr) {
                return AP_MODULE_STATUS_INVALID;
            }
            arg.text = reinterpret_cast<const char *>(input + 329 + bits);
            break;
        default:
            return AP_MODULE_STATUS_INVALID;
        }
    }
    char *text = reinterpret_cast<char *>(output + 2);
    const int length = hal.util->snprintf_args(text, 256, format, arguments, count);
    if (length < 0 || length > 255) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingWire::write(output, uint16_t(length), 2);
    return AP_MODULE_STATUS_OK;
}
}
#endif
