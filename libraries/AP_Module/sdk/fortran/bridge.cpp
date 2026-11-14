// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>

using ap_module::Context;
using ap_module::ContextBase;
using ap_module::Error;
using ap_module::StringView;

extern "C" int32_t ap_module_fortran_run(const void *context);
extern "C" int32_t ap_module_run(const ContextBase *base)
{
    const Context context(*base);
    return ap_module_fortran_run(&context);
}

extern "C" int64_t ap_module_fortran_time_us(const Context *context)
{
    return int64_t(context->time_us());
}

extern "C" int32_t ap_module_fortran_sleep_ms(const Context *context, int32_t milliseconds)
{
    if (milliseconds < 0) {
        return int32_t(Error::Invalid);
    }
    return int32_t(context->sleep_ms(uint32_t(milliseconds)).error);
}

extern "C" bool ap_module_fortran_stop_requested(const Context *context)
{
    return context->stop_requested();
}

extern "C" int32_t ap_module_fortran_get_gyro(const Context *context, float *gyro)
{
    const auto result = context->ahrs().get_gyro();
    if (result) {
        gyro[0] = result.value.x;
        gyro[1] = result.value.y;
        gyro[2] = result.value.z;
    }
    return int32_t(result.error);
}

extern "C" int32_t ap_module_fortran_text(const Context *context, int32_t severity, const char *message, int32_t length)
{
    if (severity < 0 || severity > 7 || length < 0 || length > 255 ||
        (message == nullptr && length != 0)) {
        return int32_t(Error::Invalid);
    }
    for (int32_t i = 0; i < length; i++) {
        if (message[i] == 0) {
            return int32_t(Error::Invalid);
        }
    }
    // Core diagnostics must also work in modules-only firmware without GCS.
    return int32_t(context->text(uint8_t(severity), StringView(message, size_t(length))).error);
}

extern "C" int32_t ap_module_fortran_text_floats(const Context *context, int32_t severity, const char *format,
                                               int32_t length, const float *values, int32_t count)
{
    if (severity < 0 || severity > 7 || length < 0 || length > 255 || count < 0 || count > 8 ||
        (format == nullptr && length != 0) || (values == nullptr && count != 0)) {
        return int32_t(Error::Invalid);
    }
    ap_module::PrintfPacket request{};
    for (int32_t i = 0; i < length; i++) {
        if (format[i] == 0) {
            return int32_t(Error::Invalid);
        }
        request.format.data[i] = uint8_t(format[i]);
    }
    request.count = uint8_t(count);
    for (int32_t i = 0; i < count; i++) {
        request.tags.data[i] = 5; // FLOAT64 in the shared string.format_printf wire contract.
        const double promoted = values[i];
        const auto *bytes = reinterpret_cast<const uint8_t *>(&promoted);
        for (size_t b = 0; b < sizeof(promoted); b++) {
            reinterpret_cast<uint8_t *>(&request.values.data[i])[b] = bytes[b];
        }
    }
    const auto text = context->string().format_printf(request);
    if (!text) {
        return int32_t(text.error);
    }
    if (text.value.length > 255) {
        return int32_t(Error::Host);
    }
    return ap_module_fortran_text(context, severity, reinterpret_cast<const char *>(text.value.text.data), text.value.length);
}
