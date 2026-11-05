// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_STDOUT_SERIAL)
#include <stdio.h>
#endif
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
#include "Runtime.h"
#include <AP_Radio/driver_sx1280_module.h>
#endif
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

// Reserved migration-prototype ABI slots. The production receiver uses the
// SX1280 executor below; retired implementations live only in tests.
static inline int32_t binding_periph_elrs_shadow(const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}
static inline int32_t binding_periph_elrs_codec_register(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}
static inline int32_t binding_periph_elrs_codec_stats(const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

static inline int32_t binding_periph_elrs_service_register(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}
static inline int32_t binding_periph_elrs_service_stats(const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

static inline int32_t binding_periph_sx1280_register(void *context, const uint8_t *input, uint32_t input_size,
                                                        uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 8 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
    if (BindingWire::read(input, 4) != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint32_t address = BindingWire::read(input + 4, 4);
    if (address == 0) {
        SX1280Module::remove(context);
        output[0] = 1;
    } else {
        void *rw = nullptr;
        if (Runtime::callback_address(context, address, rw)) {
            output[0] = SX1280Module::install(context, reinterpret_cast<SX1280Module::Entry>(address), rw);
        }
    }
    return AP_MODULE_STATUS_OK;
#else
    (void)context;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_periph_sx1280_stats(const uint8_t *input, uint32_t input_size,
                                                     uint8_t *output, uint32_t output_size)
{
    if (input != nullptr || input_size != 0 || output == nullptr || output_size != 20) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
    uint32_t words[5]{};
    SX1280Module::counters(words);
    for (uint8_t i = 0; i < 5; i++) {
        BindingWire::write(output + 4 * i, words[i], 4);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_periph_sx1280_configure(void *context, const uint8_t *input, uint32_t input_size,
                                                       uint8_t *output, uint32_t output_size)
{
    if (!input || input_size != 12 || !output || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
    output[0] = SX1280Module::prepare(context, input);
    return AP_MODULE_STATUS_OK;
#else
    (void)context;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_periph_sx1280_snapshot(void *context, const uint8_t *input, uint32_t input_size,
                                                      uint8_t *output, uint32_t output_size)
{
    if (!input || input_size != 4 || !output || output_size != 16 || BindingWire::read(input, 4) > 240) {
        return AP_MODULE_STATUS_INVALID;
    }
#if defined(AP_MODULE_SX1280_ENABLED) && AP_MODULE_SX1280_ENABLED
    return SX1280Module::snapshot(context, BindingWire::read(input, 4), output) ?
        AP_MODULE_STATUS_OK : AP_MODULE_STATUS_UNAVAILABLE;
#else
    (void)context;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_periph_debug_write(const uint8_t *input, uint32_t input_size,
                                                uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 256 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    const char *text = BindingWire::read_cstring(input);
    if (text == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_STDOUT_SERIAL)
    // Only call on the module worker: STDOUT may wait for UART buffer space.
    printf("%s\r\n", text);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
