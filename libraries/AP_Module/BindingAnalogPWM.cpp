// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingAnalogPWM.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace AP_Module
{

static constexpr uint8_t MAX_SOURCES_PER_MODULE = 4;
static constexpr uint8_t MAX_HANDLES = 32; // Global source budget, not a module-count limit.

struct AnalogEntry {
    void *context;
    uint32_t token;
    AP_HAL::AnalogSource *source;
};

struct PWMEntry {
    void *context;
    uint32_t token;
    AP_HAL::PWMSource *source;
};

static AnalogEntry analog_registry[MAX_HANDLES];
static PWMEntry pwm_registry[MAX_HANDLES];
static HAL_Semaphore registry_sem;
static uint32_t next_analog_token = 1;
static uint32_t next_pwm_token = 1;

template <typename Entry>
static Entry *find_entry(Entry (&registry)[MAX_HANDLES], void *context, uint32_t token)
{
    if (context == nullptr || token == 0) {
        return nullptr;
    }
    for (auto &entry : registry) {
        if (entry.context == context && entry.token == token && entry.source != nullptr) {
            return &entry;
        }
    }
    return nullptr;
}

template <typename Entry>
static uint32_t allocate_token(Entry (&registry)[MAX_HANDLES], uint32_t &next_token)
{
    for (uint8_t attempt = 0; attempt <= MAX_HANDLES; attempt++) {
        const uint32_t candidate = next_token++;
        if (next_token == 0) {
            next_token = 1;
        }
        bool live = candidate == 0;
        for (const auto &entry : registry) {
            live |= entry.token == candidate && entry.context != nullptr;
        }
        if (!live) {
            return candidate;
        }
    }
    return 0;
}

int32_t binding_analog_open(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if !defined(HAL_DISABLE_ADC_DRIVER)
    if (hal.analogin == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    WITH_SEMAPHORE(registry_sem);
    AnalogEntry *free_entry = nullptr;
    uint8_t owned = 0;
    for (auto &entry : analog_registry) {
        if (entry.context == context) {
            owned++;
        } else if (free_entry == nullptr && entry.context == nullptr) {
            free_entry = &entry;
        }
    }
    if (owned >= MAX_SOURCES_PER_MODULE || free_entry == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    const uint32_t token = allocate_token(analog_registry, next_analog_token);
    if (token == 0) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    if (free_entry->source == nullptr) {
        free_entry->source = hal.analogin->channel(ANALOG_INPUT_NONE);
        if (free_entry->source == nullptr) {
            return AP_MODULE_STATUS_UNAVAILABLE;
        }
    } else {
        const bool pin_reset = free_entry->source->set_pin(ANALOG_INPUT_NONE);
        (void)pin_reset;
    }
    free_entry->context = context;
    free_entry->token = token;
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_analog_set_pin(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 5 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if !defined(HAL_DISABLE_ADC_DRIVER)
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(analog_registry, context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->source->set_pin(input[4]) ? 1 : 0;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

enum class AnalogReading : uint8_t {
    AVERAGE,
    LATEST,
    RATIOMETRIC,
};

static int32_t analog_read(void *context, const uint8_t *input, uint32_t input_size,
                           uint8_t *output, uint32_t output_size, AnalogReading reading)
{
    if (input == nullptr || input_size != 4 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if !defined(HAL_DISABLE_ADC_DRIVER)
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(analog_registry, context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    float value;
    switch (reading) {
    case AnalogReading::AVERAGE:
        value = entry->source->voltage_average();
        break;
    case AnalogReading::LATEST:
        value = entry->source->voltage_latest();
        break;
    case AnalogReading::RATIOMETRIC:
        value = entry->source->voltage_average_ratiometric();
        break;
    }
    BindingWire::write_float(output, value);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_analog_voltage_average(void *context, const uint8_t *input, uint32_t input_size,
                                       uint8_t *output, uint32_t output_size)
{
    return analog_read(context, input, input_size, output, output_size, AnalogReading::AVERAGE);
}

int32_t binding_analog_voltage_latest(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size)
{
    return analog_read(context, input, input_size, output, output_size, AnalogReading::LATEST);
}

int32_t binding_analog_voltage_average_ratiometric(void *context, const uint8_t *input, uint32_t input_size,
                                                   uint8_t *output, uint32_t output_size)
{
    return analog_read(context, input, input_size, output, output_size, AnalogReading::RATIOMETRIC);
}

int32_t binding_pwm_open(void *context, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    WITH_SEMAPHORE(registry_sem);
    PWMEntry *free_entry = nullptr;
    uint8_t owned = 0;
    for (auto &entry : pwm_registry) {
        if (entry.context == context && entry.source != nullptr) {
            owned++;
        } else if (free_entry == nullptr && entry.source == nullptr) {
            free_entry = &entry;
        }
    }
    if (owned >= MAX_SOURCES_PER_MODULE || free_entry == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    const uint32_t token = allocate_token(pwm_registry, next_pwm_token);
    if (token == 0) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *source = NEW_NOTHROW AP_HAL::PWMSource();
    if (source == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    *free_entry = {context, token, source};
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_pwm_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(pwm_registry, context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    delete entry->source;
    *entry = {};
    return AP_MODULE_STATUS_OK;
}

int32_t binding_pwm_set_pin(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 5 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(pwm_registry, context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->source->set_pin(input[4], "RustScripting") ? 1 : 0;
    return AP_MODULE_STATUS_OK;
}

static int32_t pwm_read(void *context, const uint8_t *input, uint32_t input_size,
                        uint8_t *output, uint32_t output_size, bool average)
{
    if (input == nullptr || input_size != 4 || output == nullptr || output_size != 2) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(pwm_registry, context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint16_t value = average ? entry->source->get_pwm_avg_us() : entry->source->get_pwm_us();
    BindingWire::write(output, value, 2);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_pwm_get_us(void *context, const uint8_t *input, uint32_t input_size,
                           uint8_t *output, uint32_t output_size)
{
    return pwm_read(context, input, input_size, output, output_size, false);
}

int32_t binding_pwm_get_average_us(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    return pwm_read(context, input, input_size, output, output_size, true);
}

void binding_analog_pwm_cleanup(void *context)
{
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    for (auto &entry : analog_registry) {
        if (entry.context == context) {
#if !defined(HAL_DISABLE_ADC_DRIVER)
            const bool pin_reset = entry.source->set_pin(ANALOG_INPUT_NONE);
            (void)pin_reset;
#endif
            entry.context = nullptr;
            entry.token = 0;
        }
    }
    for (auto &entry : pwm_registry) {
        if (entry.context == context) {
            delete entry.source;
            entry = {};
        }
    }
}

}
#endif
