// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingSerialLED.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_SerialLED/AP_SerialLED.h>

namespace AP_Module
{

static constexpr uint8_t CHANNELS = 16;
static constexpr uint8_t MAX_LEDS = 128;
#if AP_SERIALLED_ENABLED
static HAL_Semaphore serialled_sem;
static void *serialled_owners[CHANNELS];
#endif

enum class SerialLEDSetup : uint8_t {
    Neopixel,
    NeopixelRGB,
    Profiled,
};

static int32_t serialled_setup(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size, SerialLEDSetup setup)
{
    if (context == nullptr || input == nullptr || input_size != 2 || output == nullptr || output_size != 1 ||
        input[0] < 1 || input[0] > CHANNELS || input[1] > MAX_LEDS) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_SERIALLED_ENABLED
    static_assert(AP_SERIALLED_MAX_LEDS == MAX_LEDS, "update Rust serial-LED wire cap");
    BindingLock lock(serialled_sem);
    void *&owner = serialled_owners[input[0] - 1U];
    if (owner != nullptr && owner != context) {
        return AP_MODULE_STATUS_BUSY;
    }
    auto *serialled = AP_SerialLED::get_singleton();
    if (serialled == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    bool configured = false;
    switch (setup) {
    case SerialLEDSetup::Neopixel:
        configured = serialled->set_num_neopixel(input[0], input[1]);
        break;
    case SerialLEDSetup::NeopixelRGB:
        configured = serialled->set_num_neopixel_rgb(input[0], input[1]);
        break;
    case SerialLEDSetup::Profiled:
        configured = serialled->set_num_profiled(input[0], input[1]);
        break;
    }
    output[0] = configured;
    if (configured) {
        owner = context;
    }
    return AP_MODULE_STATUS_OK;
#else
    (void)setup;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_serialled_set_num_neopixel(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size)
{
    return serialled_setup(context, input, input_size, output, output_size, SerialLEDSetup::Neopixel);
}

int32_t binding_serialled_set_num_neopixel_rgb(void *context, const uint8_t *input, uint32_t input_size,
                                               uint8_t *output, uint32_t output_size)
{
    return serialled_setup(context, input, input_size, output, output_size, SerialLEDSetup::NeopixelRGB);
}

int32_t binding_serialled_set_num_profiled(void *context, const uint8_t *input, uint32_t input_size,
                                           uint8_t *output, uint32_t output_size)
{
    return serialled_setup(context, input, input_size, output, output_size, SerialLEDSetup::Profiled);
}

void binding_serialled_cleanup(void *context)
{
#if AP_SERIALLED_ENABLED
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(serialled_sem);
    for (void *&owner : serialled_owners) {
        if (owner == context) {
            owner = nullptr;
        }
    }
#else
    (void)context;
#endif
}

}
#endif
