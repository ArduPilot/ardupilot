// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingONVIF.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_ONVIF/AP_ONVIF.h>

namespace AP_Module
{

static constexpr uint16_t STRING_WIRE_SIZE = 256;
static constexpr uint16_t START_WIRE_SIZE = 3 * STRING_WIRE_SIZE;
#if ENABLE_ONVIF == 1
static HAL_Semaphore onvif_sem;
static void *onvif_owner;

AP_HAL::Semaphore &binding_onvif_semaphore()
{
    return onvif_sem;
}

int32_t binding_onvif_check_owner(void *context)
{
    if (context == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (onvif_owner == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    return onvif_owner == context ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_BUSY;
}
#endif

int32_t binding_onvif_start(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != START_WIRE_SIZE ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *username = BindingWire::read_cstring(input);
    const char *password = BindingWire::read_cstring(input + STRING_WIRE_SIZE);
    const char *hostname = BindingWire::read_cstring(input + 2 * STRING_WIRE_SIZE);
    if (username == nullptr || password == nullptr || hostname == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
#if ENABLE_ONVIF == 1
    BindingLock lock(onvif_sem);
    if (onvif_owner != nullptr && onvif_owner != context) {
        return AP_MODULE_STATUS_BUSY;
    }
    auto *onvif = AP_ONVIF::get_singleton();
    if (onvif == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    // start() allocates and mutates the singleton even on many false paths.
    // Claim before entering it so another module cannot inherit partial state.
    onvif_owner = context;
    output[0] = onvif->start(username, password, hostname);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

void binding_onvif_cleanup(void *context)
{
#if ENABLE_ONVIF == 1
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(onvif_sem);
    if (onvif_owner == context) {
        onvif_owner = nullptr;
    }
#else
    (void)context;
#endif
}

}
#endif
