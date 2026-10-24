// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingCustomMode.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint8_t MAX_HANDLES = 32;

struct CustomModeEntry {
    void *context;
    uint32_t token;
    AP_Vehicle::custom_mode_state *state;
    uint8_t mode_number;
};

static CustomModeEntry registry[MAX_HANDLES];
static HAL_Semaphore registry_sem;
static uint32_t next_token = 1;

static CustomModeEntry *find_entry(void *context, uint32_t token)
{
    if (context == nullptr || token == 0) {
        return nullptr;
    }
    for (auto &entry : registry) {
        if (entry.context == context && entry.token == token && entry.state != nullptr) {
            return &entry;
        }
    }
    return nullptr;
}

static uint32_t allocate_token()
{
    for (uint8_t attempt = 0; attempt <= MAX_HANDLES; attempt++) {
        const uint32_t candidate = next_token++;
        if (next_token == 0) {
            next_token = 1;
        }
        bool live = candidate == 0;
        for (const auto &entry : registry) {
            live |= entry.token == candidate && entry.state != nullptr;
        }
        if (!live) {
            return candidate;
        }
    }
    return 0;
}

static bool valid_text(const uint8_t *text, uint8_t length, uint8_t capacity)
{
    if (length == 0 || length >= capacity) {
        return false;
    }
    for (uint8_t i = 0; i < capacity; i++) {
        if ((i < length && text[i] == 0) || (i >= length && text[i] != 0)) {
            return false;
        }
    }
    return true;
}

int32_t binding_custom_mode_register(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 40 || output == nullptr || output_size != 4 ||
        !valid_text(input + 2, input[1], 32) || !valid_text(input + 35, input[34], 5)) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_SCRIPTING_BINDING_VEHICLE_ENABLED
    auto *vehicle = AP_Vehicle::get_singleton();
    if (vehicle == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    BindingLock scheduler_lock(AP::scheduler().get_semaphore());
    WITH_SEMAPHORE(registry_sem);
    for (const auto &entry : registry) {
        if (entry.state != nullptr && entry.mode_number == input[0] && entry.context != context) {
            return AP_MODULE_STATUS_BUSY;
        }
    }
    auto *state = vehicle->register_custom_mode(input[0], reinterpret_cast<const char *>(input + 2),
                                                reinterpret_cast<const char *>(input + 35), context);
    if (state == nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    CustomModeEntry *free_entry = nullptr;
    for (auto &entry : registry) {
        if (entry.context == context && entry.state == state) {
            BindingWire::write(output, entry.token, 4);
            return AP_MODULE_STATUS_OK;
        }
        if (free_entry == nullptr && entry.state == nullptr) {
            free_entry = &entry;
        }
    }
    const uint32_t token = allocate_token();
    if (free_entry == nullptr || token == 0) {
        state->allow_entry = false;
        state->owner = nullptr;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    state->allow_entry = false;
    *free_entry = {context, token, state, input[0]};
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static int32_t resolve(void *context, const uint8_t *input, uint32_t input_size,
                       AP_Vehicle::custom_mode_state *&state)
{
    state = nullptr;
    if (context == nullptr || input == nullptr || input_size < 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    state = entry->state;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_get_allow_entry(void *context, const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size)
{
    if (output == nullptr || output_size != 1 || input_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    AP_Vehicle::custom_mode_state *state;
    const int32_t status = resolve(context, input, input_size, state);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    output[0] = state->allow_entry ? 1 : 0;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_set_allow_entry(void *context, const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size)
{
    if (output != nullptr || output_size != 0 || input == nullptr || input_size != 5 || input[4] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    AP_Vehicle::custom_mode_state *state;
    const int32_t status = resolve(context, input, input_size, state);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    state->allow_entry = input[4] != 0;
    return AP_MODULE_STATUS_OK;
}

void binding_custom_mode_cleanup(void *context)
{
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(AP::scheduler().get_semaphore());
    WITH_SEMAPHORE(registry_sem);
    for (auto &entry : registry) {
        if (entry.context == context) {
            if (entry.state->owner == context) {
                entry.state->allow_entry = false;
                entry.state->owner = nullptr;
            }
            entry = {};
        }
    }
}

}
#endif
