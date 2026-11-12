// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingCustomMode.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <math.h>
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

// resolve a handle whose state is still owned by context; caller holds the scheduler lock
static AP_Vehicle::custom_mode_state *owned(void *context, const uint8_t *input, uint32_t input_size)
{
    AP_Vehicle::custom_mode_state *state;
    if (resolve(context, input, input_size, state) != AP_MODULE_STATUS_OK || state->owner != context) {
        return nullptr;
    }
    return state;
}

int32_t binding_custom_mode_set_request(void *context, const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    using Request = AP_Vehicle::custom_mode_state::Request;
    if (input == nullptr || input_size != 9 || output == nullptr || output_size != 2 ||
        input[4] > uint8_t(Request::LOITER)) {
        return AP_MODULE_STATUS_INVALID;
    }
    const float takeoff_alt_m = BindingWire::read_float(input + 5);
    if (!isfinite(takeoff_alt_m)) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    // requests only apply to the running mode with a live owner
    if (!state->active || state->owner_lost) {
        return AP_MODULE_STATUS_OK;
    }
    state->generation = state->generation == UINT16_MAX ? 1 : state->generation + 1;
    state->request = Request(input[4]);
    state->takeoff_alt_m = takeoff_alt_m;
    BindingWire::write(output, state->generation, 2);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_status(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (output == nullptr || output_size != 10 || input_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    BindingLock lock(AP::scheduler().get_semaphore());
    const auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = uint8_t(state->request);
    BindingWire::write(output + 1, state->generation, 2);
    BindingWire::write(output + 3, state->applied_generation, 2);
    output[5] = state->init_ok;
    output[6] = state->complete;
    output[7] = state->active;
    output[8] = state->takeoff_event;
    output[9] = state->owner_lost;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_set_options(void *context, const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    using State = AP_Vehicle::custom_mode_state;
    if (output != nullptr || output_size != 0 || input == nullptr || input_size != 5 ||
        (input[4] & ~(State::NO_EXTERNAL_GUIDED | State::EKF_FS_DISARM | State::ARMING_GATED)) != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    state->options = input[4];
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_set_arming_ready(void *context, const uint8_t *input, uint32_t input_size,
                                             uint8_t *output, uint32_t output_size)
{
    if (output != nullptr || output_size != 0 || input == nullptr || input_size != 5 || input[4] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    state->arming_ready = input[4] != 0;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_set_supervision(void *context, const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size)
{
    if (output != nullptr || output_size != 0 || input == nullptr || input_size != 6) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    state->keepalive_ms = AP_HAL::millis();
    state->supervision_ms = BindingWire::read(input + 4, 2);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_keepalive(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size)
{
    if (output != nullptr || output_size != 0 || input_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    state->keepalive_ms = AP_HAL::millis();
    return AP_MODULE_STATUS_OK;
}

int32_t binding_custom_mode_ack_takeoff_event(void *context, const uint8_t *input, uint32_t input_size,
                                              uint8_t *output, uint32_t output_size)
{
    if (output == nullptr || output_size != 1 || input_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = state->takeoff_event;
    state->takeoff_event = false;
    return AP_MODULE_STATUS_OK;
}

// arming can delay while the log opens, so it is done on the main thread;
// arm false cancels a pending request
int32_t binding_custom_mode_request_arm(void *context, const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || output_size != 1 || input_size != 5 || input[4] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    BindingLock lock(AP::scheduler().get_semaphore());
    auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (input[4] == 0) {
        state->arm_request = false;
        output[0] = 1;
        return AP_MODULE_STATUS_OK;
    }
    const bool ok = state->active && !state->owner_lost;
    state->arm_request |= ok;
    output[0] = ok;
    return AP_MODULE_STATUS_OK;
}

// handle, NED pos/vel/accel, use_yaw, yaw_deg, use_yaw_rate, yaw_rate_degs, yaw_relative
int32_t binding_custom_mode_set_target_posvelaccel(void *context, const uint8_t *input, uint32_t input_size,
                                                   uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 51 || output == nullptr || output_size != 1 ||
        input[40] > 1 || input[45] > 1 || input[50] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    float v[11];
    for (uint8_t i = 0; i < 9; i++) {
        v[i] = BindingWire::read_float(input + 4 + 4*i);
    }
    v[9] = BindingWire::read_float(input + 41);
    v[10] = BindingWire::read_float(input + 46);
    for (const float f : v) {
        if (!isfinite(f)) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    output[0] = 0;
#if AP_SCRIPTING_BINDING_VEHICLE_ENABLED
    auto *vehicle = AP_Vehicle::get_singleton();
    if (vehicle == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    BindingLock lock(AP::scheduler().get_semaphore());
    const auto *state = owned(context, input, input_size);
    if (state == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = !state->owner_lost &&
                vehicle->custom_mode_set_target_posvelaccel_NED(state, Vector3f(v[0], v[1], v[2]), Vector3f(v[3], v[4], v[5]),
                                                                Vector3f(v[6], v[7], v[8]), input[40] != 0, v[9],
                                                                input[45] != 0, v[10], input[50] != 0);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
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
                entry.state->arm_request = false;
                // supervised modes handle the loss on the main thread
                entry.state->owner_loss_event = entry.state->supervision_ms != 0;
            }
            entry = {};
        }
    }
}

}
#endif
