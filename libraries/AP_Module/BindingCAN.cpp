// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingCAN.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Scripting/AP_Scripting_CANSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint8_t MAX_BUFFER_LENGTH = 25;
static constexpr uint8_t FRAME_DATA_SIZE = 64;
static constexpr uint8_t FRAME_WIRE_SIZE = 4 + 1 + 1 + FRAME_DATA_SIZE;

#if AP_SCRIPTING_CAN_SENSOR_ENABLED
static constexpr uint8_t MAX_BUFFERS = 16;
static constexpr uint32_t MAX_TIMEOUT_US = 1000000U;
struct CANRegistryEntry {
    void *context;
    uint32_t token;
    ScriptingCANSensor *sensor;
    ScriptingCANBuffer *buffer;
};

static CANRegistryEntry registry[MAX_BUFFERS];
static HAL_Semaphore registry_sem;
static uint32_t next_token = 1;

static CANRegistryEntry *find_entry(void *context, uint32_t token)
{
    for (auto &entry : registry) {
        if (entry.context == context && entry.token == token && entry.buffer != nullptr) {
            return &entry;
        }
    }
    return nullptr;
}

static bool allocate_token(uint32_t &token)
{
    for (uint8_t attempts = 0; attempts <= MAX_BUFFERS;) {
        const uint32_t candidate = next_token++;
        if (candidate == 0) {
            continue;
        }
        attempts++;
        bool used = false;
        for (const auto &entry : registry) {
            if (entry.buffer != nullptr && entry.token == candidate) {
                used = true;
                break;
            }
        }
        if (!used) {
            token = candidate;
            return true;
        }
    }
    return false;
}

static void write_frame(uint8_t *output, const AP_HAL::CANFrame &frame)
{
    BindingWire::write(output, frame.id, 4);
    output[4] = frame.dlc;
    output[5] = frame.canfd;
    const uint8_t native_size = uint8_t(sizeof(frame.data));
    const uint8_t data_size = native_size < FRAME_DATA_SIZE ? native_size : FRAME_DATA_SIZE;
    memcpy(output + 6, frame.data, data_size);
}

static bool read_frame(const uint8_t *input, AP_HAL::CANFrame &frame)
{
    const uint8_t dlc = input[4];
    const bool can_fd = input[5] != 0;
    if (input[5] > 1 || dlc > 15 || (!can_fd && dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen) ||
        AP_HAL::CANFrame::dlcToDataLength(dlc) > sizeof(frame.data)) {
        return false;
    }
#if !HAL_CANFD_SUPPORTED
    if (can_fd) {
        return false;
    }
#endif
    frame.id = BindingWire::read(input, 4);
    frame.dlc = dlc;
    frame.canfd = can_fd;
    memcpy(frame.data, input + 6, sizeof(frame.data));
    return true;
}
#endif

static int32_t get_device(void *context, uint8_t index, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 1 || output == nullptr || output_size != 4 ||
        index > 1 || input[0] == 0 || input[0] > MAX_BUFFER_LENGTH) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    auto *sensor = get_scripting_can_sensor(index);
    if (sensor == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    if (!sensor->initialized()) {
        return AP_MODULE_STATUS_OK;
    }
    WITH_SEMAPHORE(registry_sem);
    CANRegistryEntry *free_entry = nullptr;
    for (auto &entry : registry) {
        if (entry.buffer == nullptr) {
            free_entry = &entry;
            break;
        }
    }
    if (free_entry == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *buffer = sensor->add_buffer(input[0]);
    if (buffer == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    uint32_t token;
    if (!allocate_token(token)) {
        sensor->remove_buffer(buffer);
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    *free_entry = {context, token, sensor, buffer};
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_can_get_device(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    return get_device(context, 0, input, input_size, output, output_size);
}

int32_t binding_can_get_device2(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    return get_device(context, 1, input, input_size, output, output_size);
}

int32_t binding_can_read_frame(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr ||
        output_size != FRAME_WIRE_SIZE + 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    AP_HAL::CANFrame frame;
    if (entry->buffer->read_frame(frame)) {
        output[0] = 1;
        write_frame(output + 1, frame);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_can_write_frame(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 + FRAME_WIRE_SIZE + 4 ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    AP_HAL::CANFrame frame;
    const uint32_t timeout_us = BindingWire::read(input + 4 + FRAME_WIRE_SIZE, 4);
    if (!read_frame(input + 4, frame) || timeout_us > MAX_TIMEOUT_US) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->buffer->write_frame(frame, timeout_us);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_can_add_filter(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 12 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->buffer->add_filter(BindingWire::read(input + 4, 4),
                                          BindingWire::read(input + 8, 4));
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_can_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (!entry->sensor->remove_buffer(entry->buffer)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    *entry = {};
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

void binding_can_cleanup(void *context)
{
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    for (auto &entry : registry) {
        if (entry.context == context) {
            entry.sensor->remove_buffer(entry.buffer);
            entry = {};
        }
    }
#else
    (void)context;
#endif
}

#if defined(AP_MODULE_BINDING_TESTING)
void binding_can_test_set_next_token(uint32_t token)
{
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    WITH_SEMAPHORE(registry_sem);
    next_token = token;
#else
    (void)token;
#endif
}
#endif

}
#endif
