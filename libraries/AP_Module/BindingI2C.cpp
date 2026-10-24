// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingI2C.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace AP_Module
{

static constexpr uint8_t MAX_DEVICES_PER_MODULE = 4;
static constexpr uint8_t MAX_HANDLES = 32; // Global device budget, not a module-count limit.
static constexpr uint16_t TRANSFER_MAX = 255;

struct I2CEntry {
    void *context;
    uint32_t token;
    AP_HAL::I2CDevice *device;
};

static I2CEntry registry[MAX_HANDLES];
static HAL_Semaphore registry_sem;
static uint32_t next_token = 1;

static I2CEntry *find_entry(void *context, uint32_t token)
{
    if (context == nullptr || token == 0) {
        return nullptr;
    }
    for (auto &entry : registry) {
        if (entry.context == context && entry.token == token && entry.device != nullptr) {
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
        if (candidate == 0) {
            continue;
        }
        bool live = false;
        for (const auto &entry : registry) {
            live |= entry.token == candidate && entry.device != nullptr;
        }
        if (!live) {
            return candidate;
        }
    }
    return 0;
}

static int32_t resolve(void *context, const uint8_t *input, uint32_t input_size,
                       uint32_t expected_size, AP_HAL::I2CDevice *&device)
{
    device = nullptr;
    if (context == nullptr || input == nullptr || input_size != expected_size) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    device = entry->device;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_i2c_open(void *context, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 7 || output == nullptr || output_size != 4 ||
        input[0] > 4 || input[1] > 128 || input[6] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    if (hal.i2c_mgr == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    WITH_SEMAPHORE(registry_sem);
    I2CEntry *free_entry = nullptr;
    uint8_t owned = 0;
    for (auto &entry : registry) {
        if (entry.context == context && entry.device != nullptr) {
            owned++;
        } else if (free_entry == nullptr && entry.device == nullptr) {
            free_entry = &entry;
        }
    }
    if (owned >= MAX_DEVICES_PER_MODULE || free_entry == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    const uint32_t token = allocate_token();
    if (token == 0) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *device = hal.i2c_mgr->get_device_ptr(input[0], input[1], BindingWire::read(input + 2, 4), input[6] != 0);
    if (device == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    *free_entry = {context, token, device};
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_i2c_close(void *context, const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    delete entry->device;
    *entry = {};
    return AP_MODULE_STATUS_OK;
}

static int32_t setting(void *context, const uint8_t *input, uint32_t input_size,
                       uint8_t *output, uint32_t output_size, bool retries)
{
    if (output != nullptr || output_size != 0 || input == nullptr || input_size != 5 ||
        (retries && input[4] > 20)) {
        return AP_MODULE_STATUS_INVALID;
    }
    AP_HAL::I2CDevice *device;
    const int32_t status = resolve(context, input, input_size, 5, device);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    BindingLock lock(*device->get_semaphore());
    if (retries) {
        device->set_retries(input[4]);
    } else {
        device->set_address(input[4]);
    }
    return AP_MODULE_STATUS_OK;
}

int32_t binding_i2c_set_retries(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    return setting(context, input, input_size, output, output_size, true);
}

int32_t binding_i2c_set_address(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    return setting(context, input, input_size, output, output_size, false);
}

int32_t binding_i2c_write_register(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    AP_HAL::I2CDevice *device;
    const int32_t status = resolve(context, input, input_size, 6, device);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    BindingLock lock(*device->get_semaphore());
    output[0] = device->write_register(input[4], input[5]) ? 1 : 0;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_i2c_read_registers(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (output == nullptr || output_size != TRANSFER_MAX + 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    AP_HAL::I2CDevice *device;
    const int32_t status = resolve(context, input, input_size, 6, device);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    const uint8_t length = input[5];
    if (length == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(*device->get_semaphore());
    if (device->read_registers(input[4], output + 1, length)) {
        output[0] = 1;
    }
    return AP_MODULE_STATUS_OK;
}

int32_t binding_i2c_transfer(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 4 + 2 + TRANSFER_MAX || output == nullptr ||
        output_size != TRANSFER_MAX + 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    AP_HAL::I2CDevice *device;
    const int32_t status = resolve(context, input, input_size, 4 + 2 + TRANSFER_MAX, device);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    const uint8_t send_length = input[4];
    const uint8_t receive_length = input[5];
    for (uint16_t i = send_length; i < TRANSFER_MAX; i++) {
        if (input[6 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    BindingLock lock(*device->get_semaphore());
    if (device->transfer(input + 6, send_length, output + 1, receive_length)) {
        output[0] = 1;
    }
    return AP_MODULE_STATUS_OK;
}

void binding_i2c_cleanup(void *context)
{
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    for (auto &entry : registry) {
        if (entry.context == context) {
            delete entry.device;
            entry = {};
        }
    }
}

}
#endif
