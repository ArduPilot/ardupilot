// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingSharedState.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <stdlib.h>
#include <string.h>

namespace AP_Module
{

#if AP_MODULE_SHARED_STATE_ENABLED
static constexpr uint16_t TRANSFER_MAX = 2048;
static constexpr uint32_t BLOCK_MAX = 16384;
static constexpr uint32_t TOTAL_MAX = 65536;
static constexpr uint8_t MAX_BLOCKS = 8;
static constexpr uint8_t MAX_HANDLES = 32;
static constexpr uint8_t MAX_PER_CONTEXT = 4;

struct SharedBlock {
    uint32_t id;
    uint32_t size;
    uint64_t revision;
    uint16_t references;
    uint8_t *data;
};

struct SharedHandle {
    void *context;
    uint32_t token;
    SharedBlock *block;
};

static SharedBlock blocks[MAX_BLOCKS];
static SharedHandle handles[MAX_HANDLES];
static HAL_Semaphore shared_sem;
static uint32_t next_token = 1;
static uint32_t allocated;

static uint64_t read_u64(const uint8_t *p)
{
    return uint64_t(BindingWire::read(p, 4)) | (uint64_t(BindingWire::read(p + 4, 4)) << 32);
}

static void write_u64(uint8_t *p, uint64_t value)
{
    BindingWire::write(p, uint32_t(value), 4);
    BindingWire::write(p + 4, uint32_t(value >> 32), 4);
}

// All registry access and copies are covered by shared_sem. No module pointer is retained.
static SharedHandle *find_handle(void *context, uint32_t token)
{
    for (auto &handle : handles) {
        if (handle.block != nullptr && handle.context == context && handle.token == token) {
            return &handle;
        }
    }
    return nullptr;
}

static void release(SharedHandle &handle)
{
    auto &block = *handle.block;
    if (--block.references == 0) {
        allocated -= block.size;
        free(block.data);
        block = {};
    }
    handle = {};
}

int32_t binding_shared_state_open(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 8 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint32_t id = BindingWire::read(input, 4);
    const uint32_t size = BindingWire::read(input + 4, 4);
    if (size == 0 || size > BLOCK_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(shared_sem);
    SharedHandle *slot = nullptr;
    uint8_t owned = 0;
    for (auto &handle : handles) {
        if (handle.block != nullptr && handle.context == context) {
            owned++;
        } else if (handle.block == nullptr && slot == nullptr) {
            slot = &handle;
        }
    }
    if (slot == nullptr || owned >= MAX_PER_CONTEXT || next_token == 0) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    SharedBlock *block = nullptr;
    SharedBlock *empty = nullptr;
    for (auto &candidate : blocks) {
        if (candidate.data != nullptr && candidate.id == id) {
            block = &candidate;
        } else if (candidate.data == nullptr && empty == nullptr) {
            empty = &candidate;
        }
    }
    if (block != nullptr && block->size != size) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (block == nullptr) {
        if (empty == nullptr || size > TOTAL_MAX - allocated) {
            return AP_MODULE_STATUS_NO_MEMORY;
        }
        auto *data = static_cast<uint8_t *>(calloc(size, 1));
        if (data == nullptr) {
            return AP_MODULE_STATUS_NO_MEMORY;
        }
        block = empty;
        *block = {id, size, 1, 0, data};
        allocated += size;
    }
    *slot = {context, next_token++, block};
    block->references++;
    BindingWire::write(output, slot->token, 4);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_shared_state_read(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 10 || output == nullptr || output_size != 2056) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint32_t offset = BindingWire::read(input + 4, 4);
    const uint16_t length = BindingWire::read(input + 8, 2);
    WITH_SEMAPHORE(shared_sem);
    auto *handle = find_handle(context, BindingWire::read(input, 4));
    if (handle == nullptr || length > TRANSFER_MAX || offset > handle->block->size ||
        length > handle->block->size - offset) {
        return AP_MODULE_STATUS_INVALID;
    }
    memcpy(output + 8, handle->block->data + offset, length);
    write_u64(output, handle->block->revision);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_shared_state_write(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 2066 || output == nullptr || output_size != 8) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint32_t offset = BindingWire::read(input + 4, 4);
    const uint16_t length = BindingWire::read(input + 8, 2);
    if (length > TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = length; i < TRANSFER_MAX; i++) {
        if (input[18 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    WITH_SEMAPHORE(shared_sem);
    auto *handle = find_handle(context, BindingWire::read(input, 4));
    if (handle == nullptr || offset > handle->block->size || length > handle->block->size - offset) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto &block = *handle->block;
    const uint64_t expected = read_u64(input + 10);
    if (expected != 0 && expected != block.revision) {
        return AP_MODULE_STATUS_OK;
    }
    if (block.revision == UINT64_MAX) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    memcpy(block.data + offset, input + 18, length);
    write_u64(output, ++block.revision);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_shared_state_close(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    WITH_SEMAPHORE(shared_sem);
    auto *handle = find_handle(context, BindingWire::read(input, 4));
    if (handle == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    release(*handle);
    return AP_MODULE_STATUS_OK;
}

void binding_shared_state_cleanup(void *context)
{
    WITH_SEMAPHORE(shared_sem);
    for (auto &handle : handles) {
        if (handle.block != nullptr && handle.context == context) {
            release(handle);
        }
    }
}
#else
int32_t binding_shared_state_open(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

int32_t binding_shared_state_read(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

int32_t binding_shared_state_write(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

int32_t binding_shared_state_close(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t)
{
    return AP_MODULE_STATUS_UNAVAILABLE;
}

void binding_shared_state_cleanup(void *)
{
}
#endif // AP_MODULE_SHARED_STATE_ENABLED
}
#endif // AP_MODULE_ENABLED
