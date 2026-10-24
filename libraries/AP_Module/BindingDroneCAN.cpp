// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingDroneCAN.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <stdlib.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint32_t MESSAGE_WIRE_SIZE = 269;
static constexpr uint16_t MAX_PAYLOAD = 256;

#if HAL_ENABLE_DRONECAN_DRIVERS
static constexpr uint8_t MAX_HANDLES = 16;
static constexpr uint8_t QUEUE_DEPTH = 8;

class RustDroneCANHandle {
public:
    RustDroneCANHandle(AP_DroneCAN &driver, uint64_t signature, uint16_t data_type, bool can_fd) :
        _driver(driver), _signature(signature), _data_type(data_type), _can_fd(can_fd) {}
    CLASS_NO_COPY(RustDroneCANHandle);
    ~RustDroneCANHandle()
    {
        WITH_SEMAPHORE(_driver.get_canard_iface().get_sem_rx());
        delete _subscriber;
    }

    bool subscribe()
    {
        WITH_SEMAPHORE(_driver.get_canard_iface().get_sem_rx());
        _subscriber = NEW_NOTHROW Subscriber(*this, CanardTransferTypeBroadcast);
        return _subscriber != nullptr;
    }

    bool pop(uint8_t *data, uint16_t &length, uint8_t &node_id, uint64_t &timestamp_us, bool &can_fd)
    {
        if (_subscriber == nullptr) {
            return false;
        }
        Payload payload;
        if (!_subscriber->payloads.pop(payload)) {
            return false;
        }
        length = payload.length;
        node_id = payload.node_id;
        timestamp_us = payload.timestamp_us;
        can_fd = payload.can_fd;
        memcpy(data, payload.data, length);
        free(payload.data);
        return true;
    }

    bool broadcast(const uint8_t *payload, uint16_t length)
    {
        Canard::Transfer transfer{};
        transfer.transfer_type = CanardTransferTypeBroadcast;
        transfer.data_type_signature = _signature;
        transfer.data_type_id = _data_type;
        transfer.inout_transfer_id = &_transfer_id;
        transfer.priority = 0;
        transfer.payload = payload;
        transfer.payload_len = length;
        transfer.iface_mask = CANARD_IFACE_ALL;
#if CANARD_ENABLE_CANFD
        transfer.canfd = _can_fd;
#endif
        transfer.timeout_ms = 10;
        return _driver.get_canard_iface().broadcast(transfer);
    }

    bool request(uint8_t target_node, const uint8_t *payload, uint16_t length)
    {
        auto &iface = _driver.get_canard_iface();
        Canard::Transfer transfer{};
        transfer.transfer_type = CanardTransferTypeRequest;
        transfer.data_type_signature = _signature;
        transfer.data_type_id = _data_type;
        transfer.inout_transfer_id = &_transfer_id;
        transfer.priority = 0;
        transfer.payload = payload;
        transfer.payload_len = length;
        transfer.iface_mask = CANARD_IFACE_ALL;
#if CANARD_ENABLE_CANFD
        transfer.canfd = _can_fd;
#endif
        transfer.timeout_ms = 10;

        WITH_SEMAPHORE(iface.get_sem_rx());
        delete _subscriber;
        _subscriber = NEW_NOTHROW Subscriber(*this, CanardTransferTypeResponse);
        if (_subscriber == nullptr) {
            return false;
        }
        _subscriber->node_id = target_node;
        if (!iface.request(target_node, transfer)) {
            delete _subscriber;
            _subscriber = nullptr;
            return false;
        }
        return true;
    }

private:
    struct Payload {
        uint8_t *data;
        uint16_t length;
        uint8_t node_id;
        uint64_t timestamp_us;
        bool can_fd;
    };

    class Subscriber : public Canard::HandlerList {
    public:
        Subscriber(RustDroneCANHandle &owner, CanardTransferType requested_type) :
            Canard::HandlerList(requested_type, owner._data_type, owner._signature,
                                owner._driver.get_driver_index()),
            _owner(owner),
            _transfer_type(requested_type)
        {
            link();
        }
        ~Subscriber() override
        {
            unlink();
            Payload payload;
            while (payloads.pop(payload)) {
                free(payload.data);
            }
        }
        ObjectBuffer<Payload> payloads{QUEUE_DEPTH};
        uint8_t node_id{};

    private:
        bool handle_message(const CanardRxTransfer &transfer) override
        {
            if (_transfer_type == CanardTransferTypeResponse &&
                (node_id != transfer.source_node_id ||
                 uint8_t((transfer.transfer_id + 1U) & 31U) != _owner._transfer_id)) {
                return false;
            }
            if (transfer.payload_len > MAX_PAYLOAD || payloads.space() == 0) {
                return true;
            }
            Payload payload{};
            payload.data = static_cast<uint8_t *>(malloc(transfer.payload_len));
            if (payload.data == nullptr) {
                return true;
            }
            payload.length = transfer.payload_len;
            payload.node_id = transfer.source_node_id;
            payload.timestamp_us = transfer.timestamp_usec;
#if CANARD_ENABLE_CANFD
            payload.can_fd = transfer.canfd;
#endif
            uint32_t remaining = transfer.payload_len * 8U;
            uint32_t offset = 0;
            while (remaining > 0) {
                const uint8_t bits = MIN(8U, remaining);
                canardDecodeScalar(&transfer, offset * 8U, bits, false, &payload.data[offset]);
                offset++;
                remaining -= bits;
            }
            if (!payloads.push(payload)) {
                free(payload.data);
            }
            return true;
        }
        RustDroneCANHandle &_owner;
        const CanardTransferType _transfer_type;
    };

    AP_DroneCAN &_driver;
    const uint64_t _signature;
    const uint16_t _data_type;
    const bool _can_fd;
    uint8_t _transfer_id{};
    Subscriber *_subscriber{};
};

struct RegistryEntry {
    void *context;
    uint32_t token;
    RustDroneCANHandle *handle;
};

static RegistryEntry registry[MAX_HANDLES];
static HAL_Semaphore registry_sem;
static uint32_t next_token = 1;

static RegistryEntry *find_entry(void *context, uint32_t token)
{
    for (auto &entry : registry) {
        if (entry.context == context && entry.token == token && entry.handle != nullptr) {
            return &entry;
        }
    }
    return nullptr;
}

static bool allocate_token(uint32_t &token)
{
    for (uint8_t attempts = 0; attempts <= MAX_HANDLES;) {
        const uint32_t candidate = next_token++;
        if (candidate == 0) {
            continue;
        }
        attempts++;
        bool used = false;
        for (const auto &entry : registry) {
            if (entry.handle != nullptr && entry.token == candidate) {
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

static int32_t create_handle(void *context, uint8_t driver_index, uint64_t signature, uint16_t data_type,
                             bool can_fd, bool subscribe, uint8_t *output)
{
    auto *driver = AP_DroneCAN::get_dronecan(driver_index);
    if (driver == nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    WITH_SEMAPHORE(registry_sem);
    RegistryEntry *free_entry = nullptr;
    for (auto &entry : registry) {
        if (entry.handle == nullptr) {
            free_entry = &entry;
            break;
        }
    }
    if (free_entry == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *handle = NEW_NOTHROW RustDroneCANHandle(*driver, signature, data_type, can_fd);
    if (handle == nullptr || (subscribe && !handle->subscribe())) {
        delete handle;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    uint32_t token;
    if (!allocate_token(token)) {
        delete handle;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    *free_entry = {context, token, handle};
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
}
#endif

int32_t binding_dronecan_subscribe(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size != 11 || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if HAL_ENABLE_DRONECAN_DRIVERS
    const uint64_t signature = uint64_t(BindingWire::read(input + 1, 4)) |
                               uint64_t(BindingWire::read(input + 5, 4)) << 32;
    const uint16_t data_type = uint16_t(BindingWire::read(input + 9, 2));
    return create_handle(context, input[0], signature, data_type, false, true, output);
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_dronecan_open(void *context, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size != 12 || output_size != 4 ||
        input[11] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if HAL_ENABLE_DRONECAN_DRIVERS
    const uint64_t signature = uint64_t(BindingWire::read(input + 1, 4)) |
                               uint64_t(BindingWire::read(input + 5, 4)) << 32;
    const uint16_t data_type = uint16_t(BindingWire::read(input + 9, 2));
    return create_handle(context, input[0], signature, data_type, input[11] != 0, false, output);
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_dronecan_check_message(void *context, const uint8_t *input, uint32_t input_size,
                                       uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size != 4 ||
        output_size != MESSAGE_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if HAL_ENABLE_DRONECAN_DRIVERS
    const uint32_t token = BindingWire::read(input, 4);
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, token);
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    uint16_t length;
    uint8_t node_id;
    uint64_t timestamp_us;
    bool can_fd;
    if (!entry->handle->pop(output + 3, length, node_id, timestamp_us, can_fd)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    BindingWire::write(output + 1, length, 2);
    output[259] = node_id;
    BindingWire::write(output + 260, uint32_t(timestamp_us), 4);
    BindingWire::write(output + 264, uint32_t(timestamp_us >> 32), 4);
    output[268] = can_fd;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_dronecan_broadcast(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size < 6 ||
        input_size > MAX_PAYLOAD + 6 || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const uint16_t length = uint16_t(BindingWire::read(input + 4, 2));
    if (input_size != uint32_t(length) + 6U) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_ENABLE_DRONECAN_DRIVERS
    const uint32_t token = BindingWire::read(input, 4);
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, token);
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->handle->broadcast(input + 6, length);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_dronecan_request(void *context, const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size < 7 ||
        input_size > MAX_PAYLOAD + 7 || output_size != 1 || input[4] == 0 || input[4] > 127) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const uint16_t length = uint16_t(BindingWire::read(input + 5, 2));
    if (input_size != uint32_t(length) + 7U) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_ENABLE_DRONECAN_DRIVERS
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = entry->handle->request(input[4], input + 7, length);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_dronecan_close(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_ENABLE_DRONECAN_DRIVERS
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context, BindingWire::read(input, 4));
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    delete entry->handle;
    *entry = {};
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

void binding_dronecan_cleanup(void *context)
{
#if HAL_ENABLE_DRONECAN_DRIVERS
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    for (auto &entry : registry) {
        if (entry.context == context) {
            delete entry.handle;
            entry = {};
        }
    }
#else
    (void)context;
#endif
}

#if defined(AP_MODULE_BINDING_TESTING)
void binding_dronecan_test_set_next_token(uint32_t token)
{
#if HAL_ENABLE_DRONECAN_DRIVERS
    WITH_SEMAPHORE(registry_sem);
    next_token = token;
#else
    (void)token;
#endif
}
#endif

}
#endif
