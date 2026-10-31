// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingMAVLink.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS.h>
#include <new>
#include <string.h>

namespace AP_Module
{

static constexpr uint8_t MAX_QUEUE_DEPTH = 25;
static constexpr uint8_t MAX_MESSAGE_IDS = 25;
static constexpr uint16_t MAVLINK_MESSAGE_SIZE = 298;
static constexpr uint16_t RECEIVE_WIRE_SIZE = 1 + MAVLINK_MESSAGE_SIZE + 1 + 4;
static constexpr uint16_t SEND_WIRE_SIZE = 1 + 4 + 2 + 1 + 4 + MAVLINK_MAX_PAYLOAD_LEN;

static_assert(sizeof(mavlink_message_t) == MAVLINK_MESSAGE_SIZE,
              "Rust MAVLink ABI requires the MAVLink2.1 message layout");

#if HAL_GCS_ENABLED
struct ReceivedMessage {
    mavlink_message_t msg;
    mavlink_channel_t chan;
    uint32_t timestamp_ms;
};

class ModuleMAVLink {
public:
    ModuleMAVLink(uint8_t queue_depth, uint8_t message_ids) :
        _queue(NEW_NOTHROW ObjectBuffer<ReceivedMessage>(queue_depth)),
        _message_ids(NEW_NOTHROW uint32_t[message_ids == 0 ? 1 : message_ids]),
        _message_ids_size(message_ids)
    {
        if (_message_ids != nullptr) {
            for (uint8_t i = 0; i < _message_ids_size; i++) {
                _message_ids[i] = UINT32_MAX;
            }
        }
    }

    ~ModuleMAVLink()
    {
        delete _queue;
        delete[] _message_ids;
    }

    bool valid(uint8_t queue_depth) const
    {
        return _queue != nullptr && _queue->get_size() == queue_depth && _message_ids != nullptr;
    }

    int32_t register_message(uint32_t msgid, bool &added)
    {
        added = false;
        for (uint8_t i = 0; i < _message_ids_size; i++) {
            if (_message_ids[i] == msgid) {
                return AP_MODULE_STATUS_OK;
            }
        }
        for (uint8_t i = 0; i < _message_ids_size; i++) {
            if (_message_ids[i] == UINT32_MAX) {
                _message_ids[i] = msgid;
                added = true;
                return AP_MODULE_STATUS_OK;
            }
        }
        return AP_MODULE_STATUS_NO_MEMORY;
    }

    bool pop(ReceivedMessage &message)
    {
        return _queue->pop(message);
    }

    void handle(const mavlink_message_t &msg, mavlink_channel_t chan, uint32_t timestamp_ms)
    {
        for (uint8_t i = 0; i < _message_ids_size; i++) {
            if (_message_ids[i] == UINT32_MAX) {
                return;
            }
            if (_message_ids[i] == msg.msgid) {
                const ReceivedMessage message{msg, chan, timestamp_ms};
                // Match Lua: a full queue drops the newest packet.
                _queue->push(message);
                return;
            }
        }
    }

private:
    ObjectBuffer<ReceivedMessage> *_queue;
    uint32_t *_message_ids;
    const uint8_t _message_ids_size;
};

struct RegistryEntry {
    void *context;
    ModuleMAVLink *state;
    RegistryEntry *next;
};

static RegistryEntry *registry;
static HAL_Semaphore registry_sem;

struct CommandBlock {
    uint16_t command;
    const void *owner;
    CommandBlock *next;
};

static CommandBlock *command_blocks;

static RegistryEntry *find_entry(void *context)
{
    for (auto *entry = registry; entry != nullptr; entry = entry->next) {
        if (entry->context == context) {
            return entry;
        }
    }
    return nullptr;
}
#endif

int32_t binding_mavlink_init(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 2 || output != nullptr || output_size != 0 ||
        input[0] > MAX_QUEUE_DEPTH || input[1] > MAX_MESSAGE_IDS) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_GCS_ENABLED
    WITH_SEMAPHORE(registry_sem);
    if (find_entry(context) != nullptr) {
        // Lua's init keeps its first queue and registration allocations.
        return AP_MODULE_STATUS_OK;
    }
    auto *state = NEW_NOTHROW ModuleMAVLink(input[0], input[1]);
    if (state == nullptr || !state->valid(input[0])) {
        delete state;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *entry = NEW_NOTHROW RegistryEntry{context, state, registry};
    if (entry == nullptr) {
        delete state;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    registry = entry;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_mavlink_register_rx_msgid(void *context, const uint8_t *input, uint32_t input_size,
                                          uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size != 4 || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const uint32_t msgid = BindingWire::read(input, 4);
    if (msgid >= (1U << 24)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_GCS_ENABLED
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context);
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    bool added;
    const int32_t result = entry->state->register_message(msgid, added);
    output[0] = added;
    return result;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_mavlink_receive_chan(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr ||
        output_size != RECEIVE_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if HAL_GCS_ENABLED
    WITH_SEMAPHORE(registry_sem);
    auto *entry = find_entry(context);
    if (entry == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    ReceivedMessage message;
    if (!entry->state->pop(message)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    memcpy(output + 1, &message.msg, sizeof(message.msg));
    output[1 + MAVLINK_MESSAGE_SIZE] = uint8_t(message.chan);
    BindingWire::write(output + 2 + MAVLINK_MESSAGE_SIZE, message.timestamp_ms, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_mavlink_send_chan(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || output == nullptr || input_size != SEND_WIRE_SIZE ||
        output_size != 2) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint32_t msgid = BindingWire::read(input + 1, 4);
    const uint16_t packet_len = uint16_t(BindingWire::read(input + 5, 2));
    const uint8_t have_target = input[7];
    const uint32_t target_sysid = BindingWire::read(input + 8, 4);
    if (msgid >= (1U << 24) || packet_len > MAVLINK_MAX_PAYLOAD_LEN || have_target > 1 ||
        (!have_target && target_sysid != 0)) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = packet_len; i < MAVLINK_MAX_PAYLOAD_LEN; i++) {
        if (input[12 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
#if HAL_GCS_ENABLED
    const uint8_t chan_number = input[0];
    const mavlink_channel_t chan = mavlink_channel_t(chan_number);
    if (chan_number >= gcs().num_gcs()) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr || (have_target && !(entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM))) {
        return AP_MODULE_STATUS_INVALID;
    }
    char payload[MAVLINK_MAX_PAYLOAD_LEN] {};
    memcpy(payload, input + 12, MIN(size_t(packet_len), size_t(entry->max_msg_len)));
    if (have_target) {
        payload[entry->target_system_ofs] = mavlink_msg_target_field(target_sysid);
    }
    WITH_SEMAPHORE(comm_chan_lock(chan));
    if (target_sysid > UINT8_MAX &&
        (mavlink_get_channel_status(chan)->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)) {
        return AP_MODULE_STATUS_OK;
    }
    if (comm_get_txspace(chan) < GCS_MAVLINK::packet_overhead_chan(chan) + entry->max_msg_len) {
        return AP_MODULE_STATUS_OK;
    }
    _mav_finalize_message_chan_send_target(chan, entry->msgid, payload, entry->min_msg_len,
                                           entry->max_msg_len, entry->crc_extra, target_sysid);
    output[1] = 1;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_mavlink_block_command(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 2 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if HAL_GCS_ENABLED
    const uint16_t command = uint16_t(BindingWire::read(input, 2));
    WITH_SEMAPHORE(registry_sem);
    for (const CommandBlock *item = command_blocks; item != nullptr; item = item->next) {
        if (item->command == command && item->owner == context) {
            output[0] = 1;
            return AP_MODULE_STATUS_OK;
        }
    }
    auto *item = NEW_NOTHROW CommandBlock{command, context, command_blocks};
    if (item == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    command_blocks = item;
    output[0] = 1;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

bool binding_mavlink_is_handling_command(uint16_t command)
{
#if HAL_GCS_ENABLED
    WITH_SEMAPHORE(registry_sem);
    for (const CommandBlock *item = command_blocks; item != nullptr; item = item->next) {
        if (item->command == command) {
            return true;
        }
    }
#else
    (void)command;
#endif
    return false;
}

void binding_mavlink_handle_message(const mavlink_message_t &msg, mavlink_channel_t chan)
{
#if HAL_GCS_ENABLED
    const uint32_t timestamp_ms = AP_HAL::millis();
    WITH_SEMAPHORE(registry_sem);
    for (auto *entry = registry; entry != nullptr; entry = entry->next) {
        entry->state->handle(msg, chan, timestamp_ms);
    }
#else
    (void)msg;
    (void)chan;
#endif
}

void binding_mavlink_cleanup(void *context)
{
#if HAL_GCS_ENABLED
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    CommandBlock **link = &command_blocks;
    while (*link != nullptr) {
        CommandBlock *item = *link;
        if (item->owner == context) {
            *link = item->next;
            delete item;
        } else {
            link = &item->next;
        }
    }
    for (RegistryEntry **entry_link = &registry; *entry_link != nullptr; entry_link = &(*entry_link)->next) {
        auto *entry = *entry_link;
        if (entry->context == context) {
            *entry_link = entry->next;
            delete entry->state;
            delete entry;
            break;
        }
    }
#else
    (void)context;
#endif
}

}
#endif
