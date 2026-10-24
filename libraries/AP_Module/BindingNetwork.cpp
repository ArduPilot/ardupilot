// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingNetwork.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Networking/AP_Networking.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_HAL/utility/Socket.h>
#include <errno.h>
#include <fcntl.h>
#include <new>
#include <string.h>

namespace AP_Module
{

static constexpr uint16_t TRANSFER_MAX = 1024;
static constexpr uint16_t PATH_SIZE = 256;
static constexpr uint16_t POLL_TIMEOUT_MAX_MS = 100;

#if AP_NETWORKING_ENABLED
static constexpr uint8_t MAX_SOCKETS = 50;
static constexpr uint16_t BLOCKING_IO_TIMEOUT_MS = 20;
static constexpr uint8_t TOKEN_INDEX_BITS = 6;
static constexpr uint32_t TOKEN_INDEX_MASK = (1U << TOKEN_INDEX_BITS) - 1U;
static constexpr uint32_t TOKEN_GENERATION_MASK = (1U << (32U - TOKEN_INDEX_BITS)) - 1U;
// Registry-semaphore protected. A fresh generation for every opened socket
// keeps stale handles invalid even when both an owner and its registry entry
// are freed and their addresses are reused by another generation.
static uint32_t next_generation = 1;
struct SocketSlot {
    SocketAPM *socket;
    uint32_t generation;
    bool blocking;
};

class ModuleSockets {
public:
    ~ModuleSockets()
    {
        for (auto &slot : _slots) {
            delete slot.socket;
            slot.socket = nullptr;
        }
    }

    int32_t add(SocketAPM *socket, uint32_t &token)
    {
        if (socket == nullptr || !socket->set_blocking(false)) {
            delete socket;
            return AP_MODULE_STATUS_HOST_ERROR;
        }
        for (uint8_t i = 0; i < MAX_SOCKETS; i++) {
            auto &slot = _slots[i];
            if (slot.socket == nullptr) {
                if (next_generation > TOKEN_GENERATION_MASK) {
                    // Never recycle an issued handle after freeing its owner.
                    delete socket;
                    return AP_MODULE_STATUS_NO_MEMORY;
                }
                slot.socket = socket;
                slot.blocking = false;
                slot.generation = next_generation++;
                token = (slot.generation << TOKEN_INDEX_BITS) | uint32_t(i + 1U);
                return AP_MODULE_STATUS_OK;
            }
        }
        delete socket;
        return AP_MODULE_STATUS_NO_MEMORY;
    }

    SocketSlot *resolve(uint32_t token)
    {
        const uint32_t encoded_index = token & TOKEN_INDEX_MASK;
        const uint32_t generation = token >> TOKEN_INDEX_BITS;
        if (encoded_index == 0 || encoded_index > MAX_SOCKETS || generation == 0) {
            return nullptr;
        }
        auto &slot = _slots[encoded_index - 1U];
        return slot.socket != nullptr && slot.generation == generation ? &slot : nullptr;
    }

    void release(SocketSlot &slot)
    {
        delete slot.socket;
        slot.socket = nullptr;
        slot.blocking = false;
    }

private:
    SocketSlot _slots[MAX_SOCKETS] {};
};

struct RegistryEntry {
    void *context;
    ModuleSockets *sockets;
    RegistryEntry *next;
};

static RegistryEntry *registry;
static HAL_Semaphore registry_sem;

static RegistryEntry *find_context_entry(void *context)
{
    for (auto *entry = registry; entry != nullptr; entry = entry->next) {
        if (entry->context == context) {
            return entry;
        }
    }
    return nullptr;
}

static int32_t find_or_create_entry(void *context, RegistryEntry *&entry)
{
    entry = find_context_entry(context);
    if (entry != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    auto *sockets = NEW_NOTHROW ModuleSockets;
    if (sockets == nullptr) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    entry = NEW_NOTHROW RegistryEntry;
    if (entry == nullptr) {
        delete sockets;
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    entry->context = context;
    entry->sockets = sockets;
    entry->next = registry;
    registry = entry;
    return AP_MODULE_STATUS_OK;
}

static int32_t resolve(void *context, const uint8_t *input, SocketSlot *&slot,
                       RegistryEntry *&entry)
{
    entry = find_context_entry(context);
    if (entry == nullptr || entry->sockets == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    slot = entry->sockets->resolve(BindingWire::read(input, 4));
    return slot == nullptr ? AP_MODULE_STATUS_INVALID : AP_MODULE_STATUS_OK;
}

static uint16_t operation_timeout(const SocketSlot &slot)
{
    return slot.blocking ? BLOCKING_IO_TIMEOUT_MS : 0;
}

static bool retryable_error()
{
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}
#endif

int32_t binding_network_open(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 1 || input[0] > 1 ||
        output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    RegistryEntry *entry;
    const int32_t status = find_or_create_entry(context, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    uint32_t token = 0;
    const int32_t added = entry->sockets->add(NEW_NOTHROW SocketAPM(input[0] != 0), token);
    if (added != AP_MODULE_STATUS_OK) {
        return added;
    }
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_close(void *context, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    entry->sockets->release(*slot);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_bind(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4U + PATH_SIZE + 2U ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *address = BindingWire::read_cstring(input + 4);
    const uint16_t port = uint16_t(BindingWire::read(input + 4 + PATH_SIZE, 2));
    if (address == nullptr || address[0] == 0 || port == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = slot->socket->bind(address, port);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_connect(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4U + PATH_SIZE + 2U ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *address = BindingWire::read_cstring(input + 4);
    const uint16_t port = uint16_t(BindingWire::read(input + 4 + PATH_SIZE, 2));
    if (address == nullptr || address[0] == 0 || port == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    bool connected = slot->socket->connect(address, port);
    if (!connected && slot->blocking && slot->socket->is_pending()) {
        connected = slot->socket->pollout(BLOCKING_IO_TIMEOUT_MS) && slot->socket->is_connected();
    }
    output[0] = connected;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_listen(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 5 || input[4] == 0 ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = slot->socket->listen(input[4]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_set_blocking(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 5 || input[4] > 1 ||
        output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    // Keep the descriptor nonblocking. The logical blocking mode below uses a
    // short bounded poll, so a stopped module cannot strand the manager.
    if (!slot->socket->set_blocking(false)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    slot->blocking = input[4] != 0;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_is_connected(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = slot->socket->is_connected();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static int32_t poll_socket(void *context, const uint8_t *input, uint32_t input_size,
                           uint8_t *output, uint32_t output_size, bool input_ready)
{
    if (context == nullptr || input == nullptr || input_size != 6 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const uint16_t timeout = uint16_t(BindingWire::read(input + 4, 2));
    if (timeout > POLL_TIMEOUT_MAX_MS) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = input_ready ? slot->socket->pollin(timeout) : slot->socket->pollout(timeout);
    return AP_MODULE_STATUS_OK;
#else
    (void)input_ready;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_pollin(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    return poll_socket(context, input, input_size, output, output_size, true);
}

int32_t binding_network_pollout(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    return poll_socket(context, input, input_size, output, output_size, false);
}

int32_t binding_network_accept(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    if (!slot->socket->pollin(operation_timeout(*slot))) {
        return AP_MODULE_STATUS_OK;
    }
    SocketAPM *accepted = slot->socket->accept(0);
    if (accepted == nullptr) {
        return retryable_error() ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_HOST_ERROR;
    }
    uint32_t token = 0;
    const int32_t added = entry->sockets->add(accepted, token);
    if (added != AP_MODULE_STATUS_OK) {
        return added;
    }
    BindingWire::write(output, token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_recv(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 6 || output == nullptr ||
        output_size != TRANSFER_MAX + 3U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    if (!slot->socket->pollin(operation_timeout(*slot))) {
        return AP_MODULE_STATUS_OK;
    }
    const ssize_t received = slot->socket->recv(output + 3, count, 0);
    if (received < 0) {
        return retryable_error() ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_HOST_ERROR;
    }
    if (received > count) {
        memset(output, 0, output_size);
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    BindingWire::write(output + 1, uint16_t(received), 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_send(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != TRANSFER_MAX + 6U ||
        output == nullptr || output_size != 3) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = count; i < TRANSFER_MAX; i++) {
        if (input[6 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    if (!slot->socket->pollout(operation_timeout(*slot))) {
        return AP_MODULE_STATUS_OK;
    }
    const ssize_t sent = slot->socket->send(input + 6, count);
    if (sent < 0) {
        return retryable_error() ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_HOST_ERROR;
    }
    if (sent > count) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    BindingWire::write(output + 1, uint16_t(sent), 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_sendfile(void *context, const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4U + PATH_SIZE ||
        output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    const char *path = BindingWire::read_cstring(input + 4);
    if (path == nullptr || path[0] == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED && AP_FILESYSTEM_FILE_READING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    const int fd = AP::FS().open(path, O_RDONLY);
    if (fd < 0) {
        return AP_MODULE_STATUS_OK;
    }
    if (!AP::network().sendfile(slot->socket, fd)) {
        AP::FS().close(fd);
        return AP_MODULE_STATUS_OK;
    }
    // sendfile() transfers both descriptor ownership and the socket's fd to
    // AP_Networking. Releasing the now-empty object invalidates this token.
    entry->sockets->release(*slot);
    output[0] = 1;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_recv_from(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 6 || output == nullptr ||
        output_size != TRANSFER_MAX + 10U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    if (!slot->socket->pollin(operation_timeout(*slot))) {
        return AP_MODULE_STATUS_OK;
    }
    const ssize_t received = slot->socket->recv(output + 10, count, 0);
    if (received < 0) {
        return retryable_error() ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_HOST_ERROR;
    }
    if (received > count) {
        memset(output, 0, output_size);
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    uint32_t address;
    uint16_t port;
    if (slot->socket->last_recv_address(address, port)) {
        output[3] = 1;
        BindingWire::write(output + 4, address, 4);
        BindingWire::write(output + 8, port, 2);
    }
    output[0] = 1;
    BindingWire::write(output + 1, uint16_t(received), 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_is_pending(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = slot->socket->is_pending();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_reuse_address(void *context, const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    output[0] = slot->socket->reuseaddress();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_network_send_to(void *context, const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != TRANSFER_MAX + 12U ||
        output == nullptr || output_size != 3) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = count; i < TRANSFER_MAX; i++) {
        if (input[12 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
#if AP_NETWORKING_ENABLED
    BindingLock lock(registry_sem);
    SocketSlot *slot;
    RegistryEntry *entry;
    const int32_t status = resolve(context, input, slot, entry);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    if (!slot->socket->pollout(operation_timeout(*slot))) {
        return AP_MODULE_STATUS_OK;
    }
    const uint32_t address = uint32_t(BindingWire::read(input + 6, 4));
    const uint16_t port = uint16_t(BindingWire::read(input + 10, 2));
    const ssize_t sent = slot->socket->sendto(input + 12, count, address, port);
    if (sent < 0) {
        return retryable_error() ? AP_MODULE_STATUS_OK : AP_MODULE_STATUS_HOST_ERROR;
    }
    if (sent > count) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    BindingWire::write(output + 1, uint16_t(sent), 2);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

void binding_network_cleanup(void *context)
{
#if AP_NETWORKING_ENABLED
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    for (RegistryEntry **link = &registry; *link != nullptr; link = &(*link)->next) {
        auto *entry = *link;
        if (entry->context == context) {
            *link = entry->next;
            delete entry->sockets;
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
