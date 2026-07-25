/*
 * Copyright (c) 2022 Siddharth B Purohit, CubePilot Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#pragma once
#include <stdint.h>
#include <canard.h>
#include "handler_list.h"

#ifndef CANARD_IFACE_ALL
#define CANARD_IFACE_ALL 0xFF
#endif

namespace Canard {

struct Transfer {
    CanardTransferType transfer_type; ///< Type of transfer: CanardTransferTypeBroadcast, CanardTransferTypeRequest, CanardTransferTypeResponse
    uint64_t data_type_signature; ///< Signature of the message/service
    uint16_t data_type_id; ///< ID of the message/service
    uint8_t* inout_transfer_id; ///< Transfer ID reference
    uint8_t priority; ///< Priority of the transfer
    const void* payload; ///< Pointer to the payload
    uint32_t payload_len; ///< Length of the payload
    uint8_t iface_mask; ///< Bitmask of interfaces to send the transfer on
    bool canfd; ///< true if the transfer is CAN FD
    uint32_t timeout_ms; ///< timeout in ms
};

/// @brief Interface class for Canard, its purpose is to provide a common interface for all interfaces
class Interface {
public:
    /// @brief Interface constructor
    /// @param _index index of the interface, used to identify which HandlerList to use
    /// @param _canfd true if the interface is CAN FD
    Interface(uint8_t _index, bool _canfd = false) :
        index(_index),
        canfd(_canfd)
    {}

    /// @brief Interface destructor
    virtual ~Interface() {}

    /// @brief broadcast message
    /// @param bcast_transfer transfer to broadcast
    /// @return true if broadcast was added to the queue
    virtual bool broadcast(const Transfer &bcast_transfer) = 0;

    /// @brief request message from
    /// @param destination_node_id
    /// @param req_transfer
    /// @return true if request was added to the queue
    virtual bool request(uint8_t destination_node_id, const Transfer &req_transfer) = 0;

    /// @brief respond to a request
    /// @param destination_node_id
    /// @param res_transfer
    /// @return true if response was added to the queue
    virtual bool respond(uint8_t destination_node_id, const Transfer &res_transfer) = 0;

    /// @brief check if the interface is CAN FD
    /// @return true if the interface is CAN FD
    bool is_canfd() const { return canfd; }

    /// @brief get the node ID of the interface
    /// @return returns anonymous node ID 0 if not implemented
    virtual uint8_t get_node_id() const = 0;

    /// @brief get the index of the interface
    /// @return 
    uint8_t get_index() const { return index; }

    /// @brief set canfd mode
    /// @param _canfd true if the interface is CAN FD
    void set_canfd(bool _canfd) { canfd = _canfd; }

protected:
    /// @brief forward accept_message call to indexed HandlerList
    /// @param msgid ID of the message/service
    /// @param transfer_type - transfer type
    /// @param[out] signature signature of message/service
    /// @return true if the message/service is accepted
    inline bool accept_message(uint16_t msgid, CanardTransferType transfer_type,  uint64_t &signature) {
        return HandlerList::accept_message(index, msgid, transfer_type, signature);
    }

    /// @brief forward handle_message call to indexed HandlerList
    /// @param transfer received transfer
    inline void handle_message(const CanardRxTransfer& transfer) {
        HandlerList::handle_message(index, transfer);
    }
private:
    uint8_t index; ///< index of the interface
    bool canfd; ///< true if the interface is CAN FD
};

} // namespace Canard
