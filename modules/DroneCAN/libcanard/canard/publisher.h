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
#include "interface.h"
#include <canard.h>
#include "transfer_object.h"
#include "helpers.h"

namespace Canard {

/// @brief Base class for data senders using Transfer Object, such as publishers and requesters
class Sender {
public:
    Sender(Interface &_interface) :
    interface(_interface)
    {}

    // delete copy constructor and assignment operator
    Sender(const Sender&) = delete;

    inline void set_priority(uint8_t _priority) {
        priority = _priority;
    }

    inline void set_timeout_ms(uint32_t _timeout) {
        timeout = _timeout;
    }

    inline uint32_t get_timeout_ms() {
        return timeout;
    }

protected:
    Interface &interface; ///< Interface to send the message on
    /// @brief Send a message
    /// @param Transfer message to send
    /// @return true if the message was put into the queue successfully
    bool send(Transfer& transfer, uint8_t destination_node_id = CANARD_BROADCAST_NODE_ID) NOINLINE_FUNC {
        switch (transfer.transfer_type)
        {
        case CanardTransferTypeBroadcast:
            transfer.inout_transfer_id = TransferObject::get_tid_ptr(interface.get_index(),transfer.data_type_id, CanardTransferTypeBroadcast, interface.get_node_id(), destination_node_id);
            transfer.priority = priority;
            transfer.timeout_ms = timeout;
            return interface.broadcast(transfer);
        case CanardTransferTypeRequest:
            transfer.inout_transfer_id = TransferObject::get_tid_ptr(interface.get_index(),transfer.data_type_id, CanardTransferTypeRequest, interface.get_node_id(), destination_node_id);
            transfer.priority = priority;
            transfer.timeout_ms = timeout;
            return interface.request(destination_node_id, transfer);
        case CanardTransferTypeResponse:
        default:
            return false;
        }
    }
private:
    uint8_t priority = CANARD_TRANSFER_PRIORITY_MEDIUM; ///< Priority of the message
    uint32_t timeout = 1000; ///< Timeout of the message in ms
};

template <typename msgtype>
class Publisher : public Sender {
public:
    Publisher(Interface &_interface) :
    Sender(_interface)
    {}

    // delete copy constructor and assignment operator
    Publisher(const Publisher&) = delete;

    /// @brief Broadcast a message
    /// @param msg message to send
    /// @return true if the message was put into the queue successfully
    bool broadcast(msgtype& msg) {
        return broadcast(msg, interface.is_canfd());
    }

    /// @brief Broadcast a message
    /// @param msg message to send
    /// @param canfd true if the message should be sent as CAN FD
    /// @return true if the message was put into the queue successfully
    bool broadcast(msgtype& msg, bool canfd) {
#if !CANARD_ENABLE_CANFD
        if (canfd) {
            return false;
        }
#endif
        // encode the message
        uint32_t len = msgtype::cxx_iface::encode(&msg, msg_buf 
#if CANARD_ENABLE_CANFD
        , !canfd
#elif CANARD_ENABLE_TAO_OPTION
        , true
#endif
        );
        // send the message if encoded successfully
        if (len > 0) {
            Transfer msg_transfer {};
            msg_transfer.transfer_type = CanardTransferTypeBroadcast;
            msg_transfer.data_type_id = msgtype::cxx_iface::ID;
            msg_transfer.data_type_signature = msgtype::cxx_iface::SIGNATURE;
            msg_transfer.payload = msg_buf;
            msg_transfer.payload_len = len;
#if CANARD_ENABLE_CANFD
            msg_transfer.canfd = canfd;
#endif
#if CANARD_MULTI_IFACE
            msg_transfer.iface_mask = CANARD_IFACE_ALL;
#endif
            return send(msg_transfer);
        }
        return false;
    }
private:
    uint8_t msg_buf[msgtype::cxx_iface::MAX_SIZE]; ///< Buffer to store the encoded message
};
} // namespace Canard

/// @brief Macro to create a publisher
/// @param IFACE name of the interface
/// @param PUBNAME name of the publisher
/// @param MSGTYPE type of the message
#define CANARD_PUBLISHER(IFACE, PUBNAME, MSGTYPE) \
    Canard::Publisher<MSGTYPE> PUBNAME{IFACE};
