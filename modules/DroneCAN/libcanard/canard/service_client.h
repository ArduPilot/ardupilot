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
#include "handler_list.h"
#include "interface.h"
#include "publisher.h"

namespace Canard {

/// @brief Client class to handle service requests
/// @tparam rsptype Service type
template <typename rsptype>
class Client : public HandlerList, public Sender {
public:
    /// @brief Client constructor
    /// @param _interface Interface object
    /// @param _cb Callback object
    Client(Interface &_interface, Callback<rsptype> &_cb) NOINLINE_FUNC :
    HandlerList(CanardTransferTypeResponse, rsptype::cxx_iface::ID, rsptype::cxx_iface::SIGNATURE, _interface.get_index()),
    Sender(_interface),
    server_node_id(255),
    cb(_cb) {
        // link ourselves into the handler list
        link();
    }

    // delete copy constructor and assignment operator
    Client(const Client&) = delete;

    // destructor
    ~Client() NOINLINE_FUNC {
        // unlink ourselves from the handler list
        unlink();
    }

    /// @brief handles incoming messages
    /// @param transfer transfer object of the request
    bool handle_message(const CanardRxTransfer& transfer) override NOINLINE_FUNC {
        rsptype msg {};
        if (server_node_id == transfer.source_node_id &&
            transfer_id == transfer.transfer_id &&
            !rsptype::cxx_iface::rsp_decode(&transfer, &msg)) {
                cb(transfer, msg);
                return true;
        }
        return false;
    }

    /// @brief makes service request
    /// @param destination_node_id node id of the server
    /// @param msg message containing the request
    /// @return true if the request was put into the queue successfully
    bool request(uint8_t destination_node_id, typename rsptype::cxx_iface::reqtype& msg) {
        return request(destination_node_id, msg, interface.is_canfd());
    }

    /// @brief makes service request with CAN FD option
    /// @param destination_node_id node id of the server
    /// @param msg message containing the request
    /// @param canfd true if CAN FD is to be used
    /// @return true if the request was put into the queue successfully
    bool request(uint8_t destination_node_id, typename rsptype::cxx_iface::reqtype& msg, bool canfd) NOINLINE_FUNC {
#if !CANARD_ENABLE_CANFD
        if (canfd) {
            return false;
        }
#endif
        // encode the message
        uint32_t len = rsptype::cxx_iface::req_encode(&msg, req_buf 
#if CANARD_ENABLE_CANFD
        , !canfd
#elif CANARD_ENABLE_TAO_OPTION
        , true
#endif
        );
        Transfer req_transfer;
        // send the message if encoded successfully
        req_transfer.transfer_type = CanardTransferTypeRequest;
        req_transfer.data_type_id = rsptype::cxx_iface::ID;
        req_transfer.data_type_signature = rsptype::cxx_iface::SIGNATURE;
        req_transfer.payload = req_buf;
        req_transfer.payload_len = len;
#if CANARD_ENABLE_CANFD
        req_transfer.canfd = canfd;
#endif
#if CANARD_MULTI_IFACE
        req_transfer.iface_mask = CANARD_IFACE_ALL;
#endif
        transfer_id = *TransferObject::get_tid_ptr(interface.get_index(), rsptype::cxx_iface::ID, CanardTransferTypeRequest, interface.get_node_id(), destination_node_id);
        server_node_id = destination_node_id;
        return send(req_transfer, destination_node_id);
    }

private:
    uint8_t server_node_id;

    uint8_t req_buf[rsptype::cxx_iface::REQ_MAX_SIZE];
    Callback<rsptype> &cb;
    uint8_t transfer_id;
};

} // namespace Canard
