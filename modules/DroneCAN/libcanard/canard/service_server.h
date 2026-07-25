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

namespace Canard {

/// @brief Server class to handle service requests
/// @tparam reqtype 
template <typename reqtype>
class Server : public HandlerList {

public:
    /// @brief Server constructor
    /// @param _interface Interface object
    /// @param _cb Callback object
    /// @param _index HandlerList instance id
    Server(Interface &_interface, Callback<reqtype> &_cb) NOINLINE_FUNC :
    HandlerList(CanardTransferTypeRequest, reqtype::cxx_iface::ID, reqtype::cxx_iface::SIGNATURE, _interface.get_index()),
    interface(_interface),
    cb(_cb) {
        link(); // link ourselves into the handler list
    }

    // delete copy constructor and assignment operator
    Server(const Server&) = delete;

    ~Server() NOINLINE_FUNC {
        unlink(); // unlink ourselves from the handler list
    }

    /// @brief handles incoming messages
    /// @param transfer transfer object of the request
    /// @return true if message has been handled
    bool handle_message(const CanardRxTransfer& transfer) override NOINLINE_FUNC {
        reqtype msg {};
        if (!reqtype::cxx_iface::req_decode(&transfer, &msg)) {
            transfer_id = transfer.transfer_id;
            cb(transfer, msg);
            return true;
        }
        return false;
    }

    /// @brief Send a response to the request
    /// @param transfer transfer object of the request
    /// @param msg message containing the response
    /// @return true if the response was put into the queue successfully
    bool respond(const CanardRxTransfer& transfer, typename reqtype::cxx_iface::rsptype& msg) NOINLINE_FUNC {
        // encode the message
        uint32_t len = reqtype::cxx_iface::rsp_encode(&msg, rsp_buf
#if CANARD_ENABLE_CANFD
        , !transfer.canfd
#elif CANARD_ENABLE_TAO_OPTION
        , true
#endif
        );
        // send the message if encoded successfully
        if (len > 0) {
            Transfer rsp_transfer;
#if CANARD_ENABLE_CANFD
            rsp_transfer.canfd = transfer.canfd;
#endif
#if CANARD_MULTI_IFACE
            rsp_transfer.iface_mask = iface_mask;
#endif
            rsp_transfer.transfer_type = CanardTransferTypeResponse;
            rsp_transfer.inout_transfer_id = &transfer_id;
            rsp_transfer.data_type_id = reqtype::cxx_iface::ID;
            rsp_transfer.data_type_signature = reqtype::cxx_iface::SIGNATURE;
            rsp_transfer.payload = rsp_buf;
            rsp_transfer.payload_len = len;
            rsp_transfer.priority = transfer.priority;
            rsp_transfer.timeout_ms = timeout;
            return interface.respond(transfer.source_node_id, rsp_transfer);
        }
        return false;
    }

    /// @brief Set the timeout for the response
    /// @param _timeout timeout in milliseconds
    void set_timeout_ms(uint32_t _timeout) {
        timeout = _timeout;
    }

private:
    uint8_t rsp_buf[reqtype::cxx_iface::RSP_MAX_SIZE];
    Interface &interface;
    Callback<reqtype> &cb;

    uint32_t timeout = 1000;
    uint8_t transfer_id = 0;
#if CANARD_MULTI_IFACE
    uint8_t iface_mask = CANARD_IFACE_ALL;
#endif
};

} // namespace Canard
