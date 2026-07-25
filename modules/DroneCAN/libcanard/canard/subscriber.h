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
#include "callbacks.h"
#include "handler_list.h"

namespace Canard {

/// @brief Class to handle broadcast messages
/// @tparam msgtype 
template <typename msgtype>
class Subscriber : public HandlerList {
public:
    /// @brief Subscriber Constructor
    /// @param _cb callback function
    /// @param _index HandlerList instance id
    Subscriber(Callback<msgtype> &_cb, uint8_t _index) NOINLINE_FUNC :
    HandlerList(CanardTransferTypeBroadcast, msgtype::cxx_iface::ID, msgtype::cxx_iface::SIGNATURE, _index),
    cb (_cb) {
        // link ourselves into the handler list
        link();
    }

    // delete copy constructor and assignment operator
    Subscriber(const Subscriber&) = delete;

    // destructor, remove the entry from the singly-linked list
    ~Subscriber() NOINLINE_FUNC {
        // unlink ourselves from the handler list
        unlink();
    }

    /// @brief parse the message and call the callback
    /// @param transfer transfer object
    bool handle_message(const CanardRxTransfer& transfer) override NOINLINE_FUNC {
        msgtype msg {};
        if (!msgtype::cxx_iface::decode(&transfer, &msg)) {
            cb(transfer, msg);
            return true;
        }
        return false;
    }

private:
    Callback<msgtype> &cb;
};

template <typename T, typename msgtype>
class SubscriberArgCb {
public:
    SubscriberArgCb(T* _arg, void (*_cb)(T* arg, const CanardRxTransfer&, const msgtype&), uint8_t _index) : arg_cb(_arg, _cb), _sub(arg_cb, _index) {}
    Subscriber<msgtype>& sub() { return _sub; }
private:
    ArgCallback<T, msgtype> arg_cb;
    Subscriber<msgtype> _sub;
};

/// @brief allocate an argument callback object using new
/// @tparam T type of object to pass to the callback
/// @tparam msgtype type of message handled by the callback
/// @param arg argument to pass to the callback
/// @param cb callback function
/// @param index HandlerList instance id
/// @return SubscriberArgCb object
template <typename T, typename msgtype>
SubscriberArgCb<T, msgtype>* allocate_sub_arg_callback(T* _arg, void (*_cb)(T* arg, const CanardRxTransfer& transfer, const msgtype& msg), uint8_t index) {
    return allocate<SubscriberArgCb<T, msgtype>>(_arg, _cb, index);
}

template <typename msgtype>
class SubscriberStaticCb {
public:
    SubscriberStaticCb(void (*_cb)(const CanardRxTransfer&, const msgtype&), uint8_t _index) : static_cb(_cb), _sub(static_cb, _index) {}
    Subscriber<msgtype>& sub() { return _sub; }
private:
    StaticCallback<msgtype> static_cb;
    Subscriber<msgtype> _sub;
};

/// @brief allocate a static callback object using new
/// @tparam msgtype type of message handled by the callback
/// @param cb callback function
/// @param index HandlerList instance id
/// @return SubscriberStaticCb object
template <typename msgtype>
SubscriberStaticCb<msgtype>* allocate_sub_static_callback(void (*cb)(const CanardRxTransfer&, const msgtype&), uint8_t index) {
    return allocate<SubscriberStaticCb<msgtype>>(cb, index);
}

template <typename T, typename msgtype>
class SubscriberObjCb {
public:
    SubscriberObjCb(T* _obj, void (T::*_cb)(const CanardRxTransfer&, const msgtype&), uint8_t _index) : obj_cb(_obj, _cb), _sub(obj_cb, _index) {}
    Subscriber<msgtype>& sub() { return _sub; }
private:
    ObjCallback<T, msgtype> obj_cb;
    Subscriber<msgtype> _sub;
};

/// @brief allocate an object callback object using new
/// @tparam T type of object to pass to the callback
/// @tparam msgtype type of message handled by the callback
/// @param obj object to pass to the callback
/// @param cb callback function
/// @param index HandlerList instance id
/// @return SubscriberObjCb object
template <typename T, typename msgtype>
SubscriberObjCb<T, msgtype>* allocate_sub_obj_callback(T* obj, void (T::*cb)(const CanardRxTransfer& transfer, const msgtype& msg), uint8_t index) {
    return allocate<SubscriberObjCb<T, msgtype>>(obj, cb, index);
}

} // namespace Canard
