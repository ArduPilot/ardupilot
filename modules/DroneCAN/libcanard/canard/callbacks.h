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

#include <stdint.h>

namespace Canard {

/// @brief Base class for message callbacks.
template <typename msgtype>
class Callback {
public:
    virtual ~Callback() = default;
    virtual void operator()(const CanardRxTransfer& transfer, const msgtype& msg) = 0;
};

/// @brief Static callback class.
/// @tparam msgtype type of message handled by the callback
template <typename msgtype>
class StaticCallback : public Callback<msgtype> {
public:
    /// @brief constructor
    /// @param _cb callback function
    StaticCallback(void (*_cb)(const CanardRxTransfer& transfer, const msgtype& msg)) : cb(_cb) {}

    void operator()(const CanardRxTransfer& transfer, const msgtype& msg) override {
        cb(transfer, msg);
    }
private:
    void (*cb)(const CanardRxTransfer& transfer, const msgtype& msg);
};

/// @brief allocate a static callback object using new
/// @tparam msgtype 
/// @param cb callback function
/// @return StaticCallback object
template <typename msgtype>
StaticCallback<msgtype> *allocate_static_callback(void (*cb)(const CanardRxTransfer& transfer, const msgtype& msg)) {
    return allocate<StaticCallback<msgtype>>(cb);
}

/// @brief Object callback class.
/// @tparam T type of object to call the callback on
/// @tparam msgtype type of message handled by the callback
template <typename T, typename msgtype>
class ObjCallback : public Callback<msgtype> {
public:
    /// @brief Constructor
    /// @param _obj object to call the callback on
    /// @param _cb callback member function
    ObjCallback(T* _obj, void (T::*_cb)(const CanardRxTransfer& transfer, const msgtype& msg)) : obj(_obj), cb(_cb) {}

    void operator()(const CanardRxTransfer& transfer, const msgtype& msg) override {
        if (obj != nullptr) {
            (obj->*cb)(transfer, msg);
        }
    }
private:
    T *obj;
    void (T::*cb)(const CanardRxTransfer& transfer, const msgtype& msg);
};

/// @brief allocate an object callback object using new
/// @tparam T type of object to call the callback on
/// @tparam msgtype type of message handled by the callback
/// @param obj object to call the callback on
/// @param cb callback member function
/// @return ObjCallback object
template <typename T, typename msgtype>
ObjCallback<T, msgtype>* allocate_obj_callback(T* obj, void (T::*cb)(const CanardRxTransfer& transfer, const msgtype& msg)) {
    return allocate<ObjCallback<T, msgtype>>(obj, cb);
}

/// @brief Argument callback class.
/// @tparam T type of object to pass to the callback
/// @tparam msgtype type of message handled by the callback
template <typename T, typename msgtype>
class ArgCallback : public Callback<msgtype> {
public:
    /// @brief Constructor
    /// @param _arg argument to pass to the callback
    /// @param _cb callback function
    ArgCallback(T* _arg, void (*_cb)(T* arg, const CanardRxTransfer& transfer, const msgtype& msg)) : cb(_cb), arg(_arg) {}

    void operator()(const CanardRxTransfer& transfer, const msgtype& msg) override {
        cb(arg, transfer, msg);
    }
private:
    void (*cb)(T* arg, const CanardRxTransfer& transfer, const msgtype& msg);
    T* arg;
};

/// @brief allocate an argument callback object using new
/// @tparam T type of object to pass to the callback
/// @tparam msgtype type of message handled by the callback
/// @param arg argument to pass to the callback
/// @param cb callback function
/// @return ArgCallback object
template <typename T, typename msgtype>
ArgCallback<T, msgtype>* allocate_arg_callback(T* arg, void (*cb)(T* arg, const CanardRxTransfer& transfer, const msgtype& msg)) {
    return allocate<ArgCallback<T, msgtype>>(arg, cb);
}

} // namespace Canard
