// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <bindings.hpp>

namespace ap_module {
// All resource owners borrow their Context. Keep the Context alive until after
// their destruction. Firmware also reclaims leaked handles after worker join.
class Parameter {
public:
    Parameter() : context(nullptr), storage(nullptr) {}
    Parameter(const Parameter &) = delete;
    Parameter &operator=(const Parameter &) = delete;
    Parameter(Parameter &&other) noexcept : context(other.context), storage(other.storage) { other.storage = nullptr; }
    ~Parameter() { if (storage != nullptr) { context->deallocate(storage); } }
    static Result<Parameter> create(const Context &context, StringView name) {
        auto layout = context.parameter_cache().__layout();
        if (!layout) { return {layout.error, Parameter()}; }
        const auto n = layout.value.size;
        const auto a = layout.value.alignment;
        if (n == 0 || n > 16U * 1024 * 1024 || a == 0 || a > 4096 || (a & (a - 1)) != 0 || n % a != 0) {
            return {Error::Host, Parameter()};
        }
        Parameter result;
        result.context = &context;
        result.storage = context.allocate(n, a);
        if (result.storage == nullptr) { return {Error::NoMemory, Parameter()}; }
        if (reinterpret_cast<uintptr_t>(result.storage) % a != 0) { return {Error::Host, Parameter()}; }
        const auto initialized = context.parameter_cache().init(result.address(), name);
        if (!initialized) { return {initialized.error, Parameter()}; }
        if (!initialized.value) { return {Error::Unavailable, Parameter()}; }
        return {Error::Ok, static_cast<Parameter &&>(result)};
    }
    Result<Optional<float>> get() const {
        if (storage == nullptr) { return {Error::Invalid}; }
        return context->parameter_cache().get(address());
    }
    Result<bool> set(float value) const {
        if (storage == nullptr) { return {Error::Invalid}; }
        return context->parameter_cache().set(address(), value);
    }
    Result<bool> set_and_save(float value) const {
        if (storage == nullptr) { return {Error::Invalid}; }
        return context->parameter_cache().set_and_save(address(), value);
    }
    Result<bool> set_default(float value) const {
        if (storage == nullptr) { return {Error::Invalid}; }
        return context->parameter_cache().set_default(address(), value);
    }
    Result<bool> configured() const {
        if (storage == nullptr) { return {Error::Invalid}; }
        return context->parameter_cache().configured(address());
    }
private:
    uint64_t address() const { return uint64_t(reinterpret_cast<uintptr_t>(storage)); }
    const Context *context;
    void *storage;
};

template<class Tag> class OwnedHandle {
public:
    OwnedHandle(const Context &context, uint32_t token) : context(context), token(token) {}
    OwnedHandle(const OwnedHandle &) = delete;
    OwnedHandle &operator=(const OwnedHandle &) = delete;
    OwnedHandle(OwnedHandle &&other) noexcept : context(other.context), token(other.token) { other.token = 0; }
    ~OwnedHandle() { if (token != 0) { Tag::close(context, token); } }
    uint32_t get() const { return token; }
    uint32_t release() { const auto result = token; token = 0; return result; }
private:
    const Context &context;
    uint32_t token;
};
struct CanBufferOwner { static void close(const Context &c, uint32_t n) { c.canbuffer().close(n); } };
struct DroneCanSubscriptionOwner { static void close(const Context &c, uint32_t n) { c.dronecansubscription().close(n); } };
struct DroneCanHandleOwner { static void close(const Context &c, uint32_t n) { c.dronecanhandle().close(n); } };
struct SocketOwner { static void close(const Context &c, uint32_t n) { c.network().socket_close(n); } };
struct I2cOwner { static void close(const Context &c, uint32_t n) { c.i2c().close(n); } };
struct PwmOwner { static void close(const Context &c, uint32_t n) { c.pwm_source().close(n); } };
using CanBuffer = OwnedHandle<CanBufferOwner>;
using DroneCanSubscription = OwnedHandle<DroneCanSubscriptionOwner>;
using DroneCanHandle = OwnedHandle<DroneCanHandleOwner>;
using Socket = OwnedHandle<SocketOwner>;
using I2cDevice = OwnedHandle<I2cOwner>;
using PwmSource = OwnedHandle<PwmOwner>;
} // namespace ap_module
