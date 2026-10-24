// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <HostApi.h>

namespace ap_module {
enum class Error : int32_t {
    Ok = AP_MODULE_STATUS_OK, Invalid = AP_MODULE_STATUS_INVALID,
    Unavailable = AP_MODULE_STATUS_UNAVAILABLE, Stopped = AP_MODULE_STATUS_STOPPED,
    Host = AP_MODULE_STATUS_HOST_ERROR,
    NoMemory = AP_MODULE_STATUS_NO_MEMORY, Busy = AP_MODULE_STATUS_BUSY,
};
template<class T> struct Result {
    Error error;
    T value;
    explicit operator bool() const { return error == Error::Ok; }
};
template<> struct Result<void> {
    Error error;
    explicit operator bool() const { return error == Error::Ok; }
};
template<class T> struct Optional { bool present; T value; };
template<class T, size_t N> struct Array { T data[N]; };
struct StringView {
    const char *data;
    size_t length;
    StringView(const char *s, size_t n) : data(s), length(n) {}
    template<size_t N> StringView(const char (&s)[N]) : data(s), length(0) {
        while (length < N && s[length] != 0) { length++; }
    }
};
class ContextBase {
public:
    ContextBase(const AP_ModuleHostApi &api, void *raw) : _api(api), _raw(raw) {}
    uint64_t time_us() const { return _api.time_us(_raw); }
    bool stop_requested() const { return _api.stop_requested(_raw) != 0; }
    Result<void> sleep_ms(uint32_t ms) const { return {Error(_api.sleep_ms(_raw, ms))}; }
    Result<void> text(uint8_t severity, StringView s) const {
        if (s.length > 512 || (s.data == nullptr && s.length != 0)) { return {Error::Invalid}; }
        return {Error(_api.text(_raw, severity, reinterpret_cast<const uint8_t *>(s.data), uint32_t(s.length)))};
    }
    int32_t call(uint32_t slot, const uint8_t *input, uint32_t ni, uint8_t *output, uint32_t no) const {
        return _api.binding_call(_raw, slot, input, ni, output, no);
    }
    void *allocate(uint32_t size, uint32_t alignment) const { return _api.allocate(_raw, size, alignment); }
    void deallocate(void *p) const { _api.deallocate(_raw, static_cast<uint8_t *>(p)); }
private:
    const AP_ModuleHostApi &_api;
    void *_raw;
};
[[noreturn]] void fatal(const char *message);
namespace wire {
inline uint64_t read(const uint8_t *p, size_t n) {
    uint64_t bits = 0;
    for (size_t i = 0; i < n; i++) { bits |= uint64_t(p[i]) << (8 * i); }
    return bits;
}
inline void write(uint8_t *p, uint64_t bits, size_t n) {
    for (size_t i = 0; i < n; i++) { p[i] = uint8_t(bits >> (8 * i)); }
}
template<class T> struct Codec {
    static constexpr size_t size = sizeof(T);
    static bool put(uint8_t *p, T v) { write(p, uint64_t(v), size); return true; }
    static bool get(const uint8_t *p, T &v) {
        for (size_t i = 0; i < size; i++) { reinterpret_cast<uint8_t *>(&v)[i] = p[i]; }
        return true;
    }
};
template<> struct Codec<bool> {
    static constexpr size_t size = 1;
    static bool put(uint8_t *p, bool v) { *p = v; return true; }
    static bool get(const uint8_t *p, bool &v) { if (*p > 1) { return false; } v = *p; return true; }
};
template<> struct Codec<float> {
    static constexpr size_t size = 4;
    static bool put(uint8_t *p, float v) {
        for (size_t i = 0; i < 4; i++) { p[i] = reinterpret_cast<const uint8_t *>(&v)[i]; } return true;
    }
    static bool get(const uint8_t *p, float &v) {
        for (size_t i = 0; i < 4; i++) { reinterpret_cast<uint8_t *>(&v)[i] = p[i]; } return true;
    }
};
template<class T, size_t N> struct Codec<Array<T, N>> {
    static constexpr size_t size = N * Codec<T>::size;
    static bool put(uint8_t *p, const Array<T, N> &v) {
        for (size_t i = 0; i < N; i++) { if (!Codec<T>::put(p + i * Codec<T>::size, v.data[i])) { return false; } }
        return true;
    }
    static bool get(const uint8_t *p, Array<T, N> &v) {
        for (size_t i = 0; i < N; i++) { if (!Codec<T>::get(p + i * Codec<T>::size, v.data[i])) { return false; } }
        return true;
    }
};
template<class T> struct Codec<Optional<T>> {
    static constexpr size_t size = 1 + Codec<T>::size;
    static bool put(uint8_t *p, const Optional<T> &v) { p[0] = v.present; return !v.present || Codec<T>::put(p + 1, v.value); }
    static bool get(const uint8_t *p, Optional<T> &v) {
        return Codec<bool>::get(p, v.present) && (!v.present || Codec<T>::get(p + 1, v.value));
    }
};
template<size_t N, bool Nonempty = false> inline bool string(uint8_t *p, StringView s) {
    if ((Nonempty && s.length == 0) || s.length >= N || (s.length != 0 && s.data == nullptr)) { return false; }
    for (size_t i = 0; i < s.length; i++) { if (s.data[i] == 0) { return false; } p[i] = uint8_t(s.data[i]); }
    return true;
}
} // namespace wire
} // namespace ap_module
