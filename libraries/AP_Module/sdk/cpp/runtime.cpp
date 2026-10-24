// SPDX-License-Identifier: GPL-3.0-or-later
#include "runtime.hpp"
#include "new"

namespace {
const AP_ModuleHostApi *host;
void *host_context;
void *allocate(size_t n, size_t alignment, bool may_fail)
{
    void *p = nullptr;
    if (n <= UINT32_MAX && alignment <= 4096) {
        p = host->allocate(host_context, uint32_t(n == 0 ? 1 : n), uint32_t(alignment));
    }
    if (p == nullptr && !may_fail) { ap_module::fatal("C++ allocation failed"); }
    if (p != nullptr) {
        auto *bytes = static_cast<uint8_t *>(p);
        for (size_t i = 0; i < n; i++) { bytes[i] = 0; }
    }
    return p;
}
void deallocate(void *p) { if (p != nullptr) { host->deallocate(host_context, static_cast<uint8_t *>(p)); } }
}
extern "C" int32_t ap_module_run(const ap_module::ContextBase *context);
extern "C" int32_t start(const AP_ModuleHostApi *api, void *raw)
{
    if (api == nullptr || api->version != AP_MODULE_ABI_VERSION || api->size < sizeof(*api)) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint8_t expected[] = AP_MODULE_API_FINGERPRINT;
    for (size_t i = 0; i < sizeof(expected); i++) {
        if (api->fingerprint[i] != expected[i]) { return AP_MODULE_STATUS_INVALID; }
    }
    if (api->allocate == nullptr || api->deallocate == nullptr || api->time_us == nullptr ||
        api->sleep_ms == nullptr || api->stop_requested == nullptr || api->text == nullptr ||
        api->panic == nullptr || api->binding_call == nullptr) { return AP_MODULE_STATUS_INVALID; }
    host = api;
    host_context = raw;
    const ap_module::ContextBase context(*api, raw);
    return ap_module_run(&context);
}
[[noreturn]] void ap_module::fatal(const char *message)
{
    uint32_t length = 0;
    while (length < 512 && message[length] != 0) { length++; }
    host->panic(host_context, reinterpret_cast<const uint8_t *>(message), length);
    __builtin_trap();
}
void *operator new(size_t n) { return allocate(n, alignof(max_align_t), false); }
void *operator new[](size_t n) { return ::operator new(n); }
void *operator new(size_t n, const std::nothrow_t &) noexcept { return allocate(n, alignof(max_align_t), true); }
void *operator new[](size_t n, const std::nothrow_t &t) noexcept { return ::operator new(n, t); }
void *operator new(size_t n, std::align_val_t a) { return allocate(n, size_t(a), false); }
void *operator new[](size_t n, std::align_val_t a) { return ::operator new(n, a); }
void *operator new(size_t n, std::align_val_t a, const std::nothrow_t &) noexcept { return allocate(n, size_t(a), true); }
void *operator new[](size_t n, std::align_val_t a, const std::nothrow_t &t) noexcept { return ::operator new(n, a, t); }
void operator delete(void *p) noexcept { deallocate(p); }
void operator delete[](void *p) noexcept { deallocate(p); }
void operator delete(void *p, size_t) noexcept { deallocate(p); }
void operator delete[](void *p, size_t) noexcept { deallocate(p); }
void operator delete(void *p, std::align_val_t) noexcept { deallocate(p); }
void operator delete[](void *p, std::align_val_t) noexcept { deallocate(p); }
void operator delete(void *p, size_t, std::align_val_t) noexcept { deallocate(p); }
void operator delete[](void *p, size_t, std::align_val_t) noexcept { deallocate(p); }
extern "C" void *memcpy(void *dst, const void *src, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    const auto *s = static_cast<const uint8_t *>(src);
    for (size_t i = 0; i < n; i++) { d[i] = s[i]; }
    return dst;
}
extern "C" void *memmove(void *dst, const void *src, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    const auto *s = static_cast<const uint8_t *>(src);
    if (reinterpret_cast<uintptr_t>(d) <= reinterpret_cast<uintptr_t>(s)) { return memcpy(dst, src, n); }
    for (size_t i = n; i != 0; i--) { d[i - 1] = s[i - 1]; }
    return dst;
}
extern "C" void *memset(void *dst, int c, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    for (size_t i = 0; i < n; i++) { d[i] = uint8_t(c); }
    return dst;
}
extern "C" int memcmp(const void *a, const void *b, size_t n)
{
    const auto *x = static_cast<const uint8_t *>(a);
    const auto *y = static_cast<const uint8_t *>(b);
    for (size_t i = 0; i < n; i++) { if (x[i] != y[i]) { return int(x[i]) - int(y[i]); } }
    return 0;
}
extern "C" void __cxa_pure_virtual() { ap_module::fatal("C++ pure virtual call"); }
#if defined(__arm__)
extern "C" void __aeabi_memcpy(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memcpy4(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memcpy8(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memclr(void *d, size_t n) { memset(d, 0, n); }
extern "C" void __aeabi_memclr4(void *d, size_t n) { memset(d, 0, n); }
extern "C" void __aeabi_memclr8(void *d, size_t n) { memset(d, 0, n); }
// AAPCS memory helpers use size before the fill byte, unlike memset().
extern "C" void __aeabi_memset(void *d, size_t n, int c) { memset(d, c, n); }
extern "C" void __aeabi_memset4(void *d, size_t n, int c) { memset(d, c, n); }
extern "C" void __aeabi_memset8(void *d, size_t n, int c) { memset(d, c, n); }
extern "C" void __aeabi_memmove(void *d, const void *s, size_t n) { memmove(d, s, n); }
extern "C" void __aeabi_memmove4(void *d, const void *s, size_t n) { memmove(d, s, n); }
extern "C" void __aeabi_memmove8(void *d, const void *s, size_t n) { memmove(d, s, n); }

extern "C" uint64_t ap_module_udivmod(uint64_t numerator, uint64_t denominator, uint64_t *remainder)
{
    if (denominator == 0) { ap_module::fatal("C++ division by zero"); }
    uint64_t bit = 1;
    while (denominator <= (numerator >> 1)) {
        denominator <<= 1;
        bit <<= 1;
    }
    uint64_t quotient = 0;
    while (bit != 0) {
        if (numerator >= denominator) {
            numerator -= denominator;
            quotient |= bit;
        }
        denominator >>= 1;
        bit >>= 1;
    }
    *remainder = numerator;
    return quotient;
}
extern "C" uint64_t ap_module_sdivmod(int64_t numerator, int64_t denominator, uint64_t *remainder)
{
    // Unsigned magnitudes also cover INT64_MIN without signed overflow.
    const uint64_t n = numerator < 0 ? -uint64_t(numerator) : uint64_t(numerator);
    const uint64_t d = denominator < 0 ? -uint64_t(denominator) : uint64_t(denominator);
    const auto quotient = ap_module_udivmod(n, d, remainder);
    if (numerator < 0) { *remainder = -*remainder; }
    return (numerator < 0) != (denominator < 0) ? -quotient : quotient;
}

// AEABI division returns quotient in r0:r1 and remainder in r2:r3, not a
// C++ aggregate via a hidden pointer. Keep the gate aligned and preserve r9.
#define AP_MODULE_DIVMOD_GATE(helper) \
    asm volatile("push {r4, lr}\n" \
                 "sub sp, sp, #16\n" \
                 "add r4, sp, #8\n" \
                 "str r4, [sp]\n" \
                 "bl " #helper "\n" \
                 "ldrd r2, r3, [sp, #8]\n" \
                 "add sp, sp, #16\n" \
                 "pop {r4, pc}\n")
extern "C" __attribute__((naked)) uint64_t __aeabi_uldivmod(uint64_t, uint64_t)
{
    AP_MODULE_DIVMOD_GATE(ap_module_udivmod);
}
extern "C" __attribute__((naked)) int64_t __aeabi_ldivmod(int64_t, int64_t)
{
    AP_MODULE_DIVMOD_GATE(ap_module_sdivmod);
}
#undef AP_MODULE_DIVMOD_GATE
#endif
