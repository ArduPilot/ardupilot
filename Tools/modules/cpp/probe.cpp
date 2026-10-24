// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>
#include <new>

#ifndef MODULE_PROBE_TEXT
#define MODULE_PROBE_TEXT "ready"
#endif

namespace {
uint32_t generation = 41;
struct Counter {
    virtual uint32_t next() = 0;
    virtual ~Counter() = default;
};
struct OwnedCounter : Counter {
    uint8_t *memory = new uint8_t[512];
    uint32_t next() override { return generation++; }
    ~OwnedCounter() override { delete[] memory; }
};
__attribute__((noinline)) uint32_t next(Counter &c) { return c.next(); }

__attribute__((noinline)) bool runtime_helpers()
{
    struct Division { uint64_t n, d, q, r; };
    const Division cases[] = {
        {0, 1, 0, 0}, {UINT64_MAX, 1, UINT64_MAX, 0},
        {UINT64_MAX, UINT64_MAX, 1, 0},
        {UINT64_MAX, uint64_t(1) << 63, 1, UINT64_MAX >> 1},
        {0xfedcba9876543210ULL, 0x123456789ULL, 0xe0000000ULL, 0x96543210ULL},
    };
    for (const auto &test : cases) {
        volatile uint64_t n = test.n, d = test.d;
        if (n / d != test.q || n % d != test.r) { return false; }
    }
    for (int sign = -1; sign <= 1; sign += 2) {
        volatile int64_t n = -INT64_MAX, d = int64_t(sign) * 4294967297LL;
        if (n / d != int64_t(-sign) * 2147483647LL || n % d != -2147483648LL) { return false; }
    }
    volatile int64_t minimum = INT64_MIN, one = 1;
    if (minimum / one != INT64_MIN || minimum % one != 0) { return false; }
    alignas(8) uint8_t bytes[32]{};
    volatile size_t size = 16;
    __builtin_memset(bytes, 0x5a, size);
    __builtin_memmove(bytes + 4, bytes, size);
    __builtin_memmove(bytes, bytes + 4, size);
    for (size_t i = 0; i < 32; i++) {
        if (bytes[i] != (i < 20 ? 0x5a : 0)) { return false; }
    }
    return true;
}
}

static ap_module::Result<void> run(ap_module::Context &context)
{
    OwnedCounter counter;
    if (next(counter) != 41 || !runtime_helpers()) { return {ap_module::Error::Host}; }
    ap_module::Vector3f vector{3.0f, 4.0f, 0.0f};
    auto length = context.vector3f().length(vector);
    if (!length || length.value != 5.0f) { return {ap_module::Error::Host}; }
    char message[32] = MODULE_PROBE_TEXT;
    auto sent = context.text(6, message);
    if (!sent) { return sent; }
    while (!context.stop_requested()) {
        const auto slept = context.sleep_ms(1000);
        if (!slept && slept.error != ap_module::Error::Stopped) { return slept; }
    }
    return {ap_module::Error::Ok};
}
AP_MODULE(run)
