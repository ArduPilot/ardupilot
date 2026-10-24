// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>
#include <new>

namespace {
uint32_t generations;
struct Counter {
    virtual uint32_t value() const = 0;
    virtual ~Counter() = default;
};
struct OwnedCounter : Counter {
    uint8_t *memory = new uint8_t[512];
    uint32_t value() const override { return ++generations; }
    ~OwnedCounter() override { delete[] memory; }
};
__attribute__((noinline)) uint32_t virtual_call(const Counter &counter) { return counter.value(); }
}

static ap_module::Result<void> run(ap_module::Context &context)
{
    OwnedCounter counter;
    if (virtual_call(counter) != 1) { return {ap_module::Error::Host}; }
    auto gyro = context.ahrs().get_gyro();
    if (!gyro) { return {gyro.error}; }
    auto length = context.vector3f().length(gyro.value);
    if (!length) { return {length.error}; }
    const auto sent = context.gcs().send_text(ap_module::MavSeverity::INFO, "CPP ready");
    if (!sent) { return sent; }
    while (!context.stop_requested()) {
        auto slept = context.sleep_ms(10);
        if (!slept) { return slept; }
    }
    return {ap_module::Error::Ok};
}
AP_MODULE(run)
