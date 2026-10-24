// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>

static ap_module::Result<void> run(ap_module::Context &context)
{
    while (!context.stop_requested()) {
        const auto sent = context.text(6, "C++ tick");
        if (!sent) { return sent; }
        const auto slept = context.sleep_ms(1000);
        if (!slept) { return slept; }
    }
    return {ap_module::Error::Stopped};
}
AP_MODULE(run)
