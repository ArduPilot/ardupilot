// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "runtime.hpp"
#include <bindings.hpp>
#include "resources.hpp"

// Local RAII is supported. Automatic global initialization is rejected by the packer.
#define AP_MODULE(run) \
    extern "C" int32_t ap_module_run(const ap_module::ContextBase *base) { \
        ap_module::Context context(*base); \
        return int32_t(run(context).error); \
    }
