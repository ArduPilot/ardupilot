// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>

extern "C" uint64_t ap_module_aerobatics_time_us(const ap_module::ContextBase *context)
{
    return context->time_us();
}
