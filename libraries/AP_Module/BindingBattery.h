// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BattMonitor/AP_BattMonitor_Scripting.h>

namespace AP_Module
{

static constexpr uint32_t BATTERY_SCRIPTING_WIRE_SIZE = 91U;

static inline int32_t binding_battery_handle_scripting(const uint8_t *input, uint32_t input_size,
                                                        uint8_t *output, uint32_t output_size)
{
    if (input_size != BATTERY_SCRIPTING_WIRE_SIZE || output_size != 1 || input == nullptr || output == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_BATTERY_SCRIPTING_ENABLED
    if (input[5] > 1 || input[6] > 32) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *battery = AP_BattMonitor::get_singleton();
    if (battery == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    BattMonitorScript_State state{};
    state.voltage = BindingWire::read_float(input + 1);
    state.healthy = input[5];
    state.cell_count = input[6];
    state.capacity_remaining_pct = input[7];
    state.state_of_health_pct = input[8];
    for (uint8_t i = 0; i < 32; i++) {
        state.cell_voltages[i] = uint16_t(BindingWire::read(input + 9 + i * 2, 2));
    }
    state.cycle_count = uint16_t(BindingWire::read(input + 73, 2));
    state.current_amps = BindingWire::read_float(input + 75);
    state.consumed_mah = BindingWire::read_float(input + 79);
    state.consumed_wh = BindingWire::read_float(input + 83);
    state.temperature = BindingWire::read_float(input + 87);
    output[0] = battery->handle_scripting(input[0], state);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
