// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_EFI/AP_EFI.h>
#if AP_EFI_SCRIPTING_ENABLED
#include <AP_EFI/AP_EFI_Scripting.h>
#endif

namespace AP_Module
{

static constexpr uint32_t EFI_STATE_WIRE_SIZE = 101U;

#if AP_EFI_SCRIPTING_ENABLED
static inline void binding_efi_encode_state(uint8_t *wire, const EFI_State &state)
{
    BindingWire::write(wire, state.last_updated_ms, 4);
    wire[4] = uint8_t(state.engine_state);
    wire[5] = state.general_error;
    wire[6] = uint8_t(state.crankshaft_sensor_status);
    wire[7] = uint8_t(state.temperature_status);
    wire[8] = uint8_t(state.fuel_pressure_status);
    wire[9] = uint8_t(state.oil_pressure_status);
    wire[10] = uint8_t(state.detonation_status);
    wire[11] = uint8_t(state.misfire_status);
    wire[12] = uint8_t(state.debris_status);
    wire[13] = state.engine_load_percent;
    BindingWire::write(wire + 14, state.engine_speed_rpm, 4);
    BindingWire::write_float(wire + 18, state.spark_dwell_time_ms);
    BindingWire::write_float(wire + 22, state.atmospheric_pressure_kpa);
    BindingWire::write_float(wire + 26, state.intake_manifold_pressure_kpa);
    BindingWire::write_float(wire + 30, state.intake_manifold_temperature);
    BindingWire::write_float(wire + 34, state.coolant_temperature);
    BindingWire::write_float(wire + 38, state.oil_pressure);
    BindingWire::write_float(wire + 42, state.oil_temperature);
    BindingWire::write_float(wire + 46, state.fuel_pressure);
    BindingWire::write_float(wire + 50, state.fuel_consumption_rate_cm3pm);
    BindingWire::write_float(wire + 54, state.estimated_consumed_fuel_volume_cm3);
    wire[58] = state.throttle_position_percent;
    wire[59] = state.ecu_index;
    wire[60] = uint8_t(state.spark_plug_usage);
    BindingWire::write_float(wire + 61, state.cylinder_status.ignition_timing_deg);
    BindingWire::write_float(wire + 65, state.cylinder_status.injection_time_ms);
    BindingWire::write_float(wire + 69, state.cylinder_status.cylinder_head_temperature);
    BindingWire::write_float(wire + 73, state.cylinder_status.cylinder_head_temperature2);
    BindingWire::write_float(wire + 77, state.cylinder_status.exhaust_gas_temperature);
    BindingWire::write_float(wire + 81, state.cylinder_status.exhaust_gas_temperature2);
    BindingWire::write_float(wire + 85, state.cylinder_status.lambda_coefficient);
    BindingWire::write_float(wire + 89, state.ignition_voltage);
    BindingWire::write_float(wire + 93, state.throttle_out);
    BindingWire::write_float(wire + 97, state.pt_compensation);
}
#endif

static inline int32_t binding_efi_get_state(const uint8_t *input, uint32_t input_size,
                                            uint8_t *output, uint32_t output_size)
{
    if (input != nullptr || input_size != 0 || output == nullptr || output_size != EFI_STATE_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_EFI_SCRIPTING_ENABLED
    auto *efi = AP_EFI::get_singleton();
    if (efi == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    EFI_State state{};
    efi->get_state(state);
    binding_efi_encode_state(output, state);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_efi_get_backend(const uint8_t *input, uint32_t input_size,
                                              uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 1 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_EFI_SCRIPTING_ENABLED
    auto *efi = AP_EFI::get_singleton();
    output[0] = efi != nullptr && efi->get_backend(input[0]) != nullptr;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_efi_handle_scripting(const uint8_t *input, uint32_t input_size,
                                                   uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != EFI_STATE_WIRE_SIZE + 1 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_EFI_SCRIPTING_ENABLED
    const uint8_t *wire = input + 1;
    if (wire[5] > 1 || wire[4] > 3 || wire[6] > 2 || wire[7] > 5 || wire[8] > 3 || wire[9] > 3 ||
        wire[10] > 2 || wire[11] > 2 || wire[12] > 2 || wire[60] > 3) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *efi = AP_EFI::get_singleton();
    auto *backend = efi == nullptr ? nullptr : efi->get_backend(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    EFI_State state{};
    state.last_updated_ms = BindingWire::read(wire, 4);
    state.engine_state = static_cast<Engine_State>(wire[4]);
    state.general_error = wire[5];
    state.crankshaft_sensor_status = static_cast<Crankshaft_Sensor_Status>(wire[6]);
    state.temperature_status = static_cast<Temperature_Status>(wire[7]);
    state.fuel_pressure_status = static_cast<Fuel_Pressure_Status>(wire[8]);
    state.oil_pressure_status = static_cast<Oil_Pressure_Status>(wire[9]);
    state.detonation_status = static_cast<Detonation_Status>(wire[10]);
    state.misfire_status = static_cast<Misfire_Status>(wire[11]);
    state.debris_status = static_cast<Debris_Status>(wire[12]);
    state.engine_load_percent = wire[13];
    state.engine_speed_rpm = BindingWire::read(wire + 14, 4);
    state.spark_dwell_time_ms = BindingWire::read_float(wire + 18);
    state.atmospheric_pressure_kpa = BindingWire::read_float(wire + 22);
    state.intake_manifold_pressure_kpa = BindingWire::read_float(wire + 26);
    state.intake_manifold_temperature = BindingWire::read_float(wire + 30);
    state.coolant_temperature = BindingWire::read_float(wire + 34);
    state.oil_pressure = BindingWire::read_float(wire + 38);
    state.oil_temperature = BindingWire::read_float(wire + 42);
    state.fuel_pressure = BindingWire::read_float(wire + 46);
    state.fuel_consumption_rate_cm3pm = BindingWire::read_float(wire + 50);
    state.estimated_consumed_fuel_volume_cm3 = BindingWire::read_float(wire + 54);
    state.throttle_position_percent = wire[58];
    state.ecu_index = wire[59];
    state.spark_plug_usage = static_cast<Spark_Plug_Usage>(wire[60]);
    state.cylinder_status.ignition_timing_deg = BindingWire::read_float(wire + 61);
    state.cylinder_status.injection_time_ms = BindingWire::read_float(wire + 65);
    state.cylinder_status.cylinder_head_temperature = BindingWire::read_float(wire + 69);
    state.cylinder_status.cylinder_head_temperature2 = BindingWire::read_float(wire + 73);
    state.cylinder_status.exhaust_gas_temperature = BindingWire::read_float(wire + 77);
    state.cylinder_status.exhaust_gas_temperature2 = BindingWire::read_float(wire + 81);
    state.cylinder_status.lambda_coefficient = BindingWire::read_float(wire + 85);
    state.ignition_voltage = BindingWire::read_float(wire + 89);
    state.throttle_out = BindingWire::read_float(wire + 93);
    state.pt_compensation = BindingWire::read_float(wire + 97);
    output[0] = backend->handle_scripting(state);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
