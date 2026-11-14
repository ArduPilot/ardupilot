// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Servo_Telem/AP_Servo_Telem.h>

namespace AP_Module
{

static inline int32_t binding_esc_telem_update_telem_data(const uint8_t *input, uint32_t input_size,
                                                          uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 12 || output != nullptr || output_size != 0 ||
        input[1] == 0 || (input[1] & ~0x07U) != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_WITH_ESC_TELEM
    if (input[0] >= ESC_TELEM_MAX_ESCS) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *telemetry = AP_ESC_Telem::get_singleton();
    if (telemetry == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    AP_ESC_Telem_Backend::TelemetryData data{};
    data.temperature_cdeg = int16_t(BindingWire::read(input + 2, 2));
    data.voltage = BindingWire::read_float(input + 4);
    data.current = BindingWire::read_float(input + 8);
    uint16_t present = 0;
    if ((input[1] & 0x01U) != 0) {
        present |= AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE;
    }
    if ((input[1] & 0x02U) != 0) {
        present |= AP_ESC_Telem_Backend::TelemetryType::VOLTAGE;
    }
    if ((input[1] & 0x04U) != 0) {
        present |= AP_ESC_Telem_Backend::TelemetryType::CURRENT;
    }
    telemetry->update_telem_data(input[0], data, present);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_servo_telem_get_telem(const uint8_t *input, uint32_t input_size,
                                                    uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 1 || output == nullptr || output_size != 6) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_SERVO_TELEM_ENABLED
    auto *telemetry = AP_Servo_Telem::get_singleton();
    if (telemetry == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    AP_Servo_Telem::TelemetryData data{};
    if (!telemetry->get_telem(input[0], data)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    if (data.present(AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION)) {
        output[1] = 1;
        BindingWire::write_float(output + 2, data.measured_position);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_servo_telem_update_telem_data(const uint8_t *input,
                                                             uint32_t input_size,
                                                             uint8_t *output,
                                                             uint32_t output_size)
{
    if (input == nullptr || input_size != 33 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint16_t present = uint16_t(BindingWire::read(input + 1, 2));
    if (present == 0 || (present & ~0x03FFU) != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_SERVO_TELEM_ENABLED
    auto *telemetry = AP_Servo_Telem::get_singleton();
    if (telemetry == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    AP_Servo_Telem::TelemetryData data{};
    data.present_types = present;
    data.command_position = BindingWire::read_float(input + 3);
    data.measured_position = BindingWire::read_float(input + 7);
    data.force = BindingWire::read_float(input + 11);
    data.speed = BindingWire::read_float(input + 15);
    data.voltage = BindingWire::read_float(input + 19);
    data.current = BindingWire::read_float(input + 23);
    data.duty_cycle = input[27];
    data.motor_temperature_cdeg = int16_t(BindingWire::read(input + 28, 2));
    data.pcb_temperature_cdeg = int16_t(BindingWire::read(input + 30, 2));
    data.status_flags = input[32];
    telemetry->update_telem_data(input[0], data);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
