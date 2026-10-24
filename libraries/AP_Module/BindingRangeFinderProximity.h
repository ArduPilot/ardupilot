// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_Proximity/AP_Proximity_Backend.h>

namespace AP_Module
{

static constexpr uint32_t RANGEFINDER_STATE_WIRE_SIZE = 13U;

static inline bool binding_sensor_valid(const uint8_t *input, uint32_t input_size,
                                        uint32_t expected_input, uint8_t *output,
                                        uint32_t output_size)
{
    if (input == nullptr || input_size != expected_input || output == nullptr || output_size != 1) {
        return false;
    }
    output[0] = 0;
    return true;
}

#if AP_RANGEFINDER_ENABLED
static inline AP_RangeFinder_Backend *binding_rangefinder_resolve(uint8_t index)
{
    auto *frontend = RangeFinder::get_singleton();
    return frontend == nullptr ? nullptr : frontend->get_backend(index);
}
#endif

static inline int32_t binding_rangefinder_get_backend(const uint8_t *input, uint32_t input_size,
                                                       uint8_t *output, uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_RANGEFINDER_ENABLED
    output[0] = binding_rangefinder_resolve(input[0]) != nullptr;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_type(const uint8_t *input, uint32_t input_size,
                                                uint8_t *output, uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_RANGEFINDER_ENABLED
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = uint8_t(backend->type());
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_status(const uint8_t *input, uint32_t input_size,
                                                  uint8_t *output, uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_RANGEFINDER_ENABLED
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const uint8_t status = uint8_t(backend->status());
    if (status > uint8_t(RangeFinder::Status::PoweredDown)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = status;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_orientation(const uint8_t *input, uint32_t input_size,
                                                       uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 1 || output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_RANGEFINDER_ENABLED
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    BindingWire::write(output, int32_t(backend->orientation()), 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_get_state(const uint8_t *input, uint32_t input_size,
                                                     uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 1 || output == nullptr ||
        output_size != RANGEFINDER_STATE_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint32_t i = 0; i < output_size; i++) {
        output[i] = 0;
    }
#if AP_RANGEFINDER_ENABLED && AP_SCRIPTING_ENABLED
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    RangeFinder::RangeFinder_State state{};
    backend->get_state(state);
    const uint8_t status = uint8_t(state.status);
    if (status > uint8_t(RangeFinder::Status::PoweredDown)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    BindingWire::write_float(output, state.distance_m);
    output[4] = uint8_t(state.signal_quality_pct);
    BindingWire::write(output + 5, state.voltage_mv, 2);
    output[7] = status;
    output[8] = state.range_valid_count;
    BindingWire::write(output + 9, state.last_reading_ms, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_handle_script_distance(const uint8_t *input,
                                                                  uint32_t input_size,
                                                                  uint8_t *output,
                                                                  uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 5, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_RANGEFINDER_ENABLED && AP_SCRIPTING_ENABLED
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = backend->handle_script_msg(BindingWire::read_float(input + 1));
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_rangefinder_handle_script_state(const uint8_t *input,
                                                               uint32_t input_size,
                                                               uint8_t *output,
                                                               uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, RANGEFINDER_STATE_WIRE_SIZE + 1U,
                              output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_RANGEFINDER_ENABLED && AP_SCRIPTING_ENABLED
    const uint8_t *wire = input + 1;
    if (wire[7] > uint8_t(RangeFinder::Status::PoweredDown)) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *backend = binding_rangefinder_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    RangeFinder::RangeFinder_State state{};
    state.distance_m = BindingWire::read_float(wire);
    state.signal_quality_pct = int8_t(wire[4]);
    state.voltage_mv = uint16_t(BindingWire::read(wire + 5, 2));
    state.status = static_cast<RangeFinder::Status>(wire[7]);
    state.range_valid_count = wire[8];
    state.last_reading_ms = BindingWire::read(wire + 9, 4);
    output[0] = backend->handle_script_msg(state);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

#if HAL_PROXIMITY_ENABLED
static inline AP_Proximity_Backend *binding_proximity_resolve(uint8_t index)
{
    auto *frontend = AP_Proximity::get_singleton();
    return frontend == nullptr ? nullptr : frontend->get_backend(index);
}
#endif

static inline int32_t binding_proximity_get_backend(const uint8_t *input, uint32_t input_size,
                                                     uint8_t *output, uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED
    output[0] = binding_proximity_resolve(input[0]) != nullptr;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_proximity_type(const uint8_t *input, uint32_t input_size,
                                              uint8_t *output, uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED
    auto *backend = binding_proximity_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = uint8_t(backend->type());
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_proximity_handle_script_distance(const uint8_t *input,
                                                                uint32_t input_size,
                                                                uint8_t *output,
                                                                uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 14, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
    if (input[13] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *backend = binding_proximity_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = backend->handle_script_distance_msg(BindingWire::read_float(input + 1),
                                                     BindingWire::read_float(input + 5),
                                                     BindingWire::read_float(input + 9),
                                                     input[13] != 0);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_proximity_handle_script_3d(const uint8_t *input,
                                                          uint32_t input_size,
                                                          uint8_t *output,
                                                          uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 14, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
    if (input[13] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    auto *backend = binding_proximity_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const Vector3f vector{BindingWire::read_float(input + 1),
                          BindingWire::read_float(input + 5),
                          BindingWire::read_float(input + 9)};
    output[0] = backend->handle_script_3d_msg(vector, input[13] != 0);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_proximity_set_distance_min_max(const uint8_t *input,
                                                              uint32_t input_size,
                                                              uint8_t *output,
                                                              uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 9, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
    auto *backend = binding_proximity_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = backend->set_distance_min_max(BindingWire::read_float(input + 1),
                                              BindingWire::read_float(input + 5));
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static inline int32_t binding_proximity_update_virtual_boundary(const uint8_t *input,
                                                                 uint32_t input_size,
                                                                 uint8_t *output,
                                                                 uint32_t output_size)
{
    if (!binding_sensor_valid(input, input_size, 1, output, output_size)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
    auto *backend = binding_proximity_resolve(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = backend->update_virtual_boundary();
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif // AP_MODULE_ENABLED
