// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingMotorConfig.h"

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Motors/AP_Motors_config.h>

#if AP_SCRIPTING_ENABLED && (APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_COPTER_OR_HELI)
#define AP_MODULE_MOTOR_CONFIG_ENABLED 1
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#if AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED
#include <AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h>
#endif
#else
#define AP_MODULE_MOTOR_CONFIG_ENABLED 0
#endif

#if AP_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduCopter) && AP_MOTORS_FRAME_6DOF_SCRIPTING_ENABLED
#define AP_MODULE_MOTOR_6DOF_ENABLED 1
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#else
#define AP_MODULE_MOTOR_6DOF_ENABLED 0
#endif

#if AP_SCRIPTING_BINDING_MOTORS_ENABLED && \
    (APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_COPTER_OR_HELI)
#define AP_MODULE_MOTOR_FRAME_STRING_ENABLED 1
#include <AP_Motors/AP_Motors_Class.h>
#else
#define AP_MODULE_MOTOR_FRAME_STRING_ENABLED 0
#endif

namespace AP_Module
{

static constexpr uint8_t MOTOR_FACTOR_COUNT = 32U;
static_assert(sizeof(float) == 4U, "Rust motor factors require 32-bit floats");
static constexpr uint32_t MOTOR_FACTOR_TABLE_WIRE_SIZE = 4U * MOTOR_FACTOR_COUNT * 4U;
static constexpr uint8_t MOTOR_FRAME_STRING_MAX = 32U;
#if AP_MODULE_MOTOR_CONFIG_ENABLED || AP_MODULE_MOTOR_6DOF_ENABLED || AP_MODULE_MOTOR_FRAME_STRING_ENABLED
// This serializes Rust calls, but each successful call permanently mutates shared firmware state.
static HAL_Semaphore motor_config_sem;
#endif

#if AP_MODULE_MOTOR_CONFIG_ENABLED
static_assert(AP_MOTORS_MAX_NUM_MOTORS <= MOTOR_FACTOR_COUNT,
              "Rust motor-factor wire cap is too small");
#endif

static inline int32_t binding_motor_bool_call_valid(const uint8_t *input, uint32_t input_size,
        uint32_t expected_input, uint8_t *output,
        uint32_t output_size)
{
    if (input == nullptr || input_size != expected_input || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
    return AP_MODULE_STATUS_OK;
}

int32_t binding_motors_dynamic_init(const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{
    const int32_t valid = binding_motor_bool_call_valid(input, input_size, 1, output, output_size);
    if (valid != AP_MODULE_STATUS_OK) {
        return valid;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED
    if (input[0] > AP_MOTORS_MAX_NUM_MOTORS) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = motors->init(input[0]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_dynamic_add_motor(const uint8_t *input, uint32_t input_size,
        uint8_t *output, uint32_t output_size)
{
    const int32_t valid = binding_motor_bool_call_valid(input, input_size, 2, output, output_size);
    if (valid != AP_MODULE_STATUS_OK) {
        return valid;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED
    if (input[0] >= AP_MOTORS_MAX_NUM_MOTORS || input[1] > AP_MOTORS_MAX_NUM_MOTORS) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = motors->add_motor(input[0], input[1]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_dynamic_load_factors(const uint8_t *input,
        uint32_t input_size,
        uint8_t *output,
        uint32_t output_size)
{
    if (input == nullptr || input_size != MOTOR_FACTOR_TABLE_WIRE_SIZE ||
        output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED
    AP_MotorsMatrix_Scripting_Dynamic::factor_table table {};
    constexpr uint32_t field_size = MOTOR_FACTOR_COUNT * sizeof(float);
    for (uint8_t field = 0; field < 4; field++) {
        float *destination = field == 0 ? table.roll : field == 1 ? table.pitch :
                             field == 2 ? table.yaw : table.throttle;
        for (uint8_t index = 0; index < MOTOR_FACTOR_COUNT; index++) {
            const float factor = BindingWire::read_float(input + field * field_size + index * sizeof(float));
            if (!std::isfinite(factor)) {
                return AP_MODULE_STATUS_INVALID;
            }
            if (index < AP_MOTORS_MAX_NUM_MOTORS) {
                destination[index] = factor;
            } else if (!is_zero(factor)) {
                return AP_MODULE_STATUS_INVALID;
            }
        }
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix_Scripting_Dynamic::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    motors->load_factors(table);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_matrix_init(const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    const int32_t valid = binding_motor_bool_call_valid(input, input_size, 1, output, output_size);
    if (valid != AP_MODULE_STATUS_OK) {
        return valid;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED
    if (input[0] > AP_MOTORS_MAX_NUM_MOTORS) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = motors->init(input[0]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_matrix_add_motor_raw(const uint8_t *input,
        uint32_t input_size,
        uint8_t *output,
        uint32_t output_size)
{
    if (input == nullptr || input_size != 14 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED
    const float roll = BindingWire::read_float(input + 1);
    const float pitch = BindingWire::read_float(input + 5);
    const float yaw = BindingWire::read_float(input + 9);
    if (input[0] >= AP_MOTORS_MAX_NUM_MOTORS || input[13] > AP_MOTORS_MAX_NUM_MOTORS ||
        !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    motors->add_motor_raw(int8_t(input[0]), roll, pitch, yaw, input[13]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_matrix_set_throttle_factor(const uint8_t *input,
        uint32_t input_size,
        uint8_t *output,
        uint32_t output_size)
{
    const int32_t valid = binding_motor_bool_call_valid(input, input_size, 5, output, output_size);
    if (valid != AP_MODULE_STATUS_OK) {
        return valid;
    }
#if AP_MODULE_MOTOR_CONFIG_ENABLED && AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED
    const float factor = BindingWire::read_float(input + 1);
    if (input[0] >= AP_MOTORS_MAX_NUM_MOTORS || !std::isfinite(factor) || factor < 0.0f) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = motors->set_throttle_factor(int8_t(input[0]), factor);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_6dof_init(const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size)
{
    const int32_t valid = binding_motor_bool_call_valid(input, input_size, 1, output, output_size);
    if (valid != AP_MODULE_STATUS_OK) {
        return valid;
    }
#if AP_MODULE_MOTOR_6DOF_ENABLED
    if (input[0] > AP_MOTORS_MAX_NUM_MOTORS) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix_6DoF_Scripting::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    output[0] = motors->init(input[0]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_6dof_add_motor(const uint8_t *input,
                                      uint32_t input_size,
                                      uint8_t *output,
                                      uint32_t output_size)
{
    if (input == nullptr || input_size != 27 || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_MODULE_MOTOR_6DOF_ENABLED
    float factors[6];
    for (uint8_t index = 0; index < 6; index++) {
        factors[index] = BindingWire::read_float(input + 1 + index * sizeof(float));
        if (!std::isfinite(factors[index])) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    if (input[0] >= AP_MOTORS_MAX_NUM_MOTORS || input[25] > 1 ||
        input[26] > AP_MOTORS_MAX_NUM_MOTORS) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP_MotorsMatrix_6DoF_Scripting::get_singleton();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    motors->add_motor(int8_t(input[0]), factors[0], factors[1], factors[2],
                      factors[3], factors[4], factors[5], input[25] != 0, input[26]);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_motors_set_frame_string(const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != MOTOR_FRAME_STRING_MAX + 1U ||
        output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_MODULE_MOTOR_FRAME_STRING_ENABLED
    const uint8_t length = input[0];
    if (length > MOTOR_FRAME_STRING_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint8_t index = 0; index < MOTOR_FRAME_STRING_MAX; index++) {
        if ((index < length && input[index + 1] == 0) ||
            (index >= length && input[index + 1] != 0)) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    BindingLock lock(motor_config_sem);
    auto *motors = AP::motors();
    if (motors == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    char frame[MOTOR_FRAME_STRING_MAX + 1U] {};
    memcpy(frame, input + 1, length);
    motors->set_frame_string(frame);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}

#undef AP_MODULE_MOTOR_6DOF_ENABLED
#undef AP_MODULE_MOTOR_FRAME_STRING_ENABLED
#undef AP_MODULE_MOTOR_CONFIG_ENABLED
#endif // AP_MODULE_ENABLED
