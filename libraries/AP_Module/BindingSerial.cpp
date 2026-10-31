// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingSerial.h"

#if AP_MODULE_ENABLED
#include <AP_Module/BindingConfig.h>
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_SerialManager/AP_SerialManager.h>
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
#include <AP_Scripting/AP_Scripting_SerialDevice.h>
#endif

namespace AP_Module
{

static constexpr uint32_t SIMULATED = 0x80000000U;
static constexpr uint32_t SIMULATED_MARKER = 1U;
static HAL_Semaphore serial_sem;

struct ResolvedSerial {
    AP_HAL::UARTDriver *stream;
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    AP_Scripting_SerialDevice::Port *device;
#endif
};

static int32_t resolve(uint32_t token, ResolvedSerial &resolved)
{
    resolved.stream = nullptr;
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    resolved.device = nullptr;
#endif
    if (token == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    if ((token & SIMULATED) == 0) {
        if (token > 256U) {
            return AP_MODULE_STATUS_INVALID;
        }
#if AP_SERIALMANAGER_ENABLED
        resolved.stream = AP::serialmanager().find_serial(
            AP_SerialManager::SerialProtocol_Scripting, uint8_t(token - 1U), false);
        return resolved.stream == nullptr ? AP_MODULE_STATUS_UNAVAILABLE : AP_MODULE_STATUS_OK;
#else
        return AP_MODULE_STATUS_UNAVAILABLE;
#endif
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if ((token & SIMULATED_MARKER) == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint8_t generation = uint8_t((token >> 24U) & 0x7fU);
    const int8_t protocol = int8_t((token >> 17U) & 0x7fU);
    const uint16_t instance = uint16_t((token >> 1U) & 0xffffU);
    auto *broker = AP_Scripting_SerialDevice::get_singleton();
    if (broker == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    resolved.device = broker->find_port(protocol, instance, generation);
    resolved.stream = resolved.device;
    return resolved.stream == nullptr ? AP_MODULE_STATUS_UNAVAILABLE : AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static int32_t validate_no_output(const uint8_t *input, uint32_t input_size,
                                  uint32_t expected_input, uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != expected_input || output != nullptr || output_size != 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_find(const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || input_size != 1 || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    BindingLock lock(serial_sem);
#if AP_SERIALMANAGER_ENABLED
    if (AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Scripting, input[0]) != nullptr) {
        BindingWire::write(output, uint32_t(input[0]) + 1U, 4);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_serial_find_simulated(const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || input_size != 3 || output_size != 4 || input[0] > INT8_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    BindingLock lock(serial_sem);
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    auto *broker = AP_Scripting_SerialDevice::get_singleton();
    if (broker == nullptr || broker->generation() == 0) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const uint16_t instance = uint16_t(BindingWire::read(input + 1, 2));
    if (broker->find_port(int8_t(input[0]), instance, broker->generation()) != nullptr) {
        const uint32_t token = SIMULATED | (uint32_t(broker->generation()) << 24U) |
                               (uint32_t(input[0]) << 17U) | (uint32_t(instance) << 1U) |
                               SIMULATED_MARKER;
        BindingWire::write(output, token, 4);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_serial_begin(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    const int32_t valid = validate_no_output(input, input_size, 9, output, output_size);
    if (valid != AP_MODULE_STATUS_OK || input[4] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint32_t baud = BindingWire::read(input + 5, 4);
    if ((input[4] == 0) != (baud == 0)) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
#endif
    port.stream->begin(input[4] ? baud : port.stream->get_baud_rate());
    return AP_MODULE_STATUS_OK;
}

static int32_t one_byte_setting(const uint8_t *input, uint32_t input_size,
                                uint8_t *output, uint32_t output_size, uint8_t maximum,
                                void (AP_HAL::UARTDriver::*apply)(uint8_t))
{
    const int32_t valid = validate_no_output(input, input_size, 5, output, output_size);
    if (valid != AP_MODULE_STATUS_OK || input[4] > maximum) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
#endif
    (port.stream->*apply)(input[4]);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_configure_parity(const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    return one_byte_setting(input, input_size, output, output_size, 2,
                            &AP_HAL::UARTDriver::configure_parity);
}

int32_t binding_serial_set_stop_bits(const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (input != nullptr && input_size == 5 && input[4] == 0) {
        return AP_MODULE_STATUS_INVALID;
    }
    // set_stop_bits takes int, so keep this separate from the uint8_t helper.
    const int32_t valid = validate_no_output(input, input_size, 5, output, output_size);
    if (valid != AP_MODULE_STATUS_OK || input[4] < 1 || input[4] > 2) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
#endif
    port.stream->set_stop_bits(input[4]);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_set_unbuffered_writes(const uint8_t *input, uint32_t input_size,
                                             uint8_t *output, uint32_t output_size)
{
    const int32_t valid = validate_no_output(input, input_size, 5, output, output_size);
    if (valid != AP_MODULE_STATUS_OK || input[4] > 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
#endif
    port.stream->set_unbuffered_writes(input[4] != 0);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_set_flow_control(const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    const int32_t valid = validate_no_output(input, input_size, 5, output, output_size);
    if (valid != AP_MODULE_STATUS_OK || input[4] > AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        return AP_MODULE_STATUS_OK;
    }
#endif
    port.stream->set_flow_control(AP_HAL::UARTDriver::flow_control(input[4]));
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_available(const uint8_t *input, uint32_t input_size,
                                 uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || input_size != 4 || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    uint32_t available;
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        available = port.device->device_available();
    } else
#endif
    {
        available = port.stream->available();
    }
    BindingWire::write(output, available, 4);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_read(const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr || input_size != 6 ||
        output_size != AP_MODULE_SERIAL_TRANSFER_MAX + 2U) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > AP_MODULE_SERIAL_TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    ssize_t read;
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        read = port.device->device_read(output + 2, count);
    } else
#endif
    {
        read = port.stream->read(output + 2, count);
    }
    if (read < 0 || read > count) {
        memset(output, 0, output_size);
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    BindingWire::write(output, uint16_t(read), 2);
    return AP_MODULE_STATUS_OK;
}

int32_t binding_serial_write(const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || output == nullptr ||
        input_size != AP_MODULE_SERIAL_TRANSFER_MAX + 6U || output_size != 2) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint16_t count = uint16_t(BindingWire::read(input + 4, 2));
    if (count == 0 || count > AP_MODULE_SERIAL_TRANSFER_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    for (uint16_t i = count; i < AP_MODULE_SERIAL_TRANSFER_MAX; i++) {
        if (input[6 + i] != 0) {
            return AP_MODULE_STATUS_INVALID;
        }
    }
    BindingLock lock(serial_sem);
    ResolvedSerial port;
    const int32_t status = resolve(BindingWire::read(input, 4), port);
    if (status != AP_MODULE_STATUS_OK) {
        return status;
    }
    size_t written;
#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_SERIALDEVICE_ENABLED
    if (port.device != nullptr) {
        written = port.device->device_write(input + 6, count);
    } else
#endif
    {
        written = port.stream->write(input + 6, count);
    }
    if (written > count) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    BindingWire::write(output, uint16_t(written), 2);
    return AP_MODULE_STATUS_OK;
}

}
#endif // AP_MODULE_ENABLED
