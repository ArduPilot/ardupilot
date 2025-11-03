/*
  generic object to allow a script to use a serial driver stream from both
  driver and device perspectives
 */

#include "AP_Scripting_config.h"
#include "AP_Scripting.h"
#include "AP_Scripting_SerialAccess.h"

#if AP_SCRIPTING_ENABLED

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
#define check_is_device_port() (is_device_port)
#define ON_DEVICE_PORT(func, ...) (((AP_Scripting_SerialDevice::Port*)stream)->device_##func (__VA_ARGS__))
#else
#define check_is_device_port() (false)
#define ON_DEVICE_PORT(...) (0) // not executed
#endif

void AP_Scripting_SerialAccess::begin(uint32_t baud)
{
    if (!check_is_device_port()) {
        stream->begin(baud);
    }
}

void AP_Scripting_SerialAccess::begin()
{
    if (!check_is_device_port()) {
        stream->begin(stream->get_baud_rate());
    }
}

void AP_Scripting_SerialAccess::configure_parity(uint8_t parity) {
    if (!check_is_device_port()) {
        stream->configure_parity(parity);
    }
}

void AP_Scripting_SerialAccess::set_stop_bits(uint8_t stop_bits) {
    if (!check_is_device_port()) {
        stream->set_stop_bits(stop_bits);
    }
}

void AP_Scripting_SerialAccess::set_unbuffered_writes(bool on) {
    if (!check_is_device_port()) {
        stream->set_unbuffered_writes(on);
    }
}

size_t AP_Scripting_SerialAccess::write(uint8_t c)
{
    return write(&c, 1);
}

size_t AP_Scripting_SerialAccess::write(const uint8_t *buffer, size_t size)
{
    if (!check_is_device_port()) {
        return stream->write(buffer, size);
    }
    return ON_DEVICE_PORT(write, buffer, size);
}

int16_t AP_Scripting_SerialAccess::read(void)
{
    uint8_t c;
    if (read(&c, 1) != 1) {
        return -1;
    }
    return c;
}

ssize_t AP_Scripting_SerialAccess::read(uint8_t* buffer, uint16_t count)
{
    if (!check_is_device_port()) {
        return stream->read(buffer, count);
    }
    return ON_DEVICE_PORT(read, buffer, count);
}

uint32_t AP_Scripting_SerialAccess::available(void)
{
    if (!check_is_device_port()) {
        return stream->available();
    }
    return ON_DEVICE_PORT(available);
}

void AP_Scripting_SerialAccess::set_flow_control(enum AP_HAL::UARTDriver::flow_control fcs)
{
    if (!check_is_device_port()) {
        stream->set_flow_control(fcs);
    }
}

#endif // AP_SCRIPTING_ENABLED
