/*
  generic object to allow a script to use a serial stream
 */

#include "AP_Scripting_config.h"
#include "AP_Scripting_SerialAccess.h"

#if AP_SCRIPTING_ENABLED

void AP_Scripting_SerialAccess::begin(uint32_t baud)
{
    stream->begin(baud);
}

size_t AP_Scripting_SerialAccess::write(uint8_t c)
{
    return write(&c, 1);
}

size_t AP_Scripting_SerialAccess::write(const uint8_t *buffer, size_t size)
{
    return stream->write(buffer, size);
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
    return stream->read(buffer, count);
}

uint32_t AP_Scripting_SerialAccess::available(void)
{
    return stream->available();
}

void AP_Scripting_SerialAccess::set_flow_control(enum AP_HAL::UARTDriver::flow_control fcs)
{
    stream->set_flow_control(fcs);
}

#endif // AP_SCRIPTING_ENABLED
