#include <AP_HAL/AP_HAL.h>

#include <emscripten/emscripten.h>

#include "UARTDriver.h"

using namespace HALWASM;

void UARTDriver::_begin(uint32_t /*baud*/, uint16_t /*rxSpace*/, uint16_t /*txSpace*/)
{
    _initialized = true;
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return _tx_buf.write(buffer, size);
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    return _rx_buf.read(buffer, count);
}

void UARTDriver::_end() {}
void UARTDriver::_flush() {}

bool UARTDriver::tx_pending()
{
    return !_tx_buf.is_empty();
}

uint32_t UARTDriver::_available()
{
    return _rx_buf.available();
}

bool UARTDriver::_discard_input()
{
    _rx_buf.advance(_rx_buf.available());
    return true;
}

uint32_t UARTDriver::txspace()
{
    return _tx_buf.space();
}

size_t UARTDriver::js_write(const uint8_t *buf, size_t len)
{
    return _rx_buf.write(buf, len);
}

size_t UARTDriver::js_read(uint8_t *buf, size_t max_len)
{
    return _tx_buf.read(buf, max_len);
}

size_t UARTDriver::js_read_available() const
{
    return _tx_buf.available();
}

extern "C" {

    EMSCRIPTEN_KEEPALIVE void *ardupilot_malloc(size_t size)
    {
        return malloc(size);
    }

    EMSCRIPTEN_KEEPALIVE void ardupilot_free(void *ptr)
    {
        free(ptr);
    }

} // extern "C"
