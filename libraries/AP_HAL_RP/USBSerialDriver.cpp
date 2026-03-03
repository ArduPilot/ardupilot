#include "USBSerialDriver.h"

#include "pico/stdio.h"
#include "pico/stdio_usb.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

USBSerialDriver::USBSerialDriver() :
    _baudrate(115200),
    _initialized(false),
    _readbuf{UART_RX_BUFFER_SIZE},
    _writebuf{UART_TX_BUFFER_SIZE},
    _write_mutex{}
{}

void USBSerialDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    _baudrate = baud;
    _readbuf.set_size(rxSpace >= 128 ? rxSpace : 128);
    _writebuf.set_size(txSpace >= 128 ? txSpace : 128);
    stdio_usb_init();
    _initialized = true;
}

void USBSerialDriver::_end()
{
    _initialized = false;
}

void USBSerialDriver::_poll_rx()
{
    if (!_initialized) {
        return;
    }

    while (true) {
        const int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            break;
        }
        const uint8_t c = (uint8_t)ch;
        _readbuf.write(&c, 1);
    }
}

void USBSerialDriver::_flush_tx()
{
    while (_writebuf.available() > 0) {
        uint8_t c;
        if (!_writebuf.read(&c, 1)) {
            break;
        }
        putchar_raw((char)c);
    }
}

size_t USBSerialDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || size == 0) {
        return 0;
    }

    _write_mutex.take_blocking();

    for (size_t i = 0; i < size; i++) {
        const uint8_t required_space = (buffer[i] == '\n') ? 2 : 1;
        while (_writebuf.space() < required_space) {
            if (!hal.scheduler->is_system_initialized()) {
                _flush_tx();
            } else {
                _write_mutex.give();
                hal.scheduler->delay(1);
                _write_mutex.take_blocking();
            }
        }

        if (buffer[i] == '\n') {
            const uint8_t cr = '\r';
            _writebuf.write(&cr, 1);
        }
        _writebuf.write(&buffer[i], 1);
    }

    if (!hal.scheduler->is_system_initialized()) {
        _flush_tx();
    }

    _write_mutex.give();
    return size;
}

ssize_t USBSerialDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }
    _poll_rx();
    return (ssize_t)_readbuf.read(buffer, count);
}

uint32_t USBSerialDriver::_available()
{
    _poll_rx();
    return _readbuf.available();
}

bool USBSerialDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

void USBSerialDriver::_timer_tick()
{
    if (!_initialized) {
        return;
    }

    _poll_rx();
    _write_mutex.take_blocking();
    _flush_tx();
    _write_mutex.give();
}

void USBSerialDriver::_flush()
{
    while (tx_pending()) {
        _timer_tick();
    }
}

bool USBSerialDriver::_discard_input(void)
{
    _readbuf.clear();
    return true;
}

bool USBSerialDriver::set_options(uint16_t options)
{
    _last_options = options;
    return true;
}

void USBSerialDriver::configure_parity(uint8_t v)
{
    (void)v;
}

void USBSerialDriver::set_stop_bits(int n)
{
    (void)n;
}

uint64_t USBSerialDriver::receive_time_constraint_us(uint16_t nbytes)
{
    (void)nbytes;
    return AP_HAL::micros64();
}

uint32_t USBSerialDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    return _writebuf.space();
}

void USBSerialDriver::vprintf(const char *fmt, va_list ap)
{
    if (!_initialized) {
        return;
    }

    char buffer[128];
    int n = vsnprintf(buffer, sizeof(buffer), fmt, ap);
    if (n > 0) {
        size_t len = (size_t)n;
        if (len >= sizeof(buffer)) {
            len = sizeof(buffer) - 1;
        }
        _write((const uint8_t *)buffer, len);
    }
}

size_t USBSerialDriver::write(uint8_t c)
{
    return _write(&c, 1);
}

size_t USBSerialDriver::write(const uint8_t *buffer, size_t size)
{
    return _write(buffer, size);
}
