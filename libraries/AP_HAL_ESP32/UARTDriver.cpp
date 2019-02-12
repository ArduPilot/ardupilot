#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>

using namespace ESP32;

UARTDriver::UARTDriver(uint8_t serial_num)
{
    _initialized = false;
    uart_num = (uart_port_t)serial_num;
    rx_pin = 3;
    tx_pin = 1;
}

void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (!_initialized) {
        uart_config_t config = {
            .baud_rate = (int)b,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        uart_param_config(uart_num, &config);
        uart_set_pin(uart_num,tx_pin,rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart_num, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);
        _initialized = true;
    } else {
        uart_set_baudrate(uart_num, b);
    }
}

void UARTDriver::end()
{
    uart_driver_delete(uart_num);
    _readbuf.set_size(0);
    _writebuf.set_size(0);
    _initialized = false;
}

void UARTDriver::flush()
{
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    //blocking writes do not used anywhere
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t UARTDriver::available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

int16_t UARTDriver::read()
{
    if (!_initialized) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    return byte;
}

void UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void UARTDriver::read_data()
{
    int count = 0;
    do {
        count = uart_read_bytes(uart_num, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void UARTDriver::write_data()
{
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = uart_tx_chars(uart_num, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t UARTDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }
    if (!_write_mutex.take_nonblocking()) {
        return 0;
    }
    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

