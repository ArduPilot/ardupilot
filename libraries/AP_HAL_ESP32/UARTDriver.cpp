/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>

#include "esp_log.h"

extern const AP_HAL::HAL& hal;

namespace ESP32
{

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

void UARTDriver::vprintf(const char *fmt, va_list ap)
{

    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
}

void UARTDriver::begin(uint32_t b)
{
    begin(b, 0, 0);
}


void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
        if (!_initialized) {

            uart_config_t config = {
                .baud_rate = (int)b,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            };
            uart_param_config(p, &config);
            uart_set_pin(p,
                         uart_desc[uart_num].tx,
                         uart_desc[uart_num].rx,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            //uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);

            _initialized = true;
        } else {
            flush();
            uart_set_baudrate(p, b);

        }
    }
}

void UARTDriver::end()
{
    if (_initialized) {
        uart_driver_delete(uart_desc[uart_num].port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::flush()
{
    uart_port_t p = uart_desc[uart_num].port;
    uart_flush(p);
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

ssize_t IRAM_ATTR UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }

    return ret;
}

int16_t IRAM_ATTR UARTDriver::read()
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

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    do {
        count = uart_read_bytes(p, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = uart_tx_chars(p, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::write(uint8_t c)
{
    return write(&c,1);
}

size_t IRAM_ATTR UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::discard_input()
{
    //uart_port_t p = uart_desc[uart_num].port;
    //return uart_flush_input(p) == ESP_OK;
    return false;
}

}
