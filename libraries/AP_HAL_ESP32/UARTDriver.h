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

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "driver/gpio.h"
#include "driver/uart.h"

namespace ESP32
{

struct UARTDesc {
    uart_port_t port;
    gpio_num_t rx;
    gpio_num_t tx;
};

class UARTDriver : public AP_HAL::UARTDriver
{
public:

    UARTDriver(uint8_t serial_num)
        : AP_HAL::UARTDriver()
    {
        _initialized = false;
        uart_num = serial_num;
    }

    virtual ~UARTDriver() = default;

    void vprintf(const char *fmt, va_list ap) override;

    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;

    uint32_t available() override;
    //uint32_t available_locked(uint32_t key) override;

    uint32_t txspace() override;

    ssize_t read(uint8_t *buffer, uint16_t count) override;
    int16_t read() override;
    //ssize_t read(uint8_t *buffer, uint16_t count) override;
    //int16_t read_locked(uint32_t key) override;

    void _timer_tick(void) override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    bool discard_input() override; // discard all bytes available for reading
    uint32_t bw_in_bytes_per_second() const override
    {
        return 10*1024;
    }

    //bool lock_port(uint32_t write_key, uint32_t read_key) override;

    //size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key) override;
private:
    bool _initialized;
    const size_t TX_BUF_SIZE = 1024;
    const size_t RX_BUF_SIZE = 1024;
    uint8_t _buffer[32];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    void read_data();
    void write_data();

    uint8_t uart_num;
};

}
