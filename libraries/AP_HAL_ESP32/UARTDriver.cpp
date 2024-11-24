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

#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include "UARTDriver.h"

#include "esp_log.h"

extern const AP_HAL::HAL& hal;

#include "rom/ets_sys.h" //for ets_printf
extern int ets_printf(const char* format, ...); //for ets_printf in rom

using namespace ESP32;


const UARTDriver::SerialDef UARTDriver::_serial_tab[] = {HAL_ESP32_UART_DEVICES};

// table to find UARTDrivers from serial number, used for event handling
UARTDriver *UARTDriver::serial_drivers[UART_MAX_DRIVERS];

UARTDriver::UARTDriver(uint8_t serial_num) :
    AP_HAL::UARTDriver(),
    sdef(_serial_tab[_serial_num]),
    _serial_num(serial_num)
{
    _initialized = false;
    if(_serial_num > UART_MAX_DRIVERS) {
        // ets is very low level and can't print floats etc
        ets_printf("too many UART drivers");
    }
    serial_drivers[_serial_num] = this;
    //no printf allowed in static constructor, not allowed
}

void UARTDriver::vprintf(const char *fmt, va_list ap)
{
    // the idea is that no other thread can printf to the console etc till the current one finishes its line/action/etc.
    WITH_SEMAPHORE(_sem);

    uart_port_t p = sdef.port;
    if (p == 0) {
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
    //hal.scheduler->delay_microseconds(10000);// time for hw to flush while holding sem ? todo get rid of this?
}

// disable TX/RX pins for unusued uart
void UARTDriver::disable_rxtx(void) const
{
    // nop on esp32. todo?
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    // hal.console->printf("%s:%d UART num:%d\n", __PRETTY_FUNCTION__, __LINE__, sdef.port);

    uart_port_t p = sdef.port;
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
                      sdef.tx,
                      sdef.rx,
                      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        //uart_driver_install(p, 2*UART_FIFO_LEN, 0, 0, nullptr, 0);
        uart_driver_install(p, 2*UART_HW_FIFO_LEN(p), 0, 0, nullptr, 0);
        _readbuf.set_size(RX_BUF_SIZE);
        _writebuf.set_size(TX_BUF_SIZE);

        _initialized = true;
    } else {
        flush();
        uart_set_baudrate(p, b);

    }
    
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        uart_driver_delete(sdef.port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = sdef.port;
    uart_flush(p);
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t UARTDriver::_available()
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

// set optional features, return true on success
bool UARTDriver::set_options(uint16_t options)
{
    return false;
}

// get optional features
uint16_t UARTDriver::get_options(void) const
{
    return _last_options;
}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
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
    uart_port_t p = sdef.port;
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
    uart_port_t p = sdef.port;
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

size_t IRAM_ATTR UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();


    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool UARTDriver::_discard_input()
{
    //uart_port_t p = sdef.port;
    //return uart_flush_input(p) == ESP_OK;
    return false;
}

// record timestamp of new incoming data
void IRAM_ATTR UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}


/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

