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

#include <AP_HAL_ESP32/USBSerialDriver.h>
#include <AP_Math/AP_Math.h>

#include "driver/usb_serial_jtag.h"

namespace ESP32
{

USBSerialDriver::USBSerialDriver()
{
    _initialized = false;
    _receive_timestamp_idx = 0;
    _baudrate = 0;
    _uart_owner_thd = nullptr;
}

void USBSerialDriver::vprintf(const char *fmt, va_list ap)
{
    (void)fmt;
    (void)ap;
    // USB-backed SERIAL4 is MAVLink-only.
    // Do not route console printf/log output to this transport.
}

void USBSerialDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (baud == 0 && txSpace == 0 && rxSpace == 0 && _initialized) {
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    if (!_initialized) {
        usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        config.tx_buffer_size = TX_BUF_SIZE;
        config.rx_buffer_size = RX_BUF_SIZE;
        if (!usb_serial_jtag_is_driver_installed()) {
            if (usb_serial_jtag_driver_install(&config) != ESP_OK) {
                return;
            }
        }
        _readbuf.set_size(MAX((size_t)rxSpace, RX_BUF_SIZE));
        _writebuf.set_size(MAX((size_t)txSpace, TX_BUF_SIZE));
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        _initialized = true;
    } else {
        flush();
    }

    _baudrate = baud;
}

void USBSerialDriver::_end()
{
    if (_initialized) {
        _readbuf.set_size(0);
        _writebuf.set_size(0);
        if (usb_serial_jtag_is_driver_installed()) {
            usb_serial_jtag_driver_uninstall();
        }
    }
    _initialized = false;
}

void USBSerialDriver::_flush()
{
    if (!_initialized || !usb_serial_jtag_is_driver_installed()) {
        return;
    }
    (void)usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(20));
}

bool USBSerialDriver::is_initialized()
{
    return _initialized;
}

bool USBSerialDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

uint32_t USBSerialDriver::_available()
{
    if (!_initialized || _uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t USBSerialDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result = _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

ssize_t USBSerialDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return -1;
    }
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

void USBSerialDriver::_timer_tick()
{
    if (!_initialized) {
        return;
    }
    read_data();
    write_data();
}

void USBSerialDriver::read_data()
{
    if (!usb_serial_jtag_is_driver_installed()) {
        return;
    }

    int count = 0;
    do {
        count = usb_serial_jtag_read_bytes(_buffer, sizeof(_buffer), 0);
        if (count > 0) {
            _readbuf.write(_buffer, count);
        }
    } while (count > 0);
}

void USBSerialDriver::write_data()
{
    if (!usb_serial_jtag_is_driver_installed()) {
        return;
    }

    _write_mutex.take_blocking();
    while (true) {
        int count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count <= 0) {
            break;
        }
        count = usb_serial_jtag_write_bytes(_buffer, count, 0);
        if (count <= 0) {
            break;
        }
        _writebuf.advance(count);
    }
    _write_mutex.give();
}

size_t USBSerialDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();
    const size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

bool USBSerialDriver::_discard_input()
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();
    return true;
}

void USBSerialDriver::_receive_timestamp_update()
{
    _receive_timestamp[_receive_timestamp_idx ^ 1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

uint64_t USBSerialDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        const uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

uint32_t USBSerialDriver::bw_in_bytes_per_second() const
{
    return 200 * 1024;
}

uint32_t USBSerialDriver::get_baud_rate() const
{
    return _baudrate;
}

uint32_t USBSerialDriver::get_usb_baud() const
{
    return _baudrate;
}

uint8_t USBSerialDriver::get_usb_parity() const
{
    return parity;
}

bool USBSerialDriver::is_usb_connected()
{
    return usb_serial_jtag_is_driver_installed() && usb_serial_jtag_is_connected();
}

} // namespace ESP32
