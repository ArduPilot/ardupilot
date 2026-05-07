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
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "esp_err.h"

extern const AP_HAL::HAL& hal;

namespace ESP32
{

USBSerialDriver *USBSerialDriver::_singleton;
volatile bool USBSerialDriver::_mounted;
volatile bool USBSerialDriver::_port_open;

USBSerialDriver::USBSerialDriver()
{
    _initialized = false;
    _receive_timestamp_idx = 0;
    _baudrate = 0;
    _uart_owner_thd = nullptr;
    _singleton = this;
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
        if (!ardupilot_tusb_init(&USBSerialDriver::rx_callback,
                                 &USBSerialDriver::line_state_callback,
                                 this)) {
            return;
        }

        _mounted = true;
        _port_open = false;
        const size_t read_size = MAX((size_t)rxSpace, RX_BUF_SIZE);
        const size_t write_size = MAX((size_t)txSpace, TX_BUF_SIZE);
        if (!_readbuf.set_size(read_size) || !_writebuf.set_size(write_size)) {
            ardupilot_tusb_deinit();
            return;
        }
        _initialized = true;
    } else {
        _flush();
    }

    _uart_owner_thd = xTaskGetCurrentTaskHandle();
    _baudrate = baud;
}

void USBSerialDriver::_end()
{
    if (_initialized) {
        _readbuf.set_size(0);
        _writebuf.set_size(0);
        _mounted = false;
        _port_open = false;
        ardupilot_tusb_deinit();
    }
    _initialized = false;
}

void USBSerialDriver::_flush()
{
    if (!_initialized) {
        return;
    }
    if (!ardupilot_tusb_is_cdc_connected() && !ardupilot_tusb_is_open()) {
        return;
    }
    (void)ardupilot_tusb_flush(pdMS_TO_TICKS(50));
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

    _read_mutex.take_blocking();
    const uint32_t ret = _readbuf.read(buffer, count);
    _read_mutex.give();
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
    // RX is callback-fed into the read buffer by TinyUSB.
}

void USBSerialDriver::write_data()
{
    if (!_initialized) {
        return;
    }

    static constexpr size_t flush_threshold = TX_BUF_SIZE / 2;
    bool should_flush = false;
    bool queued_any = false;
    size_t local_pending = 0;

    _write_mutex.take_blocking();
    while (true) {
        const int count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count <= 0) {
            break;
        }

        const int accepted = ardupilot_tusb_write(_buffer, count);
        if (accepted <= 0) {
            break;
        }

        _writebuf.advance(accepted);
        queued_any = true;
        if (accepted < count) {
            break;
        }
    }
    local_pending = _writebuf.available();
    _write_mutex.give();

    should_flush = (local_pending == 0 && ardupilot_tusb_tx_pending()) ||
                   (queued_any && local_pending >= flush_threshold);

    if (should_flush &&
        (ardupilot_tusb_is_cdc_connected() || ardupilot_tusb_is_open())) {
        const esp_err_t flush_ret = ardupilot_tusb_flush(0);
        if (flush_ret == ESP_ERR_NOT_FINISHED ||
            flush_ret == ESP_ERR_TIMEOUT) {
            return;
        }
        if (flush_ret != ESP_OK) {
            return;
        }
    }
}

size_t USBSerialDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();
    const size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();

    if (ret > 0 &&
        (ardupilot_tusb_is_cdc_connected() || ardupilot_tusb_is_open())) {
        write_data();
    }

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

    _read_mutex.take_blocking();
    _readbuf.clear();
    _read_mutex.give();
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
    return ardupilot_tusb_is_connected();
}

void USBSerialDriver::handle_rx()
{
    size_t rx_size = 0;
    while ((rx_size = ardupilot_tusb_read(_buffer, sizeof(_buffer))) > 0) {
        _read_mutex.take_blocking();
        _readbuf.write(_buffer, rx_size);
        _read_mutex.give();
        _receive_timestamp_update();
    }
}

void USBSerialDriver::rx_callback(void *arg)
{
    if (_singleton == nullptr || arg != _singleton) {
        return;
    }
    _singleton->handle_rx();
}

void USBSerialDriver::line_state_callback(bool dtr, bool rts, void *arg)
{
    if (_singleton == nullptr || arg != _singleton) {
        return;
    }
    _mounted = true;
    _port_open = dtr || rts;
    if (_port_open) {
        _singleton->write_data();
    }
}

} // namespace ESP32
