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
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

/* WiFi MAVLink virtual serial ports for Espressif Zephyr boards: each TCP client
 * is presented to AP_SerialManager as an ordinary UART. */

#include <AP_HAL/AP_HAL.h>

#include "hwdef.h"

#if defined(AP_ZEPHYR_WIFI_ENABLED) && AP_ZEPHYR_WIFI_ENABLED

#if !defined(CONFIG_WIFI)
#error "hwdef.dat requests WIFI_TCP/WIFI_UDP but the board's prj conf lacks CONFIG_WIFI=y (see WiFiDriver.h for the full Kconfig block)"
#endif

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_HAL_Zephyr.h"
#include "Semaphores.h"

#ifndef HAL_ZEPHYR_WIFI_SSID
#define HAL_ZEPHYR_WIFI_SSID "ardupilot"
#endif
#ifndef HAL_ZEPHYR_WIFI_PASSWORD
#define HAL_ZEPHYR_WIFI_PASSWORD "ardupilot123"
#endif
#ifndef HAL_ZEPHYR_WIFI_CHANNEL
#define HAL_ZEPHYR_WIFI_CHANNEL 6
#endif
#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 4
#endif

namespace Zephyr
{

/* One-time softAP + IP + DHCP-server bring-up shared by both drivers.
   Returns true when the AP interface is up (idempotent). */
bool wifi_softap_start(void);

class WiFiDriver : public AP_HAL::UARTDriver
{
public:
    WiFiDriver();

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000 * 1024;
    }

private:
    enum ConnectionState {
        NOT_INITIALIZED,
        INITIALIZED,
        CONNECTED
    };

    static constexpr size_t RX_BUF_SIZE = 2048;
    static constexpr size_t TX_BUF_SIZE = 2048;

    uint8_t _buffer[256];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    volatile ConnectionState _state;
    int _accept_socket;
    int _socket_list[WIFI_MAX_CONNECTION];

    bool start_listen();
    bool try_accept();
    bool read_data();
    bool write_data();
    void _wifi_thread_fn();

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override;
};

class WiFiUdpDriver : public AP_HAL::UARTDriver
{
public:
    WiFiUdpDriver();

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000 * 1024;
    }

private:
    enum ConnectionState {
        NOT_INITIALIZED,
        CONNECTED
    };

    static constexpr size_t RX_BUF_SIZE = 2048;
    static constexpr size_t TX_BUF_SIZE = 2048;

    uint8_t _buffer[256];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    volatile ConnectionState _state;
    int _socket;

    bool open_socket();
    bool read_data();
    bool write_data();
    void _wifi_thread_fn();

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override;
};

}  // namespace Zephyr

#endif  /* AP_ZEPHYR_WIFI_ENABLED */
