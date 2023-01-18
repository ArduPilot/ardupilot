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

#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 5
#endif

class ESP32::WiFiDriver : public AP_HAL::UARTDriver
{
public:
    WiFiDriver();

    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000*1024;
    }


    bool discard_input() override;

    bool _more_data;
private:
    enum ConnectionState {
        NOT_INITIALIZED,
        INITIALIZED,
        CONNECTED
    };
    const size_t TX_BUF_SIZE = 1024;
    const size_t RX_BUF_SIZE = 1024;
    uint8_t _buffer[32];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    ConnectionState _state;
    short accept_socket;
    short socket_list[WIFI_MAX_CONNECTION];
    void *_wifi_task_handle;
    void initialize_wifi();
    bool read_data();
    bool write_data();
    bool start_listen();
    bool try_accept();
    static void _wifi_thread(void* arg);
    unsigned short available_socket();
};
