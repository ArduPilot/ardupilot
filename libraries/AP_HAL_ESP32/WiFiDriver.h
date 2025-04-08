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


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 5
#endif

class ESP32::WiFiDriver : public AP_HAL::UARTDriver
{
public:
    WiFiDriver();

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000*1024;
    }

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
    tskTaskControlBlock* _wifi_task_handle;
    void initialize_wifi();
    bool read_data();
    bool write_data();
    bool start_listen();
    bool try_accept();
    static void _wifi_thread(void* arg);
    unsigned short available_socket();

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override;
    void _end() override;
    void _flush() override;
    bool _discard_input() override;
    uint32_t _available() override;
};
