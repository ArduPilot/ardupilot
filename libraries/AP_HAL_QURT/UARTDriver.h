/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_HAL_QURT.h"
#include "Semaphores.h"
#include <AP_HAL/utility/RingBuffer.h>

class QURT::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(const char *name);

    /* QURT implementations of UARTDriver virtual methods */

    void set_device_path(const char *path) {
        devname = path;
    }
    
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* QURT implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* QURT implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    void _read_callback(char *buf, size_t size);

    void timer_tick(void);
    
private:
    const char *devname;
    int fd = -1;
    Semaphore lock;

    ByteBuffer *readbuf;
    ByteBuffer *writebuf;

    bool nonblocking_writes = false;
    volatile bool in_timer = false;
    volatile bool initialised = false;

    uint64_t last_write_time_us;

    int write_fd(const uint8_t *buf, uint16_t n);
};
