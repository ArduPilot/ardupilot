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

class QURT::UDPDriver : public AP_HAL::UARTDriver {
public:
    static UDPDriver *from(AP_HAL::UARTDriver *d) {
        return static_cast<UDPDriver*>(d);
    }

    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    int16_t available();
    int16_t txspace();
    int16_t read();

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    uint32_t socket_check(uint8_t *buf, int len, uint32_t *nbytes);
    uint32_t socket_input(const uint8_t *buf, int len, uint32_t *nbytes);

    enum flow_control get_flow_control(void) { return FLOW_CONTROL_ENABLE; };
    
private:
    Semaphore lock;
    bool initialised;
    bool nonblocking_writes = true;
    
    ByteBuffer *readbuf;
    ByteBuffer *writebuf;
};
