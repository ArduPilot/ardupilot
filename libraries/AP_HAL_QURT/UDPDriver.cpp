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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include <stdlib.h>
#include <unistd.h>
#include "UDPDriver.h"
#include <stdio.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>

using namespace QURT;

extern const AP_HAL::HAL& hal;

void UDPDriver::begin(uint32_t b) 
{
    begin(b, 16384, 16384);
}

void UDPDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    rxS = constrain_int32(rxS, 16384, 30000);
    txS = constrain_int32(txS, 16384, 30000);

    /*
      allocate the read buffer
     */
    if (rxS != 0 && (readbuf == nullptr || rxS != readbuf->get_size())) {
        initialised = false;
        if (readbuf != nullptr) {
            delete readbuf;
        }
        readbuf = new ByteBuffer(rxS);
    }

    /*
      allocate the write buffer
    */
    if (txS != 0 && (writebuf == nullptr || txS != writebuf->get_size())) {
        initialised = false;
        if (writebuf != nullptr) {
            delete writebuf;
        }
        writebuf = new ByteBuffer(txS);
    }
    
    if (readbuf && writebuf) {
        initialised = true;
    }
}

void UDPDriver::end() 
{
    initialised = false;
    if (readbuf) {
        delete readbuf;
        readbuf = nullptr;
    }
    if (writebuf) {
        delete writebuf;
        writebuf = nullptr;
    }

}

void UDPDriver::flush() 
{
}

bool UDPDriver::is_initialized() 
{ 
    return initialised; 
}

void UDPDriver::set_blocking_writes(bool blocking) 
{
    nonblocking_writes = !blocking;
}

bool UDPDriver::tx_pending()
{
    return false;
}

/* QURT implementations of Stream virtual methods */
int16_t UDPDriver::available() 
{ 
    if (!initialised) {
        return 0;
    }
    return readbuf->available();
}

int16_t UDPDriver::txspace() 
{ 
    if (!initialised) {
        return 0;
    }
    return writebuf->space();
}

int16_t UDPDriver::read() 
{ 
    uint8_t c;
    if (!initialised) {
        return -1;
    }
    if (!lock.take(0)) {
        return 0;
    }
    if (readbuf->empty()) {
        lock.give();
        return -1;
    }
    readbuf->read(&c, 1);
    lock.give();
    return c;
}

/* QURT implementations of Print virtual methods */
size_t UDPDriver::write(uint8_t c) 
{
    if (!initialised) {
        return 0;
    }
    if (!lock.take(0)) {
        return 0;
    }

    while (writebuf->space() == 0) {
        if (nonblocking_writes) {
            lock.give();
            return 0;
        }
        hal.scheduler->delay_microseconds(1000);
    }
    writebuf->write(&c, 1);
    lock.give();
    return 1;
}

size_t UDPDriver::write(const uint8_t *buffer, size_t size)
{
    if (!initialised) {
        return 0;
    }
    if (!nonblocking_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    if (!lock.take(0)) {
        return 0;
    }
    size_t ret = writebuf->write(buffer, size);
    lock.give();
    return ret;
}

uint32_t UDPDriver::socket_check(uint8_t *buf, int len, uint32_t *nbytes)
{
    if (!initialised) {
        return 1;
    }
    if (!writebuf) {
        return 1;
    }
    *nbytes = writebuf->available();
    if (*nbytes == 0) {
        return 1;
    }
    if (*nbytes > len) {
        *nbytes = len;
    }
    uint32_t n = *nbytes;
    if (writebuf->peek(0) != 254) {
        /*
          we have a non-mavlink packet at the start of the
          buffer. Look ahead for a MAVLink start byte, up to 256 bytes
          ahead
         */
        uint16_t limit = n>256?256:n;
        uint16_t i;
        for (i=0; i<limit; i++) {
            if (writebuf->peek(i) == 254) {
                n = i;
                break;
            }
        }
        // if we didn't find a MAVLink marker then limit the send size to 256
        if (i == limit) {
            n = limit;
        }
    } else {
        // this looks like a MAVLink packet - try to write on
        // packet boundaries when possible
        if (n < 8) {
            n = 0;
        } else {
            // the length of the packet is the 2nd byte, and mavlink
            // packets have a 6 byte header plus 2 byte checksum,
            // giving len+8 bytes
            uint8_t len = writebuf->peek(1);
            if (n < len+8) {
                // we don't have a full packet yet
                n = 0;
            } else if (n > len+8) {
                // send just 1 packet at a time (so MAVLink packets
                // are aligned on UDP boundaries)
                n = len+8;
            }
        }        
    }

    *nbytes = n;
    writebuf->read(buf, *nbytes);
    return 0;
}

uint32_t UDPDriver::socket_input(const uint8_t *buf, int len, uint32_t *nbytes)
{
    if (!initialised) {
        return 1;
    }
    if (!readbuf) {
        return 1;
    }
    *nbytes = readbuf->write(buf, len);
    return 0;
}
#endif // CONFIG_HAL_BOARD

