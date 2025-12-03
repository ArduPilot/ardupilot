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
/*
  base class for serially-attached simulated devices
*/

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>

#include "SIM_SerialDevice.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

using namespace SITL;

SerialDevice::SerialDevice(uint16_t tx_bufsize, uint16_t rx_bufsize)
{
    to_autopilot = NEW_NOTHROW ByteBuffer{tx_bufsize};
    from_autopilot = NEW_NOTHROW ByteBuffer{rx_bufsize};
}

bool SerialDevice::init_sitl_pointer()
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        if (_sitl == nullptr) {
            return false;
        }
    }
    return true;
}

#if AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED
ssize_t SerialDevice::corrupt_transfer(char *buffer, const ssize_t ret, const size_t size) const
{
    if (ret <= 0) {
        // nothing to corrupt
        return ret;
    }

    // probabilities in percent (kept simple and local to this function)
    const int pct_stall = 2;   // return 0 bytes even though data was available
    const int pct_short = 2;   // return only a short prefix of the data
    const int pct_drop  = 2;   // drop one byte
    const int pct_add   = 2;   // insert one byte
    const int pct_flip  = 2;   // flip one byte
    const int pct_burst = 2;   // corrupt a small burst of bytes

    const ssize_t short_max = 8; // maximum bytes to return on short-read
    const size_t  burst_max = 4; // maximum burst length

    // 1) stall: caller sees "no data" although there was some
    if ((rand() % 100) < pct_stall) {
        fprintf(stderr, "stall (ret=%d -> 0)\n", int(ret));
        return 0;
    }

    // 2) short-read: only return part of the available bytes
    if ((rand() % 100) < pct_short && ret > 1) {
        ssize_t max_short = ret < short_max ? ret : short_max;
        ssize_t newret = 1 + (rand() % max_short);
        if (newret < ret) {
            fprintf(stderr, "short-read (ret=%d -> %d)\n",
                    int(ret), int(newret));
            // first newret bytes stay unchanged, remaining bytes are ignored
            return newret;
        }
    }

    // 3) drop: remove a single byte from the stream
    if ((rand() % 100) < pct_drop && ret > 1) {
        const size_t ofs = rand() % ret;
        fprintf(stderr, "dropping byte at offset %u (ret=%d -> %d)\n",
                unsigned(ofs), int(ret), int(ret-1));
        memmove(&buffer[ofs], &buffer[ofs+1], ret - ofs - 1);
        return ret - 1;
    }

    // 4) add: insert a random byte into the stream (if there is room)
    if ((rand() % 100) < pct_add && size_t(ret) < size) {
        const size_t ofs = rand() % ret;
        fprintf(stderr, "adding byte at offset %u (ret=%d -> %d)\n",
                unsigned(ofs), int(ret), int(ret+1));
        memmove(&buffer[ofs+1], &buffer[ofs], ret - ofs);
        buffer[ofs] = uint8_t(rand() & 0xFF);
        return ret + 1;
    }

    // 5) flip: corrupt a single byte in the stream
    if ((rand() % 100) < pct_flip) {
        const size_t ofs = rand() % ret;
        fprintf(stderr, "flipping byte at offset %u\n", unsigned(ofs));
        buffer[ofs] = uint8_t(rand() & 0xFF);
        return ret;
    }

    // 6) burst: corrupt several consecutive bytes
    if ((rand() % 100) < pct_burst && ret > 0) {
        size_t len = 1 + (rand() % burst_max);
        const size_t ofs = rand() % ret;
        size_t end = ofs + len;
        if (end > (size_t)ret) {
            end = (size_t)ret;
        }
        fprintf(stderr, "burst-flip [%u..%u)\n",
                unsigned(ofs), unsigned(end));
        for (size_t i = ofs; i < end; i++) {
            buffer[i] = uint8_t(rand() & 0xFF);
        }
        return ret;
    }

    // no corruption applied
    return ret;
}
#endif

ssize_t SerialDevice::read_from_autopilot(char *buffer, const size_t size) const
{
    ssize_t ret = from_autopilot->read((uint8_t*)buffer, size);
#if AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED
    ret = corrupt_transfer(buffer, ret, size);
#endif

    // if (ret > 0) {
    //     ::fprintf(stderr, "SIM_SerialDevice: read from autopilot (%u): (", (unsigned)ret);
    //     for (ssize_t i=0; i<ret; i++) {
    //         const uint8_t x = buffer[i];
    //         ::fprintf(stderr, "%02X", (unsigned)x);
    //     }
    //     ::fprintf(stderr, " ");
    //     for (ssize_t i=0; i<ret; i++) {
    //         ::fprintf(stderr, "%c", buffer[i]);
    //     }
    //     ::fprintf(stderr, ")\n");
    // }
    return ret;
}

ssize_t SerialDevice::write_to_autopilot(const char *buffer, const size_t size) const
{
    if (!is_match_baud()) {
        return -1;
    }

    const ssize_t ret = to_autopilot->write((uint8_t*)buffer, size);
    return ret;
}

ssize_t SerialDevice::read_from_device(char *buffer, const size_t size) const
{
    if (!is_match_baud()) {
        return -1;
    }

    ssize_t ret = to_autopilot->read((uint8_t*)buffer, size);
#if AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED
    ret = corrupt_transfer(buffer, ret, size);
#endif
    // ::fprintf(stderr, "read_from_device: (");
    // for (ssize_t i=0; i<ret; i++) {
    //     ::fprintf(stderr, "%02X", (uint8_t)buffer[i]);
    // }
    // ::fprintf(stderr, ") (\n");
    // for (ssize_t i=0; i<ret; i++) {
    //     ::fprintf(stderr, "%c", (uint8_t)buffer[i]);
    // }
    // ::fprintf(stderr, ")\n");
    return ret;
}
ssize_t SerialDevice::write_to_device(const char *buffer, const size_t size) const
{
    const ssize_t ret = from_autopilot->write((uint8_t*)buffer, size);
    return ret;
}

/**
 * baudrates match
 * 
 * @retval true matched baudreate
 * @retval false  unmatched baudreate
 */
bool SerialDevice::is_match_baud() const
{
    if (device_baud() != 0 && autopilot_baud != 0 && device_baud() != autopilot_baud) {
        return false;
    }
    return true;
}
