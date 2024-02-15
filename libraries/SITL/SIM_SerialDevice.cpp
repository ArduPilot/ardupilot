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
    to_autopilot = new ByteBuffer{tx_bufsize};
    from_autopilot = new ByteBuffer{rx_bufsize};
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
    if (ret > 0 && (rand() % 100) < 2) {
        // drop a random byte from returned data:
        const size_t byte_ofs_to_drop = rand() % ret;
        fprintf(stderr, "dropping byte at offset %u\n", unsigned(byte_ofs_to_drop));
        memmove(&buffer[byte_ofs_to_drop], &buffer[byte_ofs_to_drop+1], ret - byte_ofs_to_drop - 1);
        return ret - 1;
    }

    if (ret > 0 && size_t(ret) < size && (rand() % 100) < 2) {
        // add a random byte to the stream:
        const size_t byte_ofs_to_add = rand() % ret;
        fprintf(stderr, "adding byte at offset %u\n", unsigned(byte_ofs_to_add));
        memmove(&buffer[byte_ofs_to_add+1], &buffer[byte_ofs_to_add], ret - byte_ofs_to_add);
        buffer[byte_ofs_to_add] = rand()*256;
        return ret + 1;
    }

    if (ret > 0 && unsigned(ret) < size && (rand() % 100) < 2) {
        // corrupt a random byte in the stream:
        const size_t byte_ofs_to_corrupt = rand() % ret;
        fprintf(stderr, "corrupting byte at offset=%u\n", unsigned(byte_ofs_to_corrupt));
        buffer[byte_ofs_to_corrupt] = rand()*256;
        return ret;
    }

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
