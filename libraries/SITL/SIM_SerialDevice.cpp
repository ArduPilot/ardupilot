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
    // devices attached to a simulated serial port have this set when
    // the autopilot opens that port:
    autopilot_baud = 0;
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

#if AP_SIM_SERIALDEVICE_NETWORK_ENABLED
/*
  attach this device to a TCP server socket.  The autopilot connects to
  this socket (e.g. with a NET_Pn port configured as a TCP client)
  instead of talking to the device over a simulated serial port
 */
bool SerialDevice::listen_on_tcp_port(const uint16_t port)
{
    listener = NEW_NOTHROW SocketAPM_native(false);
    if (listener == nullptr) {
        return false;
    }
    listener->reuseaddress();
    if (!listener->bind("127.0.0.1", port) ||
        !listener->listen(1) ||
        !listener->set_blocking(false)) {
        ::fprintf(stderr, "SIM: failed to listen on TCP port %u: %m\n", unsigned(port));
        delete listener;
        listener = nullptr;
        return false;
    }
    ::printf("SIM: device listening for autopilot on TCP port %u\n", unsigned(port));
    return true;
}

/*
  move bytes between the network socket and this device.  This performs
  the same role the SITL UART driver performs for serially-attached
  devices
 */
void SerialDevice::network_update()
{
    if (listener == nullptr) {
        // not attached to a socket
        return;
    }

    if (sock == nullptr) {
        sock = listener->accept(0);
        if (sock == nullptr) {
            return;
        }
        sock->set_blocking(false);
    }

    char buffer[512];

    // device to autopilot:
    while (true) {
        const ssize_t nread = read_from_device(buffer, sizeof(buffer));
        if (nread <= 0) {
            break;
        }
        if (sock->send(buffer, nread) != nread) {
            // if the autopilot is not keeping up we simply drop the data
            break;
        }
    }

    // autopilot to device:
    while (true) {
        const ssize_t nread = sock->recv(buffer, sizeof(buffer), 0);
        if (nread == 0) {
            // autopilot closed the connection; wait for it to reconnect
            delete sock;
            sock = nullptr;
            return;
        }
        if (nread < 0) {
            break;
        }
        write_to_device(buffer, nread);
    }
}
#endif  // AP_SIM_SERIALDEVICE_NETWORK_ENABLED

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
