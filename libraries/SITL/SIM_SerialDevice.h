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

#pragma once

#include "SIM_config.h"

#include <unistd.h>
#include <AP_HAL/utility/RingBuffer.h>

#if AP_SIM_SERIALDEVICE_NETWORK_ENABLED
#include <AP_HAL/utility/Socket_native.h>
#endif  // AP_SIM_SERIALDEVICE_NETWORK_ENABLED

namespace SITL {

class SerialDevice {
public:

    SerialDevice(uint16_t tx_bufsize=512, uint16_t rx_bufsize=512);


    // methods for autopilot to use to talk to device:
    ssize_t read_from_device(char *buffer, size_t size) const;
    ssize_t write_to_device(const char *buffer, size_t size) const;
    void set_autopilot_baud(uint32_t baud) { autopilot_baud = baud; }

    // methods for simulated device to use:
    ssize_t read_from_autopilot(char *buffer, size_t size) const;
    virtual ssize_t write_to_autopilot(const char *buffer, size_t size) const;
    virtual uint32_t device_baud() const { return 0; }  // 0 meaning unset

#if AP_SIM_SERIALDEVICE_NETWORK_ENABLED
    // attach this device to a TCP server socket rather than to a
    // simulated serial port.  This simulates a device which the
    // autopilot reaches over the network (e.g. via a NET_Pn port)
    // rather than over one of its serial ports.  Returns true on success
    bool listen_on_tcp_port(uint16_t port) WARN_IF_UNUSED;

    // true if this device is attached to the autopilot via a network
    // socket rather than via a simulated serial port
    bool is_network_attached() const { return listener != nullptr; }

    // move bytes between the network socket and this device.  Does
    // nothing if the device is not attached to a socket
    void network_update();
#else
    bool is_network_attached() const { return false; }
#endif  // AP_SIM_SERIALDEVICE_NETWORK_ENABLED

protected:

    class SIM *_sitl;

    ByteBuffer *to_autopilot;
    ByteBuffer *from_autopilot;

    bool init_sitl_pointer() WARN_IF_UNUSED;

private:

    bool is_match_baud(void) const;

    // baudrate the autopilot has this device open at; zero if the
    // device is not attached to a simulated serial port
    uint32_t autopilot_baud;

#if AP_SIM_SERIALDEVICE_NETWORK_ENABLED
    SocketAPM_native *listener = nullptr;  // socket the autopilot connects to, nullptr if serially attached
    SocketAPM_native *sock = nullptr;      // socket to the connected autopilot, nullptr if not connected
#endif  // AP_SIM_SERIALDEVICE_NETWORK_ENABLED

    ssize_t corrupt_transfer(char *buffer, const ssize_t ret, const size_t size) const;
};

}
