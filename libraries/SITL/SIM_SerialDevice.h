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

#include <unistd.h>
#include <AP_HAL/utility/RingBuffer.h>

namespace SITL {

class SerialDevice {
public:

    SerialDevice();


    // methods for autopilot to use to talk to device:
    ssize_t read_from_device(char *buffer, size_t size) const;
    ssize_t write_to_device(const char *buffer, size_t size) const;
    void set_autopilot_baud(uint32_t baud) { autopilot_baud = baud; }

    // methods for simulated device to use:
    ssize_t read_from_autopilot(char *buffer, size_t size) const;
    virtual ssize_t write_to_autopilot(const char *buffer, size_t size) const;
    virtual uint32_t device_baud() const { return 0; }  // 0 meaning unset

protected:

    class SIM *_sitl;

    ByteBuffer *to_autopilot;
    ByteBuffer *from_autopilot;

    bool init_sitl_pointer();

private:

    uint32_t autopilot_baud;
};

}
