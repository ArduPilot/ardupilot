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
  Replay serial capture files
*/

#include <AP_Math/AP_Math.h>

#include "SIM_CapReplay.h"
#include <GCS_MAVLink/GCS.h>
#include "SITL.h"

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

CapReplay::CapReplay(const char *_filepath) : SerialDevice::SerialDevice()
{
    strncpy(filepath, _filepath, ARRAY_SIZE(filepath)-1);
}

void CapReplay::rewind()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CapReplay: rewind %s", filepath);
    base_time_ms = AP_HAL::millis();
    ::lseek(_fd, 0, SEEK_SET);
}

void CapReplay::update()
{
    if (_fd == -2) {
        _fd = ::open(filepath, O_RDONLY);
    }
    if (_fd == -1) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    while (_fd > 0) {
        if (header.length == 0) {
            const auto read_ret = ::read(_fd, &header, sizeof(header));
            if (read_ret == -1) {
                _fd = -1;
                return;
            }
            if (read_ret != sizeof(header)) {
                rewind();
                return;
            }
            if (header.magic != 0xEAEF0D0F) {
                _fd = -1;
                return;
            }
        }
        if (header.time_ms > now_ms - base_time_ms) {
            return;
        }

        char buf[1024];
        while (header.length > 0) {
            const auto num_bytes_to_read = MIN(header.length, ARRAY_SIZE(buf));
            const auto num_read = ::read(_fd, buf, num_bytes_to_read);
            if (num_read == -1) {
                _fd = -1;
                break;
            }
            if (write_to_autopilot(buf, num_read) != num_read) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CapReplay: Short write to autopilot");
            }
            if (num_read < (signed)num_bytes_to_read) {
                rewind();
                break;
            }
            header.length -= num_read;
        }
    }
}
