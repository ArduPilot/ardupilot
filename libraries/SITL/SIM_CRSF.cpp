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
  Base class for CRSF telemetry
*/

#include "SIM_config.h"

#if AP_SIM_CRSF_ENABLED

#include "SIM_CRSF.h"

using namespace SITL;

extern const AP_HAL::HAL& hal;

const char *CRSF::dataid_string(DataID id, ssize_t& len)
{
    switch (id) {
        case DataID::VTX_FRAME: {
            static const uint8_t vtx_frame[] = { 0xC8, 0x8, 0xF, 0xCE, 0x30, 0x8, 0x16, 0xE9, 0x0, 0x5F }; // VTX
            len = sizeof(vtx_frame);
            return (const char*)vtx_frame;
        }
            break;
        case DataID::VTX_TELEM: {
            static const uint8_t vtx_frame[] = { 0xC8, 0x7, 0x10, 0xCE, 0xE, 0x16, 0x65, 0x0, 0x1B }; // VTX Telem
            len = sizeof(vtx_frame);
            return (const char*)vtx_frame;
        }
            break;
        case DataID::VTX_UNKNOWN: {
            static const uint8_t vtx_frame[] = { 0xC8, 0x9, 0x8, 0x0, 0x9E, 0x0, 0x0, 0x0, 0x0, 0x0, 0x95}; // Battery 15.8v
            len = sizeof(vtx_frame);
            return (const char*)vtx_frame;
        }
        default:
            break;
    }

    return "UNKNOWN";
}

void CRSF::update()
{
    const ssize_t n = read_from_autopilot(&_buffer[_buflen], ARRAY_SIZE(_buffer) - _buflen - 1);
    if (n != -1) {
        _buflen += n;
    }

    // update every 400ms
    uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms < 400) {
        return;
    }

    _last_update_ms = now;

    ssize_t len = 0;
    ssize_t index = 0;
    const char* bytes = dataid_string(DataID(_id), len);

    while (len > 0) {
        const ssize_t nwrite = write_to_autopilot(&bytes[index], len);
        len -= nwrite;
        index += nwrite;
    }

    _id = (_id + 1) % MAX_DATA_FRAMES;

    if (_buflen == 0) {
        return;
    }
}

#endif  // AP_SIM_CRSF_ENABLED
