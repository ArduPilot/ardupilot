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

#include "AP_RCProtocol_Debug.h"
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

void AP_RCProtocol_Debug::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

void AP_RCProtocol_Debug::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    buffer[buflen] = byte;
    buflen++;
    last_data_us = timestamp_us;

    if (buflen >= DEBUG_FRAMELEN_MAX) {
        log_data(AP_RCProtocol::NONE, timestamp_us, buffer, buflen);
        buflen = 0;
    }
}

/*
  process a byte provided by a uart
 */
void AP_RCProtocol_Debug::process_byte(uint8_t byte, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}
