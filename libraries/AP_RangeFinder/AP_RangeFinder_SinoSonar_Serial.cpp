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
  driver for SinoSonar_Serial : ME007YS; A02YYUW;
 */
#include "AP_RangeFinder_SinoSonar_Serial.h"

#if AP_RANGEFINDER_SINOSONAR_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

static constexpr float SINOSONAR_SERIAL_MAX_RANGE_M = 4.5;
static constexpr uint8_t SINOSONAR_HEADER = 0xFF;

// read - return last value measured by sensor
bool AP_RangeFinder_SinoSonar_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // format is: [ 0xFF | DATA_H | DATA_L | SUM ]

    // read any available lines from the lidar
    for (auto i=0; i<8192; i++) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (buf_len == 0 && b != SINOSONAR_HEADER) {
            // discard
            continue;
        }
        buf[buf_len++] = b;
        if (buf_len == sizeof(buf)) {
            buf_len = 0;
            const uint16_t crc = (buf[0] + buf[1] + buf[2]) & 0x00FF;
            if (crc != buf[3]) {
                // bad CRC, discard
                continue;
            }
            reading_m = ((buf[1] << 8) + buf[2]) * 0.001;
            return reading_m <= SINOSONAR_SERIAL_MAX_RANGE_M;
        }
    }
    return false;
}

#endif  // AP_RANGEFINDER_SINOSONAR_SERIAL_ENABLED
