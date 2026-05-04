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
  driver for serial rangefinder like : ME007YS; A02YYUW; JSN-SR04T; AJ-SR04M;
 */
#include "AP_RangeFinder_A02YYUW.h"

#if AP_RANGEFINDER_A02YYUW_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

static constexpr float A02YYUW_MAX_RANGE_M = 4.5;
static constexpr uint8_t A02YYUW_HEADER = 0xFF;

// read - return average of all valid readings since last call
bool AP_RangeFinder_A02YYUW::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_request_ms > 50) {
        uart->write(0x55);
        last_request_ms = now_ms;
    }

    float sum_range = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // format is: [ 0xFF | DATA_H | DATA_L | SUM ]

    // read any available frames from the sensor
    for (auto i=0; i<8192; i++) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (buf_len == 0 && b != A02YYUW_HEADER) {
            // discard
            continue;
        }
        buf[buf_len++] = b;
        if (buf_len == sizeof(buf)) {
            buf_len = 0;
            const uint8_t crc = (buf[0] + buf[1] + buf[2]) & 0x00FF;
            if (crc != buf[3]) {
                // bad CRC, discard
                continue;
            }
            const float dist_m = ((buf[1] << 8) | buf[2]) * 0.001f;
            if (dist_m > A02YYUW_MAX_RANGE_M) {
                count_out_of_range++;
                continue;
            }
            sum_range += dist_m;
            count++;
        }
    }

    if (count > 0) {
        reading_m = sum_range / count;
        no_signal = false;
        return true;
    }

    // readings beyond the sensor's rated range are not trusted,
    // report out-of-range-high
    if (count_out_of_range > 0) {
        reading_m = MAX(A02YYUW_MAX_RANGE_M, max_distance()) + 1.0f;
        no_signal = true;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_A02YYUW_ENABLED
