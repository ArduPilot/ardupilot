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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ERDI_ENABLED

#include "AP_RangeFinder_ERDI.h"
#include <AP_HAL/AP_HAL.h>

#define ERDI_FRAME_HEADER       0x5C
#define ERDI_FRAME_LEN          4
#define ERDI_DIST_OUT_OF_RANGE  0xFFFF  // reported by the H series and LRF10VB when out of range
#define ERDI_START_CMD_INTERVAL_MS 1000

extern const AP_HAL::HAL& hal;

/*
  "Start Ranging" command from the VB series manuals: 5A 0A 02 02 00 F1
  The H series streams from power-up and has no start command.
 */
void AP_RangeFinder_ERDI::send_start_command()
{
    static const uint8_t cmd[] { 0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1 };
    uart->write(cmd, sizeof(cmd));
}

/*
  discard the first byte of the line buffer and move the buffer start
  to the next frame header candidate
 */
void AP_RangeFinder_ERDI::resync()
{
    for (uint8_t i = 1; i < linebuf_len; i++) {
        if (linebuf[i] == ERDI_FRAME_HEADER) {
            memmove(&linebuf[0], &linebuf[i], linebuf_len - i);
            linebuf_len -= i;
            return;
        }
    }
    linebuf_len = 0;
}

/*
  read all available frames and return the average distance in metres.
  The H series streams at up to 1kHz, so averaging is used to reduce
  noise and keep the UART buffer drained.
 */
bool AP_RangeFinder_ERDI::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (_model == Model::VB &&
        now_ms - last_valid_frame_ms > ERDI_START_CMD_INTERVAL_MS &&
        now_ms - last_start_cmd_ms > ERDI_START_CMD_INTERVAL_MS) {
        send_start_command();
        last_start_cmd_ms = now_ms;
    }

    const float scale = (_model == Model::VB) ? 0.001f : 0.01f;
    float sum_m = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    for (uint16_t j = 0; j < 8192; j++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (linebuf_len == 0 && c != ERDI_FRAME_HEADER) {
            continue;
        }
        linebuf[linebuf_len++] = c;
        if (linebuf_len < ERDI_FRAME_LEN) {
            continue;
        }

        const uint8_t checksum = ~uint8_t(linebuf[1] + linebuf[2]);
        if (checksum != linebuf[3]) {
            // probably locked onto a data byte that looked like a header
            resync();
            continue;
        }

        const uint16_t raw = uint16_t(linebuf[2]) << 8 | linebuf[1];
        linebuf_len = 0;
        last_valid_frame_ms = now_ms;

        const float frame_m = raw * scale;
        if (raw == ERDI_DIST_OUT_OF_RANGE || frame_m > max_distance()) {
            // 0xFFFF is the H series out-of-range code. A factory bench
            // log of an LRF50H also showed occasional frames with a valid
            // checksum but a distance far beyond the rated range, so any
            // frame past RNGFNDx_MAX is counted as out-of-range instead of
            // being averaged into the reading.
            count_out_of_range++;
            continue;
        }
        sum_m += frame_m;
        count++;
    }

    if (count > 0) {
        reading_m = sum_m / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // report just beyond the configured maximum so that the
        // status becomes OutOfRangeHigh
        reading_m = max_distance() + 1.0f;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_ERDI_ENABLED
