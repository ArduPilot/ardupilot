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

#include "AP_RangeFinder_NoopLoop.h"

#if AP_RANGEFINDER_NOOPLOOP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define NOOPLOOP_FRAME_HEADER 0x57
#define NOOPLOOP_FRAME_HEADER_1 0x00
#define NOOPLOOP_FRAME_LENGTH 16
#define NOOPLOOP_DIST_MAX_MM 8000

// format of serial packets received from NoopLoop TOF Sense P and F lidar
//
// Data Bit             Definition             Description
// -----------------------------------------------------------------------
// byte 0               Frame header           0x57
// byte 1               Function Mark          0x00
// byte 2               (Reserved)
// byte 3               Id                     id
// byte 4-7             System time            System time in ms
// byte 8-10            Distance*1000
// byte 11              Status                 Strength high 8 bits
// byte 12-13           Signal Strength
// byte 14              (Reserved)
// byte 15              Checksum

// distance returned in reading_m
bool AP_RangeFinder_NoopLoop::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    uint32_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        // if buffer is empty and this byte is 0x57, add to buffer
        if (linebuf_len == 0) {
            if (c == NOOPLOOP_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x00, add it to buffer
            // if not clear the buffer
            if (c == NOOPLOOP_FRAME_HEADER_1) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 16 items try to decode it
            if (linebuf_len == NOOPLOOP_FRAME_LENGTH) {
                // calculate checksum
                uint8_t checksum = 0;
                for (uint8_t i=0; i<NOOPLOOP_FRAME_LENGTH-1; i++) {
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if (checksum == linebuf[NOOPLOOP_FRAME_LENGTH-1]) {
                    // calculate distance
                    const int32_t dist = (int32_t)(linebuf[8] << 8 | linebuf[9] << 16 | linebuf[10] << 24) / 256;
                    const uint8_t valid = (linebuf[11]);
                    if (dist > 0 && reading_m < NOOPLOOP_DIST_MAX_MM && valid < 255) {
                        reading_m = dist * 0.001f;
                        linebuf_len = 0;
                        uart->discard_input();
                        return true;
                    }
                }
                // clear buffer
                linebuf_len = 0;
                uart->discard_input();
            }
        }
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_NOOPLOOP_ENABLED
