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

#include "AP_RangeFinder_Benewake.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9
#define BENEWAKE_DIST_MAX_CM 32768
#define BENEWAKE_OUT_OF_RANGE_ADD_CM 100

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x59
// byte 1               Frame header    0x59
// byte 2               DIST_L          Distance (in cm) low 8 bits
// byte 3               DIST_H          Distance (in cm) high 8 bits
// byte 4               STRENGTH_L      Strength low 8 bits
// bute 4 (TF03)        (Reserved)
// byte 5               STRENGTH_H      Strength high 8 bits
// bute 5 (TF03)        (Reserved)
// byte 6 (TF02)        SIG             Reliability in 8 levels, 7 & 8 means reliable
// byte 6 (TFmini)      Distance Mode   0x02 for short distance (mm), 0x07 for long distance (cm)
// byte 6 (TF03)        (Reserved)
// byte 7 (TF02 only)   TIME            Exposure time in two levels 0x03 and 0x06
// byte 8               Checksum        Checksum byte, sum of bytes 0 to bytes 7

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_Benewake::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x59, add to buffer
        if (linebuf_len == 0) {
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x59, add it to buffer
            // if not clear the buffer
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 9 items try to decode it
            if (linebuf_len == BENEWAKE_FRAME_LENGTH) {
                // calculate checksum
                uint8_t checksum = 0;
                for (uint8_t i=0; i<BENEWAKE_FRAME_LENGTH-1; i++) {
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if (checksum == linebuf[BENEWAKE_FRAME_LENGTH-1]) {
                    // calculate distance
                    uint16_t dist = ((uint16_t)linebuf[3] << 8) | linebuf[2];
                    if (dist >= BENEWAKE_DIST_MAX_CM) {
                        // this reading is out of range
                        count_out_of_range++;
                    } else if (!has_signal_byte()) {
                        // no signal byte from TFmini so add distance to sum
                        sum_cm += dist;
                        count++;
                    } else {
                        // TF02 provides signal reliability (good = 7 or 8)
                        if (linebuf[6] >= 7) {
                            // add distance to sum
                            sum_cm += dist;
                            count++;
                        } else {
                            // this reading is out of range
                            count_out_of_range++;
                        }
                    }
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_cm = sum_cm / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_cm = MAX(model_dist_max_cm(), max_distance_cm() + BENEWAKE_OUT_OF_RANGE_ADD_CM);
        return true;
    }

    // no readings so return false
    return false;
}
