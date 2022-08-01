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

#include "AP_RangeFinder_TeraRanger_Serial.h"

#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED


#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define FRAME_HEADER 0x54
#define FRAME_LENGTH 5
#define DIST_MAX_CM 3000
#define OUT_OF_RANGE_ADD_CM 1000
#define STATUS_MASK 0x1F
#define DISTANCE_ERROR 0x0001

// format of serial packets received from rangefinder
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x54
// byte 1               DIST_H          Distance (in mm) high 8 bits
// byte 2               DIST_L          Distance (in mm) low 8 bits
// byte 3               STATUS          Status,Strengh,OverTemp
// byte 4               CRC8            packet CRC

// distance returned in reading_m, set to true if sensor reports a good reading
bool AP_RangeFinder_TeraRanger_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_mm = 0;
    uint16_t count = 0;
    uint16_t bad_read = 0;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x57, add to buffer
        if (linebuf_len == 0) {
            if (c == FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        // buffer is not empty, add byte to buffer
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 5 items try to decode it
            if (linebuf_len == FRAME_LENGTH) {
                // calculate CRC8 (tbd)
                uint8_t crc = 0;
                crc =crc_crc8(linebuf,FRAME_LENGTH-1);
                // if crc matches, extract contents
                if (crc == linebuf[FRAME_LENGTH-1]) {
                    // calculate distance
                    uint16_t dist = ((uint16_t)linebuf[1] << 8) | linebuf[2];
                    if (dist >= DIST_MAX_CM *10) {
                        // this reading is out of range and a bad read
                        bad_read++;
                    } else {
                        // check if reading is good, no errors, no overtemp, reading is not the special case of 1mm
                        if ((STATUS_MASK & linebuf[3]) == 0 && (dist != DISTANCE_ERROR)) {
                            // add distance to sum
                            sum_mm += dist;
                            count++;
                        } else {
                            // this reading is bad
                            bad_read++;
                        }
                    }
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings since last update
        reading_m = (sum_mm * 0.001f) / count;
        return true;
    }

    if (bad_read > 0) {
        // if a bad read has occurred this update overwrite return with larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(DIST_MAX_CM, max_distance_cm() + OUT_OF_RANGE_ADD_CM) * 0.01f;
        return true;
    }

    // no readings so return false
    return false;
}

#endif // AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
