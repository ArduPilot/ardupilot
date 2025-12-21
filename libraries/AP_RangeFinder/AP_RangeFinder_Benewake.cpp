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

#if AP_RANGEFINDER_BENEWAKE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9
#define BENEWAKE_DIST_MAX_CM 32768
#define BENEWAKE_OUT_OF_RANGE_ADD 1.00  // metres

// format of serial packets received from benewake lidar
// Model       | Byte 0-1  | Byte 2 | Byte 3 | Byte 4     | Byte 5     | Byte 6     | Byte 7         | Byte 8
// ------------+-----------+--------+--------+------------+------------+------------+----------------+---------
// TFmini      | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | dist mode* | reserved       | Checksum
// TF02        | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | SIG*       | Exposure Time* | Checksum
// TF-Nova     | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp       | Confidence*    | Checksum
// ------------+-----------+--------+--------+------------+------------+------------+----------------+---------
// TFmini-S    | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L     | Temp_H         | Checksum
// TFmini-plus | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L     | Temp_H         | Checksum
// TF02-Pro    | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L     | Temp_H         | Checksum
// TFS20-L     | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L     | Temp_H         | Checksum
// TF-Luna     | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | Temp_L     | Temp_H         | Checksum
// TF03        | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | reserved   | reserved       | Checksum
// TF350       | 0x59 0x59 | Dist_L | Dist_H | Strength_L | Strength_H | reserved   | reserved       | Checksum
//
// Notes:
// TFmini (discontinued) : Byte 6 : Distance mode is 0x02 for short distance (mm), 0x07 for long distance (cm)
// TF02 (discontinued) : Byte 6 : SIG byte is reliability in 8 levels, 7 & 8 means reliable
// TF02 (discontinued) : Byte 7 : Exposure Time in two levels 0x03 and 0x06
// TF-Nova : Byte 7 : Confidence from 0 ~ 100 (100 being highest confidence)
//
// Strength values per model:
// TFmini: No documentation
// TF02: 0 ~ 3000, reliable if value >=20 and <= 2000
// TF-Nova: No documentation
// TFmini-S: 0 ~ 65535. reliable if value >=100 and <65535, 65535 (-1) means signal strength saturation (bad)
// TFmini-plus: 0 ~ 65535. reliable if value >=100 and <65535, 65535 (-1) means signal strength saturation (bad)
// TF02-Pro: 0 ~ 65535, no documentation
// TFS20-L: 0 ~ 65535, no documentation
// TF-Luna: 0 ~ 65535. reliable if value >=100 and <65535, 65535 (-1) means signal strength saturation (bad)
// TF03: 0 ~ 3500, reliable if >=40 ~ 1200.  if high reflectivity object will be 1500
// TF350: 0 ~ 3500, reliable if >=40 ~ 1200.  if high reflectivity object will be 1500

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_Benewake::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read any available lines from the lidar
    for (auto j=0; j<8192; j++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
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
                    if (dist >= BENEWAKE_DIST_MAX_CM || dist == uint16_t(model_dist_max_cm())) {
                        // this reading is out of range. Note that we
                        // consider getting exactly the model dist max
                        // is out of range. This fixes an issue with
                        // the TF03 which can give exactly 18000 cm
                        // when out of range
                        count_out_of_range++;
                    } else if (!has_signal_byte()) {
                        // no signal byte so can immediately add distance to sum
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
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm() * 0.01, max_distance() + BENEWAKE_OUT_OF_RANGE_ADD);
        return true;
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_BENEWAKE_ENABLED
