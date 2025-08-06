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

#include "AP_RangeFinder_NRA24_Serial.h"

#if AP_RANGEFINDER_NRA24_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>

#define FRAME_HEADER 0xAA
#define FRAME_TERMINATOR 0x55
#define FRAME_LENGTH 14
#define FRAME_MSG_L 2
#define FRAME_MSG_H 3
#define DIST_H 6
#define DIST_L 7
#define FRAME_CHECKBYTE 11
#define MSG_TARGET_INFO_L 0x0C
#define MSG_TARGET_INFO_H 0x07
#define DIST_MAX 50.00
#define OUT_OF_RANGE_ADD 1.00

extern const AP_HAL::HAL& hal;

// format of serial packets received from rangefinder
// Note that the NRA24 is mixed-endian.
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0xAA
// byte 1               Frame header    0xAA
// byte 2               MessageId_L     Low 8 bits for command
// byte 3               MessageId_H     High 8 bits for command
// byte 4               Target ID       0x01 (always 1 for NRA24?)
// byte 5               RCS             Radar cross section = value * 0.5 - 50
// byte 6               DIST_H          Distance (in cm) high 8 bits
// byte 7               DIST_L          Distance (in cm) low 8 bits
// byte 8               reserved
// byte 9               Roll count      Roll count = (value & 0xE) >> 5
// byte 9               VELOCITY_H      Bottom 3 bits are velocity in 0.05m/s = value & 0x07
// byte 10              VELOCITY_L      Velocity in 0.05m/s, low 8 bits
// byte 11              Checksum        Low byte of sum of bytes 4 to 10 sum(B4..B10) & 0xFF
// byte 12              Frame end       0x55
// byte 13              Frame end       0x55

void AP_RangeFinder_NRA24_Serial::update(void)
{
    if (get_reading(state.distance_m)) {
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        if (AP_HAL::millis() - last_heartbeat_ms > read_timeout_ms()) {
            // no heartbeat, must be disconnected
            set_status(RangeFinder::Status::NotConnected);
        } else {
            // Have heartbeat but no data; likely no relative motion
            // This case has special pre-arm check handling
            set_status(RangeFinder::Status::NoData);
        }
    }
}

// distance returned in reading_m, set to true if sensor reports a good reading
bool AP_RangeFinder_NRA24_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t bad_read = 0;

    // read any available lines from the radar
    // we get 42 bytes for each reading (messages 0x070A, 0x070B, the 0x070C, which is sent only if data exist)
    // At 115200 bps, 8192 is about 0.5s of data
    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        // if buffer is empty and this byte is 0xAA, add to buffer
        if (linebuf_len == 0) {
            if (c == FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        // buffer is not empty, add byte to buffer
        } else if (linebuf_len == 1) {
            if (c == FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                // Not a header; reset buffer
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // When buffer has FRAME_LENGTH items, try to decode it
            if (linebuf_len == FRAME_LENGTH) {
                // Check for target message
                if( linebuf[FRAME_MSG_L] == MSG_TARGET_INFO_L && linebuf[FRAME_MSG_H] == MSG_TARGET_INFO_H ) {
                    // calculate checksum
                    uint8_t checksum = 0;
                    for (auto j=4; j<11; j++) {
                        checksum += linebuf[j];
                    }
                    checksum = checksum & 0xFF;
                    // if checksum matches, extract contents
                    if (checksum == linebuf[FRAME_CHECKBYTE]) {
                        // calculate distance
                        uint16_t dist = ((uint16_t)linebuf[DIST_H] << 8) | linebuf[DIST_L];
                        if (dist > DIST_MAX * 100 ) {
                            // this reading is out of range and a bad read
                            bad_read++;
                        } else {
                            // add distance to sum
                            sum_cm += dist;
                            count++;
                        }
                    } else {
                        // this reading is bad
                        bad_read++;
                    }
                } else if( linebuf[FRAME_MSG_L] == 0x0A && linebuf[FRAME_MSG_H] == 0x06 ) {
                    // Heartbeat
                    last_heartbeat_ms = AP_HAL::millis();
                }
            }
            // clear buffer
            linebuf_len = 0;
        }
    }

    if (count > 0) {
        // return average distance of readings since last update
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (bad_read > 0) {
        // if a bad read has occurred this update overwrite return with larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(DIST_MAX, max_distance() + OUT_OF_RANGE_ADD);
        return true;
    }

    // no readings so return false
    return false;
}

#endif // AP_RANGEFINDER_NRA24_SERIAL_ENABLED
