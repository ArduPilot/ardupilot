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

#include "AP_RangeFinder_LightWareSerial.h"

#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareSerial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum = 0;              // sum of all readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // max distance the sensor can reliably measure - read from parameters
    const auto distance_cm_max = max_distance()*100;

    // read any available lines from the lidar
    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        // use legacy protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            if (c == '\r') {
                linebuf[linebuf_len] = 0;
                const float dist = strtof(linebuf, nullptr);
                if (!is_negative(dist) && !is_lost_signal_distance(dist * 100, distance_cm_max)) {
                    sum += dist;
                    valid_count++;
                    // if still determining protocol update legacy valid count
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    invalid_count++;
                }
                linebuf_len = 0;
            } else if (isdigit(c) || c == '.' || c == '-') {
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
                }
            }
        }

        // use binary protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            bool msb_set = BIT_IS_SET(c, 7);
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } else {
                // received the low byte which should be second
                if (high_byte_received) {
                    const int16_t dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    if (dist >= 0 && !is_lost_signal_distance(dist, distance_cm_max)) {
                        sum += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } else {
                        invalid_count++;
                    }
                }
                high_byte_received = false;
            }
        }
    }

    // protocol set after 10 successful reads
    if (protocol_state == ProtocolState::UNKNOWN) {
        if (binary_valid_count > 10) {
            protocol_state = ProtocolState::BINARY;
        } else if (legacy_valid_count > 10) {
            protocol_state = ProtocolState::LEGACY;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = sum / valid_count;
        no_signal = false;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        reading_m = MAX(LIGHTWARE_DIST_MAX_CM, distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM);
        no_signal = true;
        return true;
    }

    // no readings so return false
    return false;
}

// check to see if distance returned by the LiDAR is a known lost-signal distance flag
bool AP_RangeFinder_LightWareSerial::is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max)
{
    if (distance_cm < distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM) {
        // in-range
        return false;
    }
    const int16_t bad_distances[] { 13000, 16000, 23000, 25000 };
    for (const auto bad_distance_cm : bad_distances) {
        if (distance_cm == bad_distance_cm) {
            return true;
        }
    }
    return false;
}

#endif
