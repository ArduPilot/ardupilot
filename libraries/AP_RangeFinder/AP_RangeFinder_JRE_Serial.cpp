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

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include "AP_RangeFinder_JRE_Serial.h"
#include <AP_Math/AP_Math.h>

#define FRAME_HEADER_1    'R'    // 0x52
#define FRAME_HEADER_2_A  'A'    // 0x41
#define FRAME_HEADER_2_B  'B'    // 0x42
#define FRAME_HEADER_2_C  'C'    // 0x43

#define DIST_MAX 50.00
#define OUT_OF_RANGE_ADD   1.0  // metres

void AP_RangeFinder_JRE_Serial::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<data_buff_ofs; i++) {
        if (data_buff[i] == FRAME_HEADER_1) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(data_buff, &data_buff[i], data_buff_ofs-i);
    data_buff_ofs = data_buff_ofs - i;
}

bool AP_RangeFinder_JRE_Serial::get_reading(float &reading_m)
{
    // uart instance check
    if (uart == nullptr) {
        return false;  // not update
    }

    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings
    float sum = 0.0f;

    float fft_value = 0.0f;
    float sum_fft_value = 0.0f;
    uint16_t sq_pct;

    // read a maximum of 8192 bytes per call to this function:
    uint16_t bytes_available = MIN(uart->available(), 8192U);

    while (bytes_available > 0) {
        // fill buffer
        const auto num_bytes_to_read = MIN(bytes_available, ARRAY_SIZE(data_buff) - data_buff_ofs);
        const auto num_bytes_read = uart->read(&data_buff[data_buff_ofs], num_bytes_to_read);
        if (num_bytes_read == 0) {
            break;
        }
        if (bytes_available < num_bytes_read) {
            // this is a bug in the uart call.
            break;
        }
        bytes_available -= num_bytes_read;
        data_buff_ofs += num_bytes_read;

        // move header frame header in buffer
        move_preamble_in_buffer(0);

        // ensure we have a packet type:
        if (data_buff_ofs < 2) {
            continue;
        }

        // determine packet length for incoming packet:
        uint8_t packet_length;
        switch (data_buff[1]) {
        case FRAME_HEADER_2_A:
            packet_length = 16;
            break;
        case FRAME_HEADER_2_B:
            packet_length = 32;
            break;
        case FRAME_HEADER_2_C:
            packet_length = 48;
            break;
        default:
            move_preamble_in_buffer(1);
            continue;
        }

        // check there are enough bytes for message type
        if (data_buff_ofs < packet_length) {
            continue;
        }

        // check the checksum
        const uint16_t crc = crc16_ccitt_r(data_buff, packet_length - 2, 0xffff, 0xffff);
        if (crc != ((data_buff[packet_length-1] << 8) | data_buff[packet_length-2])) {
            // CRC failure
            move_preamble_in_buffer(1);
            continue;
        }

        // check random bit to for magic value:
        if (data_buff[packet_length-3] & 0x02) { // NTRK
            invalid_count++;
            // discard entire packet:
            move_preamble_in_buffer(packet_length);
            continue;
        }

        // good message, extract rangefinder reading:
        reading_m = (data_buff[4] * 256 + data_buff[5]) * 0.01f;
        sum += reading_m;
        fft_value = (data_buff[10] * 256 + data_buff[11]);
        sum_fft_value += fft_value;
        valid_count++;
        move_preamble_in_buffer(packet_length);
    }

    // return average of all valid readings
    if (valid_count > 0) {
        no_signal = false;
        reading_m = sum / valid_count;
        sq_pct = (uint16_t)((sum_fft_value / valid_count) * (100.0f / 512.0f) + 0.5f);
        if (sq_pct >= 100) {
            signal_quality_pct = 100;
        } else {
            signal_quality_pct = (int8_t)sq_pct;
        }
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        no_signal = true;
        reading_m = MAX(DIST_MAX, max_distance() + OUT_OF_RANGE_ADD);
        return true;
    }

    return false;
}
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
