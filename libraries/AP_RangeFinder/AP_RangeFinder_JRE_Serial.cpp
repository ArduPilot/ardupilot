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

#define FRAME_HEADER_1      'R'     // 0x52
#define FRAME_HEADER_2_A    'A'     // 0x41
#define FRAME_HEADER_2_B    'B'     // 0x42
#define FRAME_HEADER_2_C    'C'     // 0x43
#define FRAME_HEADER_2_X    'X'     // 0x58
#define FRAME_HEADER_3      'A'     // 0x41
#define FRAME_HEADER_4_A    'A'     // 0x41
#define FRAME_HEADER_4_B    'B'     // 0x42
#define FRAME_HEADER_4_C    'C'     // 0x43

#define DIST_MAX            50.00f
#define OUT_OF_RANGE_ADD    1.0f    // metres

#define MODE_A_LEN_V1       16      // 1Data Mode packet length
#define MODE_B_LEN_V1       32      // 3Data Mode packet length
#define MODE_C_LEN_V1       48      // 5Data Mode packet length
#define ALT_OFFSET_V1       4       // offset Altitude data
#define RADIO_POW_OFFSET_V1 10      // offset Radio Power data
#define NTRK_STAT_FROM_END_OFFSET_V1    2       // Offset from the end of NTRK Status (Status(L) is 13th or 29th or 45th Byte)
#define NTRK_STAT_BIT_MASK_V1           0x02    // NTRK bit mask

#define MODE_A_LEN_V2       34      // 1Data Mode packet length
#define MODE_B_LEN_V2       54      // 3Data Mode packet length
#define MODE_C_LEN_V2       74      // 5Data Mode packet length
#define ALT_OFFSET_V2       11      // offset Altitude data
#define RADIO_POW_OFFSET_V2 19      // offset Radio Power data
#define NTRK_STAT_FROM_END_OFFSET_V2    8       // offset from the end of NTRK Status (Status is 25th or 45th or 65th Byte)
#define NTRK_STAT_BIT_MASK_V2           0x02    // NTRK bit mask

#define SQ_NORM_UPPER_LIMIT 512.0f  // upper limit for normalizing Radio Power to Signal Quality (Radio Power is over 512 -> Signal Quality is 100.0%)

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

    float radio_pow = 0.0f;
    float sum_radio_pow = 0.0f;
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
        if (data_buff_ofs < 4) {
            continue;
        }

        // determine packet length for incoming packet:
        bool is_v2_flag = false;
        uint8_t packet_length;
        switch (data_buff[1]) {
        case FRAME_HEADER_2_A:
            packet_length = MODE_A_LEN_V1;
            break;
        case FRAME_HEADER_2_B:
            packet_length = MODE_B_LEN_V1;
            break;
        case FRAME_HEADER_2_C:
            packet_length = MODE_C_LEN_V1;
            break;
        case FRAME_HEADER_2_X:
            if (data_buff[2] == FRAME_HEADER_3) {
                switch (data_buff[3]) {
                case FRAME_HEADER_4_A:
                    packet_length = MODE_A_LEN_V2;
                    break;
                case FRAME_HEADER_4_B:
                    packet_length = MODE_B_LEN_V2;
                    break;
                case FRAME_HEADER_4_C:
                    packet_length = MODE_C_LEN_V2;
                    break;
                default:
                    move_preamble_in_buffer(3);
                    continue;
                }

                is_v2_flag = true;
            } else {
                move_preamble_in_buffer(2);
                continue;
            }
            break;
        default:
            move_preamble_in_buffer(1);
            continue;
        }

        uint8_t alt_offset;
        uint8_t radio_pow_offset;
        uint8_t ntrk_offset;
        uint8_t ntrk_bit_mask;

        if (is_v2_flag) {
            alt_offset = ALT_OFFSET_V2;
            radio_pow_offset = RADIO_POW_OFFSET_V2;
            ntrk_offset = packet_length - NTRK_STAT_FROM_END_OFFSET_V2 - 1;
            ntrk_bit_mask = NTRK_STAT_BIT_MASK_V2;
        } else {
            alt_offset = ALT_OFFSET_V1;
            radio_pow_offset = RADIO_POW_OFFSET_V1;
            ntrk_offset = packet_length - NTRK_STAT_FROM_END_OFFSET_V1 - 1;
            ntrk_bit_mask = NTRK_STAT_BIT_MASK_V1;
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

        // check NTRK bit to for status value:
        if (data_buff[ntrk_offset] & ntrk_bit_mask) {   // NTRK
            invalid_count++;
            // discard entire packet:
            move_preamble_in_buffer(packet_length);
            continue;
        }

        // good message, extract rangefinder reading:
        reading_m = (data_buff[alt_offset] * 256 + data_buff[alt_offset + 1]) * 0.01f;
        sum += reading_m;
        radio_pow = (data_buff[radio_pow_offset] * 256 + data_buff[radio_pow_offset + 1]);
        sum_radio_pow += radio_pow;
        valid_count++;
        move_preamble_in_buffer(packet_length);
    }

    // return average of all valid readings
    if (valid_count > 0) {
        no_signal = false;
        reading_m = sum / valid_count;
        sq_pct = (uint16_t)((sum_radio_pow / valid_count) * (100.0f / SQ_NORM_UPPER_LIMIT) + 0.5f);
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
