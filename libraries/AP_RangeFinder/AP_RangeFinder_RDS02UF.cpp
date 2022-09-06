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

/**
 * RDS02UF Note:
 * Sensor range scope 1.5m~20.0m
 * Azimuth Coverage ±17°,Elevation Coverage ±3°
 * Frame Rate 20Hz
 */
#include "AP_RangeFinder_RDS02UF.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define RDS02_HEAD1                 0x55
#define RDS02_HEAD2                 0x55
#define RDS02_END                   0xAA

#define RDS02UF_HEAD_LEN            2
#define RDS02UF_ERROR_CODE_INDEX    3
#define RDS02UF_PRODUCTS_FC_INDEX_L 4
#define RDS02UF_PRODUCTS_FC_INDEX_H 5
#define RDS02UF_PRE_DATA_LEN        6
#define RDS02UF_DATA_LEN            10
#define RDS02_DATA_Y_INDEX_L        13
#define RDS02_DATA_Y_INDEX_H        14
#define RDS02_TARGET_FC_INDEX_L     8
#define RDS02_TARGET_FC_INDEX_H     9
#define RDS02_TARGET_INFO_FC        0x070C
#define RDS02UF_DIST_MAX_M          20.0
#define RDS02UF_DIST_MIN_M          1.5
#define RDS02UF_IGNORE_ID_BYTE      0x0F0F
#define RDS02UF_UAV_PRODUCTS_ID     0x03FF
#define RDS02UF_TIMEOUT_MS          200


extern const AP_HAL::HAL& hal;

// return last value measured by sensor
bool AP_RangeFinder_RDS02UF::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available data from the lidar
    float sum = 0.0f;
    uint16_t count = 0;
    uint32_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t c = uart->read();
        if (c < 0) {
            continue;
        }
        if (decode(c)) {
            sum += _distance_m;
            count++;
        }
    }

    if (AP_HAL::millis() - state.last_reading_ms > RDS02UF_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        reading_m = 0.0f;
    } else {
        // return average of all measurements
        reading_m = sum / count;
        update_status();
    }

    // return false if no new readings
    if (count == 0) {
        return false;
    }

    // return average of all measurements
    reading_m = sum / count;
    return true;
}


// add a single character to the buffer and attempt to decode
// returns true if a distance was successfully decoded
bool AP_RangeFinder_RDS02UF::decode(uint8_t c)
{
    uint8_t data = c;
    bool ret = false;
    switch (decode_state)
    {
        case RDS02UF_PARSE_STATE::STATE0_SYNC_1:
            if (data == RDS02_HEAD1) {
                parser_buffer[parser_index] = data;
                parser_index++;
                decode_state = RDS02UF_PARSE_STATE::STATE1_SYNC_2;
            }
            break;
        case RDS02UF_PARSE_STATE::STATE1_SYNC_2:
            if (data == RDS02_HEAD2) {
                parser_buffer[parser_index] = data;
                parser_index++;
                decode_state = RDS02UF_PARSE_STATE::STATE2_ADDRESS;
            } else {
                parser_index = 0;
                decode_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
            }
            break;
        case RDS02UF_PARSE_STATE::STATE2_ADDRESS: // address
            parser_buffer[parser_index] = data;
            parser_index++;
            decode_state = RDS02UF_PARSE_STATE::STATE3_ERROR_CODE;
            break;
        case RDS02UF_PARSE_STATE::STATE3_ERROR_CODE: // error_code
            parser_buffer[parser_index] = data;
            parser_index++;
            decode_state = RDS02UF_PARSE_STATE::STATE4_FC_CODE_L;
            break;
        case RDS02UF_PARSE_STATE::STATE4_FC_CODE_L: // fc_code low
            parser_buffer[parser_index] = data;
            parser_index++;
            decode_state = RDS02UF_PARSE_STATE::STATE5_FC_CODE_H;
            break;
        case RDS02UF_PARSE_STATE::STATE5_FC_CODE_H: // fc_code high
            parser_buffer[parser_index] = data;
            parser_index++;
            decode_state = RDS02UF_PARSE_STATE::STATE6_LENGTH_L;
            break;
        case RDS02UF_PARSE_STATE::STATE6_LENGTH_L: // lengh_low
            parser_buffer[parser_index] = data;
            parser_index++;
            decode_state = RDS02UF_PARSE_STATE::STATE7_LENGTH_H;
            break;
        case RDS02UF_PARSE_STATE::STATE7_LENGTH_H: // lengh_high
        {
            uint8_t read_len = data << 8 | parser_buffer[parser_index-1];
            if ( read_len == RDS02UF_DATA_LEN)	// rds02uf data length is 10
            {
                parser_buffer[parser_index] = data;
                parser_index++;
                decode_state = RDS02UF_PARSE_STATE::STATE8_REAL_DATA;
            }else{
                parser_index = 0;
                decode_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
            }
            break;
        }
        case RDS02UF_PARSE_STATE::STATE8_REAL_DATA: // real_data
            parser_buffer[parser_index] = data;
            parser_index++;
            if ( parser_index == (RDS02UF_HEAD_LEN + RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN) ) {
                decode_state = RDS02UF_PARSE_STATE::STATE9_CRC;
            }
            break;
        case RDS02UF_PARSE_STATE::STATE9_CRC: // crc
        {
	        uint8_t crc_data;
            crc_data = crc8(&parser_buffer[2], RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN);
            parser_buffer[parser_index] = data;
            if (crc_data == data || data == 0xff) {
                parser_index++;
                decode_state = RDS02UF_PARSE_STATE::STATE10_END_1;
            } else {
                parser_index = 0;
                decode_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
            }
            break;
        }
        case RDS02UF_PARSE_STATE::STATE10_END_1: //
            if (data == RDS02_END) {
                parser_buffer[parser_index] = data;
                parser_index++;
                decode_state = RDS02UF_PARSE_STATE::STATE11_END_2;
            } else {
                parser_index = 0;
                decode_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
            }
            break;
        case RDS02UF_PARSE_STATE::STATE11_END_2: //
        {
            uint16_t fc_code = (parser_buffer[RDS02UF_PRODUCTS_FC_INDEX_H] << 8 | parser_buffer[RDS02UF_PRODUCTS_FC_INDEX_L]);
            uint8_t err_code = parser_buffer[RDS02UF_ERROR_CODE_INDEX];

            if (data == RDS02_END)
            {
                if (fc_code == RDS02UF_UAV_PRODUCTS_ID && err_code == 0) {// get targer information
                    uint16_t read_info_fc = (parser_buffer[RDS02_TARGET_FC_INDEX_H] << 8 | parser_buffer[RDS02_TARGET_FC_INDEX_L]);
                    if((read_info_fc & RDS02UF_IGNORE_ID_BYTE) == RDS02_TARGET_INFO_FC){	// read_info_fc = 0x70C + ID * 0x10, ID: 0~0xF
                        _distance_m = (parser_buffer[RDS02_DATA_Y_INDEX_H] * 256 + parser_buffer[RDS02_DATA_Y_INDEX_L]) / 100.0f;
                        if (_distance_m > RDS02UF_DIST_MAX_M)
                        {
                            _distance_m = RDS02UF_DIST_MAX_M;
                            set_status(RangeFinder::Status::OutOfRangeHigh);
                        }else if (_distance_m < RDS02UF_DIST_MIN_M)
                        {
                            _distance_m = RDS02UF_DIST_MIN_M;
                            set_status(RangeFinder::Status::OutOfRangeLow);
                        }
                        ret = true;
                        state.last_reading_ms = AP_HAL::millis();
                    }

                }
            }
			parser_index = 0;
			decode_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
			break;
        }
    }

    return ret;
}

uint8_t AP_RangeFinder_RDS02UF::crc8(uint8_t* pbuf, int32_t len)
{
     uint8_t* data = pbuf;
     uint8_t crc = 0;
     while ( len-- )
        crc = crc8_table[crc^*(data++)];
     return crc;
}