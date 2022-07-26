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

#define RDS02_HEAD1 0x55
#define RDS02_HEAD2 0x55
#define RDS02_END 0xAA

#define RDS02UF_HEAD_LEN 2
#define RDS02UF_PRE_DATA_LEN 6
#define RDS02UF_DATA_LEN 10
#define RDS02_DATA_Y_INDEX_L 13
#define RDS02_DATA_Y_INDEX_H 14
#define RDS02_DATA_FC_L    8
#define RDS02_DATA_FC_H    9
#define RDS02_TARGET_INFO  0x70C
#define RDS02UF_DIST_MAX_CM           2000
#define RDS02UF_DIST_MIN_CM           150
#define AP_RANGEFINDER_RDS02UF_TIMEOUT_MS   200


extern const AP_HAL::HAL& hal;

AP_RangeFinder_RDS02UF::AP_RangeFinder_RDS02UF(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params):
    AP_RangeFinder_Backend_Serial(_state, _params)
{
    
}

// return last value measured by sensor
bool AP_RangeFinder_RDS02UF::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0.0f;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        if (decode(c)) {
            sum += _distance_m;
            count++;
        }
    }

    // return false on failure
    if (count == 0) {
        return false;
    }

    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_RDS02UF_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        reading_m = 0.0f;
    } else {
        // return average of all measurements
        reading_m = sum / count;
        update_status();
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
        case STATE0_SYNC_1:
            if (data == RDS02_HEAD1) {
                rev_buffer[parserbuf_index] = data;
                parserbuf_index++;
                decode_state = STATE1_SYNC_2;
            }
            break;
        case STATE1_SYNC_2:
            if (data == RDS02_HEAD2) {
                rev_buffer[parserbuf_index] = data;
                parserbuf_index++;
                decode_state = STATE2_ADDRESS;
            } else {
                parserbuf_index = 0;
                decode_state = STATE0_SYNC_1;
            }
            break;
        case STATE2_ADDRESS: // address
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            decode_state = STATE3_ERROR_CODE;
            break;
        case STATE3_ERROR_CODE: // error_code
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            decode_state = STATE4_FC_CODE_L;
            break;
        case STATE4_FC_CODE_L: // fc_code low
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            decode_state = STATE5_FC_CODE_H;
            break;
        case STATE5_FC_CODE_H: // fc_code high
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            decode_state = STATE6_LENGTH_L;
            break;
        case STATE6_LENGTH_L: // lengh_low
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            decode_state = STATE7_LENGTH_H;
            break;
        case STATE7_LENGTH_H: // lengh_high
        {
            uint8_t read_len = data << 8 | rev_buffer[parserbuf_index-1];
            if ( read_len == RDS02UF_DATA_LEN)	// rds02uf data length is 10
            {
                rev_buffer[parserbuf_index] = data;
                parserbuf_index++;
                decode_state = STATE8_REAL_DATA;
            }else{
                parserbuf_index = 0;
                decode_state = STATE0_SYNC_1;
            }
            break;
        }
        case STATE8_REAL_DATA: // real_data
            rev_buffer[parserbuf_index] = data;
            parserbuf_index++;
            if ( parserbuf_index == (RDS02UF_HEAD_LEN + RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN) ) {
                decode_state = STATE9_CRC;
            }
            break;
        case STATE9_CRC: // crc
        {
	        uint8_t crc_data = 0;
            crc_data = crc8(&rev_buffer[2], RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN);
            rev_buffer[parserbuf_index] = data;
            if (crc_data == data || data == 0xff) {
                parserbuf_index++;
                decode_state = STATE10_END_1;
            } else {
                parserbuf_index = 0;
                decode_state = STATE0_SYNC_1;
            }
            break;
        }
        case STATE10_END_1: //
            if (data == RDS02_END) {
                rev_buffer[parserbuf_index] = data;
                parserbuf_index++;
                decode_state = STATE11_END_2;
            } else {
                parserbuf_index = 0;
                decode_state = STATE0_SYNC_1;
            }
            break;
        case STATE11_END_2: //
        {
            uint16_t fc_code = (rev_buffer[STATE5_FC_CODE_H] << 8 | rev_buffer[STATE4_FC_CODE_L]);
            uint8_t err_code = rev_buffer[STATE3_ERROR_CODE];
            if (data == RDS02_END)
            {
                if (fc_code == 0x03ff && err_code == 0) {// get targer information
                    uint16_t data_fc = (rev_buffer[RDS02_DATA_FC_H] << 8 | rev_buffer[RDS02_DATA_FC_L]);
                    if((data_fc & 0xf0f) == RDS02_TARGET_INFO){	// data_fc = 0x70C + ID * 0x10, ID: 0~0xF
                        float distance = (rev_buffer[RDS02_DATA_Y_INDEX_H] * 256 + rev_buffer[RDS02_DATA_Y_INDEX_L]) / 100.0f;
                        _distance_m = distance;
                        _distance_m = MIN(MAX(RDS02UF_DIST_MIN_CM, uint16_t(_distance_m*100)), RDS02UF_DIST_MAX_CM) * 0.01f;
                        ret = true;
                        state.last_reading_ms = AP_HAL::millis();
                    }

                }
            }
			parserbuf_index = 0;
			decode_state = STATE0_SYNC_1;
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