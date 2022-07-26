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

#include "AP_RangeFinder_RDS02UF.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

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
    switch (step)
    {
        case STATE1_SYNC_1:
            if (data == RDS02_HEAD1)
            {
                rev_buffer[step] = data;
                step++;
            }
            break;
        case STATE2_SYNC_2:
            if (data == RDS02_HEAD2)
            {
                rev_buffer[step] = data;
                step++;
            }
            else
            {
                ParseDataFailed(rev_buffer, sizeof(rev_buffer));
            }
            break;
        case STATE3_ADDRESS: // address
            rev_buffer[step] = data;
            step++;
            break;
        case STATE4_ERROR_CODE: // error_code
            rev_buffer[step] = data;
            err_code = data;
            step++;
            break;
        case STATE5_FC_CODE_L: // fc_code low
            rev_buffer[step] = data;
            fc_code = data;
            step++;
            break;
        case STATE6_FC_CODE_H: // fc_code high
            rev_buffer[step] = data;
            fc_code = fc_code | (data << 8);
            step++;
            break;
        case STATE7_LENGTH_L: // lengh_low
            data_len = data;
            rev_buffer[step] = data;
            step++;
            break;
        case STATE8_LENGTH_H: // lengh_high
            data_len = data << 8 | data_len;
            if(data_len == 10){
                rev_buffer[step] = data;
                step++;
            }
            else
            {
                ParseDataFailed(rev_buffer, sizeof(rev_buffer));
            }
            break;
        case STATE9_REAL_DATA: // real_data
            rev_buffer[step + data_index] = data;

            data_index++;
            if (data_index == data_len)
            {
                step++;
            }
            break;
        case STATE10_CRC: // crc
            crc_data = crc8(&rev_buffer[2], 6 + data_len);
            rev_buffer[step + data_index] = data;
            if (crc_data == data || data == 0xff)
            {
                step++;
            }
            else
            {
                ParseDataFailed(rev_buffer, sizeof(rev_buffer));
            }
            break;
        case STATE11_END_1: //
            if (data == RDS02_END)
            {
                rev_buffer[step + data_index] = data;
                step++;
            }
            else
            {
                ParseDataFailed(rev_buffer, sizeof(rev_buffer));
            }

            break;
        case STATE12_END_2: //
            if (data == RDS02_END)
            {
                if (fc_code == 0x03ff && err_code == 0)
                {   
                    float distance = (rev_buffer[RDS02_ValueY_H] * 256 + rev_buffer[RDS02_ValueY_L]) / 100.0f;
                    _distance_m = distance;
                    ret = true;
                    
                }
                ParseDataFailed(rev_buffer, sizeof(rev_buffer));
            }
            break;
    }

    return ret;
}

void AP_RangeFinder_RDS02UF::ParseDataFailed(uint8_t *data_in, uint8_t datalengh)
{
    memset(data_in, 0, datalengh);
    step = 0;
    address = 0;
    data_len = 0;
    data_index = 0;
    crc_data = 0;
    fc_code = 0;
    err_code = 0;
}

uint8_t AP_RangeFinder_RDS02UF::crc8(uint8_t* pbuf, int32_t len)
{
     uint8_t* data = pbuf;
     uint8_t crc = 0;
     while ( len-- )
     crc = crc8_table[crc^*(data++)];
     return crc;
}