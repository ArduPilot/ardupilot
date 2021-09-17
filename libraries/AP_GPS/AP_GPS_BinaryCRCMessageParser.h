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

#pragma once

#include "AP_GPS.h"
#include <vector>

/*
    parser abstract class for parse binary crc32 gps message
*/
class AP_GPS_BinaryCRCMessageParser
{
public:
    explicit AP_GPS_BinaryCRCMessageParser(const std::vector<uint8_t> &preambles, uint8_t min_header_length, uint8_t max_header_length, uint16_t max_body_length);
    ~AP_GPS_BinaryCRCMessageParser();

protected:
    virtual uint16_t get_message_body_length_from_header_buff() = 0;
    virtual uint8_t get_message_header_length_from_header_buff() = 0;
    virtual uint8_t *get_message_header_buff() = 0;
    virtual uint8_t *get_message_body_buff() = 0;
    bool parse(uint8_t data);
    virtual bool process_message() = 0;

private:
    void reset_parse();

    const std::vector<uint8_t> _preambles;
    uint8_t _min_header_length;
    uint8_t _max_header_length;
    uint16_t _max_body_length;

    uint8_t _decode_step;   // step number of message decode
    uint32_t _msg_crc;      // message crc data
    uint32_t _read_size;    // readed size of current message
    uint8_t _preamble_step; // step of preamble decode
    uint8_t _real_header_length;    //header length from real message header
    uint16_t _real_body_length;     //body length from read message header
};

