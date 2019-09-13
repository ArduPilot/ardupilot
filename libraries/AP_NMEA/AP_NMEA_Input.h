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

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_NMEA_Input {

public:
    // constructor. This incorporates initialization as well
    // protocol and port
    AP_NMEA_Input(AP_SerialManager::SerialProtocol protocol, uint8_t serial_instance = 0);
    // uart pointer
    AP_NMEA_Input(AP_HAL::UARTDriver *uart);

    // See if we can read in some data
    bool update();

    // return true if we have new data
    // assume consumer immediately reads the data
    bool new_data();

protected:
    // protocol specific decoding
    virtual void decode_latest_term() {};

    // write decoded data to consumer
    virtual void write() {};

    // pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr; 

    // try and decode NMEA message
    bool decode(char c);

    char _term[64];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _data_valid;          // is current data is valid so far
    uint16_t _sentence_length; // number of characters in current sentence
};
