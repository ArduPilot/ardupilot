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

#include "AP_NMEA_Input.h"

// optionally log all NMEA data for debug purposes
// #define NMEA_LOG_PATH "nmea.log"

#ifdef NMEA_LOG_PATH
#include <stdio.h>
#endif

// init - see if we can find the correct serial port
AP_NMEA_Input::AP_NMEA_Input(AP_SerialManager::SerialProtocol protocol, uint8_t serial_instance)
{
    _uart = AP::serialmanager().find_serial(protocol, serial_instance);

    if (_uart != nullptr) {
        _uart->begin(AP::serialmanager().find_baudrate(protocol, serial_instance));
    }
}

// init - pass in a serial port pointer
AP_NMEA_Input::AP_NMEA_Input(AP_HAL::UARTDriver *uart)
{
    _uart = uart;
}

bool AP_NMEA_Input::update()
{
    if (_uart == nullptr) {
        return false;
    }

    // read any available lines from the uart
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
#ifdef NMEA_LOG_PATH
        static FILE *logf = nullptr;
        if (logf == nullptr) {
            logf = fopen(NMEA_LOG_PATH, "wb");
        }
        if (logf != nullptr) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (decode(c) && _data_valid) {
            // we have decoded a full message and the data is valid
            write();
        }
    }
    return true;
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_NMEA_Input::decode(char c)
{
    _sentence_length++;

    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        // null terminate and decode latest term
        _term[_term_offset] = 0;
        bool valid_sentence = false;

        // handle the last term in a message
        if (_term_is_checksum) {
            uint8_t nibble_high = 0;
            uint8_t nibble_low  = 0;
            if (!hex_to_uint8(_term[0], nibble_high) || !hex_to_uint8(_term[1], nibble_low)) {
                return false;
            }
            const uint8_t checksum = (nibble_high << 4u) | nibble_low;
            valid_sentence = checksum == _checksum;
        } else {
            decode_latest_term();
        }

        // move onto next term
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        // return true if we have decoded a valid NMEA sentence
        // this check's the sentence only not the data
        return valid_sentence;
    }

    case '$': // sentence begin
        _sentence_length = 0;
        _data_valid = false;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        return false;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_term) - 1) {
        _term[_term_offset++] = c;
    }
    if (!_term_is_checksum) {
        _checksum ^= c;
    }

    return false;
}
