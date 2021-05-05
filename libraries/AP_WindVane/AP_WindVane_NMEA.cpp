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

#include <AP_HAL/AP_HAL.h>
#include "AP_WindVane_NMEA.h"
#include <AP_SerialManager/AP_SerialManager.h>

/*
    NMEA wind vane library, tested with Calypso Wired sensor,
    should also work with other NMEA wind sensors using the MWV message,
    heavily based on RangeFinder NMEA library
*/

// constructor
AP_WindVane_NMEA::AP_WindVane_NMEA(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
}

// init - performs any required initialization for this instance
void AP_WindVane_NMEA::init(const AP_SerialManager& serial_manager)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_WindVane, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_WindVane, 0));
    }
}

void AP_WindVane_NMEA::update_direction()
{
    // Only call update from here if it has not been called already by update speed
    if (_frontend._speed_sensor_type.get() != _frontend.Speed_type::WINDSPEED_NMEA) {
        update();
    }
}

void AP_WindVane_NMEA::update_speed()
{
    update();
}

void AP_WindVane_NMEA::update()
{
    if (uart == nullptr) {
        return;
    }

    // read any available lines from the windvane
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (decode(c)) {
            // user may not have NMEA selected for both speed and direction
            if (_frontend._direction_type.get() == _frontend.WindVaneType::WINDVANE_NMEA) {
                _frontend._direction_apparent_raw = wrap_PI(radians(_wind_dir_deg + _frontend._dir_analog_bearing_offset.get()));
            }
            if (_frontend._speed_sensor_type.get() == _frontend.Speed_type::WINDSPEED_NMEA) {
                _frontend._speed_apparent_raw = _speed_ms;
            }
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_WindVane_NMEA::decode(char c)
{
    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        if (_sentence_done) {
            return false;
        }

        // null terminate and decode latest term
        _term[_term_offset] = 0;
        bool valid_sentence = decode_latest_term();

        // move onto next term
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '$': // sentence begin
        _sentence_valid = false;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        _wind_dir_deg = -1.0f;
        _speed_ms = -1.0f;
        _sentence_done = false;
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

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_WindVane_NMEA::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        _sentence_done = true;
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        return ((checksum == _checksum) && _sentence_valid);
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
             // unknown ID (we are actually expecting II)
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "MWV") == 0) {
            // we found the sentence type for wind
            _sentence_valid = true;
        }
        return false;
    }

    // if this is not the sentence we want then wait for another
    if (!_sentence_valid) {
        return false;
    }

    switch (_term_number) {
        case 1:
            _wind_dir_deg = strtof(_term, NULL);
            // check for sensible value
            if (is_negative(_wind_dir_deg) || _wind_dir_deg > 360.0f) {
                _sentence_valid = false;
            }
            break;

        case 2:
            // we are expecting R for relative wind 
            // (could be T for true wind, maybe add in the future...)
            if (_term[0] != 'R') {
                _sentence_valid = false;
            }
            break;

        case 3:
            _speed_ms = strtof(_term, NULL);
            break;

        case 4:
            if (_term[0] == 'K') {
                // convert from km/h to m/s
                _speed_ms *= KM_PER_HOUR_TO_M_PER_SEC;
            } else if (_term[0] == 'N') {
                // convert from Knots to m/s
                _speed_ms *= KNOTS_TO_M_PER_SEC;
            }
            // could also be M for m/s, but we want that anyway so nothing to do
            // check for sensible value
            if (is_negative(_speed_ms) || _speed_ms > 100.0f) {
                _sentence_valid = false;
            }
            break;

        case 5:
            // expecting A for data valid
            if (_term[0] != 'A') {
                _sentence_valid = false;
            }
            break;

    }
    return false;
}

// return the numeric value of an ascii hex character
int16_t AP_WindVane_NMEA::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}
