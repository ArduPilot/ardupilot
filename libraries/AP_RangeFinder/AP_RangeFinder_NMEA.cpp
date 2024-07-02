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

#include "AP_RangeFinder_NMEA.h"

#if AP_RANGEFINDER_NMEA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

void AP_RangeFinder_NMEA::init_serial(uint8_t serial_instance)
{
    AP_RangeFinder_Backend_Serial::init_serial(serial_instance);
    AP_NMEA_Input::init(uart);
}

// return last value measured by sensor
bool AP_RangeFinder_NMEA::get_reading(float &reading_m)
{
    sum = 0;
    count = 0;

    AP_NMEA_Input::update();

    // return false on failure
    if (count == 0) {
        return false;
    }

    // return average of all measurements
    reading_m = sum / count;
    return true;
}

// get temperature reading
bool AP_RangeFinder_NMEA::get_temp(float &temp) const
{
    uint32_t now_ms = AP_HAL::millis();
    if ((_temp_readtime_ms == 0) || ((now_ms - _temp_readtime_ms) > read_timeout_ms())) {
        return false;
    }
    temp = _temp;
    return true;
}

bool AP_RangeFinder_NMEA::start_sentence_type(const char *term_type)
{
    const char *valid_sentences[] {
        sentence_dbt,
        sentence_dpt,
        sentence_mtw,
        sentence_hded,
    };
    for (auto valid_sentence : valid_sentences) {
        if (strcmp(term_type, valid_sentence)) {
            _current_sentence_type = valid_sentence;
            _distance_m = -1;
            _temp_unvalidated = -600;
            return true;
        }
    }
    return false;
}

// decode the most recently consumed term
// returns true if new distance sentence has just passed checksum test and is validated
bool AP_RangeFinder_NMEA::handle_term(uint8_t _term_number, const char *_term)
{
    if (_current_sentence_type == sentence_dbt) {
        // parse DBT messages
        if (_term_number == 3) {
            _distance_m = strtof(_term, NULL);
        }
    } else if (_current_sentence_type == sentence_dpt) {
        // parse DPT messages
        if (_term_number == 1) {
            _distance_m = strtof(_term, NULL);
        }
    } else if (_current_sentence_type == sentence_mtw) {
        // parse MTW (mean water temperature) messages
        if (_term_number == 1) {
            _temp_unvalidated = strtof(_term, NULL);
        }
    } else if (_current_sentence_type == sentence_hded) {
        // parse HDED (Hondex custom message)
        if (_term_number == 4) {
            _distance_m = strtof(_term, NULL);
        }
    }

    return true;
}

void AP_RangeFinder_NMEA::handle_decode_success()
{
    if (_distance_m > 0) {
        sum += _distance_m;
        count++;
    }
    if (_temp_unvalidated > -500) {
        _temp = _temp_unvalidated;
        _temp_readtime_ms = AP_HAL::millis();
    }
}

#endif  // AP_RANGEFINDER_NMEA_ENABLED
