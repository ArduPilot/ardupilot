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
/*
 *   NMEA Sensor driver for VHW ans MTW messages over Serial
 *   https://gpsd.gitlab.io/gpsd/NMEA.html#_vhw_water_speed_and_heading
 *   https://gpsd.gitlab.io/gpsd/NMEA.html#_mtw_mean_temperature_of_water
 */

#include "AP_Airspeed_NMEA.h"

#if AP_AIRSPEED_NMEA_ENABLED

#include <AP_Vehicle/AP_Vehicle.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 

#include "AP_Airspeed.h"

#define TIMEOUT_MS 2000

extern const AP_HAL::HAL &hal;

AP_Airspeed_NMEA::AP_Airspeed_NMEA(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

bool AP_Airspeed_NMEA::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AirSpeed, 0);
    if (_uart == nullptr) {
        return false;
    }

    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,0,0,0));

    _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_AirSpeed, 0));

    // make sure this sensor cannot be used in the EKF
    set_use(0);

    // must set use zero offset to pass offset check for health
    set_use_zero_offset();

    return true;
}

// read the from the sensor
bool AP_Airspeed_NMEA::get_airspeed(float &airspeed)
{
    if (_uart == nullptr) {
        return false;
    }

    uint32_t now = AP_HAL::millis();

    // read any available lines from the sensor
    float sum = 0.0f;
    uint16_t count = 0;
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            _last_update_ms = now;
            if (_sentence_type == TYPE_VHW) {
                sum += _speed;
                count++;
            } else {
                _temp_sum += _temp;
                _temp_count++;
            }
        }
    }

    if (count == 0) {
        // Cant return false because updates are too slow, return previous reading
        // Could return false after some timeout, however testing shows that the DST800 just stops sending the message at zero speed
        airspeed = _last_speed;
    } else {
        // return average of all measurements
        airspeed = sum / count;
        _last_speed = airspeed;
    }

    return (now - _last_update_ms) < TIMEOUT_MS;
}

// return the current temperature in degrees C
// the main update is done in the get_pressue function
// this just reports the value
bool AP_Airspeed_NMEA::get_temperature(float &temperature)
{
    if (_uart == nullptr) {
        return false;
    }

    if (_temp_count == 0) {
        temperature = _last_temp;
    } else {
        // return average of all measurements
        temperature = _temp_sum / _temp_count;
        _last_temp = temperature;
        _temp_count = 0;
        _temp_sum = 0;
    }

    return true;
}


// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_Airspeed_NMEA::decode(char c)
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
        if (!_sentence_done && _sentence_valid) {
            // null terminate and decode latest term
            _term[_term_offset] = 0;
            bool valid_sentence = decode_latest_term();

            // move onto next term
            _term_number++;
            _term_offset = 0;
            _term_is_checksum = (c == '*');
            return valid_sentence;
        }
        return false;
    }

    case '$': // sentence begin
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        _sentence_done = false;
        _sentence_valid = true;
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
bool AP_Airspeed_NMEA::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        _sentence_done = true;
        uint8_t nibble_high = 0;
        uint8_t nibble_low  = 0;
        if (!hex_to_uint8(_term[0], nibble_high) || !hex_to_uint8(_term[1], nibble_low)) {
            return false;
        }
        const uint8_t checksum = (nibble_high << 4u) | nibble_low;
        return checksum == _checksum;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        // actually expecting YX for MTW and VW for VHW
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "MTW") == 0) {
            _sentence_type = TPYE_MTW;
        } else if (strcmp(term_type, "VHW") == 0) {
            _sentence_type = TYPE_VHW;
        } else {
            _sentence_valid = false;
        }
        return false;
    }

    if (_sentence_type == TPYE_MTW) {
        // parse MTW messages
        if (_term_number == 1) {
            _temp = strtof(_term, NULL);
        }
    } else if (_sentence_type == TYPE_VHW) {
        // parse VHW messages
        if (_term_number == 7) {
            _speed = strtof(_term, NULL) * KM_PER_HOUR_TO_M_PER_SEC;
        }
    }

    return false;
}

#endif  // APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 

#endif  // AP_AIRSPEED_NMEA_ENABLED
