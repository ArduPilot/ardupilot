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

#include <AP_Vehicle/AP_Vehicle_Type.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 

#include "AP_Airspeed.h"
#include <AP_SerialManager/AP_SerialManager.h>

#define TIMEOUT_MS 2000

bool AP_Airspeed_NMEA::init()
{
    if (!AP_NMEA_Input::init(AP_SerialManager::SerialProtocol_AirSpeed, 0)) {
        return false;
    }

    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,0,0,0));

    // make sure this sensor cannot be used in the EKF
    set_use(0);

    // must set use zero offset to pass offset check for health
    set_use_zero_offset();

    return true;
}

// read the from the sensor
bool AP_Airspeed_NMEA::get_airspeed(float &airspeed)
{
    uint32_t now = AP_HAL::millis();

    _sum = 0;
    _count = 0;
    _temp_sum = 0;
    _temp_count = 0;

    AP_NMEA_Input::update();

    if (_count == 0) {
        // Cant return false because updates are too slow, return previous reading
        // Could return false after some timeout, however testing shows that the DST800 just stops sending the message at zero speed
        airspeed = _last_speed;
    } else {
        // return average of all measurements
        airspeed = _sum / _count;
        _last_speed = airspeed;
        _last_update_ms = now;
    }

    return (now - _last_update_ms) < TIMEOUT_MS;
}

// return the current temperature in degrees C
// the main update is done in the get_pressure function
// this just reports the value
bool AP_Airspeed_NMEA::get_temperature(float &temperature)
{
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

bool AP_Airspeed_NMEA::start_sentence_type(const char *term_type)
{
    const char *valid_sentences[] {
        sentence_mtw,
        sentence_vhw,
    };
    for (auto valid_sentence : valid_sentences) {
        if (strcmp(term_type, valid_sentence)) {
            _current_sentence_type = valid_sentence;
            _speed = -1000;
            _temp = -400;
            return true;
        }
    }
    return false;
}

bool AP_Airspeed_NMEA::handle_term(uint8_t _term_number, const char *_term)
{
    if (_current_sentence_type == sentence_mtw) {
        // parse MTW messages
        if (_term_number == 1) {
            _temp = strtof(_term, NULL);
        }
    } else if (_current_sentence_type == sentence_vhw) {
        // parse VHW messages
        if (_term_number == 7) {
            _speed = strtof(_term, NULL) * KM_PER_HOUR_TO_M_PER_SEC;
        }
    }

    return true;
}

void AP_Airspeed_NMEA::handle_decode_success()
{
    if (_speed > -900) {
        _sum += _speed;
        _count++;
    }
    if (_temp > -300) {
        _temp_sum += _temp;
        _temp_count++;
    }
}

#endif  // APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 

#endif  // AP_AIRSPEED_NMEA_ENABLED
