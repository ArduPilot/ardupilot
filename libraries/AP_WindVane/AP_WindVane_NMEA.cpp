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

#include "AP_WindVane_config.h"

#if AP_WINDVANE_NMEA_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_WindVane_NMEA.h"
#include <AP_SerialManager/AP_SerialManager.h>

/*
    NMEA wind vane library, tested with Calypso Wired sensor,
    should also work with other NMEA wind sensors using the MWV message,
    heavily based on RangeFinder NMEA library
*/

// init - performs any required initialization for this instance
void AP_WindVane_NMEA::init(const AP_SerialManager& serial_manager)
{
    AP_NMEA_Input::init(AP_SerialManager::SerialProtocol_WindVane, 0);
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

void AP_WindVane_NMEA::handle_decode_success()
{
    // user may not have NMEA selected for both speed and direction
    if (_frontend._direction_type.get() == _frontend.WindVaneType::WINDVANE_NMEA) {
        _frontend._direction_apparent_raw = wrap_PI(radians(_wind_dir_deg + _frontend._dir_analog_bearing_offset.get()));
    }
    if (_frontend._speed_sensor_type.get() == _frontend.Speed_type::WINDSPEED_NMEA) {
        _frontend._speed_apparent_raw = _speed_ms;
    }
}


bool AP_WindVane_NMEA::start_sentence_type(const char *term_type)
{
    _wind_dir_deg = -1.0f;
    _speed_ms = -1.0f;
    // wind only:
    return strcmp(term_type, "MWV") == 0;
}

bool AP_WindVane_NMEA::handle_term(uint8_t term_number, const char *term)
{
    bool ret = true;
    switch (term_number) {
        case 1:
            _wind_dir_deg = strtof(term, NULL);
            // check for sensible value
            if (is_negative(_wind_dir_deg) || _wind_dir_deg > 360.0f) {
                ret = false;
            }
            break;

        case 2:
            // we are expecting R for relative wind 
            // (could be T for true wind, maybe add in the future...)
            if (term[0] != 'R') {
                ret = false;
            }
            break;

        case 3:
            _speed_ms = strtof(term, NULL);
            break;

        case 4:
            if (term[0] == 'K') {
                // convert from km/h to m/s
                _speed_ms *= KM_PER_HOUR_TO_M_PER_SEC;
            } else if (term[0] == 'N') {
                // convert from Knots to m/s
                _speed_ms *= KNOTS_TO_M_PER_SEC;
            }
            // could also be M for m/s, but we want that anyway so nothing to do
            // check for sensible value
            if (is_negative(_speed_ms) || _speed_ms > 100.0f) {
                ret = false;
            }
            break;

        case 5:
            // expecting A for data valid
            if (term[0] != 'A') {
                ret = false;
            }
            break;

    }

    return ret;
}

#endif  // AP_WINDVANE_NMEA_ENABLED
