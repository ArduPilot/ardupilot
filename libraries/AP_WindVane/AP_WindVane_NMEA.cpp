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
void AP_WindVane_NMEA::init()
{
    NMEA_Driver = new AP_NMEA_Input_Windvane(*this, AP_SerialManager::SerialProtocol_WindVane);
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
    if (NMEA_Driver != nullptr) {
        NMEA_Driver->update();
    }
}

void AP_NMEA_Input_Windvane::write()
{
    // user may not have NMEA selected for both speed and direction
    if (Wind_vane_backend._frontend._direction_type.get() == Wind_vane_backend._frontend.WindVaneType::WINDVANE_NMEA) {
        Wind_vane_backend.direction_update_frontend(wrap_PI(radians(_wind_dir_deg + Wind_vane_backend._frontend._dir_analog_bearing_offset.get()) + AP::ahrs().yaw));
    }
    if (Wind_vane_backend._frontend._speed_sensor_type.get() == Wind_vane_backend._frontend.Speed_type::WINDSPEED_NMEA) {
        Wind_vane_backend.speed_update_frontend(_speed_ms);
    }
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
void AP_NMEA_Input_Windvane::decode_latest_term()
{
    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            // unknown ID (we are actually expecting II)
            return;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "MWV") == 0) {
            // we found the sentence type for wind
            _data_valid = true;
        }
    }

    // if this is not the sentence we want then wait for another
    if (!_data_valid) {
        return;
    }

    switch (_term_number) {
        case 1:
            _wind_dir_deg = strtof(_term, NULL);
            // check for sensible value
            if (is_negative(_wind_dir_deg) || _wind_dir_deg > 360.0f) {
                _data_valid = false;
            }
            break;

        case 2:
            // we are expecting R for relative wind 
            // (could be T for true wind, maybe add in the future...)
            if (_term[0] != 'R') {
                _data_valid = false;
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
                _data_valid = false;
            }
            break;

        case 5:
            // expecting A for data valid
            if (_term[0] != 'A') {
                _data_valid = false;
            }
            break;

    }
    return;
}
