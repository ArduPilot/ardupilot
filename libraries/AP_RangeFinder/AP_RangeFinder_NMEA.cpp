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
#include "AP_RangeFinder_LightWareSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NMEA.h"

extern const AP_HAL::HAL& hal;

// constructor initialises the rangefinder
// Note this is called after detect() returns true, so we
// already know that we should setup the rangefinder
AP_RangeFinder_NMEA::AP_RangeFinder_NMEA(RangeFinder::RangeFinder_State &_state,
                                         AP_RangeFinder_Params &_params,
                                         uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    NMEA_Driver = new AP_NMEA_Input_RangeFinder(*this, AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
}

// detect if a NMEA rangefinder by looking to see if the user has configured it
bool AP_RangeFinder_NMEA::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// update the state of the sensor
void AP_RangeFinder_NMEA::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = now;
        update_status();
    } else if ((now - state.last_reading_ms) > 3000) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// return last value measured by sensor
bool AP_RangeFinder_NMEA::get_reading(uint16_t &reading_cm)
{
    if (NMEA_Driver == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    sum = 0.0f;
    count = 0;

    NMEA_Driver->update();

    // return false on failure
    if (count == 0) {
        return false;
    }

    // return average of all measurements
    reading_cm = 100.0f * sum / count;
    return true;
}

void AP_NMEA_Input_RangeFinder::write() {
    rangefinder_backend.sum += _distance_m;
    rangefinder_backend.count++;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
void AP_NMEA_Input_RangeFinder::decode_latest_term()
{
    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = SONAR_UNKNOWN;
            return;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "DBT") == 0) {
            _sentence_type = SONAR_DBT;
            _data_valid = true;
        } else if (strcmp(term_type, "DPT") == 0) {
            _sentence_type = SONAR_DPT;
            _data_valid = true;
        } else {
            _sentence_type = SONAR_UNKNOWN;
        }
        return;
    }

    if (_sentence_type == SONAR_DBT) {
        // parse DBT messages
        if (_term_number == 3) {
            _distance_m = strtof(_term, NULL);
        }
    } else if (_sentence_type == SONAR_DPT) {
        // parse DPT messages
        if (_term_number == 1) {
            _distance_m = strtof(_term, NULL);
        }
    }

    return;
}
