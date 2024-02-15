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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NMEA_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_NMEA : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_NMEA(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /// enum for handled messages
    enum sentence_types : uint8_t {
        SONAR_UNKNOWN = 0,
        SONAR_DBT,
        SONAR_DPT,
        SONAR_MTW,  // mean water temperature
        SONAR_HDED, // hondex custom sonar message
    };

    // get a distance reading
    bool get_reading(float &reading_m) override;

    // get temperature reading in C.  returns true on success and populates temp argument
    bool get_temp(float &temp) const override;

    uint16_t read_timeout_ms() const override { return 3000; }

    // add a single character to the buffer and attempt to decode
    // returns true if a distance was successfully decoded
    // distance should be pulled directly from _distance_m member
    bool decode(char c);

    // decode the just-completed term
    // returns true if new sentence has just passed checksum test and is validated
    bool decode_latest_term();

    // message decoding related members
    char _term[15];                         // buffer for the current term within the current sentence
    uint8_t _term_offset;                   // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;                   // term index within the current sentence
    float _distance_m = -1.0f;              // distance in meters parsed from a term, -1 if no distance
    float _temp_unvalidated;                // unvalidated temperature in C (may have failed checksum)
    float _temp;                            // temperature in C (validated)
    uint32_t _temp_readtime_ms;             // system time we last read a validated temperature, 0 if never read
    uint8_t _checksum;                      // checksum accumulator
    bool _term_is_checksum;                 // current term is the checksum
    sentence_types _sentence_type;          // the sentence type currently being processed
    bool _sentence_done;                    // true if this sentence has already been decoded
};

#endif  // AP_RANGEFINDER_NMEA_ENABLED
