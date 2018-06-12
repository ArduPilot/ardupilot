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

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_NMEA : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_NMEA(RangeFinder::RangeFinder_State &_state,
                        AP_SerialManager &serial_manager,
                        uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    /// enum for handled messages
    enum sentence_types : uint8_t {
        SONAR_UNKNOWN = 0,
        SONAR_DBT,
        SONAR_DPT
    };

    // get a reading
    bool get_reading(uint16_t &reading_cm);

    // add a single character to the buffer and attempt to decode
    // returns true if a complete sentence was successfully decoded
    // distance should be pulled directly from _distance_m member
    bool decode(char c);

    // decode the just-completed term
    // returns true if new sentence has just passed checksum test and is validated
    bool decode_latest_term();

    // return the numeric value of an ascii hex character
    static int16_t char_to_hex(char a);

    AP_HAL::UARTDriver *uart = nullptr;     // pointer to serial uart
    uint32_t _last_reading_ms;              // system time of last successful reading

    // message decoding related members
    char _term[15];                         // buffer for the current term within the current sentence
    uint8_t _term_offset;                   // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;                   // term index within the current sentence
    float _distance_m;                      // distance in meters parsed from a term, -1 if no distance
    uint8_t _checksum;                      // checksum accumulator
    bool _term_is_checksum;                 // current term is the checksum
    sentence_types _sentence_type;          // the sentence type currently being processed
};
