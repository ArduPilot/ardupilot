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
#include <AP_NMEA/AP_NMEA_Input.h>

class AP_RangeFinder_NMEA : public AP_RangeFinder_Backend
{
friend class AP_NMEA_Input_RangeFinder;

public:
    // constructor
    AP_RangeFinder_NMEA(RangeFinder::RangeFinder_State &_state,
                        AP_RangeFinder_Params &_params,
                        uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    // Sum and count for average reading
    float sum = 0.0f;
    uint16_t count = 0;

private:

    // pointer to NMEA input driver
    AP_NMEA_Input *NMEA_Driver;

    // get a reading
    bool get_reading(uint16_t &reading_cm);

};

class AP_NMEA_Input_RangeFinder : public AP_NMEA_Input
{
public:
    // constructor
    AP_NMEA_Input_RangeFinder(AP_RangeFinder_NMEA &frontend, AP_SerialManager::SerialProtocol protocol, uint8_t serial_instance):
        AP_NMEA_Input(protocol),
        rangefinder_backend(frontend),
        _distance_m(-1.0f) {};

private:
    // decode the just-completed term
    // returns true if new sentence has just passed checksum test and is validated
    void decode_latest_term() override;

    void write() override;

    /// enum for handled messages
    enum sentence_types : uint8_t {
        SONAR_UNKNOWN = 0,
        SONAR_DBT,
        SONAR_DPT
    };

    // message decoding related members
    float _distance_m;                      // distance in meters parsed from a term, -1 if no distance
    sentence_types _sentence_type;          // the sentence type currently being processed

    AP_RangeFinder_NMEA &rangefinder_backend;
};
