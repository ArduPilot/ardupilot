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

class AP_RangeFinder_NMEA : public AP_RangeFinder_Backend_Serial, AP_NMEA_Input
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_NMEA(_state, _params);
    }

    void init_serial(uint8_t serial_instance) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    // methods required to be a AP_NMEA_Input
    void handle_decode_success() override;
    bool start_sentence_type(const char *term_type) override;
    bool handle_term(uint8_t term_number, const char *term) override;

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    const char *sentence_dbt = "DBT";
    const char *sentence_dpt = "DPT";
    const char *sentence_mtw = "MTW";
    const char *sentence_hded = "HDED";

    // get a distance reading
    bool get_reading(float &reading_m) override;

    // get temperature reading in C.  returns true on success and populates temp argument
    bool get_temp(float &temp) const override;

    uint16_t read_timeout_ms() const override { return 3000; }

    // variables for callbacks to accumulate into:
    float sum;
    uint16_t count;

    float _distance_m = -1.0f;              // distance in meters parsed from a term, -1 if no distance
    float _temp_unvalidated;                // unvalidated temperature in C (may have failed checksum)
    float _temp;                            // temperature in C (validated)
    uint32_t _temp_readtime_ms;             // system time we last read a validated temperature, 0 if never read

    const char *_current_sentence_type = nullptr;
};

#endif  // AP_RANGEFINDER_NMEA_ENABLED
 
