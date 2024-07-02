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

#include "AP_WindVane_config.h"

#if AP_WINDVANE_NMEA_ENABLED

#include "AP_WindVane_Backend.h"
#include <AP_NMEA_Input/AP_NMEA_Input.h>

class AP_WindVane_NMEA : public AP_WindVane_Backend, public AP_NMEA_Input
{
public:

    // constructor
    AP_WindVane_NMEA(AP_WindVane &frontend) :
        AP_WindVane_Backend(frontend),
        AP_NMEA_Input()
        { }

    // initialization
    void init(const AP_SerialManager& serial_manager) override;

    // update state
    void update_direction() override;
    void update_speed() override;

protected:

    // methods required to be a AP_NMEA_Input
    void handle_decode_success() override;
    bool start_sentence_type(const char *term_type) override;
    bool handle_term(uint8_t term_number, const char *term) override;

private:

    // latest values read in
    float _speed_ms;
    float _wind_dir_deg;
};

#endif  // AP_WINDVANE_NMEA_ENABLED
