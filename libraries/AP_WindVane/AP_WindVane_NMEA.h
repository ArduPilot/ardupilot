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

#include "AP_WindVane_Backend.h"
#include <AP_NMEA/AP_NMEA_Input.h>

class AP_WindVane_NMEA : public AP_WindVane_Backend
{
friend class AP_NMEA_Input_Windvane;

public:
    // constructor
    AP_WindVane_NMEA(AP_WindVane &frontend);

    // initialization
    void init() override;

    // update state
    void update_direction() override;
    void update_speed() override;

private:
    // pointer to NMEA input driver
    AP_NMEA_Input *NMEA_Driver;

    // See if we can read in some data
    void update();
};

class AP_NMEA_Input_Windvane : public AP_NMEA_Input
{
public:
    // constructor
    AP_NMEA_Input_Windvane(AP_WindVane_NMEA &frontend, AP_SerialManager::SerialProtocol protocol):
        AP_NMEA_Input(protocol),
        Wind_vane_backend(frontend) {};

private:
    void decode_latest_term() override;

    void write() override;

    // latest values read in
    float _speed_ms;
    float _wind_dir_deg;

    AP_WindVane_NMEA &Wind_vane_backend;
};
