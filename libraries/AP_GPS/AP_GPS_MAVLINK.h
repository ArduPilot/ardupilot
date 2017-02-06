// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

#pragma once

#include "AP_GPS_NMEA.h"

/// NMEA parser
///
class AP_GPS_MAVLINK : public AP_GPS_NMEA
{

public:
	AP_GPS_MAVLINK(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read();

    // Not for DGPS, but to actually inject the aircraft's gps position via Mavlink
    void inject_data(uint8_t *data, uint8_t len);

private:
    bool _new_data;
};
