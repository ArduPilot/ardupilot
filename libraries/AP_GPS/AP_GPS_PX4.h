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
//  GPS proxy driver for APM on PX4 platforms
//  Code by Holger Steinhaus
//
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <modules/uORB/topics/vehicle_gps_position.h>

class AP_GPS_PX4 : public AP_GPS_Backend {
public:
    AP_GPS_PX4(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read();

private:
    int                           _gps_sub;
    struct vehicle_gps_position_s _gps_pos;
};
#endif // CONFIG_HAL_BOARD
