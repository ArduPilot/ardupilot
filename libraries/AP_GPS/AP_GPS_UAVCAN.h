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
//  UAVCAN GPS driver
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_UAVCAN : public AP_GPS_Backend {
public:
    AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);
    ~AP_GPS_UAVCAN() override;

    bool read() override;

    // This method is called from UAVCAN thread
    void handle_gnss_msg(const AP_GPS::GPS_State &msg) override;
private:
    bool _new_data;

    AP_GPS::GPS_State _interm_state;
    AP_HAL::Semaphore *_sem_gnss;
};
