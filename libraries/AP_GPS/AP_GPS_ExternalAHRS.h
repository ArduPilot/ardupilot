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
//  ExternalAHRS GPS driver which accepts gps position data from an external AHRS unit
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_EXTERNAL_AHRS_ENABLED

class AP_GPS_ExternalAHRS : public AP_GPS_Backend
{
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    bool read() override;
    void handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt) override;

    const char *name() const override { return "ExternalAHRS"; }

    bool get_lag(float &lag_sec) const override;

private:
    bool new_data;
};

#endif // AP_EXTERNAL_AHRS_ENABLED

