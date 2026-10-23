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

#include "GPS_Backend.h"

#include <SITL/SITL.h>

#if AP_SIM_GPS_ENABLED

class AP_GPS_SITL : public AP_GPS_Backend
{

public:

    using AP_GPS_Backend::AP_GPS_Backend;

    bool        read() override;

    const char *name() const override { return "SITL"; }

private:

    uint32_t last_update_ms;
};

#endif  // AP_SIM_GPS_ENABLED
