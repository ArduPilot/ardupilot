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

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>

class AP_Proximity_AirSimSITL : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:
    SITL::SITL *sitl = AP::sitl();
};
#endif // CONFIG_HAL_BOARD
