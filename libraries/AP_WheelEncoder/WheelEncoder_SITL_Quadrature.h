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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "WheelEncoder_Backend.h"
#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>

class AP_WheelEncoder_SITL_Quadrature : public AP_WheelEncoder_Backend
{
public:
    // constructor
    using AP_WheelEncoder_Backend::AP_WheelEncoder_Backend;

    // update state
    void update(void) override;

private:
    int32_t  _distance_count; // distance count as number of encoder ticks
    uint32_t _total_count; // total number of encoder ticks
};

#endif // CONFIG_HAL_BOARD