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

class AP_WindVane_SITL : public AP_WindVane_Backend
{
public:

    // constructor
    using AP_WindVane_Backend::AP_WindVane_Backend;

    // update state
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        void update_direction() override;
        void update_speed() override;
    #endif
};
