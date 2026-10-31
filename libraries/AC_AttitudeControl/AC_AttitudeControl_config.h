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

// compile-time options for the attitude and position controllers

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// let the position controller's thrust vector point downward on a planned path.  This header
// is shared by every vehicle, so it cannot depend on the vehicle type; only multirotor Copter
// grants the regime, and only Copter shows its parameters
#ifndef AC_POSCONTROL_INVERTED_THRUST_ENABLED
#define AC_POSCONTROL_INVERTED_THRUST_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
