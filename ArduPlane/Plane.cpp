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
#include "Plane.h"
#include "version.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : DataFlash{FIRMWARE_STRING}
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
    auto_state.next_wp_no_crosstrack = true;
    auto_state.no_crosstrack = true;
}

Plane plane;
