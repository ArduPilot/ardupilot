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

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane plane;
AP_Vehicle& vehicle = plane;

#if AP_SCRIPTING_ENABLED

// returns true if vehicle is landing. Only used by Lua scripts
bool Plane::is_landing() const
{
    #if HAL_QUADPLANE_ENABLED
    
    return plane.quadplane.in_vtol_land_descent();

    #endif // HAL_QUADPLANE_ENABLED

    return control_mode->is_landing();
}

// returns true if vehicle is taking off. Only used by Lua scripts
bool Plane::is_taking_off() const
{
    #if HAL_QUADPLANE_ENABLED

    return plane.quadplane.in_vtol_takeoff();

    #endif // HAL_QUADPLANE_ENABLED

    return control_mode->is_taking_off();
}

#endif // AP_SCRIPTING_ENABLED
