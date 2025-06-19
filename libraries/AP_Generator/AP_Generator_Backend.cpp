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
#pragma GCC optimize("Os")

#include "AP_Generator_Backend.h"

#if HAL_GENERATOR_ENABLED

// Base class constructor
AP_Generator_Backend::AP_Generator_Backend(AP_Generator& frontend) :
    _frontend(frontend)
{
}

// Called from the subclass update function to update the frontend variables for accessing
void AP_Generator_Backend::update_frontend()
{
    // Update the values in the front end
    _frontend._voltage = _voltage;
    _frontend._current = _current;
    _frontend._consumed_mah = _consumed_mah;
    _frontend._fuel_remaining = _fuel_remaining;
    _frontend._fuel_remaining_l = _fuel_remaining_l;
    _frontend._rpm = _rpm;
    _frontend._healthy = healthy();
}
#endif
