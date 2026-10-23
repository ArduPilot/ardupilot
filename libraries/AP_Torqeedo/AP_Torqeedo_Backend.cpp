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

#include "AP_Torqeedo_Backend.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Torqeedo_Backend::AP_Torqeedo_Backend(AP_Torqeedo_Params &params, uint8_t instance) :
    _params(params),
    _instance(instance)
{}

#endif // HAL_TORQEEDO_ENABLED
