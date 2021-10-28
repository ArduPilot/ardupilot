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


#include "AP_Hygrometer_Backend.h"
#include "AP_Hygrometer.h"

AP_Hygrometer_Backend::AP_Hygrometer_Backend(AP_Hygrometer &_frontend, uint8_t _instance, AP_Hygrometer::Hygrometer_State &_state, AP_Hygrometer::Hygrometer_Param &_params) :
    frontend(_frontend),
    instance(_instance),
    state(_state),
    params(_params)
{
}


AP_Hygrometer_Backend::~AP_Hygrometer_Backend(void)
{
}
