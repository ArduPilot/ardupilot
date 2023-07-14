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

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Torqeedo_Backend::AP_Torqeedo_Backend(AP_Torqeedo& _frontend, AP_Torqeedo::Torqeedo_State& _state, AP_Torqeedo_Params& _params) :
        frontend(_frontend),
        state(_state),
        params(_params)
{
    _backend_type = (AP_Torqeedo::Type )_params.type.get();
}


// set status and update valid count
void AP_Torqeedo_Backend::set_status(AP_Torqeedo::Status status)
{
    state.status = status;
}

#endif