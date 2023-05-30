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
#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED
#include "AP_Networking_Backend.h"


/*
    base class constructor.
    This incorporates initialisation as well.
*/
AP_Networking_Backend::AP_Networking_Backend(AP_Networking &front,
                                            AP_Networking::AP_Networking_State &state,
                                            AP_Networking_Params &params):
    _front(front),
    _state(state),
    _params(params)
{
}

#endif // AP_NETWORKING_ENABLED

