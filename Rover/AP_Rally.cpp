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

#include <AP_Common/Location.h>

#include "Rover.h"

#include "AP_Rally.h"

bool AP_Rally_Rover::is_valid(const Location &rally_point) const
{
#if AP_FENCE_ENABLED
    if (!rover.fence.check_destination_within_fence(rally_point)) {
        return false;
    }
#endif
    return true;
}
