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
/*
 *   AP_Common.cpp - common utility functions
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_Common.h"

extern const AP_HAL::HAL& hal;

/*
  Return true if value is between lower and upper bound inclusive.
  False otherwise.
*/
bool is_bounded_int32(int32_t value, int32_t lower_bound, int32_t upper_bound)
{
    if ((lower_bound <= upper_bound) &&
        (value >= lower_bound) && (value <= upper_bound)) {
        return true;
    }

    return false;

}

// make sure we know what size the Location object is:
assert_storage_size<Location, 16> _assert_storage_size_Location;
