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
 * backend class for direct attached radios
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Radio_backend.h"

AP_Radio_backend::AP_Radio_backend(AP_Radio &_radio) :
    radio(_radio)
{
}

AP_Radio_backend::~AP_Radio_backend(void)
{
}
