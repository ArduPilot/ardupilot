// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_Common.h>
#include <AP_HAL.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_RangeFinder_Backend::AP_RangeFinder_Backend(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
        ranger(_ranger),
        state(_state) 
{
}
