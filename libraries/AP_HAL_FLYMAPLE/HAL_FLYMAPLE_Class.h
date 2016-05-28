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
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_FLYMAPLE_CLASS_H__
#define __AP_HAL_FLYMAPLE_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_FLYMAPLE_Namespace.h"

class HAL_FLYMAPLE : public AP_HAL::HAL {
public:
    HAL_FLYMAPLE();
    void init(int argc, char * const * argv) const;
};

extern const HAL_FLYMAPLE AP_HAL_FLYMAPLE;

#endif // __AP_HAL_FLYMAPLE_CLASS_H__

