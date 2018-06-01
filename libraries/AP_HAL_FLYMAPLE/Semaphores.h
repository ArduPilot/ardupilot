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

#ifndef __AP_HAL_FLYMAPLE_SEMAPHORE_H__
#define __AP_HAL_FLYMAPLE_SEMAPHORE_H__

#include "AP_HAL_FLYMAPLE.h"

class AP_HAL_FLYMAPLE_NS::FLYMAPLESemaphore : public AP_HAL::Semaphore {
public:
    FLYMAPLESemaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();

    bool _taken;
};

#endif // __AP_HAL_FLYMAPLE_SEMAPHORE_H__
