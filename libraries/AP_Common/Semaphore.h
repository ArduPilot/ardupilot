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
#pragma once

/*
  a method to make semaphores less error prone. The WITH_SEMAPHORE()
  macro will block forever for a semaphore, and will automatically
  release the semaphore when it goes out of scope

  Note that we have two types of semaphores. A normal semaphore can
  only be taken once. A recursive semaphore allows for the thread
  holding the semaphore to take it again. It must be released the same
  number of times it is taken.

  The WITH_SEMAPHORE() macro can be used with either type of semaphore
 */

#include <AP_HAL/Semaphores.h>

namespace AP_HAL {
class Semaphore;
}

class WithSemaphore {
public:
    WithSemaphore(AP_HAL::Semaphore *mtx) : WithSemaphore(*mtx) { }

    WithSemaphore(AP_HAL::Semaphore &mtx) : _mtx(mtx) {
        _mtx.take_blocking();
    }

    ~WithSemaphore() {
        _mtx.give();
    }
private:
    AP_HAL::Semaphore &_mtx;
};

#define WITH_SEMAPHORE(sem) WithSemaphore _getsem ## __UNIQ__(sem)

