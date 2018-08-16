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
  a method to make semaphores less error prone. The WITH_SEMAPHORE()
  macro will block forever for a semaphore, and will automatically
  release the semaphore when it goes out of scope
 */
class WithSemaphore {
public:
    WithSemaphore(HAL_Semaphore &mtx) :
    _mtx(mtx)
    {
        _mtx.take_blocking();
    }

    ~WithSemaphore() {
        _mtx.give();
    }
private:
    HAL_Semaphore &_mtx;
};

#define WITH_SEMAPHORE(sem) WithSemaphore _getsem(sem)

