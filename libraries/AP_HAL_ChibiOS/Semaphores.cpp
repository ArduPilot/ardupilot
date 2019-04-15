/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include "AP_HAL_ChibiOS.h"

#if CH_CFG_USE_MUTEXES == TRUE

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// constructor
Semaphore::Semaphore()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
}

bool Semaphore::give()
{
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    mutex_t *mtx = (mutex_t *)_lock;
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(mtx);
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    mutex_t *mtx = (mutex_t *)_lock;
    return chMtxTryLock(mtx);
}

bool Semaphore::check_owner(void)
{
    mutex_t *mtx = (mutex_t *)_lock;
    return mtx->owner == chThdGetSelfX();
}

void Semaphore::assert_owner(void)
{
    osalDbgAssert(check_owner(), "owner");
}

// constructor
Semaphore_Recursive::Semaphore_Recursive()
    : Semaphore(), count(0)
{}


bool Semaphore_Recursive::give()
{
    chSysLock();
    mutex_t *mtx = (mutex_t *)_lock;
    osalDbgAssert(count>0, "recursive semaphore");
    if (count != 0) {
        count--;
        if (count == 0) {
            // this thread is giving it up
            chMtxUnlockS(mtx);
            // we may need to re-schedule if our priority was increased due to
            // priority inheritance
            chSchRescheduleS();
        }
    }
    chSysUnlock();
    return true;
}

bool Semaphore_Recursive::take(uint32_t timeout_ms)
{
    // most common case is we can get the lock immediately
    if (Semaphore::take_nonblocking()) {
        count=1;
        return true;
    }

    // check for case where we hold it already
    chSysLock();
    mutex_t *mtx = (mutex_t *)_lock;
    if (mtx->owner == chThdGetSelfX()) {
        count++;
        chSysUnlock();
        return true;
    }
    chSysUnlock();
    if (Semaphore::take(timeout_ms)) {
        count = 1;
        return true;
    }
    return false;
}

bool Semaphore_Recursive::take_nonblocking(void)
{
    // most common case is we can get the lock immediately
    if (Semaphore::take_nonblocking()) {
        count=1;
        return true;
    }

    // check for case where we hold it already
    chSysLock();
    mutex_t *mtx = (mutex_t *)_lock;
    if (mtx->owner == chThdGetSelfX()) {
        count++;
        chSysUnlock();
        return true;
    }
    chSysUnlock();
    if (Semaphore::take_nonblocking()) {
        count = 1;
        return true;
    }
    return false;
}

#endif // CH_CFG_USE_MUTEXES

