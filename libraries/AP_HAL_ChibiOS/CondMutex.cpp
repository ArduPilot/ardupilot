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
#include "CondMutex.h"
#include <hal.h>
#include <chmtx.h>
#include <chcond.h>
#include "AP_HAL_ChibiOS.h"
#include <AP_Math/AP_Math.h>

#if CH_CFG_USE_CONDVARS == TRUE

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// constructor
CondMutex::CondMutex()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
    static_assert(sizeof(_cond) >= sizeof(condition_variable_t), "invalid condition_variable size");
    condition_variable_t *cond = (condition_variable_t *)_cond;
    chCondObjectInit(cond);
}

void CondMutex::lock_and_wait(AP_HAL::CondMutex::Condition proc)
{
    mutex_t *mtx = (mutex_t *)_lock;
    condition_variable_t *cond = (condition_variable_t *)_cond;

    chMtxLock(mtx);

    while (!proc()) {   // wait for proc to return true
       chCondWait(cond);
    }
}

void CondMutex::lock_and_signal()
{
    mutex_t *mtx = (mutex_t *)_lock;
    condition_variable_t *cond = (condition_variable_t *)_cond;
    chMtxLock(mtx);
    chCondSignal(cond);
}

void CondMutex::unlock()
{
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
}

#endif // CH_CFG_USE_CONDVARS == TRUE
