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
 * Code by @davidbuzz and Claude
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "Semaphores.h"
#include "Scheduler.h"   /* reassert_main_priority() */

using namespace Zephyr;

// constructor
Semaphore::Semaphore()
{
#ifdef __ZEPHYR__
    k_mutex_init(&_mutex);
#endif
}

bool Semaphore::give()
{
#ifdef __ZEPHYR__
    const bool ok = (k_mutex_unlock(&_mutex) == 0);
    /* k_mutex_unlock() has just put this thread back to the priority it had when
       it took the mutex (kernel/mutex.c:275). If this is main and its intended
       priority changed in between - boost_end() runs under AP_AHRS's _rsem, and
       AP_Scheduler's _rsem is taken while boosted - the kernel has just undone
       that change. ChibiOS keeps realprio in the thread for this; Zephyr keeps
       the snapshot in the mutex, so the HAL owns it. One compare on every give();
       the kernel call only when they differ. */
    Scheduler::reassert_main_priority();
    return ok;
#else
    return true;
#endif
}

#ifdef __ZEPHYR__
/* About to block on a mutex main owns, from another thread: register as a
   waiter so a give() or set_main_priority() on main meanwhile cannot put main
   below this thread while it is still pending (Scheduler.cpp, s_main_waiters).
   The owner field is read without the kernel lock: a stale read in either
   direction costs one needless or one missing registration for the length of
   the race window, never a wrong priority for longer than that. */
static inline bool note_main_waiter(const struct k_mutex &m)
{
    const k_tid_t main_tid = Scheduler::main_thread_id();
    const k_tid_t self = k_current_get();
    if (m.owner == nullptr || m.owner != main_tid || self == main_tid) {
        return false;
    }
    Scheduler::main_waiter_begin(k_thread_priority_get(self));
    return true;
}
#endif

bool Semaphore::take(uint32_t timeout_ms)
{
#ifdef __ZEPHYR__
    const bool waiter = note_main_waiter(_mutex);
    const int ret = k_mutex_lock(&_mutex, timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER ? K_FOREVER : K_MSEC(timeout_ms));
    if (waiter) {
        Scheduler::main_waiter_end();
    }
    return ret == 0;
#else
    return true;
#endif
}

bool Semaphore::take_nonblocking()
{
#ifdef __ZEPHYR__
    return k_mutex_lock(&_mutex, K_NO_WAIT) == 0;
#else
    return true;
#endif
}

BinarySemaphore::BinarySemaphore(bool initial_state)
{
#ifdef __ZEPHYR__
    /* count = initial_state ? 1 : 0, limit = 1 */
    k_sem_init(&_sem, initial_state ? 1u : 0u, 1u);
#endif
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
#ifdef __ZEPHYR__
    if (timeout_us == 0u) {
        return k_sem_take(&_sem, K_NO_WAIT) == 0;
    }
    return k_sem_take(&_sem, K_USEC(timeout_us)) == 0;
#else
    return true;
#endif
}

bool BinarySemaphore::wait_blocking()
{
#ifdef __ZEPHYR__
    return k_sem_take(&_sem, K_FOREVER) == 0;
#else
    return true;
#endif
}

void BinarySemaphore::signal()
{
#ifdef __ZEPHYR__
    k_sem_give(&_sem);
#endif
}

void BinarySemaphore::signal_ISR()
{
#ifdef __ZEPHYR__
    /* k_sem_give() is safe to call from ISR context. */
    k_sem_give(&_sem);
#endif
}


#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
