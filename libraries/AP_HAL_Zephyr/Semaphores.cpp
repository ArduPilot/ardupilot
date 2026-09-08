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
    return k_mutex_unlock(&_mutex) == 0;
#else
    return true;
#endif
}

bool Semaphore::take(uint32_t timeout_ms)
{
#ifdef __ZEPHYR__
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return k_mutex_lock(&_mutex, K_FOREVER) == 0;
    }
    return k_mutex_lock(&_mutex, K_MSEC(timeout_ms)) == 0;
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
