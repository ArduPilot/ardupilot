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
#include "ch_compat.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

/*
  One entry per thread that asks for one. Only threads that actually use the
  ChibiOS event calls get one, which today means AP_IOMCU's worker - the table
  is sized for a handful so a second user does not need a code change.
 */
#define AP_CH_COMPAT_MAX_THREADS 4

static struct ch_thread ch_threads[AP_CH_COMPAT_MAX_THREADS];
static uint8_t ch_thread_count;

struct ch_thread *chThdGetSelfX(void)
{
    const k_tid_t self = k_current_get();

    for (uint8_t i = 0; i < ch_thread_count; i++) {
        if (ch_threads[i].tid == self) {
            return &ch_threads[i];
        }
    }

    /* First call from this thread. The lock covers the allocation; the slot
       is initialised before it is published. */
    const unsigned int key = irq_lock();
    for (uint8_t i = 0; i < ch_thread_count; i++) {
        if (ch_threads[i].tid == self) {
            irq_unlock(key);
            return &ch_threads[i];
        }
    }
    if (ch_thread_count >= AP_CH_COMPAT_MAX_THREADS) {
        irq_unlock(key);
        return nullptr;
    }
    struct ch_thread *thread = &ch_threads[ch_thread_count];
    thread->tid = self;
    atomic_set(&thread->pending, 0);
    k_sem_init(&thread->sem, 0, 1);
    ch_thread_count++;
    irq_unlock(key);

    return thread;
}

void chEvtSignal(struct ch_thread *thread, eventmask_t mask)
{
    if (thread == nullptr) {
        return;
    }
    atomic_or(&thread->pending, (atomic_val_t)mask);
    /* The semaphore is only a wakeup; the mask above carries the news, so a
       count of 1 is enough however many events arrive before the wait. */
    k_sem_give(&thread->sem);
}

eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, k_timeout_t timeout)
{
    struct ch_thread *thread = chThdGetSelfX();
    if (thread == nullptr) {
        return 0;
    }
    /* Take whatever is already pending first: chEvtSignal may have run long
       before this call, and the wakeup must not be lost. */
    eventmask_t got = (eventmask_t)atomic_and(&thread->pending, ~(atomic_val_t)mask) & mask;
    if (got != 0) {
        return got;
    }
    if (k_sem_take(&thread->sem, timeout) != 0) {
        return 0;   /* timed out */
    }
    /* ChibiOS hands back the bits it consumed, so clear exactly those. */
    return (eventmask_t)atomic_and(&thread->pending, ~(atomic_val_t)mask) & mask;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
