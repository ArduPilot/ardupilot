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
/*
 * The small piece of the ChibiOS event API that AP_IOMCU uses, on Zephyr.
 *
 * AP_IOMCU drives its worker thread with per-thread event masks: the thread
 * blocks in chEvtWaitAnyTimeout(), and any other thread wakes it with
 * chEvtSignal(). Zephyr's k_event is the same idea, so this is a translation
 * rather than an emulation. Only these names are needed - AP_IOMCU.h already
 * forward-declares thread_t as struct ch_thread, so nothing else has to
 * change.
 *
 * Included only by the AP_HAL_ZEPHYR branch of AP_IOMCU.cpp, not force-included
 * anywhere: these are ChibiOS names and do not belong in every translation
 * unit.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <stdint.h>

typedef uint32_t eventmask_t;

#define EVENT_MASK(eid) ((eventmask_t)1U << (eid))

/* AP_IOMCU.h has: typedef struct ch_thread thread_t; */
struct ch_thread {
    /* Deliberately a semaphore plus an atomic mask rather than k_event.
       k_event lives in kernel/events.c, and the linker reaches libkernel.a
       before ArduPilot's archive, so nothing pulls that object in and the
       link fails on z_impl_k_event_post. Semaphores and atomics are already
       linked by the HAL. */
    struct k_sem sem;
    atomic_t pending;
    k_tid_t tid;
};

/* The calling thread's event object, created on first use. */
struct ch_thread *chThdGetSelfX(void);

/* Wake a thread. Safe from any thread; k_event_post is not blocking. */
void chEvtSignal(struct ch_thread *thread, eventmask_t mask);

/* Wait for any bit in mask on the calling thread, clearing what it returns. */
eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, k_timeout_t timeout);

#define chTimeMS2I(ms) K_MSEC(ms)

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
