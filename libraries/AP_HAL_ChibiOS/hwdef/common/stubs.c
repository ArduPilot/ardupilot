/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
* **** This file incorporates work covered by the following copyright and ****
* **** permission notice:                                                 ****
*
*  Copyright (c) 2009 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
*  28.03.09  mifi       First Version, based on the original syscall.c from
*                       newlib version 1.17.0
*  17.08.09  gdisirio   Modified the file for use under ChibiOS/RT
*  15.11.09  gdisirio   Added read and write handling
****************************************************************************/

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
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "ch.h"

#include <stdlib.h>
#include <sys/lock.h>

#if CH_CFG_USE_MUTEXES == TRUE

struct __lock {
    mutex_t mutex;
    bool dynamically_allocated;
};

struct __lock __lock___libc_recursive_mutex = {
    __MUTEX_DATA(__lock___libc_recursive_mutex.mutex), false
};

#else

// Bootloaders disable ChibiOS mutexes and execute libc calls on one thread.
struct __lock {
    uint8_t unused;
};

struct __lock __lock___libc_recursive_mutex;
static struct __lock single_thread_lock;

#endif

static int preinit_errno;

int *ap_picolibc_errno(void)
{
    if (ch_system.state != ch_sys_running) {
        return &preinit_errno;
    }
    return &chThdGetSelfX()->picolibc_errno;
}

static void lock_init(_LOCK_T *lock)
{
#if CH_CFG_USE_MUTEXES == TRUE
    struct __lock *new_lock = malloc(sizeof(*new_lock));
    if (new_lock == NULL) {
        *lock = NULL;
        return;
    }
    chMtxObjectInit(&new_lock->mutex);
    new_lock->dynamically_allocated = true;
    *lock = new_lock;
#else
    *lock = &single_thread_lock;
#endif
}

void __retarget_lock_init(_LOCK_T *lock)
{
    lock_init(lock);
}

void __retarget_lock_init_recursive(_LOCK_T *lock)
{
    lock_init(lock);
}

void __retarget_lock_close(_LOCK_T lock)
{
#if CH_CFG_USE_MUTEXES == TRUE
    if (lock != NULL && lock->dynamically_allocated) {
        free(lock);
    }
#else
    (void)lock;
#endif
}

void __retarget_lock_close_recursive(_LOCK_T lock)
{
    __retarget_lock_close(lock);
}

void __retarget_lock_acquire(_LOCK_T lock)
{
#if CH_CFG_USE_MUTEXES == TRUE
    // Startup is single-threaded until chSysInit() has completed.
    if (lock != NULL && ch_system.state == ch_sys_running) {
        chMtxLock(&lock->mutex);
    }
#else
    (void)lock;
#endif
}

void __retarget_lock_acquire_recursive(_LOCK_T lock)
{
    __retarget_lock_acquire(lock);
}

void __retarget_lock_release(_LOCK_T lock)
{
#if CH_CFG_USE_MUTEXES == TRUE
    if (lock != NULL && ch_system.state == ch_sys_running) {
        chMtxUnlock(&lock->mutex);
    }
#else
    (void)lock;
#endif
}

void __retarget_lock_release_recursive(_LOCK_T lock)
{
    __retarget_lock_release(lock);
}

int __aeabi_atexit(void *arg, void (*func)(void *), void *dso_handle)
{
    // Firmware never exits, so static destructors do not need registration.
    (void)arg;
    (void)func;
    (void)dso_handle;
    return 0;
}

/*** EOF ***/
