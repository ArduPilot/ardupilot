/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
 */
#include "Thread.h"

#include <alloca.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Scheduler.h"

#define STACK_POISON 0xBEBACAFE

extern const AP_HAL::HAL &hal;

namespace Linux {


void *Thread::_run_trampoline(void *arg)
{
    Thread *thread = static_cast<Thread *>(arg);
    thread->_poison_stack();
    thread->_run();

    return nullptr;
}

bool Thread::_run()
{
    if (!_task) {
        return false;
    }

    _task();

    return true;
}

/* Round up to the specified alignment.
 *
 * Let u be the address p rounded up to the alignment a. Then:
 *    u = p + a - 1 - r, where r = (p + a - 1) % a
 *
 * If p % a = 0, i.e. if p is already aligned, then:
 *     r = a - 1 ==> u = p
 *
 * Otherwise:
 *     r = p % a -1 ==> u = p + a - p % a
 *
 *     p can be written p = q + p % a, where q is rounded down to the
 *     alignment a. Then u = q + a.
 */
static inline void *align_to(void *p, size_t align)
{
    return (void *)(((uintptr_t)p + align - 1) & ~(align - 1));
}

void Thread::_poison_stack()
{
    pthread_attr_t attr;
    size_t stack_size, guard_size;
    void *stackp;
    uint32_t *p, *curr, *begin, *end;

    if (pthread_getattr_np(_ctx, &attr) != 0 ||
        pthread_attr_getstack(&attr, &stackp, &stack_size) != 0 ||
        pthread_attr_getguardsize(&attr, &guard_size) != 0) {
        return;
    }

    stack_size /= sizeof(uint32_t);
    guard_size /= sizeof(uint32_t);

    /* The stack either grows upward or downard. The guard part always
     * protects the end */
    end = (uint32_t *)stackp;
    begin = end + stack_size;
    curr = (uint32_t *)align_to(alloca(sizeof(uint32_t)), alignof(uint32_t));

    /* if curr is closer to @end, the stack actually grows from low to high
     * virtual address: this is because this function should be executing very
     * early in the thread's life and close to the thread creation, assuming
     * the actual stack size is greater than the guard size and the stack
     * until now is resonably small */
    if (abs(curr - begin) > abs(curr - end)) {
        std::swap(end, begin);
        end -= guard_size;

        for (p = end; p > curr; p--) {
            *p = STACK_POISON;
        }
    } else {
        end += guard_size;

        /* we aligned curr to the up boundary, make sure this didn't cause us
         * to lose some bytes */
        curr--;

        for (p = end; p < curr; p++) {
            *p = STACK_POISON;
        }
    }

    _stack_debug.start = begin;
    _stack_debug.end = end;
}

size_t Thread::get_stack_usage()
{
    uint32_t *p;
    size_t result = 0;

    /* Make sure we are tracking usage for this thread */
    if (_stack_debug.start == 0 || _stack_debug.end == 0) {
        return 0;
    }

    if (_stack_debug.start < _stack_debug.end) {
        for (p = _stack_debug.end; p > _stack_debug.start; p--) {
            if (*p != STACK_POISON) {
                break;
            }
        }
        result = p - _stack_debug.start;
    } else {
        for (p = _stack_debug.end; p < _stack_debug.start; p++) {
            if (*p != STACK_POISON) {
                break;
            }
        }
        result = _stack_debug.start - p;
    }

    return result;
}

bool Thread::start(const char *name, int policy, int prio)
{
    if (_started) {
        return false;
    }

    struct sched_param param = { .sched_priority = prio };
    pthread_attr_t attr;
    int r;

    pthread_attr_init(&attr);

    /*
      we need to run as root to get realtime scheduling. Allow it to
      run as non-root for debugging purposes, plus to allow the Replay
      tool to run
     */
    if (geteuid() == 0) {
        if ((r = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) != 0 ||
            (r = pthread_attr_setschedpolicy(&attr, policy)) != 0 ||
            (r = pthread_attr_setschedparam(&attr, &param) != 0)) {
            AP_HAL::panic("Failed to set attributes for thread '%s': %s",
                          name, strerror(r));
        }
    }

    if (_stack_size) {
        if (pthread_attr_setstacksize(&attr, _stack_size) != 0) {
            return false;
        }
    }

    r = pthread_create(&_ctx, &attr, &Thread::_run_trampoline, this);
    if (r != 0) {
        AP_HAL::panic("Failed to create thread '%s': %s",
                      name, strerror(r));
    }
    pthread_attr_destroy(&attr);

    if (name) {
        pthread_setname_np(_ctx, name);
    }

    _started = true;

    return true;
}

bool Thread::is_current_thread()
{
    return pthread_equal(pthread_self(), _ctx);
}

bool PeriodicThread::set_rate(uint32_t rate_hz)
{
    if (_started || rate_hz == 0) {
        return false;
    }

    _period_usec = hz_to_usec(rate_hz);

    return true;
}

bool Thread::set_stack_size(size_t stack_size)
{
    if (_started) {
        return false;
    }

    _stack_size = stack_size;

    return true;
}

bool PeriodicThread::_run()
{
    uint64_t next_run_usec = AP_HAL::micros64() + _period_usec;

    while (true) {
        uint64_t dt = next_run_usec - AP_HAL::micros64();
        if (dt > _period_usec) {
            // we've lost sync - restart
            next_run_usec = AP_HAL::micros64();
        } else {
            Scheduler::from(hal.scheduler)->microsleep(dt);
        }
        next_run_usec += _period_usec;

        _task();
    }

    return true;
}

}
