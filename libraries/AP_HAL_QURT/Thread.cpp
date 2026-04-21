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

#include <limits.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"

extern const AP_HAL::HAL &hal;

namespace QURT
{


void *Thread::_run_trampoline(void *arg)
{
    Thread *thread = static_cast<Thread *>(arg);
    thread->_run();

    if (thread->_auto_free) {
        delete thread;
    }

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

size_t Thread::get_stack_usage()
{
    return 0;
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

    if ((r = pthread_attr_setschedparam(&attr, &param)) != 0) {
        AP_HAL::panic("Failed to set attributes for thread '%s': %s",
                      name, strerror(r));
    }

    const uint32_t stack_alignment = 2048U;
    _stack_size = (_stack_size+stack_alignment) & ~stack_alignment;
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

    _started = true;

    return true;
}

bool Thread::is_current_thread()
{
    return pthread_equal(pthread_self(), _ctx);
}

bool Thread::join()
{
    void *ret;

    if (_ctx == 0) {
        return false;
    }

    if (pthread_join(_ctx, &ret) != 0 ||
        (intptr_t)ret != 0) {
        return false;
    }

    return true;
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

    _stack_size = MAX(stack_size, (size_t) PTHREAD_STACK_MIN);

    return true;
}

bool PeriodicThread::_run()
{
    if (_period_usec == 0) {
        return false;
    }

    uint64_t next_run_usec = AP_HAL::micros64() + _period_usec;

    while (!_should_exit) {
        uint64_t dt = next_run_usec - AP_HAL::micros64();
        if (dt > _period_usec) {
            // we've lost sync - restart
            next_run_usec = AP_HAL::micros64();
        } else {
            qurt_timer_sleep(dt);
        }
        next_run_usec += _period_usec;

        _task();
    }

    _started = false;
    _should_exit = false;

    return true;
}

bool PeriodicThread::stop()
{
    if (!is_started()) {
        return false;
    }

    _should_exit = true;

    return true;
}

}
