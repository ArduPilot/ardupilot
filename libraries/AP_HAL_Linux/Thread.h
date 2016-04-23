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
#pragma once

#include <pthread.h>
#include <inttypes.h>
#include <stdlib.h>

#include <AP_HAL/utility/functor.h>

#include "AP_HAL_Linux_Namespace.h"

namespace Linux {

/*
 * Interface abstracting threads
 */
class Thread {
public:
    FUNCTOR_TYPEDEF(task_t, void);

    Thread(task_t t) : _task(t) { }

    virtual ~Thread() { }

    bool start(const char *name, int policy, int prio);

    bool is_current_thread();

    size_t get_stack_usage();

    bool set_stack_size(size_t stack_size);

protected:
    static void *_run_trampoline(void *arg);

    /*
     * Run the task assigned in the constructor. May be overriden in case it's
     * preferred to use Thread as an interface or when user wants to aggregate
     * some initialization or teardown for the thread.
     */
    virtual bool _run();

    void _poison_stack();

    task_t _task;
    bool _started;
    pthread_t _ctx;

    struct stack_debug {
        uint32_t *start;
        uint32_t *end;
    } _stack_debug;

    size_t _stack_size;
};

class PeriodicThread : public Thread {
public:
    PeriodicThread(Thread::task_t t)
        : Thread(t)
    { }

    bool set_rate(uint32_t rate_hz);

protected:
    bool _run() override;

    uint64_t _period_usec;
};

}
