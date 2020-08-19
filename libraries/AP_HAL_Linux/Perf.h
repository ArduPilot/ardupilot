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

#include <atomic>
#include <limits.h>
#include <pthread.h>
#include <vector>

#include "AP_HAL_Linux.h"
#include "Perf_Lttng.h"
#include "Thread.h"
#include "Util.h"

namespace Linux {

class Perf_Counter {
    using perf_counter_type = AP_HAL::Util::perf_counter_type;
    using perf_counter_t = AP_HAL::Util::perf_counter_t;

public:
    Perf_Counter(perf_counter_type type_, const char *name_)
        : name{name_}
        , type{type_}
        , min{ULONG_MAX}
    {
    }

    const char *name;
    Perf_Lttng lttng;

    perf_counter_type type;

    uint64_t count;

    /* Everything below is in nanoseconds */
    uint64_t start;
    uint64_t total;
    uint64_t min;
    uint64_t max;

    double avg;
    double m2;
};

class Perf {
    using perf_counter_type = AP_HAL::Util::perf_counter_type;
    using perf_counter_t = AP_HAL::Util::perf_counter_t;

public:
    ~Perf();

    static Perf *get_singleton();

    perf_counter_t add(perf_counter_type type, const char *name);

    void begin(perf_counter_t pc);
    void end(perf_counter_t pc);
    void count(perf_counter_t pc);

    unsigned int get_update_count() { return _update_count; }

private:
    static Perf *_singleton;

    Perf();

    void _debug_counters();

    uint64_t _last_debug_msec;

    std::vector<Perf_Counter> _perf_counters;

    /* synchronize addition of new perf counters */
    pthread_rwlock_t _perf_counters_lock;

    /* allow to check if memory pool has changed */
    std::atomic<unsigned int> _update_count;
};

}
