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

#include <AP_HAL/AP_HAL.h>
#include <limits.h>

#ifdef HAVE_LTTNG_UST
#include "Perf_Lttng.h"
#endif

class AP_Perf_Backend;


class AP_Perf {
public:
    enum perf_counter_type {
        PC_COUNT,        /**< count the number of times an event occurs */
        PC_ELAPSED,      /**< measure the time elapsed performing an event */
        PC_INTERVAL      /**< measure the interval between instances of an event */
    };

    typedef void *perf_counter_t;

    class AP_Perf_Counter {
    public:

        AP_Perf_Counter(perf_counter_type type_, const char *name_)
            : name{name_}
            , type{type_}
            , min{ULONG_MAX}
        {
        }

#ifdef HAVE_LTTNG_UST
        Perf_Lttng lttng;
#endif

        const char *name;

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

    AP_Perf();

    // New API
    virtual perf_counter_t add(perf_counter_type type, const char *name);
    virtual void begin(perf_counter_t pc);
    virtual void end(perf_counter_t pc);
    virtual void count(perf_counter_t pc);

    // PX4 API
    virtual void perf_begin(perf_counter_t pc) { begin(pc); }
    virtual void perf_end(perf_counter_t pc) { end(pc); }
    virtual void perf_count(perf_counter_t pc) { count(pc); }
    virtual perf_counter_t perf_alloc(perf_counter_type t, const char *name) { return add(t, name); }

    static AP_Perf *get_instance() { return &_instance; }

private:
    static AP_Perf _instance;
    AP_Perf_Backend *_backend;
};