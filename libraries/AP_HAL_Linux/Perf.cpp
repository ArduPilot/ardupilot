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
#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "Util.h"
#include "Perf.h"
#include "Perf_Lttng.h"

#ifndef PRIu64
#define PRIu64 "llu"
#endif

using namespace Linux;

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Perf *Perf::_singleton;

static inline uint64_t now_nsec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_nsec + (ts.tv_sec * AP_NSEC_PER_SEC);
}

Perf *Perf::get_singleton()
{
    if (!_singleton) {
        _singleton = new Perf();
    }

    return _singleton;
}

void Perf::_debug_counters()
{
    uint64_t now = AP_HAL::millis64();

    if (now - _last_debug_msec < 5000) {
        return;
    }

    pthread_rwlock_rdlock(&_perf_counters_lock);
    unsigned int uc = _update_count;
    auto v = _perf_counters;
    pthread_rwlock_unlock(&_perf_counters_lock);

    if (uc != _update_count) {
        fprintf(stderr, "WARNING!! potentially wrong counters!!!");
    }

    for (auto &c : v) {
        if (!c.count) {
            fprintf(stderr, "%-30s\t"
                    "(no events)\n", c.name);
        } else if (c.type == Util::PC_ELAPSED) {
            fprintf(stderr, "%-30s\t"
                    "count: %" PRIu64 "\t"
                    "min: %" PRIu64 "\t"
                    "max: %" PRIu64 "\t"
                    "avg: %.4f\t"
                    "stddev: %.4f\n",
                    c.name, c.count, c.min, c.max, c.avg, sqrt(c.m2));
        } else {
            fprintf(stderr, "%-30s\t"
                    "count: %" PRIu64 "\n",
                    c.name, c.count);
        }
    }

    _last_debug_msec = now;
}

Perf::Perf()
{
    if (pthread_rwlock_init(&_perf_counters_lock, nullptr) != 0) {
        AP_HAL::panic("Perf: fail to initialize rw lock");
    }

    /* TODO: this number should come from vehicle code - just estimate the
     * number of perf counters for now; if we grow more, it will just
     * reallocate the memory pool */
    _perf_counters.reserve(50);

#ifdef DEBUG_PERF
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&Perf::_debug_counters, void));
#endif
}

void Perf::begin(Util::perf_counter_t pc)
{
    uintptr_t idx = (uintptr_t)pc;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters[idx];
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start != 0) {
        hal.console->printf("perf_begin() called twice on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    perf.start = now_nsec();

    perf.lttng.begin(perf.name);
}

void Perf::end(Util::perf_counter_t pc)
{
    uintptr_t idx = (uintptr_t)pc;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters[idx];
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start == 0) {
        hal.console->printf("perf_begin() called before begin() on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    const uint64_t elapsed = now_nsec() - perf.start;
    perf.count++;
    perf.total += elapsed;

    if (perf.min > elapsed) {
        perf.min = elapsed;
    }

    if (perf.max < elapsed) {
        perf.max = elapsed;
    }

    /*
     * Maintain avg and variance of interval in nanoseconds
     * Knuth/Welford recursive avg and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */
    const double delta_intvl = elapsed - perf.avg;
    perf.avg += (delta_intvl / perf.count);
    perf.m2 += (delta_intvl * (elapsed - perf.avg));
    perf.start = 0;

    perf.lttng.end(perf.name);
}

void Perf::count(Util::perf_counter_t pc)
{
    uintptr_t idx = (uintptr_t)pc;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters[idx];
    if (perf.type != Util::PC_COUNT) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_COUNT type.\n",
                            perf.name);
        return;
    }

    _update_count++;
    perf.count++;

    perf.lttng.count(perf.name, perf.count);
}

Util::perf_counter_t Perf::add(Util::perf_counter_type type, const char *name)
{
    if (type != Util::PC_COUNT && type != Util::PC_ELAPSED) {
        /*
         * Other perf counters not implemented for now since they are not
         * used anywhere.
         */
        return (Util::perf_counter_t)(uintptr_t) -1;
    }

    pthread_rwlock_wrlock(&_perf_counters_lock);
    Util::perf_counter_t pc = (Util::perf_counter_t) _perf_counters.size();
    _perf_counters.emplace_back(type, name);
    pthread_rwlock_unlock(&_perf_counters_lock);

    return pc;
}
