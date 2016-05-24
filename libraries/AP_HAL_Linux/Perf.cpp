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
#include <time.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "Util.h"
#include "Perf.h"
#include "Perf_Lttng.h"

using namespace Linux;

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Perf *Perf::_instance;

static inline uint64_t now_nsec()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_nsec + (ts.tv_sec * NSEC_PER_SEC);
}

Perf *Perf::get_instance()
{
    if (!_instance) {
        _instance = new Perf();
    }

    return _instance;
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

    if (perf.least > elapsed) {
        perf.least = elapsed;
    }

    if (perf.most < elapsed) {
        perf.most = elapsed;
    }

    /*
     * Maintain mean and variance of interval in nanoseconds
     * Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */
    const double delta_intvl = elapsed - perf.mean;
    perf.mean += (delta_intvl / perf.count);
    perf.m2 += (delta_intvl * (elapsed - perf.mean));
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
