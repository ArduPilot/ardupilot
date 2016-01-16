/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && !defined(PERF_LTTNG)

#include <limits.h>
#include <time.h>

#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "Util.h"

using namespace Linux;

struct perf_counter_base_t {
    const char *name;
    enum Util::perf_counter_type type;
};

struct perf_counter_count_t {
    struct perf_counter_base_t base;
    uint64_t count;
};

struct perf_counter_elapsed_t {
    struct perf_counter_base_t base;
    uint64_t count;
    /* Everything below is in nanoseconds */
    uint64_t start;
    uint64_t total;
    uint64_t least;
    uint64_t most;
    double mean;
    double m2;
};

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Util::perf_counter_t Util::perf_alloc(perf_counter_type type, const char *name)
{
    struct perf_counter_base_t *base;

    switch(type) {
    case PC_COUNT: {
        struct perf_counter_count_t *count;
        count = (struct perf_counter_count_t *)calloc(1, sizeof(struct perf_counter_count_t));
        if (!count) {
            return nullptr;
        }

        base = &count->base;
        break;
    }
    case PC_ELAPSED: {
        struct perf_counter_elapsed_t *elapsed;
        elapsed = (struct perf_counter_elapsed_t *)calloc(1, sizeof(struct perf_counter_elapsed_t));
        if (!elapsed) {
            return nullptr;
        }

        elapsed->least = ULONG_MAX;

        base = &elapsed->base;
        break;
    }
    default:
    case PC_INTERVAL: {
        /*
         * Not implemented now because it is not used even on PX4 specific
         * code and by looking at PX4 implementation without perf_reset()
         * the average is broken.
         */
        return nullptr;
    }
    }

    base->name = name;
    base->type = type;
    return (perf_counter_t)base;
}

static inline uint64_t timespec_to_nsec(const struct timespec *ts)
{
    return ts->tv_nsec + (ts->tv_sec * NSEC_PER_SEC);
}

void Util::perf_begin(perf_counter_t perf)
{
    struct perf_counter_elapsed_t *perf_elapsed = (struct perf_counter_elapsed_t *)perf;

    if (perf_elapsed == NULL) {
        return;
    }
    if (perf_elapsed->base.type != PC_ELAPSED) {
        hal.console->printf("perf_begin() called over a perf_counter_t(%s) that"
                            " is not of the PC_ELAPSED type.\n",
                            perf_elapsed->base.name);
        return;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    perf_elapsed->start = timespec_to_nsec(&ts);
}

void Util::perf_end(perf_counter_t perf)
{
    struct perf_counter_elapsed_t *perf_elapsed = (struct perf_counter_elapsed_t *)perf;

    if (perf_elapsed == NULL) {
        return;
    }

    if (perf_elapsed->base.type != PC_ELAPSED) {
        hal.console->printf("perf_end() called over a perf_counter_t(%s) "
                            "that is not of the PC_ELAPSED type.\n",
                            perf_elapsed->base.name);
        return;
    }
    if (perf_elapsed->start == 0) {
        hal.console->printf("perf_end() called before an perf_begin() on %s.\n",
                            perf_elapsed->base.name);
        return;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const uint64_t elapsed = timespec_to_nsec(&ts) - perf_elapsed->start;

    perf_elapsed->count++;
    perf_elapsed->total += elapsed;

    if (perf_elapsed->least > elapsed) {
        perf_elapsed->least =  elapsed;
    }

    if (perf_elapsed->most < elapsed) {
        perf_elapsed->most = elapsed;
    }

    /*
     * Maintain mean and variance of interval in nanoseconds
     * Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */
    const double delta_intvl = elapsed - perf_elapsed->mean;
    perf_elapsed->mean += (delta_intvl / perf_elapsed->count);
    perf_elapsed->m2 += (delta_intvl * (elapsed - perf_elapsed->mean));

    perf_elapsed->start = 0;
}

void Util::perf_count(perf_counter_t perf)
{
    struct perf_counter_count_t *perf_counter = (struct perf_counter_count_t *)perf;

    if (perf_counter == NULL) {
        return;
    }

    if (perf_counter->base.type != PC_COUNT) {
        hal.console->printf("perf_count() called over a perf_counter_t(%s) "
                            "that is not of the PC_COUNT type.\n",
                            perf_counter->base.name);
        return;
    }

    perf_counter->count++;
}

#endif
