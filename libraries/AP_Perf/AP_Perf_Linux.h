/*
 * Copyright (C) 2016-2017  Intel Corporation. All rights reserved.
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

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <pthread.h>
#include <atomic>
#include <vector>

#include "AP_Perf.h"
#include "AP_Perf_Backend.h"
#include "Perf_Lttng.h"

class AP_Perf_Linux : public AP_Perf_Backend {
public:
    class AP_Perf_Counter {
    public:
        AP_Perf_Counter(perf_counter_type type_, const char *name_)
            : name{name_}
            , type{type_}
            , min{ULONG_MAX}
        {
        }

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

    AP_Perf_Linux();

    perf_counter_t add(perf_counter_type type, const char *name) override;
    void begin(perf_counter_t pc) override;
    void end(perf_counter_t pc) override;
    void count(perf_counter_t pc) override;

protected:
    Perf_Lttng _lttng;

    void _debug_counters();

    uint64_t _last_debug_msec;

    /*
     * Synchronize addition of new perf counters:
     *   - additions always happen on the main thread
     *   - use of the perf counter may occur on any thread
     *   - reading the values of each perf counter happens
     *     on timer thread
     *
     *   This synchronizes additions of new perf counter with the read
     *   on the timer thread
     */
    pthread_rwlock_t _perf_counters_lock;

    /*
     * All perf counters in a single memory space
     */
    std::vector<AP_Perf_Counter> _perf_counters;

    /*
     * Allow to check if memory pool has changed: this is updated whenever
     * any perf counter changes, allowing for the read on the timer thread
     * to be lockless in a read and retry fashion:
     *
     *     do {
     *         unsigned int update_count = _update_count
     *         std::vector<> v = _perf_counters;
     *         if (update_counter == _update_counter) {
     *             // copy captured
     *             break;
     *         }
     *     } while (retries--);
     */
    std::atomic<unsigned int> _update_count;
};

#endif
