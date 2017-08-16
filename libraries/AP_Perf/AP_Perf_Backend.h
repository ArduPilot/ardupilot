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
 */
#pragma once

#include <inttypes.h>
#include <limits.h>

#include "AP_Perf.h"

class AP_Perf_Backend {
public:
    using perf_counter_type = AP_Perf::perf_counter_type;
    using perf_counter_t = AP_Perf::perf_counter_t;

    virtual perf_counter_t add(perf_counter_type type, const char *name) = 0;
    virtual void begin(perf_counter_t pc) = 0;
    virtual void end(perf_counter_t pc) = 0;
    virtual void count(perf_counter_t pc) = 0;
};
