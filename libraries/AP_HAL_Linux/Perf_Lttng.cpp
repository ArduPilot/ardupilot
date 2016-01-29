/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#pragma GCC diagnostic ignored "-Wcast-align"
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && defined(PERF_LTTNG)
#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE
#include "Perf_Lttng_TracePoints.h"

#include <string.h>
#include "Perf_Lttng.h"
#include "AP_HAL_Linux.h"
#include "Util.h"

using namespace Linux;

Perf_Lttng::Perf_Lttng(AP_HAL::Util::perf_counter_type type, const char *name)
    : _type(type)
{
    strncpy(_name, name, MAX_TRACEPOINT_NAME_LEN);
}

void Perf_Lttng::begin()
{
    if (_type != AP_HAL::Util::PC_ELAPSED) {
        return;
    }
    tracepoint(ardupilot, begin, _name);
}

void Perf_Lttng::end()
{
    if (_type != AP_HAL::Util::PC_ELAPSED) {
        return;
    }
    tracepoint(ardupilot, end, _name);
}

void Perf_Lttng::count()
{
    if (_type != AP_HAL::Util::PC_COUNT) {
        return;
    }
    tracepoint(ardupilot, count, _name, ++_count);
}

Util::perf_counter_t Util::perf_alloc(perf_counter_type type, const char *name)
{
    return new Linux::Perf_Lttng(type, name);
}

void Util::perf_begin(perf_counter_t perf)
{
    Linux::Perf_Lttng *perf_lttng = (Linux::Perf_Lttng *)perf;

    perf_lttng->begin();
}

void Util::perf_end(perf_counter_t perf)
{
    Linux::Perf_Lttng *perf_lttng = (Linux::Perf_Lttng *)perf;

    perf_lttng->end();
}

void Util::perf_count(perf_counter_t perf)
{
    Linux::Perf_Lttng *perf_lttng = (Linux::Perf_Lttng *)perf;

    perf_lttng->count();
}

#endif
