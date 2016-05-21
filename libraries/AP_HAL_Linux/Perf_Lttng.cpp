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
#ifdef HAVE_LTTNG_UST

#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE

#include <string.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux.h"
#include "Perf_Lttng_TracePoints.h"
#include "Perf_Lttng.h"

using namespace Linux;

Perf_Lttng::Perf_Lttng(const char *name)
{
    strncpy(_name, name, MAX_TRACEPOINT_NAME_LEN);
}

void Perf_Lttng::begin()
{
    tracepoint(ardupilot, begin, _name);
}

void Perf_Lttng::end()
{
    tracepoint(ardupilot, end, _name);
}

void Perf_Lttng::count()
{
    tracepoint(ardupilot, count, _name, ++_count);
}

#endif
