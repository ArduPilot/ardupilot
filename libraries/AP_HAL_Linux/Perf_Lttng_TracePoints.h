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

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ardupilot

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE <AP_HAL_Linux/Perf_Lttng_TracePoints.h>

#if !defined(_PERF_LTTNG_TRACEPOINT_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _PERF_LTTNG_TRACEPOINT_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    ardupilot,
    begin,
    TP_ARGS(
        const char*, name_arg
    ),
    TP_FIELDS(
        ctf_string(name_field, name_arg)
    )
)

TRACEPOINT_EVENT(
    ardupilot,
    end,
    TP_ARGS(
        const char*, name_arg
    ),
    TP_FIELDS(
        ctf_string(name_field, name_arg)
    )
)

TRACEPOINT_EVENT(
    ardupilot,
    count,
    TP_ARGS(
        const char*, name_arg,
        int, count_arg
    ),
    TP_FIELDS(
        ctf_string(name_field, name_arg)
        ctf_integer(int, count_field, count_arg)
    )
)

#endif

#include <lttng/tracepoint-event.h>
