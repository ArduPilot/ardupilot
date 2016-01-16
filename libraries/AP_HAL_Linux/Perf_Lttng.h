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

#pragma once
#include "AP_HAL_Linux.h"
#include <AP_HAL/Util.h>

#define MAX_TRACEPOINT_NAME_LEN 128

class Linux::Perf_Lttng {
public:
    Perf_Lttng(enum AP_HAL::Util::perf_counter_type type, const char *name);
    void begin();
    void end();
    void count();
private:
    char _name[MAX_TRACEPOINT_NAME_LEN];
    uint64_t _count;
    enum AP_HAL::Util::perf_counter_type _type;
};
