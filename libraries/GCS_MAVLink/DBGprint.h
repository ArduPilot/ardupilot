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
///
/// @file       DBGprint.h
/// @brief		convenience functions for debug prints.
///

#pragma once
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <stdio.h>
#include <stdarg.h>
#include <AP_Common/AP_Common.h>

#define DBG_CONSOLE(interval, max_prints, format, ...) \
DBGprint::printf(DBGTYPE::CONSOLE, interval, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_PRINTF(interval, max_prints, format, ...) \
DBGprint::printf(DBGTYPE::PRINTF, interval, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_GCS(interval, max_prints, format, ...) \
DBGprint::printf(DBGTYPE::GCS, interval, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_WCONSOLE(value, max_prints, format, ...) \
DBGprint::watch(DBGTYPE::CONSOLE, value, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_WPRINTF(value, max_prints, format, ...) \
DBGprint::watch(DBGTYPE::PRINTF, value, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_WGCS(value, max_prints, format, ...) \
DBGprint::watch(DBGTYPE::GCS, value, max_prints, __func__, format, ## __VA_ARGS__)

#define DBG_LOG(interval, recnam, labels, format, ...) \
DBGprint::dflog(interval, recnam, labels, format, __VA_ARGS__)

class DBGprint {
public:
    enum class OType {
        PRINTF,
        CONSOLE,
        GCS
    };

    // static methods which can be placed in fast loops, but with actual output
    // rate constrained by a minimum interval
    static void printf(const OType otype, const uint32_t interval_ms,
                       const uint32_t max_prints, const char *fname,
                       const char *fmt, ...) FMT_PRINTF(5, 6);

    // print only when value changes
    template <typename T>
    static void watch(const OType otype, T value, const uint32_t max_prints,
                      const char *fname, const char *fmt, ...) {
        static T last_value = (T)0;
        void *call_loc = __builtin_return_address(0);
        if (value != last_value) {
            last_value = value;
            va_list ap;
            va_start(ap, fmt);
            output_txt(call_loc, otype, 0, max_prints, fname, fmt, ap);
            va_end(ap);
        }
    }

    static void dflog(const uint32_t interval_ms, const char *recnam, const char *labels, const char *fmt, ...);

private:
    // constructor
    DBGprint(const uint32_t max_prints) : _max_prints(max_prints) {}

    // find instance pointer for this call_loc
    static DBGprint *get_instance(void* call_loc, const uint32_t max_prints);

    // generate output at intervals no less than interval_ms
    static void output_txt(void *call_loc, const OType otype, const uint32_t interval_ms,
                           const uint32_t max_prints, const char *fname,
                           const char *fmt, va_list ap);

    static const uint32_t MAX_INSTANCES = 10;
    static uint32_t _num_instances;
    static DBGprint *_instance[MAX_INSTANCES];
    static void *_call_loc[MAX_INSTANCES];

    uint32_t _last_millis;
    uint32_t _num_calls;
    uint32_t _num_prints;
    uint32_t _max_prints;
};

typedef DBGprint::OType DBGTYPE;
