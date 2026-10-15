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
/// @file       DBGprint.cpp
/// @brief		convenience function for debug prints.
///

#include "DBGprint.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

DBGprint *DBGprint::_instance[MAX_INSTANCES];
uint32_t DBGprint::_num_instances = 0;
void *DBGprint::_call_loc[MAX_INSTANCES];

// static methods which can be placed in fast loops, but with actual output
// rate constrained by a minimum interval
void DBGprint::printf(const OType otype, const uint32_t interval_ms,
                      const uint32_t max_prints, const char *fname, const char *fmt, ...) {
    void *call_loc = __builtin_return_address(0);
    va_list ap;
    va_start(ap, fmt);
    output_txt(call_loc, otype, interval_ms, max_prints, fname, fmt, ap);
    va_end(ap);
}

void DBGprint::dflog(const uint32_t interval_ms, const char *recnam, const char *labels, const char *fmt, ...) {
    void *call_loc = __builtin_return_address(0);
    DBGprint *instance = get_instance(call_loc, 0);
    uint32_t now = AP_HAL::millis();
    if ((instance->_last_millis == 0) ||
        (now - instance->_last_millis) >= interval_ms) {
        AP_Logger *logger = AP_Logger::get_singleton();
        if (logger) {
            instance->_last_millis = now;
            va_list ap;
            va_start(ap, fmt);
            logger->WriteV(recnam, labels, nullptr, nullptr, fmt, ap);
            va_end(ap);
        }
    }
}

// find instance pointer for this call_location
DBGprint *DBGprint::get_instance(void* call_loc, const uint32_t max_prints) {
    uint32_t index = 0;
    while ((call_loc != _call_loc[index]) && (index < _num_instances)) {
        index++;
    }
    if (index >= MAX_INSTANCES) {
        // too many instances, fail
        ::printf("too many DBGprint instances %d: %lx\n", index, (long)call_loc);
        return nullptr;
    } else if (index < _num_instances) {
        // found instance
        return _instance[index];
    } else {
        // instance not found and slot available
        // attempt to create new instance for this call location
        DBGprint *result = new DBGprint(max_prints);
        if (result != nullptr) {
            // success; increment counter and save call_location
            _num_instances++;
            _call_loc[index] = call_loc;
            _instance[index] = result;
            ::printf("construct DBGprint instance %d: %lx\n", index, (long)call_loc);
        }
        // nullptr if new() failed
        return result;
    }
}

// generate output at intervals no less than interval_ms
void DBGprint::output_txt(void *call_loc, const OType otype,
                          const uint32_t interval_ms, const uint32_t max_prints,
                          const char *fname, const char *fmt, va_list ap) {
    DBGprint *instance = get_instance(call_loc, max_prints);
    // do nothing if get_instance failed, or if print limit exceeded
    if (!instance ||
        ((instance->_max_prints > 0) && (instance->_num_prints >= instance->_max_prints))) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    uint32_t delta = now - instance->_last_millis;
    // if this is the first call, or if interval_ms has elapsed since the last call
    if ((instance->_last_millis == 0) ||
        delta >= interval_ms) {
        // generate the requested output
        instance->_last_millis = now;
        ++instance->_num_prints;
        float avg_call_rate = 0;
        if (delta != 0) {
            avg_call_rate = 1000 * instance->_num_calls / delta;
        }
        switch (otype) {
        case OType::PRINTF:
            ::printf("%lu: %s: %1.0f Hz: ", (unsigned long)now, fname, avg_call_rate);
            vprintf(fmt,ap);
            ::printf("\n");
            break;
        case OType::GCS:
#ifndef HAL_NO_GCS
            // don't prefix the GCS message; not enough space
            gcs().send_textv(MAV_SEVERITY_INFO, fmt, ap);
            break;
#endif
        case OType::CONSOLE:
            AP_HAL::get_HAL().console->printf("%lu: %s: %1.0f Hz: ", (unsigned long)now, fname, avg_call_rate);
            AP_HAL::get_HAL().console->vprintf(fmt, ap);
            AP_HAL::get_HAL().console->printf("\n");
            break;
        }
        instance->_num_calls = 1;
    } else {
        // count number of skipped calls
        instance->_num_calls++;
    }
}
