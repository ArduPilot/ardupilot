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
#include <limits.h>
#include <time.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "Util.h"
#ifdef HAVE_LIBSYSTEMD
#include "Perf.h"
#endif
#include "Perf_Lttng.h"

using namespace Linux;

struct perf_counter {
    perf_counter(const char *name_, enum Util::perf_counter_type type_)
        : name{name_}
        , type{type_}
#ifdef HAVE_LTTNG_UST
        , lttng{name_}
#endif
    {
    }

    const char *name;
    enum Util::perf_counter_type type;
#ifdef HAVE_LTTNG_UST
    Perf_Lttng lttng;
#endif
};

struct perf_counter_count : public perf_counter {
    perf_counter_count(const char *name_, Util::perf_counter_type type_)
        : perf_counter(name_, type_)
    {
    }
    uint64_t count;
};

struct perf_counter_elapsed : public perf_counter {
    perf_counter_elapsed(const char *name_, enum Util::perf_counter_type type_)
        : perf_counter(name_, type_)
        , least{ULONG_MAX}
    {
    }

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
    perf_counter *perf = nullptr;

    switch(type) {
    case PC_COUNT: {
        perf = new perf_counter_count(name, type);
        break;
    }
    case PC_ELAPSED: {
        perf = new perf_counter_elapsed(name, type);
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

#ifdef HAVE_LIBSYSTEMD
    PerfManager::getInstance()->add((Util::perf_counter_t)perf);
#endif

    return (Util::perf_counter_t)perf;
}

static inline uint64_t timespec_to_nsec(const struct timespec *ts)
{
    return ts->tv_nsec + (ts->tv_sec * NSEC_PER_SEC);
}

void Util::perf_begin(perf_counter_t perf)
{
    struct perf_counter_elapsed *perf_elapsed = (struct perf_counter_elapsed *)perf;

    if (perf_elapsed == nullptr) {
        return;
    }

    if (perf_elapsed->type != PC_ELAPSED) {
        hal.console->printf("perf_begin() called over a perf_counter_t(%s) that"
                            " is not of the PC_ELAPSED type.\n",
                            perf_elapsed->name);
        return;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    perf_elapsed->start = timespec_to_nsec(&ts);

#ifdef HAVE_LTTNG_UST
    perf_elapsed->lttng.begin();
#endif
}

void Util::perf_end(perf_counter_t perf)
{
    struct perf_counter_elapsed *perf_elapsed = (struct perf_counter_elapsed *)perf;

    if (perf_elapsed == nullptr) {
        return;
    }

    if (perf_elapsed->type != PC_ELAPSED) {
        hal.console->printf("perf_end() called over a perf_counter_t(%s) "
                            "that is not of the PC_ELAPSED type.\n",
                            perf_elapsed->name);
        return;
    }
    if (perf_elapsed->start == 0) {
        hal.console->printf("perf_end() called before perf_begin() on %s.\n",
                            perf_elapsed->name);
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

#ifdef HAVE_LTTNG_UST
    perf_elapsed->lttng.end();
#endif
}

void Util::perf_count(perf_counter_t perf)
{
    struct perf_counter_count *perf_counter = (struct perf_counter_count *)perf;
    if (perf_counter == nullptr) {
        return;
    }

    if (perf_counter->type != PC_COUNT) {
        hal.console->printf("perf_count() called over a perf_counter_t(%s) "
                            "that is not of the PC_COUNT type.\n",
                            perf_counter->name);
        return;
    }

    perf_counter->count++;

#ifdef HAVE_LTTNG_UST
    perf_counter->lttng.count();
#endif
}

#ifdef HAVE_LIBSYSTEMD

#define DBUS_NAMESPACE "org.dronecode.Ardupilot"
#define DBUS_PATH "/org/dronecode/ardupilot"

PerfManager* PerfManager::_instance = nullptr;

extern const sd_bus_vtable _perf_vtable[];

extern "C" int dbus_list_perf_trampoline(sd_bus_message *m, void *data, sd_bus_error *ret_error)
{
    PerfManager *perf_manager = (PerfManager *)data;
    return perf_manager->dbus_list_perf_callback(m, data, ret_error);
}

extern "C" int dbus_get_perf_trampoline(sd_bus_message *m, void *data, sd_bus_error *ret_error)
{
    PerfManager *perf_manager = (PerfManager *)data;
    return perf_manager->dbus_get_perf_callback(m, data, ret_error);
}

PerfManager::PerfManager()
{
    int r = sd_bus_open_system(&_bus);
    if (r < 0) {
        AP_HAL::panic("PerfManager: Unable to open system bus.");
    }

    r = sd_bus_request_name(_bus, DBUS_NAMESPACE, 0);
    if (r < 0) {
        sd_bus_unref(_bus);
        AP_HAL::panic("PerfManager: Unable to request bus name: "
                      DBUS_NAMESPACE ", probably you need add the "
                      "configuration file to /etc/dbus-1/system.d");
    }

    r = sd_bus_add_object_vtable(_bus, &_slot,
                                 DBUS_PATH,
                                 DBUS_NAMESPACE ".Perf",
                                 _perf_vtable,
                                 this);
    if (r < 0) {
        sd_bus_unref(_bus);
        AP_HAL::panic("PerfManager: Unable to register Perf interface.");
    }

    if (!_dbus_thread.start("perf manager", SCHED_OTHER, 0)) {
        sd_bus_slot_unref(_slot);
        sd_bus_unref(_bus);
        AP_HAL::panic("PerfManager: Unable to create D-Bus thread.");
    }
}

PerfManager::~PerfManager()
{
    _run_thread = false;
    sd_bus_slot_unref(_slot);
    sd_bus_unref(_bus);
    _perf_hashmap.clear();
}

void PerfManager::dbus_process_callback()
{
    while (_run_thread) {
        int r = sd_bus_process(_bus, nullptr);
        if (r < 0) {
            hal.console->printf("PerfManager: Failed to process bus.\n");
        }

        /* Wait for the next request to process */
        r = sd_bus_wait(_bus, (uint64_t) -1);
        if (r < 0) {
            hal.console->printf("PerfManager: Failed to wait bus.\n");
        }
    }
}

void PerfManager::add(Util::perf_counter_t perf)
{
    perf_counter *base = (perf_counter *)perf;
    _event_add_sem.take(0);
    /* not checking if hash already have an perf with the same name */
    _perf_hashmap[base->name] = perf;
    _event_add_sem.give();
}

PerfManager* PerfManager::getInstance()
{
    if (_instance) {
        return _instance;
    }

    _instance = new PerfManager();
    return _instance;
}

static const char* get_perf_type(perf_counter *perf)
{
    switch (perf->type) {
    case Util::PC_COUNT:
        return "count";
    case Util::PC_ELAPSED:
        return "elapsed";
    case Util::PC_INTERVAL:
        return "interval";
    default:
        return "unknown";
    }
}

int PerfManager::dbus_list_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error)
{
    sd_bus_message *reply;

    int r = sd_bus_message_new_method_return(m, &reply);
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    r = sd_bus_message_open_container(reply, 'a', "(ss)");
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    std::unordered_map<std::string, Util::perf_counter_t> hashmap;
    _event_add_sem.take(0);
    hashmap = _perf_hashmap;
    _event_add_sem.give();

    for (auto it = hashmap.begin(); it != hashmap.end(); it++) {
        perf_counter *perf = (perf_counter *)it->second;
        r = sd_bus_message_append(reply, "(ss)", perf->name,
                                  get_perf_type(perf));
        if (r < 0) {
            return sd_bus_error_set_errno(ret_error, r);
        }
    }

    r = sd_bus_message_close_container(reply);
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    return sd_bus_send(nullptr, reply, nullptr);
}

static int append_perf_specific_data(perf_counter *perf, sd_bus_message *reply)
{
    if (perf->type == Util::PC_INTERVAL) {
        hal.console->printf("Missing implementation of append_perf_data() "
                            "for PC_INTERVAL type.\n");
        return 0;
    }

    if (perf->type == Util::PC_COUNT) {
        perf_counter_count *perf_count = (perf_counter_count *)perf;
        return sd_bus_message_append(reply, "{st}", "count", perf_count->count);
    }

    perf_counter_elapsed *perf_elapsed = (perf_counter_elapsed *)perf;

    int ret = sd_bus_message_append(reply, "{st}", "count",
                                    perf_elapsed->count);
    if (ret < 0) {
        return ret;
    }

    ret = sd_bus_message_append(reply, "{st}", "total", perf_elapsed->total);
    if (ret < 0) {
        return ret;
    }

    uint64_t temp = perf_elapsed->count;
    if (temp == 0) {
        temp = 1;
    }
    temp = perf_elapsed->total / temp;
    ret = sd_bus_message_append(reply, "{st}", "average", temp);
    if (ret < 0) {
        return ret;
    }

    ret = sd_bus_message_append(reply, "{st}", "minimum", perf_elapsed->least);
    if (ret < 0) {
        return ret;
    }

    ret = sd_bus_message_append(reply, "{st}", "maximum", perf_elapsed->most);
    if (ret < 0) {
        return ret;
    }

    double variance;
    if (perf_elapsed->count > 1) {
        variance = perf_elapsed->m2 / (perf_elapsed->count - 1);
    } else {
        variance = 0.0;
    }
    temp = round(variance);
    ret = sd_bus_message_append(reply, "{st}", "variance", temp);
    if (ret < 0) {
        return ret;
    }

    temp = round(sqrt(variance));
    return sd_bus_message_append(reply, "{st}", "std-deviation", temp);
}

static int append_perf(perf_counter *perf, sd_bus_message *reply)
{
    int r = sd_bus_message_open_container(reply, SD_BUS_TYPE_STRUCT, "ssa{st}");
    if (r < 0) {
        return r;
    }

    r = sd_bus_message_append(reply, "ss", perf->name, get_perf_type(perf));
    if (r < 0) {
        return r;
    }

    r = sd_bus_message_open_container(reply, 'a', "{st}");
    if (r < 0) {
        return r;
    }

    r = append_perf_specific_data(perf, reply);
    if (r < 0) {
        return r;
    }

    /* closing a{st} */
    r = sd_bus_message_close_container(reply);
    if (r < 0) {
        return r;
    }

    /* closing (ssa{st}) */
    return sd_bus_message_close_container(reply);
}

int PerfManager::dbus_get_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error)
{
    int r = sd_bus_message_enter_container(m, SD_BUS_TYPE_ARRAY, "s");
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    sd_bus_message *reply;
    r = sd_bus_message_new_method_return(m, &reply);
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    r = sd_bus_message_open_container(reply, SD_BUS_TYPE_ARRAY, "(ssa{st})");
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    std::unordered_map<std::string, Util::perf_counter_t> hashmap;
    _event_add_sem.take(0);
    hashmap = _perf_hashmap;
    _event_add_sem.give();

    while (true) {
        const char *name;

        /* return 0 when there is no item left on array */
        r = sd_bus_message_read_basic(m, SD_BUS_TYPE_STRING, &name);
        if (r < 1) {
            break;
        }

        perf_counter *perf = (perf_counter *)hashmap[name];
        if (!perf) {
            continue;
        }

        r = append_perf(perf, reply);
        if (r < 0) {
            break;
        }
    }

    r = sd_bus_message_close_container(reply);
    if (r < 0) {
        return sd_bus_error_set_errno(ret_error, r);
    }

    return sd_bus_send(nullptr, reply, nullptr);
}

#endif
