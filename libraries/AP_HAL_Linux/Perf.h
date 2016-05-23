#pragma once

#ifdef HAVE_LIBSYSTEMD

#include <systemd/sd-bus.h>
#include <unordered_map>

#include "AP_HAL_Linux.h"
#include "Semaphores.h"
#include "Util.h"
#include "Thread.h"

class Linux::PerfManager {
public:
    ~PerfManager();

    static PerfManager *getInstance();

    void add(Util::perf_counter_t perf);

    int dbus_list_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error);
    int dbus_get_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error);
    void dbus_process_callback();

private:
    PerfManager();

    static PerfManager *_instance;

    Thread _dbus_thread{FUNCTOR_BIND_MEMBER(&PerfManager::dbus_process_callback, void)};
    bool _run_thread = true;

    std::unordered_map<std::string, Util::perf_counter_t> _perf_hashmap;
    Semaphore _event_add_sem;

    sd_bus *_bus;
    sd_bus_slot *_slot;
};

#endif
