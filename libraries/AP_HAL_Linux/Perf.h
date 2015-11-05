#pragma once

#ifdef HAVE_SYSTEMD

#include <systemd/sd-bus.h>
#include <unordered_map>

#include "AP_HAL_Linux.h"
#include "Semaphores.h"
#include "Util.h"

class Linux::PerfManager {
public:
    ~PerfManager();

    static PerfManager* getInstance();

    void add(Util::perf_counter_t perf);

    int dbus_list_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error);
    int dbus_get_perf_callback(sd_bus_message *m, void *userdata, sd_bus_error *ret_error);
    void dbus_process_callback();

private:
    PerfManager();

    static PerfManager *_instance;

    std::unordered_map<std::string, Util::perf_counter_t> _perf_hashmap;
    Semaphore _semaphore;

    sd_bus *_bus;
    sd_bus_slot *_slot;

    pthread_t _thread;
    bool _run_thread;
};

#endif
