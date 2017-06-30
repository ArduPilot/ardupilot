#pragma once

#include "AP_Perf.h"
#include <atomic>
#include <vector>

class AP_Perf_Backend {
public:
    using perf_counter_type = AP_Perf::perf_counter_type;
    using perf_counter_t = AP_Perf::perf_counter_t;

    virtual perf_counter_t add(perf_counter_type type, const char *name) { return nullptr; }
    virtual void begin(perf_counter_t pc) {}
    virtual void end(perf_counter_t pc) {}
    virtual void count(perf_counter_t pc) {}

    unsigned int get_update_count() { return _update_count; }

protected:
    std::vector<AP_Perf::AP_Perf_Counter> _perf_counters;
    /* allow to check if memory pool has changed */
    std::atomic<unsigned int> _update_count;
};