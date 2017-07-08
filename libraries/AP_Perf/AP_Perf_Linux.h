#pragma once

#include "AP_Perf_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#ifndef HAVE_LTTNG_UST
#include <pthread.h>


class AP_Perf_Linux : public AP_Perf_Backend {
public:

    AP_Perf_Linux();

    virtual perf_counter_t add(perf_counter_type type, const char *name) override;
    virtual void begin(perf_counter_t pc) override;
    virtual void end(perf_counter_t pc) override;
    virtual void count(perf_counter_t pc) override;

protected:
    void _debug_counters();
    uint64_t _last_debug_msec;

    /* synchronize addition of new perf counters */
    pthread_rwlock_t _perf_counters_lock;
};
#endif
#endif
