#pragma once

#include "AP_Perf_Backend.h"

#ifdef HAVE_LTTNG_UST
#include <pthread.h>
#include "Perf_Lttng.h"

class AP_Perf_Lttng : public AP_Perf_Backend {
public:

    AP_Perf_Lttng();

    virtual perf_counter_t add(perf_counter_type type, const char *name) override;
    virtual void begin(perf_counter_t pc) override;
    virtual void end(perf_counter_t pc) override;
    virtual void count(perf_counter_t pc) override;

private:
    /* synchronize addition of new perf counters */
    pthread_rwlock_t _perf_counters_lock;
};
#endif