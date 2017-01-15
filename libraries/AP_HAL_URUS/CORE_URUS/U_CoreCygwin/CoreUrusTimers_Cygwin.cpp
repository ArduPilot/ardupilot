
#if defined(__CYGWIN__)

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusTimers.h"
#include "CoreUrusTimers_Cygwin.h"

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <sys/times.h>
#include <sys/time.h>
#include <time.h>

static struct {
    struct timeval start_time;
} state_tv;

CLCoreUrusTimers_Cygwin::CLCoreUrusTimers_Cygwin() :
    NSCORE_URUS::CLCoreUrusTimers()
{
    gettimeofday(&state_tv.start_time, nullptr);

#if 0
    _measure_time_proccess();
#endif

}

void CLCoreUrusTimers_Cygwin::_micro_sleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}

uint64_t CLCoreUrusTimers_Cygwin::_micros64ts ()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec*1000000 + ts.tv_nsec/1000;
}

uint64_t CLCoreUrusTimers_Cygwin::_micros64tv ()
{
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e6 * ((tp.tv_sec + (tp.tv_usec * 1.0e-6)) -
                            (state_tv.start_time.tv_sec +
                             (state_tv.start_time.tv_usec * 1.0e-6)));
    return ret;
}

void CLCoreUrusTimers_Cygwin::_measure_time_proccess()
{
    uint64_t clk;
    clk = 0;

    struct timespec ts1;
    struct timespec ts2;
    struct timespec ts3;

    uint64_t now = _micros64tv();
    now = _micros64tv();

    uint64_t counter = 0;
    uint64_t nanosec = 0;

    clock_gettime(CLOCK_MONOTONIC, &ts1);

    while ((clk < 1000) || (counter == 0)) {
        clock_gettime(CLOCK_MONOTONIC, &ts3);
        clk = ts3.tv_nsec - ts1.tv_nsec;
        counter++;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts2);

    nanosec = (uint64_t)ts2.tv_nsec - (uint64_t)ts1.tv_nsec;
    uint64_t total = (uint64_t)_micros64tv() - (uint64_t)now;

    printf("measure: %lu\n", (uint64_t)total);
    printf("counter: %lu\n", (uint64_t)counter);
    printf("nanosec: %lu\n", (uint64_t)nanosec);
    printf("clkpersec: %lu\n", (unsigned long)clk);
}

uint64_t CLCoreUrusTimers_Cygwin::get_core_hrdtime ()
{
    return _micros64tv();
}

#endif // __CYGWIN__
