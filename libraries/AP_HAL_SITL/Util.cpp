#include "Util.h"

uint64_t HALSITL::Util::get_hw_rtc() const
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000000ULL + nanoseconds/1000ULL);
}
