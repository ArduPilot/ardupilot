#include "AP_HAL.h"
#include "Util.h"
#include "utility/print_vprintf.h"
#include <time.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/time.h>
#endif

/* Helper class implements AP_HAL::Print so we can use utility/vprintf */
class BufferPrinter : public AP_HAL::Print {
public:
    BufferPrinter(char* str, size_t size)  : _offs(0), _str(str), _size(size)  {}
    size_t write(uint8_t c) {
        if (_offs < _size) {
            _str[_offs] = c;
            _offs++;
            return 1;
        } else {
            return 0;
        }
    }
    size_t write(const uint8_t *buffer, size_t size) {
        size_t n = 0;
        while (size--) {
            n += write(*buffer++);
        }
        return n;
    }

    size_t _offs; 
    char* const  _str;
    const size_t _size;
};

int AP_HAL::Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int AP_HAL::Util::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    BufferPrinter buf(str, size);
    print_vprintf(&buf, format, ap);
    // null terminate if possible
    int ret = buf._offs;
    buf.write(0);
    return ret;
}

uint64_t AP_HAL::Util::get_system_clock_ms() const
{
#if defined(__APPLE__) && defined(__MACH__)
    struct timeval ts;
    gettimeofday(&ts, nullptr);
    return ((long long)((ts.tv_sec * 1000) + (ts.tv_usec / 1000)));
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000ULL + nanoseconds/1000000ULL);
#endif
}

void AP_HAL::Util::get_system_clock_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms) const
{
     // get time of day in ms
    uint64_t time_ms = get_system_clock_ms();

    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = time_ms % 1000;
    uint32_t sec_ms = (time_ms % (60 * 1000)) - ms;
    uint32_t min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);
}

// get milliseconds from now to a target time of day expressed as hour, min, sec, ms
// match starts from first value that is not -1. I.e. specifying hour=-1, minutes=10 will ignore the hour and return time until 10 minutes past 12am (utc)
uint32_t AP_HAL::Util::get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms) const
{
    // determine highest value specified (0=none, 1=ms, 2=sec, 3=min, 4=hour)
    int8_t largest_element = 0;
    if (hour != -1) {
        largest_element = 4;
    } else if (min != -1) {
        largest_element = 3;
    } else if (sec != -1) {
        largest_element = 2;
    } else if (ms != -1) {
        largest_element = 1;
    } else {
        // exit immediately if no time specified
        return 0;
    }

    // get start_time_ms as h, m, s, ms
    int32_t curr_hour, curr_min, curr_sec, curr_ms;
    get_system_clock_utc(curr_hour, curr_min, curr_sec, curr_ms);
    int32_t total_delay_ms = 0;

    // calculate ms to target
    if (largest_element >= 1) {
        total_delay_ms += ms - curr_ms;
    }
    if (largest_element == 1 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += 1000);
    }

    // calculate sec to target
    if (largest_element >= 2) {
        total_delay_ms += (sec - curr_sec)*1000;
    }
    if (largest_element == 2 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*1000));
    }

    // calculate min to target
    if (largest_element >= 3) {
        total_delay_ms += (min - curr_min)*60*1000;
    }
    if (largest_element == 3 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*60*1000));
    }

    // calculate hours to target
    if (largest_element >= 4) {
        total_delay_ms += (hour - curr_hour)*60*60*1000;
    }
    if (largest_element == 4 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (24*60*60*1000));
    }

    // total delay in milliseconds
    return static_cast<uint32_t>(total_delay_ms);
}
