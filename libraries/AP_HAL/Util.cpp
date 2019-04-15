#include "AP_HAL.h"
#include "Util.h"
#include "utility/print_vprintf.h"
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/time.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "ch.h"
#include "hal.h"
#else
#include <time.h>
#endif

/* Helper class implements AP_HAL::Print so we can use utility/vprintf */
class BufferPrinter : public AP_HAL::BetterStream {
public:
    BufferPrinter(char* str, size_t size)  : _offs(0), _str(str), _size(size)  {}

    size_t write(uint8_t c) override {
        if (_offs < _size) {
            _str[_offs] = c;
            _offs++;
            return 1;
        } else {
            return 0;
        }
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = 0;
        while (size--) {
            n += write(*buffer++);
        }
        return n;
    }

    size_t _offs;
    char* const  _str;
    const size_t _size;

    uint32_t available() override { return 0; }
    int16_t read() override { return -1; }
    uint32_t txspace() override { return 0; }
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
    if (size == 0) {
        return 0;
    }
    BufferPrinter buf(str, size-1);
    print_vprintf(&buf, format, ap);
    // null terminate
    int ret = buf._offs;
    buf.write(0);
    return ret;
}

uint64_t AP_HAL::Util::get_hw_rtc() const
{
#if defined(__APPLE__) && defined(__MACH__)
    struct timeval ts;
    gettimeofday(&ts, nullptr);
    return ((long long)((ts.tv_sec * 1000000) + ts.tv_usec));
#elif HAL_HAVE_GETTIME_SETTIME
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000000ULL + nanoseconds/1000ULL);
#endif

    // no HW clock (or not one worth bothering with)
    return 0;
}

void AP_HAL::Util::set_hw_rtc(uint64_t time_utc_usec)
{
#if HAL_HAVE_GETTIME_SETTIME
    timespec ts;
    ts.tv_sec = time_utc_usec/1000000ULL;
    ts.tv_nsec = (time_utc_usec % 1000000ULL) * 1000ULL;
    clock_settime(CLOCK_REALTIME, &ts);
#endif
}

