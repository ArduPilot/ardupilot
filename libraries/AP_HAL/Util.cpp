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
    BufferPrinter(char* str, size_t size)  :
        _offs(0), _str(str), _size(size)  {}

    size_t write(uint8_t c) override {
        if (_offs < _size) {
            _str[_offs] = c;
        }
        _offs++;
        return 1;
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
    bool read(uint8_t &b) override { return false; }
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
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
    // note that size==0 must be handled as functions like vasprintf() rely on the return
    // value being the number of bytes that would be printed if there was enough space.
    BufferPrinter buf(str, size?size-1:0);
    print_vprintf(&buf, format, ap);
    // null terminate
    size_t ret = buf._offs;
    if (ret < size) {
        // if the string did fit then nul terminate
        str[ret] = '\0';
    } else if (size > 0) {
        // if it didn't fit then terminate using passed in size
        str[size-1] = 0;
    }
    return int(ret);
}

void AP_HAL::Util::set_soft_armed(const bool b)
{
    if (b != soft_armed) {
        soft_armed = b;
        last_armed_change_ms = AP_HAL::millis();
        if (!was_watchdog_reset()) {
            persistent_data.armed = b;
        }
    }
}
