#include "BetterStream.h"

#include "print_vprintf.h"

void AP_HAL::BetterStream::printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void AP_HAL::BetterStream::vprintf(const char *fmt, va_list ap)
{
    print_vprintf(this, fmt, ap);
}

size_t AP_HAL::BetterStream::write(const char *str)
{
    return write((const uint8_t *)str, strlen(str));
}

ssize_t AP_HAL::BetterStream::read(uint8_t *buffer, uint16_t count) {
    uint16_t offset = 0;
    while (count--) {
        const int16_t x = read();
        if (x == -1) {
            return offset;
        }
        buffer[offset++] = (uint8_t)x;
    }
    return offset;
}
